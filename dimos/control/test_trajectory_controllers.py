# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Compare old PController vs new PurePursuit+PID on the same trajectory.

Uses Ivan's test_trajectory fixture to get a real 363-pose A* path through
a recorded office environment, then runs both controllers through a kinematic
simulation and compares tracking metrics.
"""

from collections.abc import Callable
from dataclasses import dataclass, field
import math

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.control.test_trajectory import get_moment  # noqa: F401 — pytest fixture
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.controllers import (
    PController,
    PIDCrossTrackController,
    PurePursuitController,
)
from dimos.navigation.replanning_a_star.path_distancer import PathDistancer
from dimos.navigation.replanning_a_star.velocity_profiler import VelocityProfiler
from dimos.utils.trigonometry import angle_diff

# Type alias for controller step function
StepFn = Callable[[PoseStamped, PathDistancer, int], Twist]


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class TickMetrics:
    cross_track_error: float
    heading_error: float
    speed: float
    distance_to_goal: float
    angular_velocity: float


@dataclass
class SimulationResult:
    ticks: list[TickMetrics] = field(default_factory=list)
    total_time: float = 0.0
    reached_goal: bool = False
    actual_path_length: float = 0.0

    mean_cross_track_error: float = 0.0
    max_cross_track_error: float = 0.0
    rms_cross_track_error: float = 0.0
    mean_speed: float = 0.0
    path_length_ratio: float = 0.0
    angular_smoothness: float = 0.0

    def compute_summary(self, planned_path_length: float) -> None:
        if not self.ticks:
            return
        cte_abs = [abs(t.cross_track_error) for t in self.ticks]
        cte_sq = [t.cross_track_error**2 for t in self.ticks]
        self.mean_cross_track_error = sum(cte_abs) / len(cte_abs)
        self.max_cross_track_error = max(cte_abs)
        self.rms_cross_track_error = math.sqrt(sum(cte_sq) / len(cte_sq))
        self.mean_speed = sum(t.speed for t in self.ticks) / len(self.ticks)
        if planned_path_length > 0:
            self.path_length_ratio = self.actual_path_length / planned_path_length
        # Angular smoothness: sum of |delta_omega| between consecutive ticks
        omegas = [t.angular_velocity for t in self.ticks]
        self.angular_smoothness = sum(abs(omegas[i] - omegas[i - 1]) for i in range(1, len(omegas)))


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def get_path_tangent_yaw(path_array: NDArray, index: int) -> float:
    if index < len(path_array) - 1:
        dx = path_array[index + 1][0] - path_array[index][0]
        dy = path_array[index + 1][1] - path_array[index][1]
    elif index > 0:
        dx = path_array[index][0] - path_array[index - 1][0]
        dy = path_array[index][1] - path_array[index - 1][1]
    else:
        return 0.0
    return float(np.arctan2(dy, dx))


# ---------------------------------------------------------------------------
# Core kinematic simulation
# ---------------------------------------------------------------------------


def simulate_controller(
    path: Path,
    step_fn: StepFn,
    dt: float,
    max_steps: int,
    goal_tolerance: float = 0.3,
    orientation_tolerance: float = 0.35,
) -> SimulationResult:
    """Run a controller through a kinematic sim on the given path."""
    path_distancer = PathDistancer(path)
    path_array = np.array([[p.position.x, p.position.y] for p in path.poses])
    planned_length = float(path_distancer._cumulative_dists[-1])

    # Start at first path pose
    x = path.poses[0].position.x
    y = path.poses[0].position.y
    yaw = path.poses[0].orientation.euler[2]

    goal_yaw = path.poses[-1].orientation.euler[2]

    result = SimulationResult()
    actual_distance = 0.0
    prev_x, prev_y = x, y

    for _ in range(max_steps):
        current_pose = make_pose(x, y, yaw)
        current_pos = np.array([x, y])

        dist_to_goal = path_distancer.distance_to_goal(current_pos)

        # Goal check
        if dist_to_goal < goal_tolerance:
            if abs(angle_diff(goal_yaw, yaw)) < orientation_tolerance:
                result.reached_goal = True
                break

        closest_idx = path_distancer.find_closest_point_index(current_pos)

        twist = step_fn(current_pose, path_distancer, closest_idx)

        # Record metrics
        cte = path_distancer.get_signed_cross_track_error(current_pos)
        tangent_yaw = get_path_tangent_yaw(path_array, closest_idx)
        heading_err = angle_diff(yaw, tangent_yaw)
        speed = math.sqrt(twist.linear.x**2 + twist.linear.y**2)

        result.ticks.append(
            TickMetrics(
                cross_track_error=cte,
                heading_error=heading_err,
                speed=speed,
                distance_to_goal=dist_to_goal,
                angular_velocity=twist.angular.z,
            )
        )

        # Kinematic update (unicycle)
        vx = twist.linear.x
        vy = twist.linear.y
        wz = twist.angular.z

        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw += wz * dt
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        step_dist = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
        actual_distance += step_dist
        prev_x, prev_y = x, y

    result.total_time = len(result.ticks) * dt
    result.actual_path_length = actual_distance
    result.compute_summary(planned_length)
    return result


# ---------------------------------------------------------------------------
# Controller wrappers
# ---------------------------------------------------------------------------


def simulate_old_controller(path: Path, global_config: GlobalConfig) -> SimulationResult:
    """Simulate the old 10Hz PController."""
    controller = PController(global_config, speed=0.55, control_frequency=10)

    def step_fn(pose: PoseStamped, distancer: PathDistancer, idx: int) -> Twist:
        lookahead = distancer.find_lookahead_point(idx)
        return controller.advance(lookahead, pose)

    return simulate_controller(path, step_fn, dt=0.1, max_steps=2000)


def simulate_new_controller(
    path: Path,
    global_config: GlobalConfig,
    frequency: float = 100.0,
) -> SimulationResult:
    """Simulate the new PurePursuit + PID cross-track controller."""
    pp = PurePursuitController(
        global_config,
        control_frequency=frequency,
        min_lookahead=0.3,
        max_lookahead=2.0,
        lookahead_gain=0.5,
        max_linear_speed=0.8,
    )
    pid = PIDCrossTrackController(
        control_frequency=frequency,
        k_p=1.5,
        k_i=0.1,
        k_d=0.2,
        max_correction=0.6,
        max_integral=0.3,
    )
    profiler = VelocityProfiler(
        max_linear_speed=0.8,
        max_angular_speed=1.5,
        max_linear_accel=1.0,
        max_linear_decel=2.0,
        max_centripetal_accel=1.0,
        min_speed=0.05,
    )
    velocity_profile = profiler.compute_profile(path)
    prev_speed = [0.0]

    def step_fn(pose: PoseStamped, distancer: PathDistancer, idx: int) -> Twist:
        speed = prev_speed[0]
        target_speed = float(velocity_profile[min(idx, len(velocity_profile) - 1)])

        lookahead = distancer.find_adaptive_lookahead_point(
            idx,
            speed,
            min_lookahead=0.3,
            max_lookahead=2.0,
        )
        curvature = distancer.get_curvature_at_index(idx)

        twist = pp.advance(lookahead, pose, current_speed=target_speed, path_curvature=curvature)

        # PID cross-track correction
        pos = np.array([pose.position.x, pose.position.y])
        cte = distancer.get_signed_cross_track_error(pos)
        correction = pid.compute_correction(cte)

        wz = float(np.clip(twist.angular.z + correction, -1.2, 1.2))

        prev_speed[0] = math.sqrt(twist.linear.x**2 + twist.linear.y**2)

        return Twist(
            linear=Vector3(twist.linear.x, twist.linear.y, 0.0),
            angular=Vector3(0.0, 0.0, wz),
        )

    dt = 1.0 / frequency
    max_steps = int(200.0 * frequency)  # 200s timeout
    return simulate_controller(path, step_fn, dt=dt, max_steps=max_steps)


# ---------------------------------------------------------------------------
# Comparison output
# ---------------------------------------------------------------------------


def print_comparison_table(old: SimulationResult, new: SimulationResult) -> None:
    print("\n" + "=" * 70)
    print(f"{'Metric':<35} {'PController':>15} {'PurePursuit+PID':>15}")
    print("-" * 70)
    rows = [
        ("Reached goal", str(old.reached_goal), str(new.reached_goal)),
        ("Time to goal (s)", f"{old.total_time:.2f}", f"{new.total_time:.2f}"),
        (
            "Mean |CTE| (m)",
            f"{old.mean_cross_track_error:.4f}",
            f"{new.mean_cross_track_error:.4f}",
        ),
        ("Max |CTE| (m)", f"{old.max_cross_track_error:.4f}", f"{new.max_cross_track_error:.4f}"),
        ("RMS CTE (m)", f"{old.rms_cross_track_error:.4f}", f"{new.rms_cross_track_error:.4f}"),
        ("Mean speed (m/s)", f"{old.mean_speed:.3f}", f"{new.mean_speed:.3f}"),
        ("Path length ratio", f"{old.path_length_ratio:.3f}", f"{new.path_length_ratio:.3f}"),
        ("Angular smoothness", f"{old.angular_smoothness:.2f}", f"{new.angular_smoothness:.2f}"),
        ("Actual path (m)", f"{old.actual_path_length:.2f}", f"{new.actual_path_length:.2f}"),
    ]
    for label, v_old, v_new in rows:
        print(f"{label:<35} {v_old!s:>15} {v_new!s:>15}")
    print("=" * 70)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def global_config():
    return GlobalConfig()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_old_controller(get_moment, global_config):
    """PController reaches the goal on the office trajectory."""
    moment = get_moment()
    result = simulate_old_controller(moment["path"], global_config)

    print(
        f"\nOld controller: reached={result.reached_goal}, "
        f"time={result.total_time:.1f}s, rms_cte={result.rms_cross_track_error:.4f}m"
    )

    assert result.reached_goal, (
        f"PController failed to reach goal. "
        f"Final distance: {result.ticks[-1].distance_to_goal:.2f}m"
    )
    assert result.path_length_ratio < 2.0


def test_new_controller(get_moment, global_config):
    """PurePursuit+PID reaches the goal on the office trajectory."""
    moment = get_moment()
    result = simulate_new_controller(moment["path"], global_config)

    print(
        f"\nNew controller: reached={result.reached_goal}, "
        f"time={result.total_time:.1f}s, rms_cte={result.rms_cross_track_error:.4f}m"
    )

    assert result.reached_goal, (
        f"PurePursuit+PID failed to reach goal. "
        f"Final distance: {result.ticks[-1].distance_to_goal:.2f}m"
    )
    assert result.path_length_ratio < 1.5


def test_new_controller_10hz(get_moment, global_config):
    """PurePursuit+PID at 10Hz — same rate as old controller for fair comparison."""
    moment = get_moment()
    result = simulate_new_controller(moment["path"], global_config, frequency=10.0)

    print(
        f"\nNew controller @10Hz: reached={result.reached_goal}, "
        f"time={result.total_time:.1f}s, rms_cte={result.rms_cross_track_error:.4f}m"
    )

    assert result.reached_goal, (
        f"PurePursuit+PID @10Hz failed to reach goal. "
        f"Final distance: {result.ticks[-1].distance_to_goal:.2f}m"
    )


def test_comparison(get_moment, global_config):
    """Compare all three variants: old 10Hz, new 10Hz, new 100Hz."""
    moment = get_moment()
    path = moment["path"]

    old_result = simulate_old_controller(path, global_config)
    new_10hz_result = simulate_new_controller(path, global_config, frequency=10.0)
    new_100hz_result = simulate_new_controller(path, global_config, frequency=100.0)

    print("\n" + "=" * 85)
    print(f"{'Metric':<35} {'PCtrl 10Hz':>15} {'PP+PID 10Hz':>15} {'PP+PID 100Hz':>15}")
    print("-" * 85)
    results = [old_result, new_10hz_result, new_100hz_result]
    rows = [
        ("Reached goal", *[str(r.reached_goal) for r in results]),
        ("Time to goal (s)", *[f"{r.total_time:.2f}" for r in results]),
        ("Mean |CTE| (m)", *[f"{r.mean_cross_track_error:.4f}" for r in results]),
        ("Max |CTE| (m)", *[f"{r.max_cross_track_error:.4f}" for r in results]),
        ("RMS CTE (m)", *[f"{r.rms_cross_track_error:.4f}" for r in results]),
        ("Mean speed (m/s)", *[f"{r.mean_speed:.3f}" for r in results]),
        ("Path length ratio", *[f"{r.path_length_ratio:.3f}" for r in results]),
        ("Angular smoothness", *[f"{r.angular_smoothness:.2f}" for r in results]),
        ("Actual path (m)", *[f"{r.actual_path_length:.2f}" for r in results]),
    ]
    for label, v1, v2, v3 in rows:
        print(f"{label:<35} {v1:>15} {v2:>15} {v3:>15}")
    print("=" * 85)

    assert old_result.reached_goal, "PController did not reach goal"
    assert new_10hz_result.reached_goal, "PurePursuit+PID @10Hz did not reach goal"
    assert new_100hz_result.reached_goal, "PurePursuit+PID @100Hz did not reach goal"

    # New controller at same frequency should still beat old controller
    assert new_10hz_result.rms_cross_track_error < old_result.rms_cross_track_error, (
        f"Expected PP+PID @10Hz RMS CTE ({new_10hz_result.rms_cross_track_error:.4f}) "
        f"< PController RMS CTE ({old_result.rms_cross_track_error:.4f})"
    )
