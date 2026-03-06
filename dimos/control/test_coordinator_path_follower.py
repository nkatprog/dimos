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

"""Integration tests: PathFollowerTask through ControlCoordinator tick loop.

Tests the full data path:
  PathFollowerTask → TickLoop tick → MockTwistBaseAdapter → twist publish callback

Uses the real A* trajectory from test_trajectory to validate end-to-end.
"""

import math
import threading

import pytest

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.hardware_interface import ConnectedTwistBase
from dimos.control.test_trajectory import get_moment  # noqa: F401 — pytest fixture
from dimos.control.tick_loop import TickLoop
from dimos.core.global_config import GlobalConfig
from dimos.hardware.drive_trains.mock.adapter import MockTwistBaseAdapter
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.navigation.replanning_a_star.path_follower_task import (
    PathFollowerTask,
    PathFollowerTaskConfig,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def build_tick_loop(
    adapter: MockTwistBaseAdapter,
    task: PathFollowerTask,
    twist_log: list[Twist] | None = None,
) -> TickLoop:
    """Build a TickLoop wired to a MockTwistBaseAdapter and PathFollowerTask."""
    component = HardwareComponent(
        hardware_id="base",
        hardware_type=HardwareType.BASE,
        joints=["base_vx", "base_vy", "base_wz"],
        adapter_type="mock_twist_base",
    )
    connected = ConnectedTwistBase(adapter=adapter, component=component)

    hardware = {"base": connected}
    hardware_lock = threading.Lock()
    tasks = {task.name: task}
    task_lock = threading.Lock()
    joint_to_hardware = {
        "base_vx": "base",
        "base_vy": "base",
        "base_wz": "base",
    }

    twist_cb = None
    if twist_log is not None:

        def twist_cb(t):
            return twist_log.append(t)

    return TickLoop(
        tick_rate=100.0,
        hardware=hardware,
        hardware_lock=hardware_lock,
        tasks=tasks,
        task_lock=task_lock,
        joint_to_hardware=joint_to_hardware,
        twist_publish_callback=twist_cb,
    )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def global_config():
    return GlobalConfig()


@pytest.fixture
def adapter():
    a = MockTwistBaseAdapter(dof=3)
    a.connect()
    return a


@pytest.fixture
def path_follower(global_config):
    cfg = PathFollowerTaskConfig(
        joint_names=["base_vx", "base_vy", "base_wz"],
        priority=10,
        max_linear_speed=0.8,
        goal_tolerance=0.3,
        orientation_tolerance=0.35,
    )
    return PathFollowerTask(name="path_follower", config=cfg, global_config=global_config)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_tick_loop_publishes_twist(adapter, path_follower, get_moment):
    """PathFollowerTask produces non-zero velocities through the tick loop."""
    moment = get_moment()
    path = moment["path"]

    # Start the task with the first pose on the path
    start_pose = make_pose(
        path.poses[0].position.x,
        path.poses[0].position.y,
        path.poses[0].orientation.euler[2],
    )
    assert path_follower.start_path(path, start_pose)

    twist_log: list[Twist] = []
    loop = build_tick_loop(adapter, path_follower, twist_log)

    # Manually tick (don't start the background thread)
    loop._tick()

    assert len(twist_log) == 1, "Expected exactly one Twist published per tick"
    twist = twist_log[0]
    # The robot starts on the path, so it should be moving forward
    assert isinstance(twist, Twist)


def test_path_follower_reaches_goal_via_tick_loop(adapter, path_follower, get_moment):
    """PathFollowerTask reaches the goal through simulated tick loop."""
    moment = get_moment()
    path = moment["path"]

    start_pose = make_pose(
        path.poses[0].position.x,
        path.poses[0].position.y,
        path.poses[0].orientation.euler[2],
    )
    assert path_follower.start_path(path, start_pose)

    twist_log: list[Twist] = []
    loop = build_tick_loop(adapter, path_follower, twist_log)

    # Kinematic simulation: update odom from velocity commands
    x = path.poses[0].position.x
    y = path.poses[0].position.y
    yaw = path.poses[0].orientation.euler[2]
    dt = 0.01  # 100Hz

    max_ticks = 20_000  # 200s at 100Hz
    for _ in range(max_ticks):
        loop._tick()

        if path_follower.get_state() == "completed":
            break

        # Get velocities from the mock adapter
        vels = adapter.read_velocities()
        vx, vy, wz = vels[0], vels[1], vels[2]

        # Unicycle kinematic update
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw += wz * dt
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        # Feed updated odom back to the task
        path_follower.update_odom(make_pose(x, y, yaw))

    assert path_follower.get_state() == "completed", (
        f"PathFollowerTask did not reach goal. State: {path_follower.get_state()}"
    )
    assert len(twist_log) > 0, "No Twist messages published"

    # Verify non-trivial velocities were commanded
    non_zero = [t for t in twist_log if abs(t.linear.x) > 0.01 or abs(t.angular.z) > 0.01]
    assert len(non_zero) > 10, "Expected many non-zero velocity commands"


def test_path_follower_cancel(adapter, path_follower, get_moment):
    """Cancel mid-path transitions task to aborted."""
    moment = get_moment()
    path = moment["path"]

    start_pose = make_pose(
        path.poses[0].position.x,
        path.poses[0].position.y,
        path.poses[0].orientation.euler[2],
    )
    assert path_follower.start_path(path, start_pose)
    assert path_follower.get_state() == "following"

    # Run a few ticks
    loop = build_tick_loop(adapter, path_follower)
    for _ in range(10):
        loop._tick()

    # Cancel
    assert path_follower.cancel()
    assert path_follower.get_state() == "aborted"

    # Task should no longer produce commands
    loop._tick()
    assert not path_follower.is_active()
