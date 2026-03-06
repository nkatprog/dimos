#!/usr/bin/env python3
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

"""Go2 blueprint with ControlCoordinator-based 100Hz path following.

Uses MockTwistBaseAdapter (in-memory) for the coordinator's hardware write
phase, and bridges velocity commands to GO2Connection via the coordinator's
cmd_vel: Out[Twist] port which autoconnect wires to GO2Connection's
cmd_vel: In[Twist].

All control compute (PurePursuit + PID + VelocityProfiler) runs at 100Hz
inside the coordinator tick loop via PathFollowerTask.
"""

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import control_coordinator
from dimos.core.blueprints import autoconnect
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

coordinator = control_coordinator(
    tick_rate=100.0,
    hardware=[
        HardwareComponent(
            hardware_id="base",
            hardware_type=HardwareType.BASE,
            joints=["base_vx", "base_vy", "base_wz"],
            adapter_type="mock_twist_base",
            auto_enable=True,
        ),
    ],
    tasks=[],  # PathFollowerTask added dynamically by nav module via set_coordinator()
)

unitree_go2_coordinator = autoconnect(
    unitree_go2_basic,
    coordinator,
    voxel_mapper(voxel_size=0.1),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
).global_config(n_workers=7, robot_model="unitree_go2")

__all__ = ["unitree_go2_coordinator"]
