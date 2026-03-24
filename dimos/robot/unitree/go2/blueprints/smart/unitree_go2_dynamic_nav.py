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

"""unitree-go2-dynamic-nav blueprint.

Extends the base ``unitree_go2`` blueprint (navigation + planner stack) by
swapping out ``VoxelGridMapper`` for ``DynamicMap`` — a Bayesian log-odds
occupancy grid with 3-D DDA ray-casting.  Dynamic obstacles (moved boulders,
pushed chairs) fade from the map as rays continue to pass through their
former locations.

Usage::

    dimos run unitree-go2-dynamic-nav
    dimos --replay run unitree-go2-dynamic-nav
"""

from dimos.core.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.navigation.dynamic_map.module import DynamicMap
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

unitree_go2_dynamic_nav = (
    autoconnect(
        unitree_go2_basic,
        DynamicMap.blueprint(),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        WavefrontFrontierExplorer.blueprint(),
        PatrollingModule.blueprint(),
    )
    .remappings(
        [
            # GO2Connection publishes on "lidar" and "odom".
            # DynamicMap expects "registered_scan" and "raw_odom".
            (DynamicMap, "registered_scan", "lidar"),
            (DynamicMap, "raw_odom", "odom"),
            # DynamicMap passes odometry through on "odom" — downstream
            # modules (planner, costmapper) read from "odom" directly so
            # no extra remapping needed there.
            # CostMapper reads from "global_map" which DynamicMap publishes.
        ]
    )
    .global_config(n_workers=7, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_dynamic_nav"]
