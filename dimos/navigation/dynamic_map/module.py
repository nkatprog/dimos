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

"""DynamicMap — NativeModule wrapper for the OctoMap-based C++ occupancy grid.

The real computation runs in ``dynamic_map_node`` (a compiled C++ subprocess).
This Python class:

* Declares the In/Out stream ports so the blueprint engine wires them up.
* Converts config fields to ``--key value`` CLI args that the subprocess parses.
* Manages the subprocess lifecycle (start / SIGTERM / watchdog).

See ``native/dynamic_map_node.cpp`` for the C++ implementation and
``DESIGN.md`` for the algorithmic background.

Build the C++ binary before running::

    cd dimos/navigation/dynamic_map/native
    cmake -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j$(nproc)
    # Requires: sudo apt install liboctomap-dev
"""

from __future__ import annotations

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class DynamicMapConfig(NativeModuleConfig):
    """Configuration for the DynamicMap native module.

    All fields below (except those in ``NativeModuleConfig``) are forwarded
    to the C++ subprocess as ``--field_name value`` CLI arguments via
    :meth:`NativeModuleConfig.to_cli_args`.
    """

    cwd: str | None = "."
    executable: str = "native/build/dynamic_map_node"
    build_command: str | None = (
        "cmake -B native/build -DCMAKE_BUILD_TYPE=Release native "
        "&& cmake --build native/build -j$(nproc)"
    )

    #: OctoMap voxel resolution in metres.
    resolution: float = 0.15
    #: Rays beyond this distance (metres) are truncated at max_range.
    max_range: float = 15.0
    #: Log-odds threshold above which a voxel is considered "occupied"
    #: and included in the published PointCloud2.
    occ_threshold: float = 0.5
    #: Map publication frequency in Hz.
    publish_rate: float = 0.5


class DynamicMap(NativeModule):
    """OctoMap-based dynamic global mapping via a C++ NativeModule subprocess.

    Integrates each lidar scan using OctoMap's ``insertPointCloud()``, which
    performs full 3-D DDA ray-casting and Bayesian log-odds updates.  Moved
    obstacles fade from the map as rays continue to pass through their former
    positions.

    The stream interface is compatible with ``VoxelGridMapper`` — both modules
    expose ``registered_scan``/``raw_odom`` inputs and ``global_map``/``odom``
    outputs — so they are drop-in interchangeable in blueprints.

    Ports
    -----
    registered_scan : In[PointCloud2]
        World-frame lidar point cloud (from GO2Connection.lidar via remapping).
    raw_odom : In[PoseStamped]
        Raw robot pose (from GO2Connection.odom via remapping).
    global_map : Out[PointCloud2]
        Occupied voxels above ``occ_threshold`` as a PointCloud2.
    odom : Out[PoseStamped]
        Pass-through odometry (no loop closure in v1).
    """

    default_config: type[DynamicMapConfig] = DynamicMapConfig  # type: ignore[assignment]

    registered_scan: In[PointCloud2]
    raw_odom: In[PoseStamped]

    global_map: Out[PointCloud2]
    odom: Out[PoseStamped]


# Blueprint factory — used by autoconnect() and dimos run
dynamic_map = DynamicMap.blueprint
