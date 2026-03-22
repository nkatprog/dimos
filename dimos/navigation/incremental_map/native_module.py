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

"""NativeModule wrapper for the native IncrementalMap executable.

Wraps the C++/Rust incremental map binary as a DimOS NativeModule.
The binary receives LCM topic names via CLI args and does pub/sub
on the LCM multicast bus directly.

Currently uses the Python stub (native/incremental_map_stub.py) as the
executable.  Replace ``executable`` with the compiled C++/Rust binary
when available.

Usage::

    from dimos.navigation.incremental_map.native_module import NativeIncrementalMap

    autoconnect(
        UnityBridgeModule.blueprint(),
        NativeIncrementalMap.blueprint(),
    ).build().loop()
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_STUB_PATH = Path(__file__).parent / "native" / "incremental_map_stub.py"


class NativeIncrementalMapConfig(NativeModuleConfig):
    """Configuration for the native IncrementalMap executable."""

    # Executable: Python stub by default; replace with C++/Rust binary
    executable: str = "python3"
    extra_args: list[str] = [str(_STUB_PATH)]  # type: ignore[assignment]

    # Map parameters (forwarded to binary via CLI)
    voxel_size: float = 0.15
    key_trans: float = 0.5
    key_deg: float = 10.0
    loop_search_radius: float = 3.0
    loop_time_thresh: float = 10.0
    loop_score_thresh: float = 0.5
    loop_submap_half_range: int = 3
    icp_max_iter: int = 30
    icp_max_dist: float = 5.0
    min_loop_detect_duration: float = 5.0
    map_publish_rate: float = 0.5
    registered_input: bool = True


class NativeIncrementalMap(NativeModule[NativeIncrementalMapConfig]):
    """NativeModule wrapper for the incremental map binary.

    Ports:
        odom (In[Odometry]): Raw odometry from robot or sim.
        registered_scan (In[PointCloud2]): World-frame registered lidar.
        global_map (Out[PointCloud2]): Accumulated corrected map.
        corrected_odom (Out[Odometry]): Loop-closure-corrected odometry.
    """

    default_config = NativeIncrementalMapConfig

    odom: In[Odometry]
    registered_scan: In[PointCloud2]
    global_map: Out[PointCloud2]
    corrected_odom: Out[Odometry]


__all__ = ["NativeIncrementalMap", "NativeIncrementalMapConfig"]
