# Copyright 2025-2026 Dimensional Inc.
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

from dataclasses import asdict, dataclass, field
import time

from reactivex import operators as ops

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs import Float32
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = field(default_factory=HeightCostConfig)


class CostMapper(Module):
    default_config = Config
    config: Config

    global_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]
    costmap_calc_ms: Out[Float32]
    costmap_latency_ms: Out[Float32]

    @rpc
    def start(self) -> None:
        super().start()

        def _on_global_map(msg: PointCloud2) -> None:
            # End-to-end latency for this module is measured from receipt of the input
            # message to completion of publishing all outputs.
            rx_time = time.monotonic()

            t0 = time.perf_counter()
            grid = self._calculate_costmap(msg)
            calc_ms = (time.perf_counter() - t0) * 1000.0

            # Primary output
            self.global_costmap.publish(grid)

            # Metrics outputs (visualized by dashboard logger modules)
            self.costmap_calc_ms.publish(Float32(calc_ms))
            latency_ms = (time.monotonic() - rx_time) * 1000.0
            self.costmap_latency_ms.publish(Float32(latency_ms))

        self._disposables.add(
            self.global_map.observable()  # type: ignore[no-untyped-call]
            .pipe(ops.filter(lambda x: x is not None))
            .subscribe(_on_global_map)
        )

    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        fn = OCCUPANCY_ALGOS[self.config.algo]
        return fn(msg, **asdict(self.config.config))


cost_mapper = CostMapper.blueprint
