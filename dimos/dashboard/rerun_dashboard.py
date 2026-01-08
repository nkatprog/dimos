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

"""Dashboard-owned Rerun initialization + global layout.

This module intentionally lives in `dimos/dashboard/` to keep `dimos/core/*` pure.

Responsibilities:
- Start the Rerun server/viewer in the main process (CLI/integration layer).
- Connect the main process to that server (so it can send blueprints).
- Define and send a single global blueprint layout (equivalent of a dashboard).
"""

from __future__ import annotations

from typing import Any

import rerun as rr
import rerun.blueprint as rrb

from dimos.core.global_config import GlobalConfig
from dimos.dashboard.rerun_init import connect_rerun, init_rerun_server
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def init_rerun_if_enabled(global_config: GlobalConfig) -> GlobalConfig:
    """Start the Rerun server/viewer if enabled and connect this process to it.

    This should be called by the CLI before `blueprint.build(...)` so modules can
    connect to the already-running server.
    """
    if not global_config.rerun_enabled:
        return global_config
    if not global_config.viewer_backend.startswith("rerun"):
        return global_config

    server_addr = init_rerun_server(viewer_mode=global_config.viewer_backend)
    global_config = global_config.model_copy(update={"rerun_server_addr": server_addr})

    # Ensure this process is connected so we can send a blueprint immediately.
    connect_rerun(global_config=global_config, server_addr=server_addr)
    logger.info("Rerun dashboard initialized", addr=server_addr)
    return global_config


def send_global_blueprint(global_config: GlobalConfig) -> None:
    """Build and send the global Rerun blueprint layout.

    The blueprint describes the UI layout only. What shows up in each view is
    determined by entity paths logged by dashboard logger modules.
    """
    if not global_config.rerun_enabled:
        return
    if not global_config.viewer_backend.startswith("rerun"):
        return

    connect_rerun(global_config=global_config, server_addr=global_config.rerun_server_addr)

    # Right column: camera + metrics (time series). These entities are expected
    # to be logged by dashboard modules (e.g. RerunLoggerModule).
    side_panels: list[Any] = [
        rrb.Spatial2DView(
            name="Camera",
            origin="world/robot/camera/rgb",
        ),
        rrb.TimeSeriesView(
            name="Costmap (ms)",
            origin="/metrics/costmap",
            contents=[
                "+ /metrics/costmap/calc_ms",
                "+ /metrics/costmap/latency_ms",
            ],
        ),
        rrb.TimeSeriesView(
            name="Voxel Pipeline (ms)",
            origin="/metrics/voxel_map",
            contents=[
                "+ /metrics/voxel_map/extract_ms",
                "+ /metrics/voxel_map/transport_ms",
                "+ /metrics/voxel_map/publish_ms",
                "+ /metrics/voxel_map/latency_ms",
            ],
        ),
        rrb.TimeSeriesView(
            name="Voxel Count",
            origin="/metrics/voxel_map",
            contents=["+ /metrics/voxel_map/voxel_count"],
        ),
    ]

    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                name="3D View",
                origin="world",
                background=[0, 0, 0],
            ),
            rrb.Vertical(
                *side_panels,
                row_shares=[2, 1, 1, 1],
            ),
            column_shares=[3, 1],
        ),
        rrb.TimePanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.BlueprintPanel(state="collapsed"),
    )

    rr.send_blueprint(blueprint)


