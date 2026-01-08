# Copyright 2025 Dimensional Inc.
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

"""Dashboard module for visualization and monitoring.

Rerun Initialization:
    The CLI initializes the Rerun server/viewer and sends a global blueprint layout.
    Dashboard modules connect to the server via connect_rerun() and log entities.

Blueprint usage:
    from dimos.dashboard import rerun_viz

    blueprint = autoconnect(
        robot_connection(),
        rerun_viz(voxel_box_size=0.1, voxel_colormap="turbo"),
    )
"""

from dimos.dashboard.rerun_autolog import autolog_to_rerun
from dimos.dashboard.rerun_init import connect_rerun, init_rerun_server, shutdown_rerun
from dimos.dashboard.rerun_logger_module import rerun_logger
from dimos.dashboard.tf_rerun_module import tf_rerun

__all__ = [
    "autolog_to_rerun",
    "connect_rerun",
    "init_rerun_server",
    "rerun_viz",
    "shutdown_rerun",
]


def rerun_viz(**kwargs):  # type: ignore[no-untyped-def]
    """Unified Rerun visualization blueprint: TF + core stream logging.

    This keeps blueprint imports clean (one thing to import) and keeps all Rerun
    policy in the dashboard layer.
    """
    from dimos.core.blueprints import autoconnect

    return autoconnect(
        tf_rerun(),
        rerun_logger(**kwargs),
    )
