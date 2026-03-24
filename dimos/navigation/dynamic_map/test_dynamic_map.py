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

"""Tests for DynamicMap NativeModule Python wrapper.

These tests cover the Python side only (config, stream declarations, CLI arg
generation, path resolution).  They do NOT require the C++ binary to be
compiled — the subprocess is never launched.
"""

from __future__ import annotations

from collections.abc import Generator
from pathlib import Path
import pickle
from typing import get_origin, get_type_hints

import pytest

from dimos.core.stream import In, Out
from dimos.navigation.dynamic_map.module import DynamicMap, DynamicMapConfig

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_module() -> DynamicMap:
    """Instantiate DynamicMap and call _resolve_paths() as NativeModule.__init__ does."""
    m = DynamicMap()
    return m


@pytest.fixture
def module() -> Generator[DynamicMap, None, None]:
    m = _make_module()
    try:
        yield m
    finally:
        m.stop()


# ---------------------------------------------------------------------------
# Config defaults
# ---------------------------------------------------------------------------


class TestDynamicMapConfig:
    def test_config_defaults(self) -> None:
        cfg = DynamicMapConfig()
        assert cfg.resolution == pytest.approx(0.15)
        assert cfg.max_range == pytest.approx(15.0)
        assert cfg.occ_threshold == pytest.approx(0.5)
        assert cfg.publish_rate == pytest.approx(0.5)

    def test_executable_default(self) -> None:
        cfg = DynamicMapConfig()
        assert "dynamic_map_node" in cfg.executable

    def test_build_command_set(self) -> None:
        """A build_command must be set so the module can self-compile."""
        cfg = DynamicMapConfig()
        assert cfg.build_command is not None
        assert "cmake" in cfg.build_command

    def test_cli_args_contain_resolution(self) -> None:
        cfg = DynamicMapConfig(resolution=0.2)
        args = cfg.to_cli_args()
        assert "--resolution" in args
        idx = args.index("--resolution")
        assert args[idx + 1] == "0.2"

    def test_cli_args_contain_max_range(self) -> None:
        cfg = DynamicMapConfig(max_range=20.0)
        args = cfg.to_cli_args()
        assert "--max_range" in args
        assert "20.0" in args

    def test_cli_args_contain_occ_threshold(self) -> None:
        cfg = DynamicMapConfig(occ_threshold=0.7)
        args = cfg.to_cli_args()
        assert "--occ_threshold" in args
        assert "0.7" in args

    def test_cli_args_contain_publish_rate(self) -> None:
        cfg = DynamicMapConfig(publish_rate=1.0)
        args = cfg.to_cli_args()
        assert "--publish_rate" in args
        assert "1.0" in args

    def test_cli_args_exclude_none_values(self) -> None:
        """Fields whose value is None must not appear in CLI args."""
        cfg = DynamicMapConfig()
        args = cfg.to_cli_args()
        # base NativeModuleConfig fields (build_command, cwd, etc.) are excluded
        # by NativeModuleConfig.to_cli_args() ignore list
        for _i, arg in enumerate(args):
            if arg == "--build_command":
                pytest.fail("build_command should not appear in to_cli_args output")


# ---------------------------------------------------------------------------
# Stream port declarations
# ---------------------------------------------------------------------------


class TestDynamicMapPorts:
    def test_in_ports_declared(self) -> None:
        hints = get_type_hints(DynamicMap)
        in_ports = {k for k, v in hints.items() if get_origin(v) is In}
        assert "registered_scan" in in_ports, f"registered_scan not in {in_ports}"
        assert "raw_odom" in in_ports, f"raw_odom not in {in_ports}"

    def test_out_ports_declared(self) -> None:
        hints = get_type_hints(DynamicMap)
        out_ports = {k for k, v in hints.items() if get_origin(v) is Out}
        assert "global_map" in out_ports, f"global_map not in {out_ports}"
        assert "odom" in out_ports, f"odom not in {out_ports}"

    def test_registered_scan_type(self) -> None:
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        hints = get_type_hints(DynamicMap)
        # In[PointCloud2].__args__[0] == PointCloud2
        assert hints["registered_scan"].__args__[0] is PointCloud2

    def test_raw_odom_type(self) -> None:
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        hints = get_type_hints(DynamicMap)
        assert hints["raw_odom"].__args__[0] is PoseStamped

    def test_global_map_type(self) -> None:
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        hints = get_type_hints(DynamicMap)
        assert hints["global_map"].__args__[0] is PointCloud2

    def test_odom_type(self) -> None:
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        hints = get_type_hints(DynamicMap)
        assert hints["odom"].__args__[0] is PoseStamped


# ---------------------------------------------------------------------------
# Module instantiation and path resolution
# ---------------------------------------------------------------------------


class TestDynamicMapModule:
    def test_module_instantiates(self, module: DynamicMap) -> None:
        assert module is not None
        assert isinstance(module, DynamicMap)

    def test_cwd_resolves_to_existing_dir(self, module: DynamicMap) -> None:
        """After _resolve_paths(), cwd must point to an existing directory."""
        cwd = module.config.cwd
        assert cwd is not None
        assert Path(cwd).exists(), f"cwd does not exist: {cwd}"
        assert Path(cwd).is_dir()

    def test_executable_path_contains_binary_name(self, module: DynamicMap) -> None:
        assert "dynamic_map_node" in module.config.executable

    def test_default_config_on_module(self, module: DynamicMap) -> None:
        assert module.config.resolution == pytest.approx(0.15)
        assert module.config.max_range == pytest.approx(15.0)

    def test_blueprint_factory_picklable(self) -> None:
        """Blueprint factory (functools.partial) must survive pickle round-trip.

        This is required for forkserver worker serialisation.
        """
        factory = DynamicMap.blueprint
        data = pickle.dumps(factory)
        factory2 = pickle.loads(data)
        assert factory2 is not None
        assert factory2.args[0] is DynamicMap

    def test_native_binary_build_cmd_references_native_dir(self, module: DynamicMap) -> None:
        """build_command must reference the native/ subdirectory."""
        cmd = module.config.build_command
        assert cmd is not None
        assert "native" in cmd
