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

"""Tests for the NativeIncrementalMap module wrapper and stub executable."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys

from dimos.navigation.incremental_map.native_module import (
    NativeIncrementalMap,
    NativeIncrementalMapConfig,
)


def test_native_incremental_map_config_defaults():
    cfg = NativeIncrementalMapConfig()
    assert cfg.voxel_size == 0.15
    assert cfg.key_trans == 0.5
    assert cfg.loop_search_radius == 3.0
    assert cfg.executable == "python3"


def test_native_incremental_map_has_ports():
    mod = NativeIncrementalMap()
    assert hasattr(mod, "odom")
    assert hasattr(mod, "registered_scan")
    assert hasattr(mod, "global_map")
    assert hasattr(mod, "corrected_odom")
    mod.stop()


def test_stub_cli_parses_correctly():
    """Stub executable should exit cleanly when given --help."""
    stub_path = Path(__file__).parent / "native" / "incremental_map_stub.py"
    assert stub_path.exists(), f"Stub not found: {stub_path}"

    result = subprocess.run(
        [sys.executable, str(stub_path), "--help"],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0
    assert "--odom" in result.stdout
    assert "--voxel_size" in result.stdout
    assert "--loop_search_radius" in result.stdout


def test_stub_starts_and_terminates():
    """Stub should start in dry-run mode and exit cleanly on SIGTERM."""
    import signal
    import time

    stub_path = Path(__file__).parent / "native" / "incremental_map_stub.py"

    proc = subprocess.Popen(
        [sys.executable, str(stub_path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    # Give it a moment to start
    time.sleep(0.3)
    assert proc.poll() is None, "Stub process should still be running"

    # Send SIGTERM
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

    assert proc.returncode in (0, -signal.SIGTERM, signal.SIGTERM), (
        f"Expected clean exit, got {proc.returncode}"
    )
