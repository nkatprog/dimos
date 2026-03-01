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

from __future__ import annotations

from dataclasses import dataclass, field

import psutil

_MB = 1024 * 1024

# Cache Process objects so cpu_percent(interval=None) has a previous sample.
_proc_cache: dict[int, psutil.Process] = {}


@dataclass(frozen=True)
class ProcessStats:
    """Resource stats for a single OS process."""

    pid: int
    alive: bool
    cpu_percent: float = 0.0
    cpu_time_user: float = 0.0
    cpu_time_system: float = 0.0
    cpu_time_iowait: float = 0.0
    pss_mb: float = 0.0
    num_threads: int = 0
    num_children: int = 0
    num_fds: int = 0
    io_read_mb: float = 0.0
    io_write_mb: float = 0.0


@dataclass(frozen=True)
class WorkerStats(ProcessStats):
    """Process stats extended with worker-specific metadata."""

    worker_id: int = -1
    modules: list[str] = field(default_factory=list)


def collect_process_stats(pid: int) -> ProcessStats:
    """Collect resource stats for a single process by PID."""
    try:
        proc = _proc_cache.get(pid)
        if proc is None or not proc.is_running():
            proc = psutil.Process(pid)
            _proc_cache[pid] = proc
        with proc.oneshot():
            cpu_pct = proc.cpu_percent(interval=None)
            ct = proc.cpu_times()
            try:
                mem_full = proc.memory_full_info()
                pss = getattr(mem_full, "pss", 0) / _MB
            except (psutil.AccessDenied, AttributeError):
                pss = 0.0
            try:
                io = proc.io_counters()
                io_r = io.read_bytes / _MB
                io_w = io.write_bytes / _MB
            except (psutil.AccessDenied, AttributeError):
                io_r = io_w = 0.0
            try:
                fds = proc.num_fds()
            except (psutil.AccessDenied, AttributeError):
                fds = 0
            return ProcessStats(
                pid=pid,
                alive=True,
                cpu_percent=cpu_pct,
                cpu_time_user=ct.user,
                cpu_time_system=ct.system,
                cpu_time_iowait=getattr(ct, "iowait", 0.0),
                pss_mb=pss,
                num_threads=proc.num_threads(),
                num_children=len(proc.children(recursive=True)),
                num_fds=fds,
                io_read_mb=io_r,
                io_write_mb=io_w,
            )
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        _proc_cache.pop(pid, None)
        return ProcessStats(pid=pid, alive=False)
