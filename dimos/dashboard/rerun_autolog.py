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

"""Rerun auto-logging utilities.

NOTE: We intentionally keep this minimal and synchronous.

The preferred pattern is:
- Producers publish normal outputs only.
- Dashboard modules subscribe and call `rr.log(..., msg.to_rerun(...))`.

This helper exists only as a small convenience for dashboard-layer code.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any, TypeVar

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

T = TypeVar("T")


def autolog_to_rerun(
    out: Any,
    entity_path: str,
    rate_limit: float | None = None,
    **to_rerun_kwargs: Any,
) -> Callable[[], None]:
    """Auto-log published messages to Rerun (sync).

    Args:
        out: Out stream to tap
        entity_path: Rerun entity path
        rate_limit: Max Hz (None = no limit)
        **to_rerun_kwargs: Passed to msg.to_rerun()

    Returns:
        Unsubscribe function
    """
    import rerun as rr

    last = 0.0

    def _log(msg: T) -> None:
        nonlocal last
        if not hasattr(msg, "to_rerun"):
            return
        if rate_limit:
            now = time.monotonic()
            if now - last < 1.0 / rate_limit:
                return
            last = now
        rr.log(entity_path, msg.to_rerun(**to_rerun_kwargs))  # type: ignore[union-attr]

    return out.tap(_log)
