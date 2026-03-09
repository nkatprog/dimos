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
from typing import TYPE_CHECKING, Any, Generic, TypeVar

if TYPE_CHECKING:
    from collections.abc import Callable

T = TypeVar("T")


# ── Lazy data sentinel ──────────────────────────────────────────────


class _Unloaded:
    """Sentinel indicating data has not been loaded yet."""

    __slots__ = ()

    def __repr__(self) -> str:
        return "<unloaded>"


_UNLOADED = _Unloaded()


# ── Observation ─────────────────────────────────────────────────────


@dataclass
class Observation(Generic[T]):
    """A single timestamped observation with optional spatial pose and metadata."""

    id: int
    ts: float
    pose: Any | None = None
    tags: dict[str, Any] = field(default_factory=dict)
    _data: T | _Unloaded = field(default=_UNLOADED, repr=False)
    _loader: Callable[[], T] | None = field(default=None, repr=False)

    @property
    def data(self) -> T:
        val = self._data
        if isinstance(val, _Unloaded):
            if self._loader is None:
                raise LookupError("No data and no loader set on this observation")
            loaded = self._loader()
            self._data = loaded
            self._loader = None  # release closure
            return loaded
        return val

    def derive(self, *, data: Any, **overrides: Any) -> Observation[Any]:
        """Create a new observation preserving ts/pose/tags, replacing data."""
        return Observation(
            id=self.id,
            ts=overrides.get("ts", self.ts),
            pose=overrides.get("pose", self.pose),
            tags=overrides.get("tags", self.tags),
            _data=data,
        )
