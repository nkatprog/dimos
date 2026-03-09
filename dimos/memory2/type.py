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
from typing import Any, Generic, Protocol, TypeVar, runtime_checkable

T = TypeVar("T")
R = TypeVar("R")


# ── Filter protocol ─────────────────────────────────────────────────


@runtime_checkable
class Filter(Protocol):
    """Any object with a .matches(obs) -> bool method can be a filter."""

    def matches(self, obs: Observation[Any]) -> bool: ...


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
    _loader: Any | None = field(default=None, repr=False)  # Callable[[], T]

    @property
    def data(self) -> T:
        if isinstance(self._data, _Unloaded):
            if self._loader is None:
                raise LookupError("No data and no loader set on this observation")
            self._data = self._loader()
            self._loader = None  # release closure
        return self._data  # type: ignore[return-value]

    def derive(self, *, data: Any, **overrides: Any) -> Observation[Any]:
        """Create a new observation preserving ts/pose/tags, replacing data."""
        return Observation(
            id=self.id,
            ts=overrides.get("ts", self.ts),
            pose=overrides.get("pose", self.pose),
            tags=overrides.get("tags", self.tags),
            _data=data,
        )


# ── Filters ─────────────────────────────────────────────────────────


@dataclass(frozen=True)
class AfterFilter:
    t: float

    def matches(self, obs: Observation[Any]) -> bool:
        return obs.ts > self.t


@dataclass(frozen=True)
class BeforeFilter:
    t: float

    def matches(self, obs: Observation[Any]) -> bool:
        return obs.ts < self.t


@dataclass(frozen=True)
class TimeRangeFilter:
    t1: float
    t2: float

    def matches(self, obs: Observation[Any]) -> bool:
        return self.t1 <= obs.ts <= self.t2


@dataclass(frozen=True)
class AtFilter:
    t: float
    tolerance: float = 1.0

    def matches(self, obs: Observation[Any]) -> bool:
        return abs(obs.ts - self.t) <= self.tolerance


@dataclass(frozen=True)
class NearFilter:
    pose: Any
    radius: float

    def matches(self, obs: Observation[Any]) -> bool:
        if obs.pose is None or self.pose is None:
            return False
        p1 = self.pose
        p2 = obs.pose
        # Support both raw (x,y,z) tuples and PoseStamped objects
        if hasattr(p1, "position"):
            p1 = p1.position
        if hasattr(p2, "position"):
            p2 = p2.position
        x1, y1, z1 = _xyz(p1)
        x2, y2, z2 = _xyz(p2)
        dist_sq = (x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2
        return dist_sq <= self.radius**2


def _xyz(p: Any) -> tuple[float, float, float]:
    """Extract (x, y, z) from various pose representations."""
    if isinstance(p, (list, tuple)):
        return (float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0)
    return (float(p.x), float(p.y), float(getattr(p, "z", 0.0)))


@dataclass(frozen=True)
class TagsFilter:
    tags: dict[str, Any]

    def matches(self, obs: Observation[Any]) -> bool:
        for k, v in self.tags.items():
            if obs.tags.get(k) != v:
                return False
        return True


@dataclass(frozen=True)
class PredicateFilter:
    """Wraps an arbitrary predicate function for use with .filter()."""

    fn: Any  # Callable[[Observation], bool] — Any to keep frozen hashable

    def matches(self, obs: Observation[Any]) -> bool:
        return self.fn(obs)


# ── StreamQuery ─────────────────────────────────────────────────────


@dataclass(frozen=True)
class StreamQuery:
    filters: tuple[Filter, ...] = ()
    order_field: str | None = None
    order_desc: bool = False
    limit_val: int | None = None
    offset_val: int | None = None
