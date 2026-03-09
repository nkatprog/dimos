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

from typing import Any, TypeVar

from dimos.core.resource import Resource
from dimos.memory2.backend import Backend, ListBackend
from dimos.memory2.stream import Stream

T = TypeVar("T")


class StreamNamespace:
    """Attribute-access proxy for session streams.

    Usage::

        session.streams.image_stream
        session.streams["image_stream"]
        list(session.streams)
        len(session.streams)
    """

    def __init__(self, session: Session) -> None:
        self._session = session

    def __getattr__(self, name: str) -> Stream[Any]:
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            return self._session._streams[name]
        except KeyError:
            available = ", ".join(self._session._streams) or "(none)"
            raise AttributeError(f"No stream named {name!r}. Available: {available}") from None

    def __getitem__(self, name: str) -> Stream[Any]:
        try:
            return self._session._streams[name]
        except KeyError:
            raise KeyError(name) from None

    def __iter__(self):
        return iter(self._session._streams.values())

    def __len__(self) -> int:
        return len(self._session._streams)

    def __contains__(self, name: str) -> bool:
        return name in self._session._streams

    def __repr__(self) -> str:
        return f"StreamNamespace({list(self._session._streams.keys())})"


class Session(Resource):
    """A session against a store. Creates and manages named streams."""

    def __init__(self, backend_factory: Any) -> None:  # Callable[[str], Backend]
        self._backend_factory = backend_factory
        self._streams: dict[str, Stream[Any]] = {}
        self._backends: dict[str, Backend[Any]] = {}

    def stream(self, name: str, payload_type: type[T] | None = None) -> Stream[T]:
        """Get or create a named stream. Returns the same Stream on repeated calls."""
        if name not in self._streams:
            backend = self._backend_factory(name)
            self._backends[name] = backend
            self._streams[name] = Stream(source=backend)
        return self._streams[name]  # type: ignore[return-value]

    def list_streams(self) -> list[Stream[Any]]:
        return list(self._streams.values())

    def delete_stream(self, name: str) -> None:
        self._streams.pop(name, None)
        self._backends.pop(name, None)

    @property
    def streams(self) -> StreamNamespace:
        return StreamNamespace(self)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def __enter__(self) -> Session:
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()


class Store(Resource):
    """Top-level entry point — wraps a storage location."""

    def session(self) -> Session:
        raise NotImplementedError

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def __enter__(self) -> Store:
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()


class ListStore(Store):
    """In-memory store for experimentation."""

    def session(self) -> Session:
        return Session(backend_factory=lambda name: ListBackend(name))
