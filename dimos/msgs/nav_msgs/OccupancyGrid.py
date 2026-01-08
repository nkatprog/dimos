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

from __future__ import annotations

from enum import IntEnum
import time
from typing import TYPE_CHECKING, Any, BinaryIO

from dimos_lcm.nav_msgs import (
    MapMetaData,
    OccupancyGrid as LCMOccupancyGrid,
)
from dimos_lcm.std_msgs import Time as LCMTime  # type: ignore[import-untyped]
import numpy as np
from PIL import Image
import rerun as rr

from dimos.msgs.geometry_msgs import Pose, Vector3, VectorLike
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray


class CostValues(IntEnum):
    """Standard cost values for occupancy grid cells.

    These values follow the ROS nav_msgs/OccupancyGrid convention:
    - 0: Free space
    - 1-99: Occupied space with varying cost levels
    - 100: Lethal obstacle (definitely occupied)
    - -1: Unknown space
    """

    UNKNOWN = -1  # Unknown space
    FREE = 0  # Free space
    OCCUPIED = 100  # Occupied/lethal space


class OccupancyGrid(Timestamped):
    """
    Convenience wrapper for nav_msgs/OccupancyGrid with numpy array support.
    """

    msg_name = "nav_msgs.OccupancyGrid"

    # Attributes
    ts: float
    frame_id: str
    info: MapMetaData
    grid: NDArray[np.int8]

    def __init__(
        self,
        grid: NDArray[np.int8] | None = None,
        width: int | None = None,
        height: int | None = None,
        resolution: float = 0.05,
        origin: Pose | None = None,
        frame_id: str = "world",
        ts: float = 0.0,
    ) -> None:
        """Initialize OccupancyGrid.

        Args:
            grid: 2D numpy array of int8 values (height x width)
            width: Width in cells (used if grid is None)
            height: Height in cells (used if grid is None)
            resolution: Grid resolution in meters/cell
            origin: Origin pose of the grid
            frame_id: Reference frame
            ts: Timestamp (defaults to current time if 0)
        """

        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()

        if grid is not None:
            # Initialize from numpy array
            if grid.ndim != 2:
                raise ValueError("Grid must be a 2D array")
            height, width = grid.shape
            self.info = MapMetaData(
                map_load_time=self._to_lcm_time(),  # type: ignore[no-untyped-call]
                resolution=resolution,
                width=width,
                height=height,
                origin=origin or Pose(),
            )
            self.grid = grid.astype(np.int8)
        elif width is not None and height is not None:
            # Initialize with dimensions
            self.info = MapMetaData(
                map_load_time=self._to_lcm_time(),  # type: ignore[no-untyped-call]
                resolution=resolution,
                width=width,
                height=height,
                origin=origin or Pose(),
            )
            self.grid = np.full((height, width), -1, dtype=np.int8)
        else:
            # Initialize empty
            self.info = MapMetaData(map_load_time=self._to_lcm_time())  # type: ignore[no-untyped-call]
            self.grid = np.array([], dtype=np.int8)

    def _to_lcm_time(self):  # type: ignore[no-untyped-def]
        """Convert timestamp to LCM Time."""

        s = int(self.ts)
        return LCMTime(sec=s, nsec=int((self.ts - s) * 1_000_000_000))

    @property
    def width(self) -> int:
        """Width of the grid in cells."""
        return self.info.width  # type: ignore[no-any-return]

    @property
    def height(self) -> int:
        """Height of the grid in cells."""
        return self.info.height  # type: ignore[no-any-return]

    @property
    def resolution(self) -> float:
        """Grid resolution in meters/cell."""
        return self.info.resolution  # type: ignore[no-any-return]

    @property
    def origin(self) -> Pose:
        """Origin pose of the grid."""
        return self.info.origin  # type: ignore[no-any-return]

    @property
    def total_cells(self) -> int:
        """Total number of cells in the grid."""
        return self.width * self.height

    @property
    def occupied_cells(self) -> int:
        """Number of occupied cells (value >= 1)."""
        return int(np.sum(self.grid >= 1))

    @property
    def free_cells(self) -> int:
        """Number of free cells (value == 0)."""
        return int(np.sum(self.grid == 0))

    @property
    def unknown_cells(self) -> int:
        """Number of unknown cells (value == -1)."""
        return int(np.sum(self.grid == -1))

    @property
    def occupied_percent(self) -> float:
        """Percentage of cells that are occupied."""
        return (self.occupied_cells / self.total_cells * 100) if self.total_cells > 0 else 0.0

    @property
    def free_percent(self) -> float:
        """Percentage of cells that are free."""
        return (self.free_cells / self.total_cells * 100) if self.total_cells > 0 else 0.0

    @property
    def unknown_percent(self) -> float:
        """Percentage of cells that are unknown."""
        return (self.unknown_cells / self.total_cells * 100) if self.total_cells > 0 else 0.0

    @classmethod
    def from_path(cls, path: Path) -> OccupancyGrid:
        match path.suffix.lower():
            case ".npy":
                return cls(grid=np.load(path))
            case ".png":
                img = Image.open(path).convert("L")
                return cls(grid=np.array(img).astype(np.int8))
            case _:
                raise NotImplementedError(f"Unsupported file format: {path.suffix}")

    def world_to_grid(self, point: VectorLike) -> Vector3:
        """Convert world coordinates to grid coordinates.

        Args:
            point: A vector-like object containing X,Y coordinates

        Returns:
            Vector3 with grid coordinates
        """
        positionVector = Vector3(point)
        # Get origin position
        ox = self.origin.position.x
        oy = self.origin.position.y

        # Convert to grid coordinates (simplified, assuming no rotation)
        grid_x = (positionVector.x - ox) / self.resolution
        grid_y = (positionVector.y - oy) / self.resolution

        return Vector3(grid_x, grid_y, 0.0)

    def grid_to_world(self, grid_point: VectorLike) -> Vector3:
        """Convert grid coordinates to world coordinates.

        Args:
            grid_point: Vector-like object containing grid coordinates

        Returns:
            World position as Vector3
        """
        gridVector = Vector3(grid_point)
        # Get origin position
        ox = self.origin.position.x
        oy = self.origin.position.y

        # Convert to world (simplified, no rotation)
        x = ox + gridVector.x * self.resolution
        y = oy + gridVector.y * self.resolution

        return Vector3(x, y, 0.0)

    def __str__(self) -> str:
        """Create a concise string representation."""
        origin_pos = self.origin.position

        parts = [
            f"▦ OccupancyGrid[{self.frame_id}]",
            f"{self.width}x{self.height}",
            f"({self.width * self.resolution:.1f}x{self.height * self.resolution:.1f}m @",
            f"{1 / self.resolution:.0f}cm res)",
            f"Origin: ({origin_pos.x:.2f}, {origin_pos.y:.2f})",
            f"▣ {self.occupied_percent:.1f}%",
            f"□ {self.free_percent:.1f}%",
            f"◌ {self.unknown_percent:.1f}%",
        ]

        return " ".join(parts)

    def __repr__(self) -> str:
        """Create a detailed representation."""
        return (
            f"OccupancyGrid(width={self.width}, height={self.height}, "
            f"resolution={self.resolution}, frame_id='{self.frame_id}', "
            f"occupied={self.occupied_cells}, free={self.free_cells}, "
            f"unknown={self.unknown_cells})"
        )

    def lcm_encode(self) -> bytes:
        """Encode OccupancyGrid to LCM bytes."""
        # Create LCM message
        lcm_msg = LCMOccupancyGrid()

        # Build header on demand
        s = int(self.ts)
        lcm_msg.header.stamp.sec = s
        lcm_msg.header.stamp.nsec = int((self.ts - s) * 1_000_000_000)
        lcm_msg.header.frame_id = self.frame_id

        # Copy map metadata
        lcm_msg.info = self.info

        # Convert numpy array to flat data list
        if self.grid.size > 0:
            flat_data = self.grid.flatten()
            lcm_msg.data_length = len(flat_data)
            lcm_msg.data = flat_data.tolist()
        else:
            lcm_msg.data_length = 0
            lcm_msg.data = []

        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> OccupancyGrid:
        """Decode LCM bytes to OccupancyGrid."""
        lcm_msg = LCMOccupancyGrid.lcm_decode(data)

        # Extract timestamp and frame_id from header
        ts = lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000)
        frame_id = lcm_msg.header.frame_id

        # Extract grid data
        if lcm_msg.data and lcm_msg.info.width > 0 and lcm_msg.info.height > 0:
            grid = np.array(lcm_msg.data, dtype=np.int8).reshape(
                (lcm_msg.info.height, lcm_msg.info.width)
            )
        else:
            grid = np.array([], dtype=np.int8)

        # Create new instance
        instance = cls(
            grid=grid,
            resolution=lcm_msg.info.resolution,
            origin=lcm_msg.info.origin,
            frame_id=frame_id,
            ts=ts,
        )
        instance.info = lcm_msg.info
        return instance

    def filter_above(self, threshold: int) -> OccupancyGrid:
        """Create a new OccupancyGrid with only values above threshold.

        Args:
            threshold: Keep cells with values > threshold

        Returns:
            New OccupancyGrid where:
            - Cells > threshold: kept as-is
            - Cells <= threshold: set to -1 (unknown)
            - Unknown cells (-1): preserved
        """
        new_grid = self.grid.copy()

        # Create mask for cells to filter (not unknown and <= threshold)
        filter_mask = (new_grid != -1) & (new_grid <= threshold)

        # Set filtered cells to unknown
        new_grid[filter_mask] = -1

        # Create new OccupancyGrid
        filtered = OccupancyGrid(
            new_grid,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

        return filtered

    def filter_below(self, threshold: int) -> OccupancyGrid:
        """Create a new OccupancyGrid with only values below threshold.

        Args:
            threshold: Keep cells with values < threshold

        Returns:
            New OccupancyGrid where:
            - Cells < threshold: kept as-is
            - Cells >= threshold: set to -1 (unknown)
            - Unknown cells (-1): preserved
        """
        new_grid = self.grid.copy()

        # Create mask for cells to filter (not unknown and >= threshold)
        filter_mask = (new_grid != -1) & (new_grid >= threshold)

        # Set filtered cells to unknown
        new_grid[filter_mask] = -1

        # Create new OccupancyGrid
        filtered = OccupancyGrid(
            new_grid,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

        return filtered

    def max(self) -> OccupancyGrid:
        """Create a new OccupancyGrid with all non-unknown cells set to maximum value.

        Returns:
            New OccupancyGrid where:
            - All non-unknown cells: set to CostValues.OCCUPIED (100)
            - Unknown cells: preserved as CostValues.UNKNOWN (-1)
        """
        new_grid = self.grid.copy()

        # Set all non-unknown cells to max
        new_grid[new_grid != CostValues.UNKNOWN] = CostValues.OCCUPIED

        # Create new OccupancyGrid
        maxed = OccupancyGrid(
            new_grid,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

        return maxed

    def copy(self) -> OccupancyGrid:
        """Create a deep copy of the OccupancyGrid.

        Returns:
            A new OccupancyGrid instance with copied data.
        """
        return OccupancyGrid(
            grid=self.grid.copy(),
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def cell_value(self, world_position: Vector3) -> int:
        grid_position = self.world_to_grid(world_position)
        x = int(grid_position.x)
        y = int(grid_position.y)

        if not (0 <= x < self.width and 0 <= y < self.height):
            return CostValues.UNKNOWN

        return int(self.grid[y, x])

    def to_rerun(self, z_offset: float = 0.01) -> Any:
        """Convert to 3D mesh on floor with Foxglove colors.

        ONE simple way - mesh overlay with blue-purple (free) to black (occupied).

        Args:
            z_offset: Height above floor (default 0.01m)

        Returns:
            rr.Mesh3D archetype for floor visualization
        """
        if self.grid.size == 0 or not np.any(self.grid != -1):
            return rr.Mesh3D(vertex_positions=[])

        # Only render known cells (not unknown = -1)
        known_mask = self.grid != -1
        gy, gx = np.where(known_mask)
        n_cells = len(gy)

        ox = self.origin.position.x
        oy = self.origin.position.y
        r = self.resolution

        # Vectorized vertex generation (4 vertices per cell)
        wx = ox + gx.astype(np.float32) * r
        wy = oy + gy.astype(np.float32) * r

        vertices = np.zeros((n_cells, 4, 3), dtype=np.float32)
        vertices[:, 0, 0] = wx
        vertices[:, 0, 1] = wy
        vertices[:, 0, 2] = z_offset
        vertices[:, 1, 0] = wx + r
        vertices[:, 1, 1] = wy
        vertices[:, 1, 2] = z_offset
        vertices[:, 2, 0] = wx + r
        vertices[:, 2, 1] = wy + r
        vertices[:, 2, 2] = z_offset
        vertices[:, 3, 0] = wx
        vertices[:, 3, 1] = wy + r
        vertices[:, 3, 2] = z_offset
        flat_vertices = vertices.reshape(-1, 3)

        # Vectorized index generation (2 triangles per cell)
        base_v = np.arange(n_cells, dtype=np.uint32) * 4
        indices = np.zeros((n_cells, 2, 3), dtype=np.uint32)
        indices[:, 0, 0] = base_v
        indices[:, 0, 1] = base_v + 1
        indices[:, 0, 2] = base_v + 2
        indices[:, 1, 0] = base_v
        indices[:, 1, 1] = base_v + 2
        indices[:, 1, 2] = base_v + 3
        flat_indices = indices.reshape(-1, 3)

        # Foxglove colors: #484981 (blue-purple) → #000000 (black)
        cell_values = self.grid[gy, gx]
        rgb = np.zeros((n_cells, 3), dtype=np.uint8)
        is_free = cell_values == 0

        rgb[is_free] = [72, 73, 129]  # Free: blue-purple

        # Occupied: gradient to black based on cost
        if np.any(~is_free):
            costs = cell_values[~is_free].astype(np.float32)
            factor = (1 - costs / 100).clip(0, 1)
            rgb[~is_free] = (np.array([72, 73, 129]) * factor[:, None]).astype(np.uint8)

        # Alpha: semi-transparent
        alpha = np.where(is_free, 180, 220).astype(np.uint8)
        colors = np.repeat(np.column_stack([rgb, alpha]), 4, axis=0)

        return rr.Mesh3D(
            vertex_positions=flat_vertices,
            triangle_indices=flat_indices,
            vertex_colors=colors,
        )
