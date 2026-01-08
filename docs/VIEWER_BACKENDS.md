# Viewer Backends

Dimos supports three visualization backends: Rerun (web or native) and Foxglove.

## Quick Start

Choose your viewer backend with the `--viewer-backend` flag:

```bash
# Rerun native viewer (default)
dimos run unitree-go2
# or explicitly:
dimos run unitree-go2 --viewer-backend rerun-native

# Rerun web viewer
dimos run unitree-go2 --viewer-backend rerun-web

# Foxglove - Use Foxglove Studio instead of Rerun
dimos run unitree-go2 --viewer-backend foxglove
```

## Blueprint API (important)

Rerun is now integrated with **strict separation of concerns**:
- Modules publish normal streams only (including metrics).
- The **dashboard layer** logs to Rerun and owns UI/layout policy.

For custom blueprints, add a single helper:

```python
from dimos.dashboard import rerun_viz

blueprint = autoconnect(
    robot_connection(),
    rerun_viz(voxel_box_size=0.1, voxel_colormap="turbo"),
)
```

### `rerun_viz(...)` API (all supported parameters)

`rerun_viz(**kwargs)` bundles:
- TF visualization (subscribes to `/tf`)
- Rerun logging for common streams (voxel map, costmap floor mesh, and metrics)

All parameters below are forwarded to the underlying `RerunLoggerModule`:

```python
rerun_viz(
    voxel_box_size: float = 0.05,
    voxel_colormap: str | None = "turbo",
    costmap_z_offset: float = 0.05,
    map_rate_limit_hz: float = 10.0,
    costmap_rate_limit_hz: float = 10.0,
)
```

- **voxel_box_size**: Box edge length used to render the voxel map in Rerun. Keep this in sync with `voxel_mapper(voxel_size=...)`.
- **voxel_colormap**: Colormap used for `PointCloud2.to_rerun(colormap=...)`. Set to `None` for no colormap.
- **costmap_z_offset**: Height above the floor for `OccupancyGrid.to_rerun(z_offset=...)`.
- **map_rate_limit_hz**: Best-effort log rate limit for `world/map`.
- **costmap_rate_limit_hz**: Best-effort log rate limit for `world/nav/costmap/floor`.

## Viewer Modes Explained

### Rerun Web (`rerun-web`)

**What you get:**
- Full dashboard at `http://localhost:7779/` (Rerun + command center in one page)
- Rerun web viewer itself at `http://localhost:9090`
- Command center alone at `http://localhost:7779/command-center`
- Works in browser, no display required (headless-friendly)

---

### Rerun Native (`rerun-native`)

**What you get:**
- Native Rerun application (separate window opens automatically)
- Command center at `http://localhost:7779/command-center`
- Better performance with larger maps/higher resolution

---

### Foxglove (`foxglove`)

**What you get:**
- Foxglove bridge on ws://localhost:8765
- No Rerun (saves resources)
- Better performance with larger maps/higher resolution
- Open layout: `dimos/assets/foxglove_dashboards/go2.json`

---

## Performance Tuning

### Symptom: Slow Map Updates

If you notice:
- Robot appears to "walk across empty space"
- Costmap updates lag behind the robot
- Visualization stutters or freezes

This happens on lower-end hardware (NUC, older laptops) with large maps.

### Increase Voxel Size

Edit [`dimos/robot/unitree_webrtc/unitree_go2_blueprints.py`](../dimos/robot/unitree_webrtc/unitree_go2_blueprints.py) to increase `_VOXEL_SIZE`:

```python
_VOXEL_SIZE = 0.05  # 5cm voxels (high detail, slower)
# ...
voxel_mapper(voxel_size=_VOXEL_SIZE)
# ...
rerun_viz(voxel_box_size=_VOXEL_SIZE, voxel_colormap="turbo")
```

**Trade-off:**
- Larger voxels = fewer voxels = faster updates
- But slightly less detail in the map
