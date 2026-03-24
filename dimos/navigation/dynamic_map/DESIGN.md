# DynamicMap — Design Notes

## Architecture: NativeModule + OctoMap

DynamicMap follows the **DimOS NativeModule pattern**: a thin Python `Module`
subclass manages a C++ subprocess.  The real mapping work runs in
`native/dynamic_map_node.cpp`; Python handles blueprint wiring and lifecycle.

```
Blueprint engine
  └── DynamicMap (Python NativeModule)
        ├── declares In/Out ports → blueprint auto-wires LCM topics
        ├── converts config → --key value CLI args
        ├── spawns subprocess: dynamic_map_node --registered_scan <topic> ...
        └── forwards SIGTERM on stop()

dynamic_map_node (C++ subprocess)
  ├── subscribes: registered_scan (PointCloud2), raw_odom (PoseStamped)
  ├── calls OctoMap::insertPointCloud() on every scan
  └── publishes: global_map (PointCloud2), odom (PoseStamped pass-through)
```

---

## Why C++ / OctoMap

* **OctoMap** is the canonical probabilistic 3-D mapping library.  Its
  `insertPointCloud()` performs full 3-D DDA ray-casting and Bayesian
  log-odds updates in a single optimised call — no hand-rolled traversal.
* **Performance**: OctoMap's C++ octree is ~10–50× faster than an equivalent
  Python `dict`-backed grid for real lidar densities (1000–50 000 pts/scan).
* **NativeModule pattern**: subprocess isolation means the mapping loop never
  blocks the Python event loop, and OOM in the C++ process doesn't kill the
  whole blueprint.

---

## Probabilistic Model

Each voxel stores a **log-odds** value `L = log(p / (1 − p))`:

| Event | ΔL | Typical |
|-------|----|---------|
| Ray passes *through* voxel | `log_odds_miss` (internal OctoMap) | −0.40 |
| Ray *terminates* in voxel | `log_odds_hit` (internal OctoMap) | +0.85 |

Values are clamped to `[log_odds_min, log_odds_max]` (configured via
`setClampingThresMin` / `setClampingThresMax` in the node).

### Dynamic obstacle removal

Rays continuously pass through voxels that are no longer occupied, gradually
decrementing their log-odds below `occ_threshold` → voxel disappears from the
published map.  The clamped floor prevents a single bad scan from clearing a
highly-confident wall.

---

## Ray-Casting: OctoMap's DDA

`OcTree::insertPointCloud()` uses the Amanatides & Woo (1987) 3-D DDA
algorithm internally.  For each ray from sensor origin `O` to endpoint `P`:

1. Identify the start and end voxel via `floor(pos / resolution)`.
2. Step one voxel at a time along the dominant axis (X, Y, or Z) until `P` is
   reached, adjusting minor axes via integer error accumulators.
3. Apply `log_odds_miss` to all traversed voxels; `log_odds_hit` to the
   endpoint.

Rays beyond `max_range` are truncated: the boundary voxel is **not** marked
occupied (no reliable obstacle there), but intermediate voxels are still
cleared.

---

## Build

```bash
# Install OctoMap
sudo apt install liboctomap-dev

# Build (from repo root or from the native/ dir)
cd dimos/navigation/dynamic_map/native
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

The Python module's `build_command` runs this automatically on first start if
the binary is missing.

---

## Stream Interface

| Stream | Direction | Type | Notes |
|--------|-----------|------|-------|
| `registered_scan` | In | `PointCloud2` | World-frame lidar (remapped from `lidar`) |
| `raw_odom` | In | `PoseStamped` | Sensor pose (remapped from `odom`) |
| `global_map` | Out | `PointCloud2` | Occupied voxels above `occ_threshold` |
| `odom` | Out | `PoseStamped` | Pass-through odometry (no loop closure v1) |

Interface is intentionally identical to `VoxelGridMapper` — the two modules
are drop-in interchangeable in blueprints.

---

## References

1. A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard,
   "OctoMap: An efficient probabilistic 3D mapping framework based on
   octrees," *Autonomous Robots* 34(3), pp. 189–206, 2013.
   https://octomap.github.io/
2. J. Amanatides and A. Woo, "A fast voxel traversal algorithm for ray
   tracing," *Eurographics* 87, pp. 3–10, 1987.
