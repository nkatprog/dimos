#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""
DepthAI / OAK-D Pro camera smoke test.

Shows:
- RGB stream (uint8 RGB)
- Depth stream (uint16 millimeters) visualized as a colormap

Quit with 'q' or ESC.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from typing import Any

# The repo targets Python 3.10+ (see pyproject). Provide a clear error for older versions.
if sys.version_info < (3, 10):
    raise SystemExit(
        f"[depthai] Python {sys.version.split()[0]} detected. This repo requires Python >= 3.10.\n"
        "Please create/activate a Python 3.10+ environment (venv/uv/conda) and retry."
    )

# Third-party deps (kept after the version check so Python<3.10 fails fast).
# We bind them to names explicitly so type checkers don't report "not defined".
cv2: Any
try:
    import cv2 as _cv2  # type: ignore[import-not-found]

    cv2 = _cv2
except ModuleNotFoundError as e:
    raise SystemExit(
        "[depthai] OpenCV (cv2) is not installed in this Python environment.\n"
        "Install it with: `pip install opencv-python` (or `pip install opencv-contrib-python`)."
    ) from e

import numpy as np  # type: ignore[import-not-found]

# Allow running directly from a source checkout without `pip install -e .`.
# (Many repo scripts assume an editable install; this makes this smoke-test friendlier.)
try:
    from dimos.hardware.sensors.camera.depthai.camera import DepthAI
    from dimos.msgs.sensor_msgs import Image
except ModuleNotFoundError:
    from pathlib import Path

    here = Path(__file__).resolve()
    for parent in [here.parent, *here.parents]:
        if (parent / "pyproject.toml").exists() and (parent / "dimos").is_dir():
            sys.path.insert(0, str(parent))
            break

    from dimos.hardware.sensors.camera.depthai.camera import DepthAI
    from dimos.msgs.sensor_msgs import Image


def _depth_to_colormap(depth_mm: np.ndarray, max_depth_mm: int) -> np.ndarray:
    """Convert uint16 depth (mm) to a BGR colormap image for display."""
    if depth_mm.dtype != np.uint16:
        depth_mm = depth_mm.astype(np.uint16, copy=False)

    d = depth_mm.astype(np.float32)
    d = np.clip(d, 0.0, float(max_depth_mm))
    d8 = (d / float(max_depth_mm) * 255.0).astype(np.uint8)
    return cv2.applyColorMap(d8, cv2.COLORMAP_TURBO)


def _check_device_available() -> bool:
    """Check if DepthAI device is available and not in use."""
    try:
        import depthai as dai  # type: ignore[import-not-found]
        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            return False
        
        # Try to get device info (doesn't open it, just checks availability)
        for d in devices:
            try:
                # Check if device state is reasonable
                state_str = str(d.state) if hasattr(d, 'state') else ''
                if 'ERROR' in state_str or 'NOT_FOUND' in state_str:
                    return False
            except Exception:
                pass
        return True
    except Exception:
        return False


def _wait_for_device_recovery(max_wait_seconds: float = 5.0) -> bool:
    """Wait for device to recover if it was in a bad state."""
    try:
        import depthai as dai  # type: ignore[import-not-found]
    except ImportError:
        return False
    
    start_time = time.time()
    while time.time() - start_time < max_wait_seconds:
        try:
            devices = dai.Device.getAllAvailableDevices()
            if devices:
                # Check if any device is in a recoverable state
                for d in devices:
                    state_str = str(d.state) if hasattr(d, 'state') else ''
                    if 'UNBOOTED' in state_str or 'BOOTED' in state_str:
                        # Device seems OK, give it a moment to stabilize
                        time.sleep(0.5)
                        return True
        except Exception:
            pass
        time.sleep(0.2)
    return False


def _check_device_available() -> bool:
    """Check if DepthAI device is available and not in use."""
    try:
        import depthai as dai
        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            return False
        
        # Try to get device info (doesn't open it, just checks availability)
        for d in devices:
            try:
                # Check if device state is reasonable
                state_str = str(d.state) if hasattr(d, 'state') else ''
                if 'ERROR' in state_str or 'NOT_FOUND' in state_str:
                    return False
            except Exception:
                pass
        return True
    except Exception:
        return False


def _wait_for_device_recovery(max_wait_seconds: float = 5.0) -> bool:
    """Wait for device to recover if it was in a bad state."""
    import depthai as dai
    
    start_time = time.time()
    while time.time() - start_time < max_wait_seconds:
        try:
            devices = dai.Device.getAllAvailableDevices()
            if devices:
                # Check if any device is in a recoverable state
                for d in devices:
                    state_str = str(d.state) if hasattr(d, 'state') else ''
                    if 'UNBOOTED' in state_str or 'BOOTED' in state_str:
                        # Device seems OK, give it a moment to stabilize
                        time.sleep(0.5)
                        return True
        except Exception:
            pass
        time.sleep(0.2)
    return False


def _kill_existing_processes() -> None:
    """Kill any existing depthai_camera_test_script processes to free the device."""
    
    try:
        # Get current process PID
        current_pid = os.getpid()
        
        # Find other processes running this script
        result = subprocess.run(
            ['pgrep', '-f', 'depthai_camera_test_script.py'],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            pids = [int(pid) for pid in result.stdout.strip().split('\n') if pid and int(pid) != current_pid]
            if pids:
                print(f"[depthai] Found {len(pids)} existing process(es) holding the device. Cleaning up...")
                for pid in pids:
                    try:
                        os.kill(pid, 15)  # SIGTERM first (graceful)
                    except ProcessLookupError:
                        pass
                    except Exception:
                        try:
                            os.kill(pid, 9)  # SIGKILL if needed
                        except Exception:
                            pass
                # Wait a moment for processes to die
                time.sleep(1.0)
                print("[depthai] Cleanup complete.")
    except Exception as e:
        # If pgrep isn't available or fails, continue anyway
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description="DepthAI / OAK-D Pro OpenCV viewer")

    parser.add_argument("--fps", type=int, default=30, help="Camera FPS (default: 30)")
    parser.add_argument(
        "--rgb-resolution",
        choices=["1080p", "4k", "720p"],
        default="1080p",
        help="RGB sensor resolution (default: 1080p)",
    )
    parser.add_argument(
        "--mono-resolution",
        choices=["400p", "720p", "800p"],
        default="720p",
        help="Mono sensor resolution for stereo depth (default: 720p)",
    )
    parser.add_argument(
        "--align-depth-to-rgb",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Align depth to RGB (default: true)",
    )

    # Stereo depth quality knobs
    parser.add_argument(
        "--lr-check",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable left-right consistency check (default: true)",
    )
    parser.add_argument(
        "--subpixel",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable subpixel disparity (default: true)",
    )
    parser.add_argument(
        "--extended-disparity",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable extended disparity (default: false)",
    )

    # IR controls (OAK-D Pro only)
    parser.add_argument(
        "--enable-ir",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable IR dot projector + flood (default: false)",
    )
    parser.add_argument(
        "--ir-dot-ma",
        type=int,
        default=0,
        help="IR dot projector brightness in mA (default: 0)",
    )
    parser.add_argument(
        "--ir-flood-ma",
        type=int,
        default=0,
        help="IR flood light brightness in mA (default: 0)",
    )

    parser.add_argument(
        "--max-depth-mm",
        type=int,
        default=3000,
        help="Depth visualization range upper bound in mm (default: 3000)",
    )
    parser.add_argument(
        "--show-depth",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show depth window (default: true)",
    )
    parser.add_argument(
        "--show-rgb",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show RGB window (default: true)",
    )
    parser.add_argument(
        "--print-stats",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Print FPS + frame info periodically (default: true)",
    )
    parser.add_argument(
        "--stats-interval-s",
        type=float,
        default=2.0,
        help="Stats print interval in seconds (default: 2.0)",
    )

    args = parser.parse_args()

    # Kill any existing processes that might be holding the device
    _kill_existing_processes()

    # Check if device is available before starting
    print("[depthai] Checking device availability...")
    if not _check_device_available():
        print("[depthai] WARNING: No DepthAI device found or device is in error state.")
        print("[depthai] Please check:")
        print("[depthai]   1. Device is connected via USB")
        print("[depthai]   2. Device is not in use by another process")
        print("[depthai]   3. Try unplugging and replugging the device")
        if not _wait_for_device_recovery():
            raise SystemExit("[depthai] Device not available. Exiting.")
        print("[depthai] Device recovered, proceeding...")
    else:
        print("[depthai] Device detected and available.")

    # Add a small delay to ensure device is fully ready
    time.sleep(0.5)

    print("[depthai] Initializing camera...")
    cam = DepthAI(
        fps=args.fps,
        rgb_resolution=args.rgb_resolution,
        mono_resolution=args.mono_resolution,
        align_depth_to_rgb=args.align_depth_to_rgb,
        lr_check=args.lr_check,
        subpixel=args.subpixel,
        extended_disparity=args.extended_disparity,
        enable_ir=args.enable_ir,
        ir_dot_projector_ma=args.ir_dot_ma,
        ir_flood_ma=args.ir_flood_ma,
    )

    lock = threading.Lock()
    latest_rgb: Image | None = None
    latest_depth: Image | None = None
    rgb_count = 0
    depth_count = 0
    last_stats_t = time.time()
    last_rgb_count = 0
    last_depth_count = 0

    def on_rgb(msg: Image) -> None:
        nonlocal latest_rgb, rgb_count
        with lock:
            latest_rgb = msg
            rgb_count += 1

    def on_depth(msg: Image) -> None:
        nonlocal latest_depth, depth_count
        with lock:
            latest_depth = msg
            depth_count += 1

    try:
        rgb_sub = cam.image_stream().subscribe(on_rgb)
        depth_sub = cam.depth_stream().subscribe(on_depth)
    except Exception as e:
        print(f"[depthai] Failed to start streams: {e}")
        print(
            "[depthai] Make sure you installed the DepthAI Python package: `pip install depthai`."
        )
        raise

    if args.show_rgb:
        cv2.namedWindow("DepthAI RGB", cv2.WINDOW_NORMAL)
    if args.show_depth:
        cv2.namedWindow("DepthAI Depth (mm)", cv2.WINDOW_NORMAL)

    print("[depthai] Running. Press 'q' or ESC to quit.")
    if args.enable_ir:
        print(
            f"[depthai] IR enabled: dot={args.ir_dot_ma} mA, flood={args.ir_flood_ma} mA (OAK-D Pro only)"
        )

    try:
        while True:
            with lock:
                rgb = latest_rgb
                depth = latest_depth
                rc = rgb_count
                dc = depth_count

            if args.show_rgb and rgb is not None:
                rgb_np = np.asarray(rgb.data)
                # DepthAI camera.py emits RGB; OpenCV expects BGR for display
                if rgb_np.ndim == 3 and rgb_np.shape[2] == 3:
                    bgr = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
                else:
                    bgr = rgb_np
                cv2.imshow("DepthAI RGB", bgr)

            if args.show_depth and depth is not None:
                try:
                    depth_mm = np.asarray(depth.data)
                    if depth_mm.ndim == 2:
                        depth_vis = _depth_to_colormap(depth_mm, max_depth_mm=args.max_depth_mm)
                        cv2.imshow("DepthAI Depth (mm)", depth_vis)
                except Exception as e:
                    # Handle errors during depth processing (e.g., device crash during read)
                    print(f"[depthai] Warning: Error processing depth frame: {e}")
                    # Skip this frame and continue
                    pass

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):  # q or ESC
                break

            if args.print_stats:
                now = time.time()
                if now - last_stats_t >= float(args.stats_interval_s):
                    dt = now - last_stats_t
                    rgb_fps = (rc - last_rgb_count) / max(dt, 1e-6)
                    depth_fps = (dc - last_depth_count) / max(dt, 1e-6)
                    rgb_shape = None if rgb is None else tuple[int, ...](np.asarray(rgb.data).shape)
                    depth_shape = None if depth is None else tuple[int, ...](np.asarray(depth.data).shape)
                    print(
                        f"[depthai] rgb_fps={rgb_fps:.1f} depth_fps={depth_fps:.1f} "
                        f"rgb_shape={rgb_shape} depth_shape={depth_shape}"
                    )
                    last_stats_t = now
                    last_rgb_count = rc
                    last_depth_count = dc

            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n[depthai] Interrupted by user. Cleaning up...")
    except Exception as e:
        print(f"\n[depthai] Error during execution: {e}")
        print("[depthai] This might indicate a device crash or communication error.")
        print("[depthai] If this persists, try:")
        print("[depthai]   1. Unplugging and replugging the device")
        print("[depthai]   2. Waiting a few seconds before restarting")
        print("[depthai]   3. Checking USB cable/port (use USB 3.0)")
        print("[depthai]   4. Ensuring no other process is using the device")
    finally:
        print("[depthai] Stopping camera and cleaning up...")
        try:
            if 'rgb_sub' in locals():
                rgb_sub.dispose()
        except Exception:
            pass
        try:
            if 'depth_sub' in locals():
                depth_sub.dispose()
        except Exception:
            pass
        try:
            cam.stop()
        except Exception as e:
            print(f"[depthai] Warning during cleanup: {e}")
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        print("[depthai] Cleanup complete.")


if __name__ == "__main__":
    main()


