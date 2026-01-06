# DepthAI / OAK-D Pro Camera Support

DimOS integration for Luxonis DepthAI/OAK-D Pro stereo RGB-D cameras.

## Overview

This module provides a complete DimOS-compatible interface for DepthAI/OAK-D Pro cameras, supporting:
- **RGB streaming** at configurable resolutions (720p, 1080p, 4K)
- **Depth streaming** with aligned depth maps (uint16 millimeters)
- **Camera intrinsics** from device calibration
- **IR illumination** control (OAK-D Pro only)
- **Automatic device booting** and recovery handling

## Features

- Full DimOS `StereoCameraHardware` interface
- Ubuntu 24.04 compatibility with proper udev rules
- Automatic device booting (handles UNBOOTED state)
- Device recovery and error handling
- Configurable stereo depth quality settings
- IR dot projector and flood illuminator control (OAK-D Pro)
- Standalone test scripts (no DimOS required)

## Quick Start

### 1. Run Setup Script

The setup script handles everything automatically:

```bash
cd dimos/hardware/sensors/camera/depthai
bash setup_depthai.sh
```

This will:
- Install udev rules for USB permissions (Ubuntu 20.04/22.04/24.04)
- Create a virtual environment
- Install all Python dependencies
- Add you to the `plugdev` group if needed

**Important:** After running setup, unplug and replug your DepthAI device.

### 2. Test the Device

Verify the device works:

```bash
source /path/to/.venv-depthai/bin/activate
python depthai_device_check.py
```

Or run the full viewer:

```bash
python depthai_camera_test_script.py
```

### 3. Use in DimOS

```python
from dimos.hardware.sensors.camera.depthai.camera import DepthAI

# Create camera with default settings
camera = DepthAI()

# Subscribe to streams
rgb_sub = camera.image_stream().subscribe(lambda img: print(f"RGB: {img.width}x{img.height}"))
depth_sub = camera.depth_stream().subscribe(lambda img: print(f"Depth: {img.width}x{img.height}"))

# Access camera info
info = camera.camera_info
print(f"Camera intrinsics: fx={info.K[0]}, fy={info.K[4]}")
```

## Configuration

### DepthAIConfig Options

```python
@dataclass
class DepthAIConfig:
    fps: int = 30                          # Frame rate (Hz)
    rgb_resolution: Literal["1080p", "4k", "720p"] = "1080p"
    mono_resolution: Literal["400p", "720p", "800p"] = "720p"
    
    align_depth_to_rgb: bool = True        # Align depth to RGB (recommended)
    lr_check: bool = True                  # Left-right consistency check
    subpixel: bool = True                  # Subpixel disparity
    extended_disparity: bool = False       # Extended disparity range
    
    enable_ir: bool = False                # Enable IR illumination
    ir_dot_projector_ma: int = 0          # IR dot projector (0-1200 mA)
    ir_flood_ma: int = 0                   # IR flood illuminator (0-1500 mA)
    
    frame_id_prefix: str | None = None     # Frame ID prefix for TF
```

### Example: Custom Configuration

```python
from dimos.hardware.sensors.camera.depthai.camera import DepthAI, DepthAIConfig

config = DepthAIConfig(
    fps=15,
    rgb_resolution="720p",
    mono_resolution="400p",
    enable_ir=True,
    ir_dot_projector_ma=200,
    ir_flood_ma=150,
    frame_id_prefix="oak_d_pro"
)

camera = DepthAI(config=config)
```

## USB Permissions (Linux)

On Linux, USB permissions are required for the device. The setup script handles this automatically, but manual setup:

### Automatic (Recommended)

```bash
bash setup_depthai.sh
```

### Manual Setup

```bash
# Add udev rules
sudo tee /etc/udev/rules.d/80-depthai.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2485", MODE="0666", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="f63b", MODE="0666", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666", GROUP="plugdev", TAG+="uaccess"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to plugdev group (if not already)
sudo usermod -a -G plugdev $USER

# Unplug and replug device
# Log out and back in if you were just added to plugdev group
```

## Device States

DepthAI devices have two USB states:

- **UNBOOTED** (`03e7:2485`): Device in bootloader mode
- **BOOTED** (`03e7:f63b`): Device running application

The camera driver automatically detects and boots UNBOOTED devices before creating the pipeline.

## Troubleshooting

### "Couldn't open stream" Error

1. **Check for zombie processes:**
   ```bash
   bash kill_depthai_processes.sh
   # or
   pkill -f depthai_camera_test_script
   ```

2. **Verify USB permissions:**
   ```bash
   lsusb | grep 03e7
   ls -la /dev/bus/usb/003/XXX  # Replace XXX with device number from lsusb
   ```

3. **Unplug and replug the device**

4. **Check udev rules:**
   ```bash
   cat /etc/udev/rules.d/80-depthai.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

### Device Crashes

If the device crashes during operation:

1. Wait 5-10 seconds for recovery
2. Kill any zombie processes: `bash kill_depthai_processes.sh`
3. Unplug and replug the device
4. Try again

See `TROUBLESHOOTING.md` for more details.

### Ubuntu 24.04 Specific Issues

Ubuntu 24.04 has stricter USB permissions. The setup script includes Ubuntu 24.04-specific fixes:

- Uses `ATTRS` instead of `ATTR` in udev rules
- Ensures user is in `plugdev` group
- Proper `TAG+="uaccess"` for systemd-logind

See `UBUNTU24_SETUP_MANUAL.md` for step-by-step manual setup.

## Test Scripts

### Standalone Device Check

Test device connectivity without DimOS:

```bash
python depthai_device_check.py [--no-show] [--frames N] [--fps FPS]
```

### OpenCV Viewer

Full RGB + depth visualization:

```bash
python depthai_camera_test_script.py [OPTIONS]
```

Options:
- `--fps FPS`: Frame rate (default: 30)
- `--rgb-resolution {1080p,4k,720p}`: RGB resolution
- `--mono-resolution {400p,720p,800p}`: Mono resolution
- `--enable-ir`: Enable IR illumination
- `--ir-dot-ma N`: IR dot projector brightness (mA)
- `--ir-flood-ma N`: IR flood illuminator brightness (mA)
- `--max-depth-mm N`: Depth visualization range (default: 3000mm)
- `--no-show-depth`: Hide depth window
- `--no-show-rgb`: Hide RGB window

## Module Integration

Use with DimOS modules:

```python
from dimos.hardware.sensors.camera.depthai.module import DepthAICameraModule

# In your module configuration
module = DepthAICameraModule(
    config=DepthAICameraModuleConfig(
        hardware=DepthAI,
        frame_id="oak_d_pro",
    )
)
```

## Requirements

- Python >= 3.10
- DepthAI >= 3.2.1
- OpenCV (for test scripts)
- Linux: USB permissions (handled by setup script)
- USB 3.0 port (recommended)

## Files

- `camera.py`: Main DepthAI camera hardware implementation
- `module.py`: DimOS module wrapper
- `setup_depthai.sh`: Automated setup script
- `requirements-depthai.txt`: Python dependencies
- `depthai_device_check.py`: Standalone device test
- `depthai_camera_test_script.py`: OpenCV viewer/test script
- `kill_depthai_processes.sh`: Cleanup utility
- `TROUBLESHOOTING.md`: Detailed troubleshooting guide
- `UBUNTU24_SETUP_MANUAL.md`: Manual setup for Ubuntu 24.04

## License

Copyright 2025 Dimensional Inc. Licensed under the Apache License, Version 2.0.
