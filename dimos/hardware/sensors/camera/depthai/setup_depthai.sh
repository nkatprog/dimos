#!/usr/bin/env bash
set -euo pipefail

# One-command setup for DepthAI/OAK bringup on Linux.
#
# What it does:
# - Installs udev rules so the device can boot (unbooted->booted)
# - Creates a lightweight venv (separate from full dimos install)
# - Installs minimal Python deps for the DepthAI test scripts in this folder
# - Optionally runs device check / viewer
# - Handles Ubuntu 24.04 compatibility automatically
#
# Usage:
#   bash setup_depthai.sh                    # Full setup
#   bash setup_depthai.sh --run-check        # Setup + run device check
#   bash setup_depthai.sh --run-viewer       # Setup + run OpenCV viewer
#   bash setup_depthai.sh --venv .venv       # Use custom venv path
#   bash setup_depthai.sh --no-udev         # Skip udev rules (if already set)
#
# Notes:
# - You will be prompted for sudo for udev rules unless --no-udev is passed.
# - After udev installation, unplug/replug the camera.
# - If added to plugdev group, log out and back in for it to take effect.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"

VENV_DIR="${REPO_ROOT}/.venv-depthai"
DO_UDEV=1
RUN_CHECK=0
RUN_VIEWER=0
PYTHON_BIN=""
NEEDS_LOGOUT=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --venv)
      VENV_DIR="$2"
      shift 2
      ;;
    --no-udev)
      DO_UDEV=0
      shift
      ;;
    --run-check)
      RUN_CHECK=1
      shift
      ;;
    --run-viewer)
      RUN_VIEWER=1
      shift
      ;;
    --python)
      PYTHON_BIN="$2"
      shift 2
      ;;
    -h|--help)
      sed -n '1,120p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown arg: $1"
      exit 2
      ;;
  esac
done

if [[ -z "${PYTHON_BIN}" ]]; then
  if command -v python3.12 >/dev/null 2>&1; then
    PYTHON_BIN="python3.12"
  elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
  else
    echo "[setup] ERROR: python3 not found."
    exit 1
  fi
fi

PY_VER="$("${PYTHON_BIN}" -c 'import sys; print(".".join(map(str, sys.version_info[:3])))')"
"${PYTHON_BIN}" - <<'PY' || {
import sys
raise SystemExit(0 if sys.version_info >= (3,10) else 1)
PY
  echo "[setup] ERROR: Need Python >= 3.10. Install python3.10+ and retry."
  exit 1
}

# Detect OS version for better compatibility messages
OS_VERSION=""
if [[ -f /etc/os-release ]]; then
    OS_VERSION=$(grep -E "^VERSION_ID=" /etc/os-release | cut -d'"' -f2 || echo "")
fi

echo "[setup] ========================================="
echo "[setup] DepthAI/OAK-D Pro Setup"
echo "[setup] ========================================="
echo "[setup] Repo root: ${REPO_ROOT}"
echo "[setup] Using python: ${PYTHON_BIN} (${PY_VER})"
if [[ -n "${OS_VERSION}" ]]; then
    echo "[setup] OS version: ${OS_VERSION}"
fi
echo "[setup] Venv: ${VENV_DIR}"
echo "[setup] ========================================="

if [[ "${DO_UDEV}" -eq 1 ]]; then
  echo "[setup] Installing udev rules for DepthAI (requires sudo)..."
  
  # Remove old udev rules to start fresh (important for Ubuntu 24.04)
  echo "[setup] Removing old udev rules (if any)..."
  sudo rm -f /etc/udev/rules.d/80-depthai.rules
  sudo rm -f /etc/udev/rules.d/80-movidius.rules
  
  # Create udev rules optimized for Ubuntu 24.04
  # Using ATTRS (not ATTR) as it's more reliable on newer udev/systemd
  echo "[setup] Creating new udev rules..."
  sudo tee /etc/udev/rules.d/80-depthai.rules >/dev/null <<'EOF'
# Luxonis / DepthAI (OAK) USB permissions for Ubuntu 24.04
# Compatible with Ubuntu 20.04, 22.04, and 24.04
#
# Unbooted (bootloader): 03e7:2485  "Movidius MyriadX"
# Booted (app running):  03e7:f63b  "Luxonis Device"
#
# Ubuntu 24.04: Using ATTRS (more reliable than ATTR on newer systemd/udev)
# TAG+="uaccess" works with systemd-logind to grant access to logged-in users
# GROUP="plugdev" provides group-based access control

# Rule for unbooted device (03e7:2485)
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2485", MODE="0666", GROUP="plugdev", TAG+="uaccess"

# Rule for booted device (03e7:f63b)
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="f63b", MODE="0666", GROUP="plugdev", TAG+="uaccess"

# Vendor-wide fallback rule (covers all 03e7 devices)
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666", GROUP="plugdev", TAG+="uaccess"
EOF

  # Set correct file permissions
  sudo chmod 644 /etc/udev/rules.d/80-depthai.rules
  
  # Reload and trigger udev rules
  echo "[setup] Reloading udev rules..."
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  
  # On Ubuntu 24.04, ensure user is in plugdev group for USB access
  if ! groups | grep -q plugdev; then
    echo "[setup] Adding user to plugdev group..."
    sudo usermod -a -G plugdev "$USER" || true
    echo "[setup] WARNING: You were added to the plugdev group."
    echo "[setup] You MUST log out and log back in (or restart) for this to take effect."
    NEEDS_LOGOUT=1
  else
    NEEDS_LOGOUT=0
  fi
  
  echo ""
  echo "[setup] ========================================="
  echo "[setup] udev rules installed successfully!"
  echo "[setup] ========================================="
  if [[ "${NEEDS_LOGOUT}" -eq 1 ]]; then
    echo "[setup] ACTION REQUIRED:"
    echo "[setup]   1. Log out and log back in (or restart)"
    echo "[setup]   2. Unplug and replug your DepthAI device"
  else
    echo "[setup] ACTION REQUIRED:"
    echo "[setup]   Unplug and replug your DepthAI device now"
  fi
  echo "[setup] ========================================="
  echo ""
fi

if [[ ! -d "${VENV_DIR}" ]]; then
  echo "[setup] Creating venv..."
  "${PYTHON_BIN}" -m venv "${VENV_DIR}"
fi

echo "[setup] Installing Python deps..."
source "${VENV_DIR}/bin/activate"
python -m pip install --upgrade pip >/dev/null
python -m pip install -r "${SCRIPT_DIR}/requirements-depthai.txt"

echo ""
echo "[setup] ========================================="
echo "[setup] Setup Complete!"
echo "[setup] ========================================="
echo "[setup] To activate the virtual environment:"
echo "[setup]   source \"${VENV_DIR}/bin/activate\""
echo ""
echo "[setup] To test the device:"
echo "[setup]   python depthai_device_check.py"
echo "[setup]   python depthai_camera_test_script.py"
echo ""
if [[ "${DO_UDEV}" -eq 1 ]]; then
    echo "[setup] REMEMBER: Unplug and replug your DepthAI device"
    if [[ "${NEEDS_LOGOUT}" -eq 1 ]]; then
        echo "[setup]          Log out and back in for group changes"
    fi
    echo ""
fi
echo "[setup] ========================================="

if [[ "${RUN_CHECK}" -eq 1 ]]; then
  echo "[setup] Running standalone device check..."
  python "${SCRIPT_DIR}/depthai_device_check.py" --no-show --frames 10
fi

if [[ "${RUN_VIEWER}" -eq 1 ]]; then
  echo "[setup] Running OpenCV viewer (press q/ESC to quit)..."
  python "${SCRIPT_DIR}/depthai_camera_test_script.py"
fi