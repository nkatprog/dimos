#!/usr/bin/env bash
# Install Unitree SDK and prerequisites into the dimos .venv (uses uv)
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PYTHON="$REPO_DIR/.venv/bin/python"
CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-$HOME/cyclonedds/install}"
SDK2_PATH="/opt/unitree_sdk2_python"

echo "=== Unitree SDK Install ==="
echo "  repo:            $REPO_DIR"
echo "  venv python:     $VENV_PYTHON"
echo "  CYCLONEDDS_HOME: $CYCLONEDDS_HOME"
echo ""

# ── 1. Verify prerequisites ─────────────────────────────────────────────────
if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "ERROR: venv not found at $REPO_DIR/.venv — run 'uv sync' first"
    exit 1
fi

if [[ ! -d "$CYCLONEDDS_HOME" ]]; then
    echo "ERROR: CycloneDDS not found at $CYCLONEDDS_HOME"
    echo "  Build it with:"
    echo "    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x ~/cyclonedds"
    echo "    cd ~/cyclonedds && mkdir -p build install && cd build"
    echo "    cmake .. -DCMAKE_INSTALL_PREFIX=../install && cmake --build . --target install -j\$(nproc)"
    exit 1
fi

if [[ ! -d "$SDK2_PATH" ]]; then
    echo "ERROR: unitree_sdk2_python not found at $SDK2_PATH"
    echo "  Clone it with:"
    echo "    sudo git clone https://github.com/unitreerobotics/unitree_sdk2_python.git $SDK2_PATH"
    echo "    sudo chown -R \$USER:\$USER $SDK2_PATH"
    exit 1
fi

# ── 2. Install CycloneDDS Python bindings ───────────────────────────────────
echo "--- Installing cyclonedds==0.10.2 ---"
CYCLONEDDS_HOME="$CYCLONEDDS_HOME" uv pip install "cyclonedds==0.10.2"

# ── 3. Install unitree_sdk2py (editable) ────────────────────────────────────
echo ""
echo "--- Installing unitree_sdk2py from $SDK2_PATH ---"
# Install without deps first (cyclonedds already handled above to avoid version conflict)
CYCLONEDDS_HOME="$CYCLONEDDS_HOME" uv pip install --no-deps -e "$SDK2_PATH"
# Install remaining deps (numpy, opencv-python) separately
uv pip install "numpy<2.0,>=1.26" "opencv-python"

# ── 4. Install unitree-webrtc-connect-leshy ──────────────────────────────────
echo ""
echo "--- Installing unitree-webrtc-connect-leshy ---"
uv pip install "unitree-webrtc-connect-leshy>=2.0.7"

echo ""
echo "=== Installation complete ==="
echo ""
echo "To validate:"
echo "  $VENV_PYTHON -c \"import unitree_sdk2py; print('unitree_sdk2py OK')\""
echo "  $VENV_PYTHON -c \"from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection; print('webrtc OK')\""
