#!/bin/bash
# UAVX Groundstation Linux Build Script
# Creates a venv, installs Python dependencies, builds standalone executable via PyInstaller

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/src"
VENV_DIR="$SCRIPT_DIR/.venv"

# Keep terminal open on error so the user can read the message
cleanup() {
    if [ $? -ne 0 ]; then
        echo ""
        echo "============================================"
        echo "  BUILD FAILED — scroll up for error details"
        echo "============================================"
        echo ""
        read -rp "Press Enter to close..."
    fi
}
trap cleanup EXIT

set -e

echo "============================================"
echo " UAVX Groundstation - Linux Build"
echo " (creates local venv, installs everything)"
echo "============================================"
echo ""

# ---------------------------------------------------------------
# Step 0: Ensure Python 3.10+ is installed
# ---------------------------------------------------------------
command -v python3 &>/dev/null || {
    echo "[0/5] Python 3 not found."
    echo ""
    echo "  Install Python 3.10+ via your package manager:"
    echo "    Debian/Ubuntu/Mint:  sudo apt install python3 python3-venv"
    echo "    Fedora:              sudo dnf install python3"
    echo "    Arch:                sudo pacman -S python"
    echo ""
    echo "  Then re-run this script."
    exit 1
}

PYVER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "  Python found: $PYVER"

python3 -c "import sys; exit(0 if sys.version_info >= (3,10) else 1)" || {
    echo "ERROR: Python 3.10 or later required (found $PYVER)"
    echo "  Upgrade via your package manager."
    exit 1
}

# ---------------------------------------------------------------
# Step 1: Ensure python3-venv is available (Debian/Ubuntu/Mint)
# ---------------------------------------------------------------
echo ""
echo "[1/5] Checking system packages..."

if command -v apt-get &>/dev/null; then
    echo "  Debian/Ubuntu/Mint detected..."
    # python3-venv is needed to create the venv
    if ! dpkg -s python3-venv &>/dev/null 2>&1; then
        echo "  Installing python3-venv..."
        sudo apt-get install -y python3-venv || {
            echo ""
            echo "  ERROR: Could not install python3-venv."
            echo "  Run manually: sudo apt install python3-venv"
            exit 1
        }
    fi
    echo "  System packages OK."
else
    echo "  (non-Debian distro, skipping apt checks)"
fi

# ---------------------------------------------------------------
# Step 2: Create venv and install Python packages
# ---------------------------------------------------------------
echo ""
echo "[2/5] Setting up virtual environment..."

if [ ! -d "$VENV_DIR" ]; then
    echo "  Creating venv at $VENV_DIR ..."
    python3 -m venv "$VENV_DIR"
else
    echo "  Venv already exists, reusing."
fi

# Activate venv
source "$VENV_DIR/bin/activate"

echo "  Upgrading pip..."
python -m pip install --upgrade pip --quiet

echo "  Installing packages..."
python -m pip install PyQt5 pyserial folium PyQtWebEngine pyttsx3 pyinstaller --quiet
echo "  Done."

# ---------------------------------------------------------------
# Step 3: Clean previous builds
# ---------------------------------------------------------------
echo ""
echo "[3/5] Cleaning previous builds..."
rm -rf "$SRC_DIR/dist" "$SRC_DIR/build"

# ---------------------------------------------------------------
# Step 4: Build executable with PyInstaller
# ---------------------------------------------------------------
echo ""
echo "[4/5] Building executable with PyInstaller (this may take a minute)..."
pushd "$SRC_DIR" &>/dev/null

pyinstaller --clean UAVX_GCS.spec

popd &>/dev/null

# ---------------------------------------------------------------
# Step 5: Copy airframes alongside executable
# ---------------------------------------------------------------
echo ""
echo "[5/5] Copying airframes..."
mkdir -p "$SRC_DIR/dist/airframes"
cp -r "$SRC_DIR/airframes/"* "$SRC_DIR/dist/airframes/" 2>/dev/null || true

# ---------------------------------------------------------------
# Done
# ---------------------------------------------------------------
deactivate 2>/dev/null || true

echo ""
echo "============================================"
echo "  SUCCESS!"
echo "  Standalone executable:"
echo "    $SRC_DIR/dist/UAVX_GCS"
echo ""
echo "  Run it:"
echo "    $SRC_DIR/dist/UAVX_GCS"
echo ""
echo "  To create a .desktop shortcut:"
echo "    ln -s \$PWD/$SRC_DIR/dist/UAVX_GCS ~/Desktop/"
echo "============================================"
echo ""
