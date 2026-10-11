#!/bin/bash
# UAVX Groundstation macOS Build Script
# Creates a venv, installs Python dependencies, builds standalone .app via PyInstaller

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/src"
VENV_DIR="$SCRIPT_DIR/.venv"

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
echo " UAVX Groundstation - macOS Build"
echo " (creates local venv, installs everything)"
echo "============================================"
echo ""

# ---------------------------------------------------------------
# Step 0: Ensure Python 3.10+ is installed
# ---------------------------------------------------------------
install_python() {
    echo "[0/5] Python 3 not found."
    echo ""
    echo "  Choose installation method:"
    echo "    1) Homebrew (recommended) — 'brew install python@3.11'"
    echo "    2) Official installer — download from python.org"
    echo ""
    read -rp "  Enter 1 or 2 [1]: " choice
    choice="${choice:-1}"

    if [ "$choice" = "1" ]; then
        if ! command -v brew &>/dev/null; then
            echo ""
            echo "  Homebrew not found. Install Homebrew first:"
            echo '    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"'
            echo ""
            echo "  Then re-run this script."
            exit 1
        fi
        echo "  Installing python@3.11 via Homebrew..."
        brew install python@3.11
        # brew doesn't add to PATH immediately; link it
        if [ -L /usr/local/bin/python3 ]; then
            :
        elif [ -f /usr/local/opt/python@3.11/bin/python3 ]; then
            ln -sf /usr/local/opt/python@3.11/bin/python3 /usr/local/bin/python3 2>/dev/null || true
        fi
    else
        echo ""
        echo "  Opening python.org download page..."
        open https://www.python.org/downloads/
        echo ""
        echo "  Download and install Python 3.10 or later."
        echo "  Make sure to check 'Install for all users' and"
        echo "  'Add Python to PATH' (the installer does this by default)."
        echo "  Then re-run this script."
        exit 0
    fi
}

if ! command -v python3 &>/dev/null; then
    install_python
fi

PYVER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "  Python found: $(python3 --version)"

python3 -c "import sys; exit(0 if sys.version_info >= (3,10) else 1)" || {
    echo "ERROR: Python 3.10 or later required (found $PYVER)"
    echo "  Upgrade from https://python.org or 'brew upgrade python'"
    exit 1
}

# ---------------------------------------------------------------
# Step 1: Create venv and install Python packages
# ---------------------------------------------------------------
echo ""
echo "[1/5] Setting up virtual environment..."

if [ ! -d "$VENV_DIR" ]; then
    echo "  Creating venv at $VENV_DIR ..."
    python3 -m venv "$VENV_DIR"
else
    echo "  Venv already exists, reusing."
fi

source "$VENV_DIR/bin/activate"

echo "  Upgrading pip..."
python -m pip install --upgrade pip --quiet

echo "  Installing packages..."
python -m pip install PyQt5 pyserial folium PyQtWebEngine pyttsx3 pyinstaller --quiet
echo "  Done."

# ---------------------------------------------------------------
# Step 2: Check for Xcode Command Line Tools (needed by PyInstaller)
# ---------------------------------------------------------------
echo ""
echo "[2/5] Checking Xcode Command Line Tools..."
if ! xcode-select -p &>/dev/null; then
    echo "  Xcode CLT not found. Installing (may take a while)..."
    xcode-select --install 2>/dev/null || true
    echo "  Please complete the Xcode CLT installation dialog, then re-run."
    exit 0
else
    echo "  Xcode CLT already installed."
fi

# ---------------------------------------------------------------
# Step 3: Clean previous builds
# ---------------------------------------------------------------
echo ""
echo "[3/5] Cleaning previous builds..."
rm -rf "$SRC_DIR/dist" "$SRC_DIR/build"

# ---------------------------------------------------------------
# Step 4: Build .app bundle with PyInstaller
# ---------------------------------------------------------------
echo ""
echo "[4/5] Building .app bundle with PyInstaller (this may take a minute)..."
pushd "$SRC_DIR" > /dev/null

pyinstaller --clean UAVX_GCS.spec

popd > /dev/null

# ---------------------------------------------------------------
# Step 5: Copy airframes alongside .app
# ---------------------------------------------------------------
echo ""
echo "[5/5] Copying airframes..."
cp -r "$SRC_DIR/airframes" "$SRC_DIR/dist/airframes"

# ---------------------------------------------------------------
# Done
# ---------------------------------------------------------------
deactivate 2>/dev/null || true

echo ""
echo "============================================"
echo "  SUCCESS!"
echo "  .app bundle created at:"
echo "    $SRC_DIR/dist/UAVX_GCS"
echo ""
echo "  Double-click the .app to launch."
echo "  (First time: right-click -> Open to bypass Gatekeeper)"
echo ""
echo "  To package as .dmg for distribution:"
echo "    brew install create-dmg"
echo "    create-dmg --volname 'UAVX_GCS' \\"
echo "      --window-pos 200 120 --window-size 600 400 \\"
echo "      --icon-size 100 --app-drop-link 450 200 \\"
echo "      '$SRC_DIR/dist/UAVX_GCS.dmg' \\"
echo "      '$SRC_DIR/dist/UAVX_GCS.app'"
echo "============================================"
echo ""
