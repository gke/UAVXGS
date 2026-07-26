#!/bin/zsh

ROOT="$(cd "$(dirname "$0")" && pwd)"
SRC="$ROOT/src"
VENV="$ROOT/.venv"

cleanup() {
    if [ $? -ne 0 ]; then
        echo ""
        echo "============================================"
        echo "  BUILD FAILED — scroll up for error details"
        echo "============================================"
        echo ""
        read -k 1 "?Press Enter to close..."
    fi
}
trap cleanup EXIT

set -e

echo "[1/6] Checking Python..."
command -v python3 >/dev/null 2>&1 || { echo "ERROR: python3 not found"; exit 1; }
python3 --version

echo "[2/6] Creating venv..."
if [ ! -d "$VENV" ]; then
    python3 -m venv "$VENV"
else
    echo "  Venv already exists, reusing."
fi
source "$VENV/bin/activate"

echo "[3/6] Installing packages..."
python -m pip install --upgrade pip --quiet
python -m pip install PyQt5 pyserial folium PyQtWebEngine pyttsx3 pyinstaller --quiet

echo "[4/6] Checking Xcode Command Line Tools..."
if ! xcode-select -p >/dev/null 2>&1; then
    echo "Xcode Command Line Tools missing"
    xcode-select --install
    exit 1
fi
echo "Xcode CLT OK"

echo "[5/6] Cleaning old builds..."
cd "$SRC"
rm -rf build dist

echo "[6/6] Building .app..."
pyinstaller --clean UAVX_GCS.spec

deactivate 2>/dev/null || true

echo
echo "================================"
echo "Build complete:"
echo "$SRC/dist/UAVX_GCS.app"
echo "================================"

open "$SRC/dist"
