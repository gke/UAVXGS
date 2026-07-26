#!/bin/bash
PYTHON=/home/gke/Documents/Flight/Code/UAVXGS/uavx-python/venv/bin/python3
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"
find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null
"$PYTHON" main.py

