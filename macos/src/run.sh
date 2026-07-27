#!/bin/bash
PYTHON=/home/gke/Documents/Flight/Code/UAVXGS/uavx-python/venv/bin/python3
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"
PIDFILE="/tmp/uavxgs.pid"
if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    exit 0
fi
echo $$ > "$PIDFILE"
trap 'rm -f "$PIDFILE"' EXIT
find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null
PYTHONDONTWRITEBYTECODE=1 "$PYTHON" main.py
