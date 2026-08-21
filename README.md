# UAVX Ground Station

Ground Control Station for UAVX flight controllers (UAVXArmQ firmware).

Python 3 / PyQt5 desktop application connecting to the FC over USB serial.

## Features

- Live telemetry: attitude, altitude, battery, GPS, nav status
- Full parameter tuning with per-airframe `.af` files (FC-native units)
- Airframe save/load and comparison against FC defaults
- Firmware flashing via STM32 DFU or UART bootloader
- Navigation setup, failsafe configuration, calibration tools

## Running

Standalone kits (PyInstaller, local venv):

| Platform | Build |
|----------|-------|
| Linux    | `cd linux && bash build.sh` |
| macOS    | `cd macos && bash build.sh` |
| Windows  | Double-click `build.bat` |

Or run from source: `python3 uavx-python/src/main.py` (requires PyQt5, pyserial).

## Documentation

See `wiki/docs/` for setup guides, flight modes, and failsafe.
