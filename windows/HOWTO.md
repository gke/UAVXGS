# UAVX Groundstation — Windows 11 Build & Install

## Quick Start (Zero Manual Setup)

Double-click **`build.bat`**. That's it.

The script will:
1. Download and install **Python 3.11** (silent, admin) if not present
2. Install **Visual C++ Redistributable** (required by Qt) if missing
3. Install all Python packages (PyQt5, pyserial, PyInstaller, etc.)
4. Build the standalone **`UAVX_GCS.exe`** via PyInstaller
5. Copy `airframes/` alongside the executable

The finished `.exe` is at `src\dist\UAVX_GCS.exe` — double-click to run.

> **Note:** Windows may show a SmartScreen warning for the unsigned `.exe`. Click **More info → Run anyway`.

## Build Windows Installer (Optional)

After `build.bat` succeeds:

1. Run `build.bat` again — it will offer to install **Inno Setup** automatically
2. Or install manually from https://jrsoftware.org/isdl.php
3. Right-click **`installer.iss`** → **Compile**
4. The installer appears in `installer_output\UAVX_GCS_Setup_0.1.0.exe`

Share this `.exe` — users just click through to install with Start Menu / Desktop shortcuts.

## File Layout

```
UAVXGS/
└── windows/                 ← YOU ARE HERE
    ├── build.bat           — builds the .exe (auto-installs everything)
    ├── installer.iss       — builds the Windows installer
    ├── HOWTO.md            — this file
    ├── src/                ← Python source + airframes (self-contained)
    │   ├── main.py         — entry point
    │   ├── ui/             — Qt UI modules
    │   ├── core/           — data manager, speech, etc.
    │   └── airframes/      — .af tuning files
    └── UAVXF4V3Q_r16.bin   ← pre-compiled FC firmware
```

## Notes

- The `.exe` is self-contained but needs `airframes/` alongside it (copied automatically)
- First launch opens a serial port selector — connect via USB to the flight controller
- All data files (telemetry logs, GPS tracks) save to the directory the `.exe` is run from
- Voice uses Windows SAPI5 (built-in) — no extra TTS engine needed
- An internet connection is required for the first build (downloads Python + packages)
