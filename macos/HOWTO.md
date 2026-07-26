# UAVX Groundstation — macOS Build & Install

## Quick Start (Zero Manual Setup)

Open **Terminal** to the `macos/` directory and run:

```bash
chmod +x build.sh && ./build.sh
```

The script will:
1. Install **Python 3.11** via Homebrew (or prompt for manual install) if not present
2. Install **Xcode Command Line Tools** (required by PyInstaller) if missing
3. Install all Python packages (PyQt5, pyserial, PyInstaller, etc.)
4. Build the standalone **`UAVX_GCS.app`** via PyInstaller
5. Copy `airframes/` alongside the `.app`

The finished `.app` is at `src/dist/UAVX_GCS` — double-click to launch.

> **Note:** First launch shows "UAVX_GCS is from an unidentified developer" — **right-click → Open** to bypass Gatekeeper once. For distribution, run `codesign` on the `.app`.

## Build .dmg Disk Image (Optional)

After `build.sh` succeeds:

```bash
brew install create-dmg
create-dmg --volname "UAVX_GCS" \
  --window-pos 200 120 --window-size 600 400 \
  --icon-size 100 --app-drop-link 450 200 \
  src/dist/UAVX_GCS.dmg \
  src/dist/UAVX_GCS.app
```

The `.dmg` appears at `src/dist/UAVX_GCS.dmg` — share this file. Users drag the `.app` to their Applications folder.

## File Layout

```
UAVXGS/
└── macos/                  ← YOU ARE HERE
    ├── build.sh           — builds the .app (auto-installs everything)
    ├── HOWTO.md           — this file
    ├── src/               ← Python source + airframes (self-contained)
    │   ├── main.py        — entry point
    │   ├── ui/            — Qt UI modules
    │   ├── core/          — data manager, speech, etc.
    │   └── airframes/     — .af tuning files
    └── UAVXF4V3Q_r16.bin  ← pre-compiled FC firmware
```

## Notes

- The `.app` is self-contained but needs `airframes/` alongside it (copied automatically by `build.sh`)
- First launch opens a serial port selector — connect via USB to the flight controller
- Telemetry logs and GPS tracks save to the directory the `.app` is run from
- Voice uses the built-in macOS `NSSpeechSynthesizer` — no extra TTS engine needed
- An internet connection is required for the first build (downloads Python + packages)
