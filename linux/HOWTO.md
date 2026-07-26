# UAVX Groundstation — Linux Build

## Zero-Setup Build

1. Open a terminal in this directory (`linux/`)
2. Run:
   ```bash
   chmod +x build.sh
   ./build.sh
   ```

The script will:
- Check Python 3.10+ is installed
- Install/verify system packages (PyQt5, pyserial) via apt if on Debian/Ubuntu
- Install Python packages via pip
- Build a standalone executable with PyInstaller

## Output

The standalone executable is at `src/dist/UAVX_GCS`. Run it with:

```bash
./src/dist/UAVX_GCS
```

## Manual Installation (if build script fails)

### Dependencies

```bash
# Debian/Ubuntu
sudo apt install python3 python3-pip python3-pyqt5 python3-serial
pip3 install PyQt5 pyserial folium PyQtWebEngine pyttsx3 pyinstaller

# Fedora
sudo dnf install python3 python3-pip python3-qt5
pip3 install PyQt5 pyserial folium PyQtWebEngine pyttsx3 pyinstaller

# Arch
sudo pacman -S python python-pip qt5-base
pip3 install PyQt5 pyserial folium PyQtWebEngine pyttsx3 pyinstaller
```

### Build

```bash
cd src
python3 -m PyInstaller \
    --onefile --windowed \
    --name "UAVX_GCS" \
    --add-data "airframes:airframes" \
    --hidden-import PyQt5.QtWebEngineWidgets \
    --hidden-import serial \
    --hidden-import folium \
    --collect-submodules core \
    --collect-submodules ui \
    main.py
```

The executable will be at `src/dist/UAVX_GCS`.

## Running from Source (no build needed)

```bash
cd src
python3 main.py
```

## Desktop Shortcut

```bash
ln -s "$(pwd)/src/dist/UAVX_GCS" ~/Desktop/UAVX_GCS
```

Or create `~/.local/share/applications/uavx-gcs.desktop`:

```ini
[Desktop Entry]
Name=UAVX Groundstation
Exec=/path/to/UAVX_GCS
Type=Application
Categories=Utility;
```
