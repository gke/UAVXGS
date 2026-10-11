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

## Flight-Controller Firmware

The kit bundles the current flight-controller firmware image, `UAVXF4V3Q_r0.bin`
(STM32F4 targets). To flash it:

1. Install `dfu-util` (or use STM32CubeProgrammer):

   ```bash
   sudo apt install dfu-util     # Debian/Ubuntu
   sudo dnf install dfu-util     # Fedora
   ```

2. Put the board into DFU/bootloader mode: hold **BOOT0** and tap reset (or
   power-cycle while BOOT0 is held), then release.
3. Flash the image:

   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D UAVXF4V3Q_r0.bin
   ```
4. Power-cycle the board; it should respond in the GCS.

When updating over USB, the GCS **BOOTLOAD** button can put a running board into
bootloader mode, so no BOOT0 link is needed.

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
