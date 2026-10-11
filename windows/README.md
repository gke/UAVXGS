# UAVX Ground Station — Windows Kit

This kit contains everything needed to run the **UAVX Ground Station (GCS)** on
Windows 11, together with a prebuilt **UAVXArmQ** firmware image for STM32F4
flight controllers.

## What's in the kit

| Path | Contents |
|------|----------|
| `src/` | GCS source (Python 3 / PyQt5) |
| `build.bat` | One-click build (installs Python, packages, builds the `.exe`) |
| `installer.iss` | Inno Setup script for the Windows installer |
| `HOWTO.md` | Build & install instructions — start here |
| `GCS_Guide.md` | Screen-by-screen user guide |
| `What_Is_Different_GCS.md` | Full old-vs-new GCS change list |
| `What_Is_Different_FC.md` | Full old-vs-new firmware change list |
| `UAVXF4V3Q_r0.bin` / `.elf` | Prebuilt flight-controller firmware |

## Quick start

1. Double-click **`build.bat`** — it installs everything and builds
   `src\dist\UAVX_GCS.exe`.
2. Run `UAVX_GCS.exe` (or build `installer.iss` for a Start Menu installer).

See [HOWTO.md](HOWTO.md) for details.

## What's different from the old UAVXGUI / firmware

- Cross-platform Python 3 / PyQt5 GCS — no .NET or Mono.
- Unified float32 parameters (all 128) and tagged `.af` airframe files.
- OpenStreetMap navigation (no API key), KML track export and a CSV flight log.
- Combined IMU + magnetometer calibration with live sensor readback.
- Built-in DFU firmware flasher, raw packet monitor, tuning display and voice
  feedback.
- Firmware: quaternion attitude, unified float parameters, reassigned RC
  channels (Dive = rapid descent, Trace = diagnostic capture, RateGain =
  in-flight rate scaling), fixed-wing throttle cut and control-surface
  authority.

## Flight-controller firmware

The kit bundles `UAVXF4V3Q_r0.bin`. Put the board into DFU mode (hold **BOOT0**,
tap reset, release) and flash:

```
dfu-util -a 0 -s 0x08000000:leave -D UAVXF4V3Q_r0.bin
```

The GCS **BOOTLOAD** button can also enter the bootloader over USB.

## License

Distributed under the GNU General Public License v3.
