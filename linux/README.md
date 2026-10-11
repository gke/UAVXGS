# UAVX Ground Station — Linux Kit

This kit contains everything needed to run the **UAVX Ground Station (GCS)** on
Linux, together with a prebuilt **UAVXArmQ** firmware image for STM32F4 flight
controllers.

## What's in the kit

| Path | Contents |
|------|----------|
| `src/` | GCS source (Python 3 / PyQt5) |
| `build.sh` | One-click build (installs packages, builds the executable) |
| `HOWTO.md` | Build & install instructions — start here |
| `PORTING_KIT.md` | Notes on the Python port |
| `UAVXF4V3Q_r0.bin` / `.elf` | Prebuilt flight-controller firmware |

## Quick start

```sh
chmod +x build.sh && ./build.sh
```

The executable is built at `src/dist/UAVX_GCS`. To run from source instead:
`cd src && python3 main.py`. See [HOWTO.md](HOWTO.md) for details.

## What's different from the old UAVXGUI / firmware

- Cross-platform Python 3 / PyQt5 GCS — no .NET or Mono.
- Unified float32 parameters (all 128) and tagged `.af` airframe files.
- OpenStreetMap navigation (no API key), KML track export and a CSV flight log.
- Combined IMU + magnetometer calibration with live sensor readback.
- Built-in DFU firmware flasher, raw packet monitor, tuning display and voice
  feedback.
- Firmware: quaternion attitude, unified float parameters, reassigned RC
  channels (Dive, Trace, RateGain), fixed-wing throttle cut and control-surface
  authority.

## Flight-controller firmware

The kit bundles `UAVXF4V3Q_r0.bin`. Install `dfu-util`, put the board into DFU
mode (hold **BOOT0**, tap reset, release) and flash:

```
dfu-util -a 0 -s 0x08000000:leave -D UAVXF4V3Q_r0.bin
```

The GCS **BOOTLOAD** button can also enter the bootloader over USB.

## License

Distributed under the GNU General Public License v3.
