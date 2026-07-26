# UAVX Project — Windows Porting Package

## For the Collaborator

This package contains everything needed to evaluate and test the UAVX flight controller firmware and its cross-platform ground control station on Windows.

---

## What's in the Box

```
windows/
├── build.bat              # One-click Windows .exe builder
├── HOWTO.md               # Build & install quick-start
├── installer.iss          # Windows installer script (Inno Setup)
├── PORTING_KIT.md         # This file
├── src/                   # GCS Python source (self-contained)
│   ├── main.py            # Entry point
│   ├── packet_parser.py   # Telemetry deserialiser
│   ├── protocol_enums.py  # Packet tags, parameter indices, states
│   ├── parameters.py      # Display multipliers and limits
│   ├── ui/                # Qt5 UI modules
│   ├── core/              # Data management, logging, speech
│   ├── widgets/           # Attitude indicator, bars, gauges
│   ├── airframes/         # Parameter set files (.af)
│   └── logger/            # CSV + KML telemetry logging
├── UAVXF4V3Q_r16.bin     # FC firmware — ready to flash to hardware
└── UAVXF4V3Q_r16.elf     # FC firmware with debug symbols
```

---

## What Does This System Do?

The UAVX project is a flight controller (FC) + ground control station (GCS) pair for multirotor aircraft.

- **The FC** (`UAVXArmQ/`) runs on an STM32F405 microcontroller on the flight controller board. It reads sensors (gyros, accelerometers, magnetometer, barometer, GPS), runs the control loops, and drives the motors.

- **The GCS** (`src/`) runs on a PC connected to the FC via USB serial. It displays telemetry (attitude, altitude, battery, GPS, throttle, rate tracking), allows parameter editing, mission planning on a map, calibration, and data logging.

- **The two communicate** over a framed serial protocol (115200 baud) at ~50 Hz.

---

## Quick Start on Windows

### 1. Build and Run the GCS

On a Windows machine with Python 3.10+ installed:

```
> cd src
> pip install PyQt5 pyserial folium PyQtWebEngine pyttsx3 pyinstaller
> python main.py
```

Or double-click `windows\build.bat` to produce a standalone `UAVX_GCS.exe`.

The GCS launches with no FC connected — it enters emulation mode automatically and generates synthetic flight data. This is the fastest way to see the instrument panel working without hardware.

### 2. Flash the FC Firmware

1. Power the flight controller board via USB
2. Put it in DFU mode (boot button + power cycle, or use the GCS bootloader command)
3. Flash `UAVXF4V3Q_r16.bin` using the built-in DFU flasher in the GCS, or an external ST-Link

Supported board: **UAVXF4V3** (STM32F405). The `.elf` contains debug symbols for GDB/OpenOCD.

### 3. Connect and Fly

1. Connect the FC via USB — it enumerates as a serial port (COMx)
2. Launch the GCS, select the COM port
3. Telemetry appears immediately — attitude sphere, altimeter, GPS, battery
4. Arm the aircraft and test altitude hold, navigation modes, RTH

---

## What to Test

### GCS on Windows

| Test | What to Look For |
|------|-----------------|
| Launch GCS with no FC | GCS enters emulation mode, synthetic data appears |
| Serial port enumeration | All COM ports listed, FC identifiable |
| Connect to FC | Telemetry streams at ~50 Hz, no CRC errors |
| Attitude indicator | Pitch/roll sphere tracks real FC attitude |
| Altitude display | Baro + GPS altitude tapes |
| Parameter window | Read all 128 params, edit a value, write back |
| Calibration | IMU and magnetometer calibration workflows |
| Map view | OpenStreetMap tiles load, aircraft position tracks |
| Mission planning | Create waypoints, upload to FC, read back |
| Telemetry logging | CSV and KML files saved to disk |
| Voice feedback | TTS announces events (arm, RTH, low battery) |

### Protocol Compatibility

| Test | What to Look For |
|------|-----------------|
| Packet framing | ESC unstuffing correct — no false sync on 0xFF in payload |
| Checksum | Every packet validated before parsing |
| All tags 0–78 | No unhandled tags, no parser crashes |
| Tuning packet (57) | Rate tracking data displays correctly |
| Config packet (63) | Firmware revision string received on connect |
| Parameter round-trip | Written values read back identically |

---

## How the Protocol Works (Summary)

```
FC                           GCS
 │                           │
 ├─ continuous broadcast ───→│  Flight (78), Guidance (59),
 │  every ~20 ms             │  Tuning (57), AltCtrl (60),
 │                           │  RC data (15), Control (16),
 │                           │  ExecTime (67), LinkStats (55)
 │                           │
 │←─ request (tag + subtag) ─┤  To read params, config, origin,
 │                           │  waypoints, or black-box data
 │                           │
 │←─ command (tag 50/51) ────┤  Calibrate IMU, mag, bootloader,
 │                           │  force defaults, cycle BB log
 │                           │
 │←─ param write (tag 17) ───┤  Single float32 parameter write
 │←─ param commit (tag 72) ──┤  Flash + condition + reset
```

The full framing specification is at `src/packet_parser.py` (the deserialiser) and `fc_src/telem.c` (the serialiser).

---

## Key Architectural Decisions

| Decision | Rationale |
|----------|-----------|
| Python 3 / PyQt5 | Cross-platform from day one. No .NET, no Mono, no Windows-only APIs. |
| 115200 baud | Balances throughput (~6 KB/s) with wire reliability. |
| Scaled int16 on wire | Float32 would double bandwidth. ×1000 scaling gives 0.1° resolution. |
| ESC stuffing (3 bytes) | Lightweight framing — 0xFF (sentinel) is valid payload, no byte-stuff for it. |
| XOR checksum | Simple, fast, catches single-bit errors. Not cryptographic. |
| Request-only for config | Static string (firmware revision) doesn't need 50 Hz broadcast. |

---

## Files Referenced

| File | Purpose |
|------|---------|
| `UAVXArmQ/src/telem.c` | FC-side telemetry generation — all packet serialisers |
| `UAVXArmQ/src/telem.h` | Packet tag enum, type-safe Tx macros |
| `UAVXArmQ/src/control.c` | Control loop — tuning packet replaces legacy attitude sends |
| `src/packet_parser.py` | GCS-side deserialiser — all packet parsers |
| `src/protocol_enums.py` | Shared enums (tags, states, config bits) |
| `src/ui/main_window.py` | Main GCS window — packet dispatch and display |
