# UAVXGS — Ground Station User Guide

## Overview

UAVXGS presents five main windows plus several instrument panels. This guide describes each window's function and how to engage with it.

---

## 1. Main Window — Instrument Panel

The primary window that opens on launch. It has four areas:

### Toolbar (top)

| Control | Function |
|---------|----------|
| **Connect** | Opens serial port selector; click again to disconnect |
| **COM:** | Serial port selector — auto-detect or manual `/dev/ttyXXX` / `COMx` |
| **Baud:** | Baud rate (115200 for UAVXArmQ) |
| **Params** | Opens the Parameter Window |
| **Nav** | Opens the Navigation / Mission Planning Window |
| **Calib** | Opens the Calibration Window |
| **Misc** | Opens the Raw Packet Monitor |
| **Flash** | Opens the DFU Firmware Flasher |
| **KML** | Checkbox — enables KML track logging |
| **Log** | Checkbox — enables CSV telemetry logging |
| **BB** | Checkbox — starts black-box dump from FC |

### Attitude Sphere (left, large)

3D artificial horizon showing roll and pitch. The aircraft icon is fixed; the horizon tilts. A compass rose rotates around the rim showing magnetic heading.

Below the sphere:
- **Exec bar** — CPU load bargraph
- **Revision label** — firmware name (e.g. "UAVX r16" or airframe name)

### Altitude Display (below sphere)

Large numeric altitude readout in metres (Kalman-filtered baro+acc). Beside it:
- **ROC** — rate of climb in m/s
- Sub-labels show the source (Altitude / ROC)

### Instrument Panels (right)

**State & Battery Row**
- Flight state (Idle / InFlight / Monitor)
- Nav state (PIC / Hold / RTH / WP)
- Current waypoint number
- Alarm status
- Battery voltage, current draw, consumed mAh

**Controls Row**
- Four miniature bargraphs: Throttle, Roll, Pitch, Yaw
- Green bars show normalised output (0–100%)

**IMU Box**
- Angles: Roll, Pitch, Yaw in degrees
- Gyros: Raw rates in rad/s
- Accels: L/R, F/B, D/U in m/s²
- MPU temperature
- Acc confidence (0–100%)

**Navigation Box**
- Next waypoint ID, bearing, distance, cross-track error
- Guidance bearing, elevation, home distance, hint
- Wind speed and direction (when WindEstValid)
- Magnetic variation (WMM)

**Altitude Box**
- Kalman-filtered altitude, ROC
- Baro variance, accel variance, bias variance
- Desired altitude (target)

**GPS Box**
- Latitude, Longitude (decimal degrees)
- Ground speed (m/s), course over ground (°)
- Satellites, HDOP, fix type
- 3D speed
- Altitude (GPS MSL)

**Flags Box**
- LED-style indicators: Armed, InFlight, AngleMode, AltHold, AltActive, GPS, HomeSet, WindEstValid, MagGood, HaveMagCal, UsingGPSAlt, HaveRangeFinder

**Voice Feedback Level**
- Dropdown in the status bar: Off, Errors, Warnings, Info, All
- Uses espeak (Linux) / NSSpeechSynthesizer (macOS) / SAPI5 (Windows)

---

## 2. Parameter Window

Opened via the **Params** toolbar button.

### Layout

Tabular view with parameter rows grouped by category tabs:

| Tab | Parameters |
|-----|-----------|
| Basic | Airframe type, Rx mode, battery, emulation |
| Gains | Roll/Pitch/Yaw PIDs, feedforward, rate limits |
| Rates | Max roll/pitch/yaw rates, throttle curve |
| Altitude | Altitude hold gains, baro config, KF tuning |
| Navigation | Waypoint radii, velocity limits, bank angle limits |
| Misc | Serial ports, telemetry rate, speaker, LED config |
| Tuning | Cruise throttle, accel/decel gains, vibration damping |

### Operations

| Action | How |
|--------|-----|
| **Read** | Reads all 128 params from FC |
| **Write** | Writes all changed params to FC |
| **Save** | Saves current set to `.af` file |
| **Load** | Loads a `.af` file into the table (not written to FC until Write) |
| **Commit** | Writes + conditions + flashes + resets FC |
| **Edit** | Double-click a value cell, type new display value, press Enter |
| **Revision** | Displays current firmware revision at top of window |

### Display vs FC Units

Parameters are shown in display units (degrees, percent, metres, etc.) and converted to FC-native units (radians, 0–1, etc.) on write. The conversion multiplier is shown in the table header.

---

## 3. Navigation Window

Opened via the **Nav** toolbar button.

### Map View

- OpenStreetMap tiles loaded via `QWebEngineView`
- Aircraft position shown as a blue aircraft icon with heading arrow
- Home position shown as a green H
- Waypoints shown with ID labels and connecting lines

### Mission Controls

| Action | How |
|--------|-----|
| **Read** | Downloads the current mission from FC |
| **Write** | Uploads the current mission to FC |
| **Clear** | Removes all waypoints |
| **Load** | Opens a mission file |
| **Save** | Saves current mission + map screenshot |
| **Add WP** | Adds a waypoint at the map centre |
| **Delete WP** | Removes selected waypoint |
| **GoTo** | Single-click on map creates a 1-point GoTo mission |
| **Right-click menu** | Set waypoint type: VIA, ORBIT, PERCH, SETPOI, SURVEY |

### Waypoint Types

| Type | Behaviour |
|------|-----------|
| **VIA** | Fly through at altitude, pause for T seconds |
| **ORBIT** | Orbit at radius R with velocity V |
| **PERCH** | Land at waypoint, then take off and resume |
| **SETPOI** | Sets a point of interest — aircraft points camera here |
| **SURVEY** | Emits periodic pulse on Aux1 pin |

### Emulation

When no FC is connected, the GCS auto-starts emulation mode. The FC simulator generates synthetic GPS and flight data. The Nav window shows a default mission that can be edited and tested without hardware.

---

## 4. Calibration Window

Opened via the **Calib** toolbar button.

### Sensors

| Button | Calibration | What to Do |
|--------|-------------|------------|
| **Calibrate IMU** | Acc + Gyro | Place aircraft level, click, wait for green |
| **Calibrate Acc** | Accelerometer only | Same as IMU |
| **Calibrate Gyro** | Gyroscope only | Aircraft still, click, wait |
| **Calibrate Mag** | Magnetometer | Rotate aircraft in all orientations, click, wait |
| **Calibrate Level** | Level trim | Aircraft level, click |

### Indicators

- Each sensor shows a status LED: Red (uncalibrated) → Yellow (in progress) → Green (done)
- Live sensor readback during calibration
- All calibration buttons are disabled when aircraft is armed

### Utility Buttons

| Button | Function |
|--------|----------|
| **Force Defaults** | Resets all parameters to factory defaults |
| **Init GPS** | Re-initialises the GPS module |
| **GPS Passthrough** | Enters GPS configuration passthrough mode |
| **Bootloader** | Jumps to DFU bootloader (for firmware flash) |

---

## 5. Misc Window — Raw Packet Monitor

Opened via the **Misc** toolbar button.

### Display

A scrolling table showing every received packet:

| Column | Content |
|--------|---------|
| Tag | Decimal tag number |
| Name | Human-readable name (Flight, Tuning, Param, etc.) |
| Dir | R (Received) or T (Transmitted) |
| Body | Hex dump of raw packet body |

### Use Cases

- Protocol debugging — verify all expected tags are arriving at correct rate
- Bandwidth analysis — count packets per second by tag
- New feature development — check that new packet parsers work correctly
- Signal integrity — look for checksum errors or partial packets

---

## 6. DFU Flasher Window

Opened via the **Flash** toolbar button.

### Function

Uploads firmware to the STM32 flight controller via the UART DFU bootloader. No external programmer (ST-Link) required.

### Workflow

1. Select the firmware `.bin` file via the file picker
2. Put the FC in DFU mode (hold boot button, power cycle, or click "Enter DFU" in the flasher)
3. Click **Flash**
4. Progress bar shows upload progress
5. Click **Reset** to restart the FC with new firmware

---

## 7. Status Bar (bottom of main window)

| Element | Function |
|---------|----------|
| Status message | Brief text showing current operation / errors |
| **Debug** | Checkbox to toggle debug messages |
| **Level** | Debug verbosity: Info, Warnings, Errors, All |
| **Speech** | Voice feedback level selector |
| **Connected indicator** | Red/green dot with connect status |
