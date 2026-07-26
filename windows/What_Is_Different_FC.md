# What Is Different — FC: UAVXArm32F4 → UAVXArmQ

## Overview

UAVXArmQ is a quaternion-based fork of the original UAVXArm32F4 flight controller firmware. The attitude representation changed from Euler angles to quaternions internally, but the most visible differences from a user and parameter perspective are listed below.

---

## Attitude Representation

| Aspect | UAVXArm32F4 (old) | UAVXArmQ (new) |
|--------|-------------------|----------------|
| Internal attitude | Euler angles (roll, pitch, yaw) | Quaternion (q0, q1, q2, q3) |
| Gimbal lock risk | Yes (singularity at 90° pitch) | No |
| Computational cost | Lower (3 DOF) | Higher (4 DOF) |
| Sensor fusion | Complementary filter | Quaternion-based complementary filter |

**Impact**: No gimbal lock. The aircraft can execute extreme manoeuvres (loops, flips) without the attitude estimate collapsing. The flight behaviour is otherwise identical from the pilot's perspective.

---

## Parameter System — Unified Float

### Old System (UAVXArm32F4 + Old GCS)

Parameters were a mix of `float32` and `uint8` depending on the parameter index:
- Some parameters stored as 0–255 integers (e.g. gains, rates)
- Other parameters stored as native floats (e.g. altitudes, voltages)
- The GCS needed to know which format each parameter used
- A **Legacy checkbox** in the old GCS toggled between uint8 integer display and float display

### New System (UAVXArmQ + UAVXGS)

All 128 parameters are `float32`:
- Every parameter reads and writes as a decimal float over the wire
- No uint8 conversion, no `LEGACY_TAGS`, no `PARAM_SCALES`
- The **Legacy checkbox is gone** — there is nothing to toggle

### What This Means for the User

- **If you had a `.af` file from the old system**: The values that were uint8 will now appear as equivalent floats (e.g. `150` → `150.0`). The FC behaviour is identical — the FC internally casts `(float32)(uint8)value` on the old uint8 params, so the round-trip is exact for 0–255.
- **If you used the Legacy checkbox**: You no longer need it. All params display as decimals. The conversion table that mapped uint8 ranges to display values has been removed — the display multiplier and limits now apply uniformly.
- **Parameter limits**: Previously, uint8 params had a hard limit of 0–255. Now all params use the full float32 range, limited per-index by `PARAM_LIMITS` × `PARAM_DISPLAY_MULT`.

---

## Telemetry Changes

| Aspect | UAVXArm32F4 | UAVXArmQ |
|--------|-------------|----------|
| Attitude control debug | Three separate per-axis packets (tag 68) at ~2.5 Hz each | Single tuning packet (tag 57) at ~50 Hz with all axes |
| Config / revision | Broadcast every other telemetry cycle (~25 Hz) | Sent only on request (once on GCS connect) |
| Tuning data | Not available as a single frame | Rate tracking (desired/actual/out) for all axes in one 22-byte packet |

**Impact**: Less wire bandwidth wasted on debug data that no one reads. The tuning data is now available in a single predictable packet that the GCS can plot or log.

---

## Cruise Throttle Feedforward

| Aspect | UAVXArm32F4 | UAVXArmQ |
|--------|-------------|----------|
| Tracking target | `DesiredThrottle + AltHoldThrComp - pCruiseThrFF` | `AltHoldThrComp` (direct) |
| Tracking rate | 0.5%/s | 2%/s |
| ROC guard | Yes (could stall learning during gentle climbs) | No |
| Persistence | On disarm via `ConfigChanged` | Same, but not overridden by `ApplyParameters()` |
| `pMaxAltHoldThrComp` default | 15% | 25% |

**Impact**: Faster convergence to the correct hover throttle. The feedforward term is learned more aggressively and is no longer reset at parameter commit.

---

## Arming

| Aspect | UAVXArm32F4 | UAVXArmQ |
|--------|-------------|----------|
| Arming channel | Usually Channel 5 | Channel 6 (NavQualificationRC) |
| Switch arming | Physical arm switch | Physical arm switch on Ch6, AltHold always on, WPNav always on |
| Tx arming (pot) | Variable threshold | Arm >10%, AltHold >40% (in-flight), WPNav >60% |
| AltControlEnabled assignments | In `MapRC()` only | Same — no change, confirmed |

**Impact**: Arming is now consistently on Ch6 across both methods. The switch method enables all features unconditionally; the pot method provides progressive mode access.

---

## Failsafe

| Aspect | UAVXArm32F4 | UAVXArmQ |
|--------|-------------|----------|
| SBus failsafe detection | `sbusDecode()` sets `RCFailsafe` | Same |
| Timeout detection | `GenerateFailsafePacket()` loads FS | Same |
| FS presets | NavQual=1650, NavMode=RTH, sticks=neutral, throttle=idle | Same |
| Override at NavQual block | Forces AltHold + AngleMode | Same |
| PassThru behaviour | Overrides all failsafe | Same |

**Impact**: No change. Failsafe behaviour is identical.

---

## Emulation Constants (`emu.h`)

| Aspect | UAVXArm32F4 | UAVXArmQ |
|--------|-------------|----------|
| Default hover throttle | Generic | 0.55 (Ecks 2208+8×6, 800 g) |
| Default mass | Generic | 0.8 kg |
| Heavy model | N/A | EM_MODEL_HEAVY: 0.60 thr, 1.0 kg |

**Impact**: Emulation now matches a real airframe out of the box. The heavy variant matches the Racerstar 2205+5×3×3 build.
