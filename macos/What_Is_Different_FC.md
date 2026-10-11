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

All 128 parameters are `float32` on the wire — there is no per-index uint8/float split. Every parameter reads and writes as a decimal float; no uint8 packing is performed by the GCS.

The **Legacy checkbox has not gone away** — it is still on the Advanced Parameters page, but its job has changed. It is now purely a **display** toggle:
- **Off (the new default):** every parameter is shown as its unified raw float value, scaled by `PARAM_DISPLAY_MULT`.
- **On:** the parameters listed in `LEGACY_TAGS` are shown in the classic legacy view instead — the raw value divided by that parameter's `PARAM_SCALES` factor (e.g. a rate gain that reads `0.20` raw shows as `20`), matching the old GCS's integer presentation.

It is **display-only**: the raw float32 value written to, and read back from, the FC is identical in both modes. A spinbox edited in Legacy mode is rescaled to raw on write, so the checkbox never changes what the FC stores or how the aircraft flies.

The Legacy view is **transitional**. It lets you read and edit in the old familiar units if you are used to them, but it is best avoided: the per-parameter `PARAM_SCALES` factors are arbitrary historical constants, not physically meaningful units, so the numbers it shows carry no more information than the raw value. Prefer the default unified raw-float view.

### What This Means for the User

- **If you had a `.af` file from the old system**: The values that were uint8 will now appear as equivalent floats (e.g. `150` → `150.0`). The FC behaviour is identical — the FC internally casts `(float32)(uint8)value` on the old uint8 params, so the round-trip is exact for 0–255.
- **If you used the Legacy checkbox**: it is still there, but now defaults **off** (unified raw float). Tick it to see the classic legacy-scaled values for the legacy-tagged parameters; the value sent to the FC is unchanged either way. Treat it as a transitional crutch — the legacy scaling is arbitrary, so do not rely on it.
- **Parameter limits**: Previously, uint8 params had a hard limit of 0–255. Now all params use the full float32 range, limited per-index by `PARAM_LIMITS` × the active display multiplier (`PARAM_DISPLAY_MULT`, or `1/PARAM_SCALES` for legacy-tagged params in Legacy mode).

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

## Fixed-Wing Throttle Cut

Fixed-wing has no idle throttle — these are electric airframes, so a commanded zero genuinely stops the propeller and acts as a brake. UAVXArmQ adds a machine-commanded FW throttle shaper so a low throttle command cuts cleanly to zero instead of idling or chattering at the bottom of the range.

| Aspect | UAVXArm32F4 | UAVXArmQ |
|--------|-------------|----------|
| Low-throttle command | Followed continuously | Snaps to 0 below `Thr Cut` |
| Re-open | N/A | Requires `Thr Cut + Hyst` (hysteresis) |
| Open rate | N/A | Rate-limited by `Ramp` (%/s) |
| Scope | — | FW altitude-hold / dive-recovery only |

- Below **`Thr Cut`** the shaped FW command snaps to **0**.
- Re-opening requires exceeding **`Thr Cut + Hyst`**, so the motor does not chatter at the threshold.
- Closing is an instant snap; opening is rate-limited by **`Ramp`** so a cut is recovered smoothly rather than as a step.
- **Manual pass-through is not shaped**, and neither is FW waypoint/nav throttle, so a low navigation command cannot be cut to zero. VTOL is untouched.

| Param | Slot | Meaning | Default | Range |
|-------|-----:|---------|--------:|-------|
| `FWThrCut` | 67 | Cut threshold | 20 % | 0..100 % |
| `FWThrRampPS` | 112 | Re-open slew rate | 33 %/s | 0..50 %/s |
| `FWThrHystBand` | 113 | Hysteresis band | 3 % | 0..100 % |

These three slots previously held the retired `FWMaxClimbAngle` (67), `FWStickScale` (112) and `FWRollControlPitchLimit` (113) parameters, which are no longer used.

**Impact**: The motor can be braked on demand (for example, on final approach) without special mixing, and the cut is recovered smoothly rather than as a step.

---

## Fixed-Wing Control-Surface Authority

The commissioned fixed-wing airframes were over-effective in pitch and roll: relative to the gains they were tuned around, the control surfaces produced roughly four times too much authority, making the aircraft twitchy. UAVXArmQ adds a per-axis output scale for the stabilised FW path.

| Param | Slot | Meaning | Default | Range |
|-------|-----:|---------|--------:|-------|
| `FWRollAuthority` | 75 | Roll throw scale | 25 % | 25..125 % |
| `FWPitchAuthority` | 124 | Pitch throw scale | 25 % | 25..125 % |
| `FWYawAuthority` | 126 | Yaw throw scale | 25 % | 25..125 % |

- Applied only to the stabilised FW path, after the output clamp. `100 %` means no correction; `25 %` means one-quarter throw.
- Manual pass-through keeps full pilot throw (Tx dual rates / expo still apply).
- MR / VTOL / Land are unaffected.

**Impact**: The same airframe gains now produce a sensible surface throw, so fixed-wing models no longer need heavy Tx rate reduction to fly smoothly.

---

## Fixed-Wing Commissioning Defaults

The shipped FW defaults were re-commissioned so a fresh fixed-wing model is flyable without hand-tuning first:

| Param | Slot | Old | New |
|-------|-----:|----:|----:|
| `FWClimbThrottle` | 21 | 0 | 60 % |
| `FWPitchThrottleFF` | 65 | 1 % | 0 |
| `FWAileronDifferential` | 70 | 25 % | 40 % conventional / 0 tailless |
| `FWAileronRudderMix` | 86 | 20 % | 0 |

**Impact**: Climb and altitude-hold have real throttle authority out of the box; pitch-to-throttle coupling and aileron-to-rudder mixing are off (they are airframe-specific), and aileron differential is set for the airframe type.

---

## Magnetometer Warm-Up Retry

| Aspect | UAVXArm32F4 | UAVXArmQ |
|--------|-------------|----------|
| Mag probe failure at boot | Magnetometer stays inactive for the flight | Probed again up to 6 times at 5 s intervals while inactive and disarmed |

**Impact**: A magnetometer that is slow to come up, or that misses the first probe, recovers automatically on the bench before the flight.

---

## Arming

The arming-qualification behaviour carried over from `NavQualificationRC` is
unchanged; the channel **functions** have been reassigned. The single
`NavQualificationRC` channel (`RxAux2Ch`) is replaced by dedicated channels.

| Aspect | UAVXArm32F4 | UAVXArmQ |
|--------|-------------|----------|
| Arming / mode channel | One channel: `NavQualificationRC` (`RxAux2Ch`) | Reassigned: `eArmingRC` (map index 4), `eAttitudeModeRC` (5), `eNavModeRC` (6) |
| Switch arming | Physical arm switch | `pArmingMode == eSwitchArming` (+ `eArmedSel` GPIO) |
| Tx arming (pot) | Variable threshold | `eArmingRC` > 10 % |
| Altitude hold | `NavQual` > 40 % (in flight) | `F.AltControlEnabled` enabled unconditionally |
| WPNav enable | `NavQual` > 60 % | `eNavModeRC` three-position switch |
| `AltControlEnabled` assignments | In `MapRC()` | In `UpdateControls()`/init — not `MapRC()` |

**Impact**: Same NavQual behaviour; the functions now sit on dedicated channels with reassigned defaults instead of one shared channel.

### Other reassigned channels

- **Dive** (`eDiveRC`, map index 8) — the pilot Dive-mode trigger. Above 70 %
  stick, on a multirotor and outside pass-through, it raises `F.DiveMode`, which
  runs the supervised rapid-descent profile in `dive.c` (Idle → Active →
  Recover, with an optional Spiral), flaring back to level by `pDiveRecoverAlt`
  (default 15 m).
- **Trace** (`eTraceRC`, map index 9) — the pilot diagnostic-capture trigger on
  the Aux2 output. The channel value is mirrored to `PW[Aux2C]`; when
  `pTraceType` is not `None`, a rising edge of its high half starts a trace
  capture.
- **RateGain** (`eTransitionRC`, map index 10) — sets `RateGainScale` to 4 to
  the power (2x − 1), an in-flight multiplier from 0.25× to 4× applied to the
  rate controllers' P+D output (`control.c`).

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
