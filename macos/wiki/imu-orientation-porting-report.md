# IMU Orientation Porting Guide — UAVXArmQ

**Status:** RESOLVED for SpeedyBee F405 Wing; mechanism documented for future targets
**Scope:** `UAVXArmQ/src/sensors/*`, `src/boards/targets/*.inc`
**Date:** 2026-08-24

---

## 1. Summary

The SpeedyBee F405 Wing port presented a persistent axis cross-swap (pitch-up read as
roll, roll-right read as pitch). Resolution took several bench iterations because three
independent defects stacked:

1. **Legacy driver bug (all IMU drivers):** `RotateSensor()` was applied only to the
   accelerometer pair; the gyro pair at `B[4..6]` was never rotated. Inert on every
   previously flown board because they all ran `IMUQuadrant = 0` (a no-op), but the
   first non-zero-quadrant target exposed it immediately.
2. **Wrong quadrant guess in the target file** (`CW270_DEG` copied rather than derived).
3. **Wrong mounting assumption:** the BMI270 is bottom-mounted on this PCB. The initial
   flip patch assumed a face-up hinge family and negated the wrong axes.

Final proven configuration: **`IMUQuadrant = 2` (CW180_DEG), `IMUSensorFlip = false`.**

Key geometry lesson: a bottom-side-mounted chip whose package face points *down* already
reads board-down on its Z axis — it needs **no Z inversion at all**, just the in-plane
rotation. Flip-style patches (`CW180_DEG_FLIP` family) are for mounts where the chip's
Z sense is inverted relative to the contract.

---

## 2. The Contract — `ScaleAccAndRate()`

All sensor drivers must deliver post-transform values satisfying the project's body
frame convention. Non-VTOL branch (`imu.c:53–62`, NED, P/R/Y vs BF/LR/UD):

| Channel | Expression | Meaning |
|---|---|---|
| `Rate[ePitch]` | `+IMURate[X]` | pitch rate from sensor X |
| `Rate[eRoll]` | `+IMURate[Y]` | roll rate from sensor Y |
| `Rate[eYaw]` | `-IMURate[Z]` | yaw rate from negated sensor Z |
| `Acc[LR]` | `+IMUAcc[X]` | lateral from sensor X |
| `Acc[BF]` | `+IMUAcc[Y]` | fore-aft from sensor Y |
| `Acc[UD]` | `-IMUAcc[Z]` | up-axis from negated Z |

Level expectation: `Acc[UD] = +1g`, i.e. raw post-transform `Z = -1g` at rest
(the reference MPU-6050 top-mount sense).

The VTOL branch maps the same chips differently (`imu.c:42–51`) — drivers must stay
generic; all axis semantics live in `ScaleAccAndRate()`.

---

## 3. Mechanism

### 3.1 `RotateSensor()` — quadrants

`filters.c`: rotates a signed int16 pair by yaw quadrant:

| `Q` | Transform | iNav/Betaflight equivalent |
|---|---|---|
| 0 | identity | `CW0_DEG` |
| 1 | `(x, y) → (y, -x)` | `CW090_DEG` |
| 2 | `(x, y) → (-x, -y)` | `CW180_DEG` |
| 3 | `(x, y) → (-y, x)` | `CW270_DEG` |

### 3.2 `IMUSensorFlip` — the underside case

Equivalent to iNav/Betaflight `CW180_DEG_FLIP`: chip rotated 180° about its own Y axis
relative to a top mount ⇒ negate **X and Z of both instruments identically**
(one die, one mounting — asymmetric patches invert yaw or decouple the estimator's
accel/gyro frames).

Application order in the drivers: unpack → both `RotateSensor()` calls → flip block.
The accel and gyro pairs must ALWAYS receive identical treatment.

### 3.3 Driver coverage

| Driver | Accel rotate | Gyro rotate | Flip block |
|---|---|---|---|
| `bmi270.c` | ✓ :132 | ✓ :133 | ✓ :139 (symmetric X/Z) |
| `icm426xx.c` | ✓ :123 | ✓ :124 | ✓ :131 (ICM-specific empirical patch) |
| `mpu6xxx.c` | ✓ :124 | ✓ :125 | – (no flipped boards use MPU) |
| `hmc5xxx_mag.c` | `MagQuadrant` only :137 | n/a | n/a |

The ICM flip block differs deliberately (negates accel X, gyro X/Z): it compensates the
ICM-42688's own package axis layout versus the MPU reference, not pure mount geometry.
Do not copy it to other parts blindly — derive per chip from the bench grid below.

---

## 4. Current Target Values

| Target | `IMUQuadrant` | `IMUSensorFlip` | `MagQuadrant` |
|---|---|---|---|
| `speedybeef405wing` | **2** | false | 0 |
| `flyingrcf4wingmini` | 0 | **true** (bottom-mount ICM) | 0 |
| `DevEBoxF4` | 3 | false | 3 |
| `omnibusf4nxt` / `omnibusf4v1` | 1 | false | 1 |
| `matekf405te` | 0 | false | 0 |
| `discoveryf4` | 0 | false | 0 |
| `uavxf4v3` / `uavxf4v4` | 0 | false | 0 |

Orientation constants are compile-time only (baked into the `.inc`). There are no
runtime params by design: anyone who needs different orientation is creating a new
target, and target creation requires the toolchain anyway.

---

## 5. Detection Procedure — Three Moves

For a new target, run this once on the bench with any reasonable starting config:

1. **Yaw right** (crisp rotation about the vertical axis):
   - Heading/yaw responds correctly → Z family right, no flip needed.
   - Yaw reads reversed → Z-inverted mount branch: the answer includes
     `IMUSensorFlip = true` (or the chip-specific equivalent patch).

2. **Pitch up ~30°, hold ~2 s** (quasi-static — the accel dominates):
   - Reads pitch up → horizontal mapping correct.
   - Reads pitch down → 180° off: toggle quadrant between 0 ↔ 2.
   - Reads roll left/right → ±90° off: pick quadrant 1 or 3 by direction.

3. **Set down level**: horizon must settle flat (`Acc[UD] → +1g`); gyro offsets are
   captured separately by `ErectRateGyros()` — not an orientation concern.

One pitch observation plus one yaw observation uniquely determine the configuration;
the optional third check (roll-right tilt) adds redundancy before first flight.

---

## 6. Worked Example — SpeedyBee F405 Wing BMI270

| Attempt | Config | Observation | Verdict |
|---|---|---|---|
| Port default | Q=3, accel-only rotation | pitch-up → roll-left | legacy gyro bug + wrong Q |
| Fix pair bug | Q=1 both | same cross | accel frame changed 180°, display didn't budge ⇒ downstream of sensors? No — stale flash suspicion |
| Fresh flash | Q=3 both + flip{Ax⁻, Gx⁻, Gz⁻} | cross flipped chirality; **yaw reversed** | yaw datum convicts the copied gyro-Z negation |
| Derived | Q=0 + symmetric X/Z negation | — | mathematically unique proper solution given face-up assumption |
| **Bench truth** | **Q=2, flip=false** | **all axes + yaw correct** | face-down bottom mount: no inversion needed |

---

## 7. Lessons

- **Unexercised mechanisms rot silently**: every flown board had `Q=0`, making the
  missing gyro rotation invisible until the first non-trivial target. Bench-check
  axes on ANY new board before flight, regardless of how "standard" it looks.
- **Fingerprint your firmware**: the census line's `CCR=` value (400 kHz build shows
  `CCR=8023`) instantly distinguishes fresh flash from stale. Keep such markers.
- **Don't copy empirical patches across chip families** — the ICM underside block is
  chip-specific compensation, not generic geometry.
- **Concurrent source editing invalidates bench data**: know which binary produced
  which observation.

---

## 8. Planned — GCS Orientation Calculator

A Calibration Window tab that walks a porter through the three moves above using live
telemetry and prints the resulting `.inc` lines. GCS-only; no FC writes; user applies
the values in source themselves (consistent with baked-in policy). Solver: candidate
transforms form a small finite group; three observed signed channel-mappings select the
correction uniquely from any starting configuration.

*Not yet implemented.*
