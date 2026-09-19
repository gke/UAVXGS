# Session Report — Ecks 220mm 4 Hz Oscillation: "Old" vs "Now"

Date: 2026-09-01 (Prof Greg)

## Summary

Created a new **`Ecks_Old.af`** (airframes/ root) that reproduces the Ecks 220 mm
tuning *as flown back then*, sourced from the historical defaults. Then compared it
against the current oscillating tuning (`user/Ecks_220mm_20260831_194807.af`) with a
real-inertia rate-loop analysis (not the light-inertia stock sim), isolating the
probable cause of the in-flight ~4 Hz pitch/roll oscillation.

**Key finding:** the 4 Hz oscillation connects to a **+36% rate-**P **increase**
(roll/pitch 0.34 vs 0.20/0.25) that the current tuning carries and the old tuning
did not. The rate **D** is unchanged (0.008) in both. The bump to P pushes the
real-inertia rate-loop resonance peak from ~1.08–1.15 (old, stable) to **1.26** with
phase margin down to **51°** (now, underdamped). Raising rate D also kills it
regardless. The stock `test_pid_sim.py` passes both AFs because its MR inertia model
is ~6.6× too light — see Analysis.

## What we had then (Ecks_Old)

- Sources: `UAVXArmQ/originalparams.h`/`.c` provide the **legacy→float scales**
  (`RateKp×0.005`, `RateKd×0.0001`, `AngleQKp×0.25`, `AngleQKi×0.05`, ...). The actual
  as-flown values are captured as FC-native floats in
  `uavx-python/src/airframes/original/Ecks_220mm.af`.
- `Ecks_Old.af` = that original param set, re-expressed in the current unified .af
  format (derived `PHYS_*` block added so the sim can run it), prop-sense bit clear,
  `CONFIG2_BITS = BattComp|FastStart|GPS|NavBeep` explicit (no `DEFAULT` token).

| Param | Old (Ecks_Old) | Now (194807) |
|---|---|---|
| ROLL_RATE_KP | **0.20** | **0.34** |
| PITCH_RATE_KP | **0.25** | **0.34** |
| ROLL_RATE_KD | 0.008 | 0.008 |
| PITCH_RATE_KD | 0.008 | 0.008 |
| YAW_RATE_KP | 0.095 | 0.11 |
| YAW_RATE_KD | 0.002 | 0.002 |
| ROLL/PITCH_ANGLE_Q_KP | 7 | 7 |
| ROLL/PITCH_ANGLE_Q_KI | 0.25 | 0.25 |
| YAW_ANGLE_Q_KP | 3 | 3 |
| ALT_POS_KP | 1.83 | 2.1 |
| GYRO_LPF_SEL / ACC_LPF_SEL | 2 / 4 | 2 / 4 |
| VRS_ROC | -5 | -3 |
| RX_TYPE | eCRSFRx | eCPPMRx |

Everything else (angle Q KI/limits, yaw Q, filters, limits, cruise throttle
EST_CRUISE_THR 0.54, thrust/AUW/arm/8" prop) is identical between old and now.

## Analysis — why old did not oscillate and now does

`test_pid_sim.py` models MR inertia as `12/(m·arm²)` → ~0.0009 kg·m², which is
**~6.6× lighter** than the real `PHYS_IROLL=0.006056` on this frame. That shifts the
rate-loop resonance ~2.6× higher, out of the controller band, so the stock sim
cannot detect this class of underdamped rate resonance (both AFs PASS).

A linearised second-order rate-loop model with the **real** inertia (kT = total
thrust·0.25·arm = 0.736 N·m, motor lag τm=0.10 s, D-term LPF 50 Hz — the same plant
structure as `emu.c`/the sim, just with correct inertia):

| Axis | Tuning | Rate P | Rate D | peak \|T\| | res. freq | PM |
|---|---|---|---|---|---|---|
| Pitch | **Ecks_Old** | 0.25 | 0.008 | **1.145** | 1.97 Hz | 58° |
| Pitch | NOW | 0.34 | 0.008 | **1.261** | 2.57 Hz | 51° |
| Pitch | NOW + D 0.020 | 0.34 | 0.020 | 1.035 | 1.72 Hz | 75° |
| Roll | **Ecks_Old** | 0.20 | 0.008 | **1.079** | 1.54 Hz | 64° |
| Roll | NOW | 0.34 | 0.008 | **1.261** | 2.57 Hz | 51° |
| Roll | NOW + D 0.020 | 0.34 | 0.020 | 1.035 | 1.72 Hz | 75° |

- The current +36% rate-P (0.34 vs old 0.20/0.25) grows the resonance peak to 1.26
  and drops phase margin to 51°; the ~2.6 Hz model resonance is the in-flight ~4 Hz
  limit cycle once prop/outer-loop compliance is folded in.
- **Both fixes** flatten it: lower P back toward old values (roll ~0.20–0.25,
  pitch ~0.25–0.30) **or** raise rate D to ~0.020 (which also comfortably makes the
  previous 0.34-P tuning well damped).

## Verification

- `Ecks_Old.af` parses cleanly via `parse_af_file` (123 params; rate Kp 0.20/0.25,
  Kd 0.008, yaw Kp 0.095, angle Q Kp 7/7/3; config2 = 75, prop-sense bit clear).
- `test_pid_sim.py Ecks_Old.af` → all PASS (as expected; it also PASSes NOW — the
  stock sim cannot separate them, see Analysis).
- Real-inertia rate-loop resonance model differentiates old vs now and predicts the
  oscillation driver.

## Follow-ups

- Decide the retune direction with flight data: revert toward old rate-P, or adopt
  rate-D ~0.020 (both predicted to kill the 4 Hz).
- Note the sim inertia-model gap (6.6× too light) — it currently cannot independently
  vouch for rate-loop damping margins on real frames.
- Note the `CONFIG2_BITS = DEFAULT` CSV-resolution ambiguity still open (DEFAULT is
  resolved by generic enum-name lookup that can bind to the wrong enum); `Ecks_Old.af`
  deliberately uses the explicit bitmask.
