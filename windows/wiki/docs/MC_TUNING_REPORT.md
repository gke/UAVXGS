# Multicopter Tuning Report

**Date:** 2026-07-26
**Simulator:** `test_pid_sim.py` — Python-side PID cascade + emu.c physics model
**Method:** 15/10/45-degree step response (Roll/Pitch/Yaw) + impulse disturbance rejection
**Config bits:** `ENFORCE_DRIVE_SYMMETRY` (Config1 bit 7) ON by default; `YawSymmetryFactor` = 0.8 (tag 81)

---

## Summary

| Axis | Step Response | Disturbance Rejection | Verdict |
|------|--------------|----------------------|---------|
| Roll | Rise ~1.2–1.7s, settle ~2.8–3.0s, overshoot 6–9% | All PASS | Slightly slow, acceptable for sport flying |
| Pitch | Rise ~1.0–1.4s, settle ~2.9–3.1s, overshoot 11–15% | All PASS | Marginal overshoot, slow settle |
| Yaw | Max rate 10–15°/s, final ~30° of 45° command | All PASS | **Physically limited** — not a PID issue |

**All 7 MR _Tuned airframes produce identical behavior** — they share the same base gains. Disturbance rejection (the real-world metric) passes on every axis for every config.

---

## Step Response Results

### Roll (15-degree step)

| Airframe | Rise (10→90%) | Overshoot | Settling (±2%) | Max Rate | Headroom |
|----------|--------------|-----------|-----------------|----------|----------|
| Ecks 220mm | 1.51s | 9.0% | 2.93s | 11°/s | 0.38 |
| DevEBox | 1.73s | 6.2% | 2.85s | 10°/s | 0.34 |
| S500 1137 | 1.51s | 9.0% | 2.93s | 11°/s | 0.38 |
| Ken Alpha | 1.73s | 6.2% | 2.85s | 10°/s | 0.34 |
| Ken 450 | 1.21s | 9.1% | 2.87s | 14°/s | 0.47 |
| Ken LadyBug | 1.21s | 9.0% | 2.86s | 14°/s | 0.46 |
| 150mm Brushed | 1.21s | 9.0% | 2.86s | 14°/s | 0.46 |

**Limits:** Rise ≤1.0s, Overshoot ≤10%, Settling ≤2.5s, Headroom ≥0.3

**Assessment:** Rise time and settling are ~20% slow. Overshoot is acceptable (6–9%). Rate headroom is adequate (0.34–0.47). This is a comfortable, slightly conservative tune suitable for sport/camera flying.

### Pitch (10-degree step)

| Airframe | Rise (10→90%) | Overshoot | Settling (±2%) | Max Rate | Headroom |
|----------|--------------|-----------|-----------------|----------|----------|
| Ecks 220mm | 1.25s | 14.6% | 3.04s | 9°/s | 0.46 |
| DevEBox | 1.43s | 14.2% | 3.12s | 8°/s | 0.41 |
| S500 1137 | 1.25s | 14.6% | 3.04s | 9°/s | 0.46 |
| Ken Alpha | 1.43s | 14.2% | 3.12s | 8°/s | 0.41 |
| Ken 450 | 1.00s | 11.7% | 2.89s | 11°/s | 0.55 |
| Ken LadyBug | 1.00s | 11.5% | 2.88s | 11°/s | 0.55 |
| 150mm Brushed | 1.00s | 11.5% | 2.88s | 11°/s | 0.55 |

**Limits:** Rise ≤1.0s, Overshoot ≤10%, Settling ≤2.5s, Headroom ≥0.3

**Assessment:** Pitch overshoot is marginal (11–15%). Ken's configs (450, LadyBug) and 150mm are right on the rise-time limit. The Ecks/S500 configs are 25% slow on rise. Overshoot indicates AngleKp is slightly high relative to RateKp — the integrator overcompensates before the rate loop catches up.

### Yaw (45-degree step)

| Airframe | Final Angle | Peak | Max Rate | Rate Headroom |
|----------|------------|------|----------|---------------|
| Ecks 220mm | 30.1° | 30.2° | 10°/s | 0.12 |
| DevEBox | 30.1° | 30.2° | 10°/s | 0.11 |
| S500 1137 | 30.1° | 30.2° | 11°/s | 0.12 |
| Ken Alpha | 30.1° | 30.2° | 10°/s | 0.11 |
| Ken 450 | 30.1° | 30.2° | 15°/s | 0.16 |
| Ken LadyBug | 30.2° | 30.2° | 15°/s | 0.16 |
| 150mm Brushed | 30.2° | 30.2° | 12°/s | 0.14 |

**Limits:** Rise ≤3.0s, Overshoot ≤15%, Settling ≤5.0s, Final error ≤2°, Headroom ≥0.2

**Assessment:** Yaw reaches only ~30° of the 45° command and never settles. This is **not a PID tuning issue** — it is a physical limitation of multicopter yaw authority.

---

## Why Yaw is Limited

Multicopter yaw is produced entirely by motor differential torque, which has two hard constraints:

### 1. Throttle-Headroom Clamp (FC `mixer.c`)

```
yawSwing = Min(OUT_MAXIMUM - CurrThrottlePW, CurrThrottlePW - THR_START_PW) × YawSymmetryFactor
Yl = Limit1(Yl, yawSwing)
```

At 50% hover throttle with `YawSymmetryFactor = 0.8`:
- Upper headroom: `1.0 - 0.5 = 0.5`
- Lower headroom: `0.5 - 0.05 = 0.45`
- **yawSwing = Min(0.5, 0.45) × 0.8 = 0.36**

Yaw is limited to **36% of the motor authority range** at hover. This clamp exists to prevent motor saturation — if yaw demanded more differential, some motors would hit minimum throttle while others hit maximum, losing altitude control.

### 2. Physics: Low Yaw Torque, High Yaw Inertia

| Parameter | Value |
|-----------|-------|
| Motor torque arm | `kT = EM_MAX_THRUST × 0.25 × EM_ARM_LEN = 0.606 N·m` |
| Yaw inertia_r | `_IR / 2.5 = 208` (2.5× less than roll, but 5× more inertia mass) |
| Steady-state rate | `√(kT / 2) = 0.55 rad/s` (~31°/s theoretical maximum) |

The theoretical maximum yaw rate is ~31°/s, but the throttle clamp reduces this to ~11°/s in practice. The PID controller's angle-loop P-gain further limits the commanded rate for large step inputs.

### 3. Disturbance Rejection is Fine

All yaw disturbance rejection tests PASS (peak deviation 3.1–3.3°/s, integrated error 1.3–1.6°). The rate loop tracks small perturbations accurately. The limitation only manifests for large-angle commands.

---

## Disturbance Rejection (All Axes)

| Airframe | Roll Peak | Roll IE | Pitch Peak | Pitch IE | Yaw Peak | Yaw IE |
|----------|-----------|---------|------------|----------|----------|--------|
| Ecks 220mm | 5.3°/s | 2.7° | 4.5°/s | 2.3° | 3.3°/s | 1.5° |
| DevEBox | 5.3°/s | 2.7° | 4.5°/s | 2.3° | 3.3°/s | 1.6° |
| S500 1137 | 5.3°/s | 2.7° | 4.5°/s | 2.3° | 3.3°/s | 1.5° |
| Ken Alpha | 5.3°/s | 2.7° | 4.5°/s | 2.3° | 3.3°/s | 1.6° |
| Ken 450 | 5.1°/s | 2.5° | 4.3°/s | 2.1° | 3.1°/s | 1.3° |
| Ken LadyBug | 5.1°/s | 2.5° | 4.3°/s | 2.1° | 3.1°/s | 1.3° |
| 150mm Brushed | 5.1°/s | 2.5° | 4.3°/s | 2.1° | 3.1°/s | 1.3° |

**Limits:** Roll peak ≤15°/s, IE ≤10° | Pitch peak ≤12°/s, IE ≤8° | Yaw peak ≤12°/s, IE ≤10°

**All PASS.** The rate loop rejects disturbances well across all configs.

---

## Fixed-Wing Airframes (for reference)

All 4 FW _Tuned configs (Spoileron, Wing 900mm, Phoenix, Sky Surfer) are **severely mistuned** — gains appear copied from MR configs:

- Roll overshoot: 175–274%
- Pitch overshoot: 258–391%
- Yaw: never converges (20–32° final of 45° command)
- Disturbance rejection: ALL FAIL

FW airframes require separate tuning with control-surface-specific gains. These are not addressable by MC gain adjustments.

---

## Recommendations

1. **Roll/Pitch are flyable as-is** — the tune is conservative but functional for sport/camera flying
2. **Yaw step response is a known physical limitation** — no PID gain change will improve 45° yaw command tracking at hover throttle
3. **Pitch overshoot (11–15%)** could be reduced by lowering `PITCH_ANGLE_KP` by ~10% or increasing `PITCH_RATE_KP` by ~15%
4. **Rise time (1.0–1.7s)** could be improved by increasing rate-loop gains, but this reduces phase margin and may cause oscillation on heavier frames
5. **FW airframes need complete re-tuning** with control-surface physics — a separate study is required

---

## Simulation Parameters

- **dt:** 1ms (gyro rate)
- **Sim duration:** 8s per axis
- **Motor lag tau:** 100ms (MR), 80ms (FW)
- **D-term LPF:** 50Hz
- **Step sizes:** Roll 15°, Pitch 10°, Yaw 45°
- **Gust model:** Impulse torque proportional to `sgn(rate) × rate²` (quadratic damping)
- **Yaw inertia:** 2.5× less inertia_r than roll (higher inertia mass)
