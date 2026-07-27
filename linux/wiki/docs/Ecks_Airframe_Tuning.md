# Ecks Quad Airframe Tuning Guide

## Overview

Four airframe configurations are provided, covering two different motor/prop combinations
at two tuning levels each:

| File | Motor | Prop | AUW | Hover | Tuning |
|------|-------|------|-----|-------|--------|
| `Ecks_800g_Moderate.af` | 2208 1000kv | 8x6 two-blade | 800g | ~55% | Stable cruise |
| `Ecks_800g_Sport.af` | 2208 1000kv | 8x6 two-blade | 800g | ~55% | Aggressive sport |
| `Ecks_1kg_Moderate.af` | Racerstar BR2205 2300kv | Cyclone 5x3x3 | 1kg | ~60% | Stable cruise |
| `Ecks_1kg_Sport.af` | Racerstar BR2205 2300kv | Cyclone 5x3x3 | 1kg | ~60% | Aggressive sport |

**GCS default**: `Ecks_800g_Moderate.af` (actively flight-tested)
**FC emulation default**: `EM_MASS=0.8`, `EM_THR_CRUISE_STICK=0.55` (2208 + 8x6)

---

## Airframe 1: 2208 1000kv + 8x6 two-blade (800g, ~55% hover)

Lower-powered build: large slow-turning props on a lightweight frame.

### Motor data

- Motor: 2208 1000kv (~35g, 22×8mm stator)
- Prop: 8x6 two-blade (8", low pitch)
- Battery: 3S 2200mAh LiPo
- Per-motor thrust at full: ~350g (installed, accounting for voltage sag across 4 motors)
- Total max thrust: ~1400g (14.3 N)
- TWR: 1.75:1
- Hover throttle: ~55%

### Moderate tuning (`Ecks_800g_Moderate.af`)

Designed for stable, predictable flight. Suitable for aerial photography,
waypoint navigation, and general cruising.

Key characteristics:
- Roll rate P 0.15 / D 0.012 — moderate P with high D for big-prop damping
- Pitch rate P 0.18 / D 0.012
- Max roll rate 360°/s, max pitch rate 180°/s — controlled rotation
- Max angle 30° — conservative tilt envelope
- Nav max angle 25° — stable turns in auto mode
- Gyro LPF 2, ACC LPF 4 — standard filtering
- Alt hold: POS Kp 0.45 / Ki 0.003 — responsive but not twitchy
- Yaw angle Kp 6 — stronger yaw authority for larger props

### Sport tuning (`Ecks_800g_Sport.af`)

Designed for aggressive sport flying. Higher rates, lower damping, wider angle limits.

Key differences from moderate:
- Roll rate P 0.24 (+60%) / D 0.008 (-33%) — snappier, less damping
- Pitch rate P 0.28 (+56%) / D 0.008 (-33%)
- Max roll rate 600°/s (+67%), max pitch rate 360°/s (+100%)
- Max angle 45° (+50%)
- Nav max angle 30°
- Gyro LPF 1, ACC LPF 3 — reduced filtering for less delay
- Horizon 0.5 — stronger self-leveling feel
- FW stick scale 0.5 — more stick sensitivity
- Yaw angle Kp 8 — crisp yaw response

---

## Airframe 2: Racerstar BR2205 2300kv + Cyclone 5x3x3 (1kg, ~60% hover)

Higher-powered build: small aggressive props on a heavier airframe (larger battery/payload).

### Motor data

- Motor: Racerstar BR2205 2300kv (28g, 22×5mm stator)
- Prop: DAL Cyclone T5040C/T5045C tri-blade (5", 3-blade)
- Battery: 3S 2200-3000mAh LiPo
- Per-motor thrust at full: ~400g (installed)
- Total max thrust: ~1600g (15.7 N)
- TWR: 1.6:1
- Hover throttle: ~60%

### Moderate tuning (`Ecks_1kg_Moderate.af`)

Designed for stable flight at higher AUW with high-kv motors.

Key characteristics:
- Roll rate P 0.20 / D 0.006 — higher P to move the mass, moderate D for small props
- Pitch rate P 0.22 / D 0.008
- Max roll rate 360°/s, max pitch rate 180°/s
- Max angle 30°
- Nav max angle 25°
- Gyro LPF 2, ACC LPF 4 — standard filtering
- Alt hold: POS Kp 0.55 / Ki 0.003 — stronger for heavier airframe
- Yaw angle Kp 4 — small props limit yaw authority
- ALT_THROTTLE_COMP_LIMIT 0.22

### Sport tuning (`Ecks_1kg_Sport.af`)

Designed for sport flying with the high-kv motors. Aggressive rates and gains.

Key differences from moderate:
- Roll rate P 0.30 (+50%) / D 0.004 (-33%)
- Pitch rate P 0.32 (+45%) / D 0.005 (-38%)
- Max roll rate 600°/s, max pitch rate 360°/s
- Max angle 45°
- Nav max angle 30°
- Gyro LPF 1, ACC LPF 3
- Horizon 0.5
- FW stick scale 0.5

---

## Tuning Philosophy

### Moderate (cruise)

- Rate P gains are set conservatively — the system should not oscillate
  under any flight condition
- Rate D gains are elevated to provide active damping for clean step
  response without overshoot
- Max rates are limited — the aircraft will feel controlled rather than
  twitchy
- Gyro filtering is at standard levels (LPF 2, ACC LPF 4) to reject
  motor noise while maintaining adequate phase margin
- Altitude hold gains are tuned for minimal bounce on engagement and
  smooth altitude tracking

### Hot (sport)

- Rate P gains are increased 45-60% above moderate — the aircraft
  responds immediately to stick inputs
- Rate D gains are reduced — less damping means less phase lag and a
  crisper feel, at the cost of potential overshoot on hard stops
- Max rates are increased 67-100% — full-rate flips and rolls are
  possible
- Gyro filtering is reduced one step (LPF 1, ACC LPF 3) — lower group
  delay for better feel, at the cost of more noise reaching the motors
- Horizon level is increased — the aircraft self-levels more aggressively
  when sticks are released

### Airframe-specific tuning notes

- The **800g 8x6 build** (2208 1000kv) has big props with high rotational
  inertia. It needs more D gain to damp the prop gyroscopic effects. The
  larger props also provide better yaw authority, allowing higher yaw
  angle Kp. Alt hold compensation limit is higher (0.25) to cover the
  wider throttle range from the slow-turning motors.

- The **1kg Racerstar build** (2205 2300kv) is heavier with small,
  low-inertia props. It needs higher P gains to move the mass, but less
  D gain because the props don't fight direction changes. Yaw authority
  is limited by the small props, so yaw rate Kp is kept moderate.

- EST_CRUISE_THR (tag 19) must match the actual hover throttle —
  setting this correctly is critical for AltHold performance. The FC uses
  it as the feedforward baseline; `TrackCruiseThrottle()` will converge
  the runtime value during stable hover.

---

## Flight Test Recommendations

1. **Start with `Ecks_800g_Moderate.af`** — this is the default for a reason.
   It's the configuration being actively flight-tested and is the safest
   starting point.

2. **If the aircraft feels sluggish**: Move to `Ecks_800g_Sport.af` for the
   800g build, or `Ecks_1kg_Sport.af` for the 1kg build. The increased
   rate P gains and reduced filtering will improve responsiveness.

3. **If the aircraft oscillates (hot tuning)**: Reduce rate P gains by 20%
   or increase rate D gains by 30%. Oscillation on hard stops means too
   much P or not enough D. High-frequency oscillation (prop noise) means
   too little filtering — increase GYRO_LPF_SEL or ACC_LPF_SEL.

4. **AltHold tuning**: The moderate alt hold settings (Kp 0.45-0.55,
   Ki 0.003) should provide smooth altitude tracking. If you see
   altitude bounce, reduce ALT_POS_KP. If you see droop, increase
   ALT_POS_KI.

5. **Transitioning between airframes**: When switching .af files, the FC
   will receive all 128 parameter values. Always disarm before loading
   a new airframe. After loading, re-check EST_CRUISE_THR in the GCS
   and verify the hover throttle stick position matches.

---

## Known Issues

- **AH overshoot on descent** — being tuned via `ALT_HOLD_THR_COMP_DECAY`
  and `ALT_THROTTLE_COMP_LIMIT`. The moderate profiles use a decay of
  0.03/ps and comp limits of 0.22-0.25.

- **Nav gain tuning** — `NAV_POS_KP`, `NAV_POS_KI`, `NAV_MAX_VELOCITY`,
  and `NAV_MAX_BANK_ANGLE` may need adjustment for your specific flight
  envelope. The moderate profiles use conservative values (Kp 0.22-0.25,
  Ki 0.012, max angle 25°).

---

## Parameter Reference

Significant parameters that differ between profiles:

| Param | 0800 Mod | 0800 Sport | 1kg Mod | 1kg Sport |
|-------|----------|------------|---------|-----------|
| EST_CRUISE_THR | 0.55 | 0.55 | 0.60 | 0.60 |
| ROLL_RATE_KP | 0.15 | 0.24 | 0.20 | 0.30 |
| ROLL_RATE_KD | 0.012 | 0.008 | 0.006 | 0.004 |
| PITCH_RATE_KP | 0.18 | 0.28 | 0.22 | 0.32 |
| PITCH_RATE_KD | 0.012 | 0.008 | 0.008 | 0.005 |
| YAW_RATE_KP | 0.18 | 0.26 | 0.15 | 0.25 |
| YAW_RATE_KD | 0.008 | 0.005 | 0.002 | 0.002 |
| MAX_ROLL_RATE | 4.19 | 4.19 | 4.19 | 4.19 |
| MAX_PITCH_RATE | 2.09 | 2.09 | 2.09 | 2.09 |
| MAX_YAW_RATE | 1.57 | 3.14 | 1.57 | 3.14 |
| MAX_ROLL_ANGLE | 0.524 | 0.785 | 0.524 | 0.785 |
| MAX_PITCH_ANGLE | 0.524 | 0.785 | 0.524 | 0.785 |
| NAV_POS_KP | 0.22 | 0.28 | 0.25 | 0.30 |
| NAV_POS_KI | 0.012 | 0.015 | 0.012 | 0.015 |
| NAV_VEL_KP | 0.25 | 0.35 | 0.30 | 0.40 |
| NAV_MAX_ANGLE | 0.436 | 0.524 | 0.436 | 0.524 |
| ALT_POS_KP | 0.45 | 0.50 | 0.55 | 0.65 |
| ALT_POS_KI | 0.003 | 0.004 | 0.003 | 0.004 |
| ALT_POS_INT_LIMIT | 0.25 | 0.30 | 0.35 | 0.40 |
| ALT_THROTTLE_COMP_LIMIT | 0.25 | 0.28 | 0.22 | 0.24 |
| GYRO_LPF_SEL | 2 | 1 | 2 | 1 |
| ACC_LPF_SEL | 4 | 3 | 4 | 3 |
| YAW_LPF_HZ | 50 | 60 | 50 | 60 |
| YAW_ANGLE_KP | 6 | 8 | 4 | 5 |
| YAW_ANGLE_INT_LIMIT | 0.025 | 0.03 | 0.015 | 0.02 |
| HORIZON | 0.3 | 0.5 | 0.3 | 0.5 |
| FW_STICK_SCALE | 0.4 | 0.5 | 0.4 | 0.5 |
| AH_THROTTLE_MOVING_TRIGGER | 0.2 | 0.15 | 0.2 | 0.15 |
