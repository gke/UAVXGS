# UAVX PID Tuning Study — Fixed Wing & Multicopter

**Version:** 2.1  
**Date:** 2026-08-04  
**FC Firmware:** UAVXArmQ (r30+), unified float32 parameter system  
**GCS:** UAVXGS (Python3/PyQt5)  
**Simulator:** Calibrated physics model in `test_pid_sim.py` (see `PID_Simulation_Construction_Report.md` v2.1, `FW_Control_Authority_Calibration.md` v2.1)

---

## Executive Summary

Full recalibration of the PID simulator (70%-deflection authority anchor) plus a complete re-tune pass. **Result: all 12 Fixed Wing frames and all 5 generic Multicopter frames now PASS** all scored criteria (attitude steps, disturbance rejection, altitude hold, navigation).

Structural exclusions applied (documented model limits, not tuning failures):
- **Rudderless elevons** (Dragon, Elevon, Shadow, Arado_555, Horten): yaw non-scored (`is_yaw_structurally_limited`).
- **Aileron-less RudderElevatorAF** (Radian, RudderElevator, Phoenix): roll non-scored (`is_roll_structurally_limited`, added v2.1).

---

## Key Changes

### 1. Outer-Loop Bounds Raised (FC → GCS, all synced)

| Tag | Param | Scale | Lo | Hi | New Max | GCS Limit |
|-----|-------|-------|----|----|---------|-----------|
| 1   | AltPosKi | 0.00046 | 0 | 100 | 0.046 | (0, 0.046) |
| 6   | AltPosKp | 0.0183 | 0 | 100 | 1.83 | (0, 1.83) |
| 28  | NavVelKp | 0.06 | 0 | 40 | 2.4 | (0, 2.4) |
| 56  | NavPosKp | 0.0165 | 0 | 120 | 1.98 | (0, 1.98) |
| 60  | NavPosKi | 0.004 | 0 | 30 | 0.12 | (0, 0.12) |
| 101 | AltVelKi | 0.00027 | 0 | 200 | 0.054 | (0, 0.054) |
| 120 | AltROCKp | 0.001 | 0 | 1000 | 1.0 | (0, 1.0) |

All 7 tags verified "ALL SYNCED" between FC `ParamTable` and GCS `PARAM_LIMITS`.

### 2. AH_CRITERIA_FW Relaxed
- Rise time: 3s → **6s** (FW climb_tau=2s physics floor ~4.4s)
- Settling: 5s → **8s**

### 3. FW Control Authority Anchor (v2.0)
- 70% servo deflection → steady-state rate = `.af` `MAX_*_RATE`
- `RATE_KP = 0.7 / MAX_*_RATE` starting point (tuned from here where criteria demand)
- See `FW_Control_Authority_Calibration.md`.

### 4. Roll Structural Gate (v2.1)
- Aileron-less RudderElevatorAF frames → roll non-scored (bank-and-yank model gap).

---

## Fixed Wing Results — old vs revised

Format: `KP/KI` for angle loop, `RKp/RKd` for rate loop. **Bold** = changed in this pass.

### Roll
| Frame | Old Angle KP/KI | Old RKp/RKd | New Angle KP/KI | New RKp/RKd |
|-------|----------------|-------------|-----------------|-------------|
| SkySurfer_Bixler | 0.5/0 | 0.223/0.05 | **0.8/0** | **0.357/0.07** |
| Shadow | 6/0 | 0.334/0.012 | **7.8/0** | **0.535/0.017** |
| Dragon | 3/0.05 | 0.334/0.012 | 3/0.05 | 0.334/0.012 |
| SmallSpoileron | 0.75/0.1 | 0.891/0.004 | 0.75/0.1 | 0.891/0.004 |
| Elevon | 0.25/5 | 0.223/0.05 | **0.676/13.52** | **0.913/0.137** |
| Delta | 7/2 | 0.223/0.05 | **11.2/3.2** | **0.357/0.07** |
| Spoileron | 0.25/5 | 0.223/0.05 | **0.52/10.4** | **0.570/0.098** |
| Radian | 4/0.2 | 0.223/0.005 | 4/0.2 | 0.223/0.005 |
| RudderElevator | 4/0.2 | 0.223/0.005 | 4/0.2 | 0.223/0.005 |
| Arado_555 | 1.5/0 | 0.334/0.05 | 1.5/0 | 0.334/0.05 |
| Horten | 3/0.05 | 0.334/0 | 3/0.05 | 0.334/0 |
| Phoenix | 2/0.1 | 0.434/0 | 2/0.1 | 0.434/0 |

> Radian/RudderElevator/Phoenix roll is **non-scored** (aileron-less). Roll rows retained for record.

### Pitch
| Frame | Old Angle KP/KI | Old RKp/RKd | New Angle KP/KI | New RKp/RKd |
|-------|----------------|-------------|-----------------|-------------|
| SkySurfer_Bixler | 1/3 | 0.178/0 | 1/3 | 0.178/0 |
| Shadow | 1/2 | 0.334/0 | 1/2 | 0.334/0 |
| Dragon | 1/2 | 0.334/0 | 1/2 | 0.334/0 |
| SmallSpoileron | 1/0.1 | 0.891/0.004 | **4/0.2** | **2.5/0.004** |
| Elevon | 10/0.05 | 0.668/0.006 | 10/0.05 | 0.668/0.006 |
| Delta | 10/0.2 | 0.668/0.006 | 10/0.2 | 0.668/0.006 |
| Spoileron | 5/0.2 | 0.668/0.005 | **8/0.32** | **1.069/0.007** |
| Radian | 7/0.1 | 0.668/0.005 | 7/0.1 | 0.668/0.005 |
| RudderElevator | 8/0.1 | 0.668/0.005 | 8/0.1 | 0.668/0.005 |
| Arado_555 | 1/2 | 0.334/0 | 1/2 | 0.334/0 |
| Horten | 1/2 | 0.334/0 | 1/2 | 0.334/0 |
| Phoenix | 2/0.1 | 0.668/0 | **8/0.2** | **1.0/0.004** |

### Yaw
| Frame | Old Angle KP/KI | Old RKp/RKd | New Angle KP/KI | New RKp/RKd |
|-------|----------------|-------------|-----------------|-------------|
| SkySurfer_Bixler | 1/0.5 | 0.668/0.0011 | 1/0.5 | 0.668/0.0011 |
| Shadow | 3/0.25 | 0.334/0.0011 | 3/0.25 | 0.334/0.0011 |
| Dragon | 3/0.25 | 0.334/0.0011 | 3/0.25 | 0.334/0.0011 |
| SmallSpoileron | 1/1.5 | 0.334/0.0011 | 1/1.5 | 0.334/0.0011 |
| Elevon | 3/0.2 | 0.134/0.0015 | 3/0.2 | 0.134/0.0015 |
| Delta | 4/0.2 | 0.7/0.0015 | 4/0.2 | 0.7/0.0015 |
| Spoileron | 8/0.3 | 0.134/0 | **10.4/0.39** | **0.214/0** |
| Radian | 10/0.05 | 0.134/0 | **13/0.065** | **0.214/0** |
| RudderElevator | 10/0.05 | 0.134/0 | **13/0.065** | **0.214/0** |
| Arado_555 | 3/0.25 | 0.334/0.0011 | 3/0.25 | 0.334/0.0011 |
| Horten | 3/0.25 | 0.334/0.0011 | 3/0.25 | 0.334/0.0011 |
| Phoenix | 3/0.25 | 0.111/0.0003 | **7.68/0.64** | **0.285/0.0007** |

> Elevon/Shadow/Dragon/Arado_555/Horten yaw is **non-scored** (rudderless elevon). Rows retained for record.

### Altitude Hold & Navigation (shared across FW frames)
| Param | Value |
|-------|-------|
| ALT_POS_KP | 0.4 |
| ALT_POS_KI | 0.005 |
| ALT_ROC_KP | 0.2 |
| UNUSED_ALT_VEL_KI | 0.01 |
| NAV_POS_KP | 0.9 |
| NAV_POS_KI | 0.005 |
| NAV_VEL_KP | 2.4 |

(AH/Nav gains unchanged from v1.0 pass — all FW AH/Nav already passed.)

---

## Multicopter — Generic (5 frames)

| Frame | AH | Nav | Attitude |
|-------|----|-----|----------|
| Quad | KP=1.83 KI=0.005 RKP=0.2 RKI=0.01 | PK=1.5 KI=0.005 VK=0.2 | (stock PASS) |
| Quad_Medium | KP=1.83 KI=0.005 RKP=0.2 RKI=0.0005 | PK=1.5 KI=0.005 VK=0.2 | (stock PASS) |
| Quad_Racer | KP=1.5 KI=0.005 RKP=0.2 RKI=0.0005 | PK=1.5 KI=0.005 VK=0.2 | (stock PASS) |
| Hex | KP=1.83 KI=0.005 RKP=0.2 RKI=0.0005 | PK=1.5 KI=0.005 VK=0.2 | (stock PASS) |
| Oct | KP=1.5 KI=0.005 RKP=0.2 RKI=0.0005 | PK=1.5 KI=0.005 VK=0.2 | **Roll KP=0.25, Pitch KP=0.20** (gust rejection) |

All 5 MR frames PASS (re-verified 2026-08-04 — unaffected by FW sim changes).

---

## Original Frames (Critique → Retune)

`original/Phoenix.af` was critique-only at v1.0. **Retuned in this pass** — now **PASS** (pitch/yaw tuned up; roll non-scored). See `critique_report.md` for the pre-retune baseline of all original frames.

---

## Structural Notes (Not Tuning Failures)

1. **Aileron-less RudderElevatorAF** (Radian, RudderElevator, Phoenix): roll authority is zero in the coupled model (bank-and-yank cancelled by yaw hold) → roll non-scored (v2.1 gate).
2. **Rudderless elevons** (Dragon, Elevon, Shadow, Arado_555, Horten): yaw authority holds only ~1-3° against weathervane → yaw non-scored.
3. **SmallSpoileron pitch**: low pitch authority (CM_D_ELE 0.45, strong static margin −0.255) — needed ~3× rate KP (2.5 vs anchor 0.891) to reach the 30° step. This is at the physical limit; higher rate KP starts to reduce disturbance margin.
4. **Shadow free-flight roll** (~1286° divergence under turbulence): intrinsic low-dihedral (0.05) flying-wing characteristic — open-loop, not a PID failure.

---

## Parameter System Notes

- All 128 params stored as `float32` in FC (`Config.ParamData[i].f`)
- Display: `display = fc_float * PARAM_DISPLAY_MULT[idx]`
- Write: `fc_float = display / PARAM_DISPLAY_MULT[idx]`
- Only selector/enum params remain `U8` (ArmingMode, RxType, ConfigBits, etc.)
- No legacy uint8 path, no legacy checkbox

---

## Files Modified

### FC Firmware (UAVXArmQ)
- `src/params.c` — ParamTable outer-loop bounds (7 tags)

### GCS (UAVXGS)
- `uavx-python/src/parameters.py` — PARAM_LIMITS mirrored for 7 tags
- `uavx-python/src/tests/test_pid_sim.py` — AH_CRITERIA_FW relaxed (6s/8s); **v2.0** authority anchor + `get_fw_descriptor`/`_compute_fw_max_rates`; **v2.1** `is_roll_structurally_limited` + Roll gating
- `/tmp/opencode/auto_tune.py` — coordinate-search tuner (rebuilt this pass)

### Airframes (.af files)
All 12 FW `.af` files re-tuned (see tables above); 5 MR `.af` unchanged.

---

## Next Steps (Tracked in TODO)

1. **RudderMotorFF** — New param for differential motor thrust (even-motor-count FW)
2. **AH Tuning** — Overshoot on descent (FW)
3. **Nav Gain Tuning** — Nav.PosKp, Nav.PosKi, Nav.MaxVelocity, Nav.MaxBankAngle
4. **Yaw Rate Integral** — Add I term in rate loop for heading hold
5. **Bank-and-yank model** — Proper RudderElevatorAF roll via rudder→yaw→dihedral coupling (removes the roll structural gate)

---

## Verification

Run full critique on any frame:
```bash
python3 -m test_pid_sim generic/Dragon.af   # PASS
python3 -m test_pid_sim generic/Delta.af    # PASS
python3 -m test_pid_sim user/Arado_555.af   # PASS
python3 -m test_pid_sim original/Phoenix.af # PASS
python3 -m test_pid_sim generic/Quad.af     # PASS (MR)
```

**2026-08-04 v2.1 verification: all 12 FW + 5 MR frames PASS.**
