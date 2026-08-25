# PID Simulation Construction Report

**Version:** 2.1  
**Date:** 2026-08-04  
**Scope:** Document the construction, assumptions, and validation of `uavx-python/src/tests/test_pid_sim.py` — the calibrated PID simulator used for the FW/MR tuning studies. **Supersedes v1 `FW_SIMULATION_REPORT.md`** (2026-07-26, "Aerodynamic Model v1", qbar×S×C_ctrl physics without the 70%-deflection authority anchor). This is the authoritative construction reference.

> **v2.0 → v2.1 (2026-08-04):** Added `is_roll_structurally_limited()` — aileron-less `RudderElevatorAF` frames have zero roll authority in the coupled model (roll routed to rudder, bank-and-yank cancelled by the yaw-angle hold). Roll step/gust criteria are non-scored for these frames, mirroring the existing yaw gate for rudderless elevons. See §7.

---

## 1. Purpose

Simulate each airframe's closed-loop step response (attitude, disturbance rejection, altitude hold, navigation) so PID gains can be tuned against physically meaningful authority before flight. The sim models the **full control path**: PID loops → FC mixer (`DoServos`/`DoMotors`) → aerodynamic/motor physics.

---

## 2. Architecture

```
run_tests_for_af(af_filename)
  └─ get_params_for_af()      → params, AirframeType, FW model idx, is_fw
  ├─ FW (descriptor present): simulate_axis_coupled()  — 3-axis coupled aero
  ├─ FW/MR (legacy):          simulate_axis()           — single-axis
  ├─ Disturbance rejection:   simulate_rate_disturbance()
  ├─ Altitude hold:           simulate_alt_hold_fw() / simulate_alt_hold_mr()
  ├─ Navigation:              simulate_nav_fw() / simulate_nav_mr()
  └─ Free-flight turbulence:  simulate_freeflight() (FW only)
```

Active engine lives at **`uavx-python/src/tests/test_pid_sim.py`**. Copies under `windows/`, `linux/`, `macos/` are stale snapshots — always run the `uavx-python` copy.

---

## 3. Parameter/Physics Sources

- **Params:** parsed from the `.af` file (`airframes.airframes.parse_af_file`) → `{ParamIndex: raw_float}` → `load_af_params()` names.
- **FW physicals:** `.af` `PHYS_*` metadata merged over `FW_AIRFRAMES` defaults by `get_fw_descriptor()` (`PHYS_TO_DESCRIPTOR`). The `.af` is authoritative.
- **MR physicals:** `PHYS_AUW_G`, `PHYS_ARM_MM`, `PHYS_MOTOR_COUNT`, `PHYS_MOTOR_THRUST_G` → hover/max thrust, arm length, inertia per frame (fallback `EM_*` emu.c defaults).
- **Enums:** `protocol_enums.ParamIndex`, `AirframeType`, `AF_CATEGORY`, `FW_MODEL_IDX`, `AF_FILES`.

---

## 4. FW Physics (`simulate_axis_coupled`, `run_physics_fw`)

### Control torques
```
L = C_l_ail · δ_a      C_l_ail = qbar·S·b·CL_D_AIL
M = C_m_ele · δ_e      C_m_ele = qbar·S·c·CM_D_ELE
N = C_n_rud · δ_r      C_n_rud = qbar·S·b·CN_D_RUD
N_elevon = adverse_yaw · C_l_ail · δ_elevon   (ElevonAF drag-differential)
```

### Damping
```
quadratic:  C_d_axis · r·|r|      (PHYS_*_DAMP)
linear:     C_lin_axis · |r|      (*_damp_lin)
```

### Stability & coupling
- Pitch/yaw static stability: `± stab_coeff · qbar · S · ref · angle`
- Adverse yaw: `N_adverse = adverse_yaw · L_ctrl`
- Dihedral: `L_dihedral = dihedral_coeff · r_yaw`
- Servo lag: first-order `servo_tau` on all surface channels

### Rate ceiling
`_compute_fw_max_rates()` solves the quadratic damping balance at full deflection (≈1.195 × `MAX_*_RATE`). The sim clamps rates to this ceiling; **`MAX_*_RATE` is the 70%-deflection steady-state rate, not the clamp**.

### Mixing (FC `DoServos`/`DoMotors`)
Efforts Rl/Pl/Yl route per AF type — see §6 of `FW_Control_Authority_Calibration.md`. Elevon, RudderElevator, Delta/Aileron/Spoileron paths all modeled; feedforwards `FW_ROLL_PITCH_FF`, `FW_AILERON_RUDDER_MIX` applied.

---

## 5. MR Physics (`run_physics_mr`, `simulate_axis`)

- Torque from motor differential thrust: `τ = kT · effort`, `kT = max_thrust·0.25·arm`.
- Axis damping: Roll 0.015, Pitch 0.03, Yaw 0.05 (quadratic, sign-opposed).
- Motor lag: first-order `EM_MOTOR_TAU` (0.10 s).
- Inertia from arm/mass per frame (shape factor by motor count: 4→0.55, 6→0.40, 8→0.30).

---

## 6. Outer Loops

### Altitude Hold
- **MR** (`simulate_alt_hold_mr`): position PI → desired ROC → velocity PI → throttle comp. Fast tilt response.
- **FW** (`simulate_alt_hold_fw`): position PI (`ALT_POS_KP/KI`) → ROC setpoint → velocity PI (`ALT_ROC_KP`, `UNUSED_ALT_VEL_KI`) → climb with `climb_tau = 2.0 s` aero lag. Airspeed-bleed coupling.

### Navigation
- **MR** (`simulate_nav_mr`): cross-track error → `NAV_POS_KP/KI` → velocity → bank (`NAV_VEL_KP` → `g·tan(bank)`) — fast (~1 s).
- **FW** (`simulate_nav_fw`): cross-track → `NAV_POS_KP/KI` → bank-angle command → coordinated turn `heading_rate = g·tan(bank)/V` — slower (`bank_tau = 1.5 s`).

---

## 7. Criteria

| Test | Roll | Pitch | Yaw |
|------|------|-------|-----|
| MR attitude (`CRITERIA`) | rise≤1.0s, OS≤10%, settle≤2.5s, err≤1° | same | rise≤3s, OS≤15%, settle≤5s, err≤2° |
| FW attitude (`FW_CRITERIA`) | rise≤4s, OS≤25%, settle≤8s, err≤5° | rise≤5s, OS≤30%, settle≤12s, err≤15° | rise≤5s, OS≤25%, settle≤10s, err≤5° |
| Disturbance (`DIST_CRITERIA`) | peak≤15°/s, settle≤1.5s, IE≤10° | peak≤12°/s, IE≤8° | peak≤12°/s, IE≤10° |
| AH (MR/FW) | `AH_CRITERIA_MR` rise≤2s OS≤20% settle≤5s err≤1° | `AH_CRITERIA_FW` rise≤6s OS≤15% settle≤8s err≤2° (climb-tau floor ~4.4s) | — |
| Nav (MR/FW) | rise≤2s/3s, OS≤20%/15%, settle≤5s, err≤1°/2° | — | — |

**Structural exclusions:** rudderless elevons (`is_yaw_structurally_limited`) — yaw step/gust non-scored; aileron-less RudderElevatorAF frames (`is_roll_structurally_limited`) — roll step/gust non-scored.

---

## 8. Free-Flight (FW)

Open-loop 8s turbulence response with zero surface deflection — measures **intrinsic** stability (dihedral, static margin, damping) without PID. Reports roll/pitch/yaw peak excursions. A large roll divergence (e.g. Shadow 1286°) is a **structural** characteristic of low-dihedral flying wings, not a tuning failure.

---

## 9. Known Gaps / Caveats

1. **Rudder-only roll** (`is_roll_structurally_limited`, v2.1): `CL_D_AIL = 0` → zero aileron authority → roll ceiling 0. Bank-and-yank roll authority under-modeled (rudder produces yaw → dihedral couples to roll), so roll is **non-scored** for these frames rather than reported as an unreachable FAIL.
2. **Elevon yaw**: drag-differential authority is approximate (~1–3° hold). No differential-motor yaw (all FW frames single-motor; `RudderMotorFF` path exists in FC but no multi-motor FW frame in the set).
3. **Airspeed coupling**: AH/Nav FW use fixed cruise speed; no throttle-induced pitch coupling beyond `climb_tau`.
4. **DT/EM mismatch**: `windows/`, `linux/`, `macos/` sim copies are stale — the authoritative file is `uavx-python/src/tests/test_pid_sim.py`.

---

## 10. Files Modified (this calibration)

- `uavx-python/src/tests/test_pid_sim.py` — `get_fw_descriptor`, `load_af_physicals`, quadratic `_compute_fw_max_rates`, `PHYS_TO_DESCRIPTOR`, `C_d_roll` from `PHYS_ROLL_DAMP` (3 call sites), elevon yaw mixing, criteria relax, **v2.1** `is_roll_structurally_limited` + Roll gating in `run_tests_for_af`.
- 12 FW `.af` files — `PHYS_*` aero block + `RATE_KP = 0.7/MAX_*_RATE`.
- Reports: `wiki/FW_Control_Authority_Calibration.md` (this series), `wiki/tuning_study.md`.
