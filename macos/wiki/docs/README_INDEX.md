# UAVXGS — wiki/docs Report Index

Central registry for the simulation / tuning report lineage. Every report tracks its
version, date, scope, and the simulator mechanics it reflects. When the simulation
mechanics change, update the affected report(s) here and bump their version.

**Rendering:** `./md2pdf.sh <file>` (pandoc → PDF, DejaVu Sans).

---

## Simulation & Calibration Series

| Report | Version | Date | Sim mechanics covered | Status |
|--------|---------|------|----------------------|--------|
| `PID_Simulation_Construction_Report.md` | **v2.1** | 2026-08-04 | Full sim construction: FW 3-axis coupled aero (quadratic damping, servo lag, mixing, stability/coupling), MR physics, AH/Nav outer loops, criteria, free-flight, known gaps. **v2.1:** `is_roll_structurally_limited` gate | **Current** — updated on every sim-mechanics change |
| `FW_Control_Authority_Calibration.md` | **v2.1** | 2026-08-04 | 70%-servo-deflection → steady-state-rate authority anchor; `PHYS_*` `.af` storage; `RATE_KP = 0.7/MAX_*_RATE`; FC mixer paths per AF type; final tuned RATE_KP table | **Current** — anchor table updated when `.af` gains change |
| `Cascade_Integrity_Checks.md` | v1 | 2026-08-08 | Attitude cascade safety cross-checks: outer IntLim ≤ max commanded rate (cascade windup), QP P-path demand ≤ max rate (loop separation), IntLim ≈ 20–30% of max rate as a *ceiling*; GCS issues caveats (not clamps), sim test validates; caught the legacy IntLimit=10 migration bug | **Current** — caveats in `parameter_window.compute_defaults`, checks in `test_pid_sim.check_cascade_integrity` |

## Tuning & Critique Series

| Report | Date | Scope | Status |
|--------|------|-------|--------|
| `tuning_study.md` | 2026-08-04 | v2.1 full pass: AH/Nav + attitude tuning across 12 FW + 5 MR generic frames; outer-loop bounds raise; AH_CRITERIA_FW 6s/8s; old-vs-revised tables; **all 12 FW + 5 MR PASS** | **Current** |
| `critique_report.md` | 2026-08-06 | Critique of `original/` frames — Phoenix FW retuned to PASS (v2.1); 9 MR frames legacy-baseline documented; **v3.1**: regenerated `user/_Tuned` fleet (mislabeled FW frames fixed, `get_fw_descriptor` `_Tuned`-suffix fix); **v4.0 (08-08)**: retired `user/_Tuned` → `backup/_retired_tuned/` (fleet-wide `IntLim=10` cascade-windup bomb, see `Cascade_Integrity_Checks.md`); FC ParamTable defaults canonical, mirrored into GCS `PARAM_DEFAULTS` | **Current** |

## GCS / Other Docs

Ecks_Airframe_Tuning.md, MC_TUNING_REPORT.md,
GCS_Guide.md, GCS_Conversion.md, What_Is_Different_GCS.md.

| Doc | Date | Scope | Status |
|-----|------|-------|--------|
| `Force_GPS_Init_Removal.md` | 2026-08-19 | Force GPS Init removed (blocked main loop ~9.4 s → IWDG reset → USB loss); `miscInitialiseGPS` slot 9 reserved; GPS config bit default changed to ON in FC ParamTable + GCS defaults. **Supp.1 (08-19):** `fc_build.py` default board fixed → `FLYINGRCF4WINGMINI` (was wrong-target `UAVXF4V3`, USART1 GPS never fed); GPS packet decode + Kalman moved out of USART1 ISR into `UpdateInertial` (ISR = byte FSM + flag only). **Supp.2 (08-20):** GPS pass-through fully removed (FC + GCS, selector slot retained as unused); **root cause of dead USART RX found** — same-port RX pad never set to `GPIO_Mode_AF` in `InitSerialPort` (bulk analog init left it disconnected; USB targets unaffected); UAVXF4V3 GPS path (USART2/PA2-3, iNav UBX init) verified, UBX-only parser | **Implemented** — builds clean (FC `fc_build.py` OK, GCS `py_compile` OK) |

## Design Series

| Doc | Version | Date | Scope | Status |
|-----|---------|------|-------|--------|
| `ParamClass_Bounds_Design.md` | v0.1 | 2026-08-05 | Class-based per-param bounds: FC tier-1 hard clamp, `.af [LIMITS]` tier-2 tuning range, GCS derived `PARAM_LIMITS` tier-3 fallback; full 128-param class mapping; migration plan | **Draft** — review before implementation |

## Soaring / Wind / ExcessLift Series

| Doc | Date | Scope | Status |
|-----|------|-------|--------|
| `Soaring_Research_ArduPilot_vs_iNav.md` | 2026-08-06 | ArduPilot full auto-soar vs iNav motor-stop model; P0–P3 adoption plan for UAVX ArmQ | **Research** |
| `ExcessLift_And_Wind_Implementation_Report.md` | 2026-08-06 | Two-source climb detector (`control.c`), Premerlani wind-triangle estimator (`wind/wind.c`), `WPAltFail` escape ladder (`auto.c`); rationale + emulator test plan | **Implementation** |

---

## Versioning Rules

1. **Sim mechanics change** (physics, mixing, criteria, authority anchor) → bump + update
   `PID_Simulation_Construction_Report.md` (and `FW_Control_Authority_Calibration.md`
   if authority is involved).
2. **Tuned gains change** → update `FW_Control_Authority_Calibration.md` RATE_KP table +
   `tuning_study.md` result tables.
3. Always keep the index table in sync so any report is resolvable from here.
