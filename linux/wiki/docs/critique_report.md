# UAVX PID Critique Report — Original Airframes

**Date:** 2026-08-08  
**Version:** 4.0  
**Scope:** Critique-only pass on `original/` airframes (legacy gains, not retuned) + v3.1 regeneration/validation of `user/_Tuned` + **v4 retirement & default-table sync**
**Criteria:** FW_CRITERIA (attitude), AH_CRITERIA_FW (altitude hold), NAV_CRITERIA_FW (navigation)

> **v3.0 update:** Re-run of the full critique. The simulation uses the **angle KP as
> regular proportional gains** (`rate_desired = angle_error × KP`, exactly as in legacy
> `DoAngleControl`). This was already the case — the critique sim does **not** apply any
> quaternion `2·Qa` factor. (The quaternion cascade form only exists in the separate
> `test_quat_sim.py`.) Results below supersede v2.1.

> **v3.1 update (2026-08-06):** The legacy `user/*_Tuned.af` fleet was deleted and
> **regenerated** as faithful copies of the `original/` airframes (correct `AF_TYPE`, no
> mislabeling). Fixed the simulator's `_Tuned`-suffix handling in `get_fw_descriptor` so
> a `user/X_Tuned.af` resolves its FW descriptor by stripping the suffix (previously
> `user/Phoenix_Tuned.af` etc. lost the aileron-less / rudderless structural gate and were
> wrongly scored — 17–20 FAILs). The MR frames carry their inherited legacy gains with a
> single defensible change (yaw-rate KP normalized toward default: `YAW_RATE_KP = 0.25`).
> Results below reconcile the regenerated fleet.

> **v4.0 update (2026-08-08):** The `user/*_Tuned.af` fleet is **retired to
> `airframes/backup/_retired_tuned/`** — every tuned frame shipped the legacy raw
> `ROLL/PITCH/YAW_ANGLE_Q_INT_LIMIT = 10` (10 rad/s) value in place of the intended
> ≈0.0026/0.0087 rad/s conversion, so all of them fail `check_cascade_integrity` as a
> cascade-windup bomb (integrator above max commanded rate can never discharge). See
> `Cascade_Integrity_Checks.md`. Retune restarts from a clean baseline (`original/`).
> FC `params.c` ParamTable defaults for the Q angle loop are declared canonical and
> mirrored verbatim into GCS `PARAM_DEFAULTS` (tags 2/4/7/9/23/24/96/97/98): the GCS
> shipped 7.0/0.25 vs FC 1.75/0.0125 — divergence was 7–20×, now synced. Non-tune
> parameter limits (FC `lo/hi` ceilings + GCS `PARAM_LIMITS`) deliberately untouched
> (see `Airframe_Tuning_Normalization.md` §5).

---

## v4.0 — Retire `user/_Tuned` fleet + default-table sync (2026-08-08)

### The failure that stopped the retune

Running `check_cascade_integrity` (the cross-check from `Cascade_Integrity_Checks.md`)
against `original/` (11 base frames incl. `user/Arado_555.af`, `user/Horten.af`) red-flags
**every MR frame** with the same I-limit windup bomb:

```
original/Ecks_220mm.af : ! Yaw I-limit 10.00 > max commanded 3.67 — windup
original/Ken_450_1165.af: ! I-limit 28.00 > max commanded 7.33 — windup
...
generic/Dragon.af       : ! I-limit 10.00 > max commanded ... — windup
```

The value is not per-airframe-tuned; it is the legacy `uint8` `10` written straight into
the unified-float field (3800× over-budget). Every `_Tuned.af` inherited it, so no basis
for per-frame retil was trustworthy. Hence the **decision to abandon the whole tuned
fleet** and re-enter from the `original/` surface.

### Decision and actions

1. **Retired** all 13 `user/*_Tuned.af` → `airframes/backup/_retired_tuned/` (the 2
   unmatched `user/Arado_555.af` / `user/Horten.af` are kept in place as the retursource).
2. **FC params.c defaults are the canonical Q-gain baseline** (AGENTS.md: FC is source of
   truth). The GCS `PARAM_DEFAULTS` for the Q angle loop now mirrors FC exactly:

   | tag | param | FC default | GCS default (after) | was (before) |
   |-----|-------|-----------|--------------------|--------------|
   | 2/7 | Roll/PitchAngleQKp | 1.75 | 1.75 | 7.0 |
   | 4/9 | Roll/PitchAngleQIntLimit | 0.01 | 0.01 | 0.0026 |
   | 23/24 | Roll/PitchAngleQKi | 0.0125 | 0.0125 | 0.25 |
   | 96 | YawAngleQKp | 0.75 | 0.75 | 3.0 |
   | 97 | YawAngleQKi | 0.0125 | 0.0125 | 1.0 |
   | 98 | YawAngleQIntLimit | 0.03 | 0.03 | 0.0087 |

   Note these are *schema* defaults only (`UseDefaultParameters` fresh-provisioning);
   they do not override the `.af` files' live flight values (unit-spinbox path).
   Non-tune limits unchanged (FC `PARAMLimits` / GCS `PARAM_LIMITS` stay where the
   check `test_param_limits.py` validates them).
3. **Next:** regenerate the tuned fleet from a cascade-corrected baseline and re-run the
   RIG until 0 hard fails.

## Simulation convention (v3.0)

The attitude loop in `test_pid_sim.py` is the legacy angle-cascade:

```
run_angle_loop:   pi.Error = clamp(stick*Max + NavCorr) - angle
                  rate_desired = pi.Error * ANGLE_KP + ITerm       # regular KP, rad/s per rad
run_rate_pd:      out = clamp((rate_desired - rate) * RATE_KP + DTerm, -1, 1)
```

- `ANGLE_KP` is used **directly** (rate command per rad of angle error) — no `2·Qa` half-angle
  scaling, no quaternion-to-Euler conversion. A 10° error with `ANGLE_KP=7` commands ~1.22 rad/s.
- This matches legacy `UAVXArm32F4/src/control.c` `DoAngleControl`:
  `P->PTerm = P->Error * P->Kp` → `R.Desired = PTerm + ITerm` → `ControlRate` (`R.Kp` in series).
- For small angles the quaternion controller's `2·Qa·QGain` ≈ `angle_error·QGain`, so treating
  angle KP as regular gains is the correct interpretation for tuning; it does **not** under-size
  authority.

---

## Fixed Wing — Original (1 frame)

### original/Phoenix.af
**Result (v3.0):** **PASS**

| Test | Result | Details |
|------|--------|---------|
| Pitch @ 30° | PASS | — |
| Yaw @ 15° | PASS | — |
| Pitch disturbance rejection | PASS | — |
| Yaw disturbance rejection | PASS | — |
| Alt Hold 5m | PASS | — |
| Nav 10m | PASS | — |

**Note:** aileron-less RudderElevatorAF — roll step is a structural gate (non-scored).

---

## Multicopter — Original (8 frames)

### original/Ecks_220mm.af — **FAIL**
- Roll @ 15°: PASS · Pitch @ 10°: PASS · Yaw @ 45°: **ISSUES** (OS 17.5%, settle 8s)
- Roll disturbance: **ISSUES** (peak 17.8°/s) · Pitch disturbance: PASS · Yaw: **ISSUES** (peak 17.2°/s)
- AH 5m: **ISSUES** (rise 2.03s) · Nav 10m: PASS

### original/Ken_450_1165.af — **FAIL**
- Roll @ 15°: PASS · Pitch @ 10°: PASS · Yaw @ 45°: **ISSUES** (settle 8s)
- Roll disturbance: **ISSUES** (peak 16.2°/s) · Pitch disturbance: PASS · Yaw: **ISSUES** (peak 13.4°/s)
- AH 5m: **ISSUES** (rise 2.33s) · Nav 10m: PASS

### original/Ken_Alpha_Test.af — **FAIL**
- Roll @ 15°: PASS · Pitch @ 10°: PASS · Yaw @ 45°: **ISSUES** (settle 8s)
- Roll disturbance: **ISSUES** (peak 19.3°/s) · Pitch disturbance: PASS · Yaw: **ISSUES** (peak 16.0°/s)
- AH 5m: **ISSUES** (rise 3.34s) · Nav 10m: PASS

### original/Ken_LadyBug.af — **FAIL**
- Roll/Pitch @ step: **ISSUES** (sim diverges — Roll final=NaN, peak=inf)
- Yaw @ 45°: **ISSUES**
- Roll/Yaw disturbance: **ISSUES** · Pitch disturbance: PASS
- AH 5m: **ISSUES** · Nav 10m: PASS
- **Note:** sim physics diverges on this frame (NaN/inf on roll step) — gains need review.

### original/Ming_DevEBox.af — **FAIL**
- Roll @ 15°: PASS · Pitch @ 10°: PASS · Yaw @ 45°: **ISSUES** (settle 8s)
- Roll disturbance: **ISSUES** (peak 19.3°/s) · Pitch disturbance: PASS · Yaw: **ISSUES** (peak 16.0°/s)
- AH 5m: **ISSUES** (rise 2.09s) · Nav 10m: PASS

### original/S500_1137.af — **FAIL**
- Roll @ 15°: PASS · Pitch @ 10°: PASS · Yaw @ 45°: **ISSUES** (settle 8s, OS 10.2%)
- Roll disturbance: **ISSUES** (peak 17.8°/s) · Pitch disturbance: PASS · Yaw: **ISSUES**
- AH 5m: **ISSUES** · Nav 10m: PASS

### original/WLToys_LadyBug.af — **FAIL**
- Roll/Pitch @ step: **ISSUES** (same NaN/inf sim divergence as Ken_LadyBug — same gains/physicals family)
- Yaw @ 45°: **ISSUES**
- Roll/Yaw disturbance: **ISSUES** · Pitch disturbance: PASS
- AH 5m: **ISSUES** · Nav 10m: PASS

### original/Rok_Quad.af — **FAIL**
- Roll @ 15°: PASS · Pitch @ 10°: PASS · Yaw @ 45°: **ISSUES** (settle 8s)
- Roll disturbance: **ISSUES** (peak 16.0°/s) · Pitch disturbance: PASS · Yaw: **ISSUES** (peak 16°/s)
- AH 5m: **ISSUES** (rise 3.79s) · Nav 10m: PASS
- **Note:** physicals updated (AUW=1344g, arm=260mm, prop=9x4, 5000mAh 3S, thrust=610g); gains not retuned.

---

## Summary Table

| Airframe | Category | Attitude | AH | Nav | Notes |
|----------|----------|----------|----|-----|-------|
| Phoenix | FW | PASS | PASS | PASS | Roll structurally non-scored |
| Ecks_220mm | MR | Partial | ISSUES | PASS | Yaw settles 8s; roll/yaw gusts high |
| Ken_450_1165 | MR | Partial | ISSUES | PASS | Yaw settles 8s |
| Ken_Alpha_Test | MR | Partial | ISSUES | PASS | Yaw settles 8s |
| Ken_LadyBug | MR | FAIL | ISSUES | PASS | Roll sim diverges (NaN) |
| Ming_DevEBox | MR | Partial | ISSUES | PASS | Yaw settles 8s |
| S500_1137 | MR | Partial | ISSUES | PASS | Yaw settles 8s |
| WLToys_LadyBug | MR | FAIL | ISSUES | PASS | Roll sim diverges (NaN) |
| Rok_Quad | MR | Partial | ISSUES | PASS | Yaw settles 8s |

**Common failure pattern:** all MR frames pass attitude roll/pitch steps and Nav, but fail on
**yaw settle time** (8s vs 5s limit), **roll/yaw gust peak deviation** (16–19°/s vs 12–15°/s
limits), and **AH rise time** (2.0–3.8s vs 2s limit). These point at under-gained yaw-rate and
altitude-ROC loops in the legacy gains, not the angle KP scaling (which is regular and correct).

---

## Recommendations

1. **Do not deploy original/ airframes with legacy gains** — they fail yaw/AH criteria.
2. **If any original airframe is to be used**, run the full tuning pipeline (attitude + AH + Nav)
   as done for generic/ frames — yaw KP/KI and alt-ROC KP are the primary targets.
3. **Ken_LadyBug / WLToys_LadyBug** — sim physics diverges on roll step (NaN/inf); gains and
   physicals need review before they can be tuned.
4. **Rok_Quad** — physicals updated; needs full retune for the new mass/prop configuration.
5. **Phoenix** — passes with the v2.1 retune; unchanged in v3.0.

---

## Verification

Run any original airframe critique:
```bash
python3 -m test_pid_sim original/Rok_Quad.af
python3 -m test_pid_sim original/Phoenix.af
```

Expected: PASS for Phoenix; FAIL/ISSUES on yaw, roll/yaw gust, and AH rise for the MR frames
(Nav now passes).

---

## v3.1 — Regenerated `user/*_Tuned.af` fleet (2026-08-06)

The active tree previously carried mislabeled `user/_Tuned` frames (e.g. `Radian_Tuned.af`,
`Arado_555_Tuned.af` were `QUAD_X` MR configs under FW names — these would select the wrong
mixer/motor map and crash on spin-up). They were **deleted**, then **regenerated** as
faithful copies of the `original/` airframes with their true `AF_TYPE`, so each is a valid
read-write tuning baseline (Radian_Tuned → `RUDDER_ELEVATOR`, Arado_555_Tuned → `ELEVON`).

### Simulator fix
`test_pid_sim.py::get_fw_descriptor` and `is_yaw_structurally_limited` now strip a
`_Tuned.af` suffix before the basename match. Without this, a regenerated FW tuned copy
failed to resolve its aileron-less / rudderless descriptor, so the structural gates
(`is_roll_structurally_limited` / `is_yaw_structurally_limited`) did not fire and the frame
was scored with strict roll/`overhead limits` (max 10% overshoot) it could never meet,
yielding 17–20 FALSE FAILs for `Phoenix_Tuned`, `Radian_Tuned`, `Arado_555_Tuned`.

`user/Rok_Quad_Tuned.af` was additionally mapped to `QUAD_X` (matching `original/Rok_Quad.af`);
the earlier `QUAD` mapping triggered `AF_CATEGORY` KeyError → no results.

### Validation (default gains, `python3 tests/test_pid_sim.py <file>`, no slider)

| File | FAIL / PASS | Notes |
|------|------------|-------|
| user/Phoenix_Tuned.af | 0 / 45 | **PASS** — retuned baseline |
| user/Radian_Tuned.af | 0 / 45 | **PASS** — retuned baseline |
| user/Arado_555_Tuned.af | 1 / 44 | 1 artifact: free-flight roll settle (see below) |
| user/Ecks_220mm_Tuned.af | 3 / 42 | MW yaw / roll-gust / AH (inherited legacy) |
| user/Ken_450_1165_Tuned.af | 3 / 42 | MW yaw / roll-gust / AH |
| user/Ken_Alpha_Test_Tuned.af | 3 / 42 | MW yaw / roll-gust / AH |
| user/Ming_DevEBox_Tuned.af | 3 / 42 | MW yaw / roll-gust / AH |
| user/S500_1137_Tuned.af | 3 / 42 | MW yaw / roll-gust / AH |
| user/Rok_Quad_Tuned.af | 4 / 41 | MW yaw / roll-gust / AH |
| user/Ken_LadyBug_Tuned.af | 7 / 36 | brushed-micro limits (see below) |
| user/WLToys_LadyBug_Tuned.af | 8 / 35 | brushed-micro limits (see below) |

**Notes**

1. **FW frames PASS after the descriptor fix** — the earlier 17–20 FAILs were classifier
   artefacts, not real gain problems. Their tuning carries over from the original baseline.
2. **Arado_555_Tuned single FAIL** is a free-flight (no PID) roll *settle* report: turbulence
   keeps the already-small peak (~2.8°) above 5% of its own peak within the 15s window, so
   the "-settle 5%" metric never triggers. It is a measurement/sampling artefact, not a gain
   deficiency — `PHYS_ROLL_DAMP` / `PHYS_DIHEDRAL` variants barely move it (7.8–7.1s across
   an 8× damping change). Arado's real geometry (45° sweep, inboard +7° dihedral, outboard
   −5° anhedral → effective dihedral small, sweep-induced adverse yaw large) is consistent
   with the stored `PHYS_DIHEDRAL=0.05` / `PHYS_ADVERSE_YAW=0.05`.
3. **MR micro frames (Ken_LadyBug / WLToys_LadyBug)** retain the brushed-motor divide:
   overshoot on roll/pitch step, yaw/roll-gust peak deviation, and slow AH. Raising rate/angle
   gains did not clear them (brushed-actuator + low inertia limit), so they are documented as
   physical — their original frames show the same pattern (see multicopter section above).
4. Yaw-rate for the MR `_Tuned` copies was normalized to `YAW_RATE_KP = 0.25` (default mid;
   was ~0.067), a single defensible change. `YAW_ANGLE_Q_KP` was left at the source value
   (2.25–3).
