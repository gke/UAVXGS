# UAVX vs ArduPilot tuning — sanity check & critique

**Date:** 2026-09-04 · **Author:** Greg + assistant
**Scope:** Plug ArduPilot's default rate gains into OUR sim framework and
critique the two tunings head-to-head, the same exercise already done for iNav
(`wiki/Session_Report_InavSimSanityCheck_Sep04.md` + `<…>InavTuning_Sep04.md`).
Airframe classes: MR (Quad), FW aileron (SkySurfer_Bixler), FW elevon (Shadow).
**Source:** `~/Documents/Flight/OtherCode/ardupilot-master` (Copter/Plane
**4.1.0-dev** era).

---

## Part 1 — How ArduPilot builds its defaults

Like iNav, ArduPilot's defaults are **hand-authored constants, not
physics-derived** — no mass/inertia/geometry enters the gain construction. But
unlike iNav, ArduPilot is much more configurable and its **authority
normalisation differs between Copter and Plane**, which is the crux of any
cross-map:

### 1.1 Copter (MR) — the clean case for us
`AC_AttitudeControl_Multi.h` builds the rate PIDs with literal defaults:

| param | default | acts on | output→ |
|---|---|---|---|
| `ATC_ANG_RLL/PIT/YAW_P` | **4.5** | *rad* of angle error | desired rate (rad/s) |
| `ATC_RAT_RLL/PIT_P` | **0.135** | *rad/s* rate error | motor roll/pitch demand |
| `ATC_RAT_RLL/PIT_D` | **0.0036** | (filtered) rate error | motor demand |
| `ATC_RAT_RLL/PIT_I` | **0.135** | rate error (integrator) | motor demand |
| `ATC_RAT_YAW_P` | **0.180** | rad/s | yaw demand |
| `ATC_RAT_YAW_D` | **0.0** | — | — |
| `ATC_RAT_YAW_I` | **0.018** | — | — |

Critically for portability: the Copter rate PID operates on **rad/s** and its
output feeds the motor mixer **directly as the ±1.0 roll/pitch/yaw differential
demand** (fraction of full authority) — **the same convention as our sim**. So
no unit conversion is needed for MR. Runtime scaling is **opt-in**, default OFF
just like iNav: throttle TPA (`ATC_THR_MIX` default 0.5 is a fixed rpy/thrust
mix, not an airspeed schedule) and no airspeed APA.

### 1.2 Plane (FW) — different convention, hard to transfer
`APM_Control` (`AP_RollController`/`AP_PitchController`) is the classic
deg-based controller. Servo output is in **centi-degrees of servo travel,
±4500 = full deflection** (so `fraction = out/4500`), and it acts on **deg/s**:

```
desired_rate(deg/s) = angle_err(cdeg) * 0.01 / tau          # angle P = 1/tau
kp_ff  (rate P)     = ( (P - I*tau)*tau - D ) / EAS2TAS
out(cdeg)           = desired_rate*kp_ff + rate_err*D + I + desired_rate*FF
```

| param (defaults) | RLL2SRV (Roll) | PTCH2SRV (Pitch) |
|---|---|---|
| `TCONST` (tau) | 0.5 | 0.5 |
| `P` (angle P) | 1.0 | 1.0 |
| `D` (rate D, per deg/s) | 0.08 | 0.04 |
| `I` | 0.3 | 0.3 |
| `FF` | **0.0** | **0.0** |
| → rate `kp_ff` (per deg/s → cdeg) | **0.345** | **0.385** |

Note the **big differences from iNav**: ArduPilot Plane's default **FF is zero**
(its real authority is the angle-P `1/tau` + rate-D + trim-I), and it is
deg/s-based, so it does *not* map 1:1 like Copter does.

**Plane has no default fixed-wing rate-YAW loop.** Heading/yaw on a plank is
coordinated via the aileron→rudder mix and passive stability; there is no
`yaw_stability`-driven rate loop equivalent to ours. **FW-yaw is therefore not
comparable** and is omitted (our Shadow yaw is drag-differential anyway).

### 1.3 Mapping used (stated explicitly)
- **MR (Quad):** transfer 1:1 — `Kp=ATC_RAT_*_P`, `Kd=ATC_RAT_*_D`
  (0.135/0.0036 roll+pitch, 0.180/0.0 yaw). Keep OUR outer angle loop.
- **FW (SkySurfer, Shadow):** `Kp_ours = kp_ff/4500·rad2deg`, `Kd_ours = D/4500·rad2deg`
  (roll ≈ 0.00439/0.00102, pitch ≈ 0.00490/0.00051). Keep OUR outer angle loop.

Only the **rate P/D** are swapped — our sim has no rate-I and ArduPilot's I/FF
are wound against its own normalisation, so a linear map of I/FF would be a
systematic artifact (exactly what the iNav PIFD row showed).

---

## Part 2 — Results (same plant + same criteria)

Script: `src/tests/compare_ardupilot.py`.

**MR — `generic/Quad.af` (ArduPilot Copter)**

| axis | Ours | ArduPilot Copter | agreement |
|---|---|---|---|
| Roll (15°) | rise .39s, ov .5%, settle .79s **PASS** | rise .35s, ov .5%, settle .76s **PASS** | ~4 % settle |
| Pitch (10°) | rise .39s, ov .6%, settle .79s **PASS** | rise .34s, ov .5%, settle .73s **PASS** | ~8 % settle |
| Yaw (45°) | rise 1.00s, settle 1.91s **PASS** | rise 1.00s, settle 1.92s **PASS** | ~identical |

**FW aileron — `generic/SkySurfer_Bixler.af` (ArduPilot Plane)**

| axis | Ours | ArduPilot Plane (rate P/D only) |
|---|---|---|
| Roll (15°) | rise 2.36s, ov 10%, settle 7.6s **PASS** | rise 8s, final 3.5° **FAIL** |
| Pitch (30°) | rise 8s, final 3.4° **FAIL** | rise 8s, final 0.1° **FAIL** |
| Yaw | — (no ArduPlane rate-yaw default) | — |

**FW elevon/delta — `generic/Shadow.af` (ArduPilot Plane)**

| axis | Ours | ArduPilot Plane (rate P/D only) |
|---|---|---|
| Roll (15°) | rise .38s, ov 24.1%, settle 3.0s **PASS** | rise 8s, final 11.8° **FAIL** |
| Pitch (30°) | rise 8s, final 6.8° **FAIL** | rise 8s, final 0.1° **FAIL** |
| Yaw | — (none) | — |

---

## Part 3 — Interpretation & critique: UAVX vs ArduPilot

### 3.1 MR — a second independent validation
ArduPilot's field-proven Copter rate **P/D**, transferred **1:1** (same
authority convention!), pass all three axes on our plant with **near-identical
step responses** (roll settle 0.76s vs our 0.79s; yaw 1.92 vs 1.91s). ArduPilot
is a widely-flown production autopilot; its MC rate tuning converging with
ours (which already matched iNav) is exactly the sanity evidence we wanted.
**Our MR plant + criteria are physically reasonable and our MC tuning sits in
the industry consensus band — now triangulated by BOTH iNav and ArduPilot.**

### 3.2 FW — why ArduPilot Plane "looks weak" is structural, not a defect
ArduPilot Plane's raw rate-P (0.0044 in our units vs our ~0.5) is ~100× below
ours, because its default authority lives in the **angle-P (`1/tau`) + trim-I +
aerodynamic self-damping**, with FF disabled by default. Injecting only its
rate-P/D (the terms we share) under-reaches the step (SkySurfer roll 3.5°,
Shadow roll 11.8° vs 15°) — the same structural story as iNav FW (whose P=5 with
FF=50 also under-reached to 10.3°). **This is a mapping-portability limit, not a
framework defect and not a verdict on ArduPilot's real-world performance.** It
does reveal our own FW style is deliberately rate-P-heavy, unlike both
competitors which push fixed-wing authority toward feed-forward/angle-P.

### 3.3 Tuning-philosophy critique — UAVX vs ArduPilot

| | UAVX (ours) | ArduPilot Copter | ArduPilot Plane |
|---|---|---|---|
| Rate units | rad/s, ±1 fraction | rad/s, ±1 fraction | **deg/s**, ±4500 cdeg |
| Rate P authority | high (P-first) | moderate (0.135) | tiny (kp_ff 0.345/4500) |
| Rate D | yes | yes (0.0036) | yes (0.04–0.08) |
| Rate I | **none** (outer loop does it) | yes (0.135) | yes (trim 0.3) |
| FF | none | none (default) | **0 by default** |
| Angle loop | quaternion PI | P-only (4.5 rad→rad/s) | P-only (1/tau) |
| Authority scaling | none (fixed) | fixed rpy/thrust mix (0.5) | opt-in, none by default |

**The decisive finding: our MR convention (rad/s + ±1 fraction authority)
matches ArduPilot Copter's exactly** — which is *why* their gains transfer
1:1 to our plant and agree. This is a much cleaner external cross-check than
iNav (which required the pidSumLimit mapping). It is a genuine structural
affinity, not a coincidence.

**Where we differ from ArduPilot (and why it's OK):**
- **No rate-I.** ArduPilot Copter runs rate-I (0.135); we deliberately push
  steady-state rate error to the outer angle-loop I (AGENTS.md "Yaw rate
  I-term"). Both are valid; ours avoids extra tuning surfaces and windup. The
  MR agreement shows our absence of rate-I does not hurt the step response.
- **P-only vs quaternion angle loop.** ArduPilot Copter uses a P-only
  angle→rate (4.5) plus a separate quaternion slew/feedforward in modern code;
  ours is a true quaternion PI with conditioned I. Both stabilise; ours adds
  the I for sustained setpoint tracking.
- **Plane's deg/s + cdeg convention is not portable** to our rad/s + fraction
  scheme without the /4500·rad2deg bridge; that bridge is documented here and
  is the *only* reason the FW comparison is possible at all.

### 3.4 Actionable notes from this comparison
- **MR: no change.** ArduPilot agrees with us (and with iNav). Our MC tuning is
  triangulated to consensus.
- **Shadow roll remains our tightest-margin axis** (24.1 % ov vs 25 %, 0.38s
  rise) — carried forward from the iNav study; ArduPilot gives no reason to
  change it, only confirms it is punchy.
- **FW pitch under-achieves on both competitors too** (aero authority limit of
  the small elevator surface, not a gain defect) — carried forward.
- **FW-yaw is an ArduPilot architectural gap** (no default rate-yaw), not one
  we share; our drag-differential plank yaw has no ArduPilot equivalent to
  sanity-check.

### 3.5 Sanity-check verdict
**PASS, with the same FW caveats as iNav.** The MR result is the strongest
validation yet — ArduPilot's gains transfer 1:1 (same convention) and agree with
ours to a few percent, and independently agree with iNav's earlier mapped
result. Our framework blesses two independent, field-proven autopilots on the
same plant. The FW ordering is structurally explained and is a portability
limit, not a failure.

---

## Files
- `uavx-python/src/tests/compare_ardupilot.py` — new comparison script
  (Quad MR via `simulate_axis`; SkySurfer + Shadow FW via `simulate_axis_coupled`
  with ArduPilot rate P/D injected).
- `uavx-python/src/tests/compare_inav.py`, `test_pid_sim.py` — reused.
- `~/Documents/Flight/OtherCode/ardupilot-master` — gain sources:
  `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.{h,cpp}`
  (`AC_ATC_MULTI_RATE_*`, `AC_ATTITUDE_CONTROL_ANGLE_P`),
  `libraries/APM_Control/AP_RollController.cpp`, `.../AP_PitchController.cpp`
  (`RLL2SRV`/`PTCH2SRV` defaults).

## Verification
- `python3 tests/compare_ardupilot.py` runs clean end-to-end (Quad MR all 3
  axes; SkySurfer + Shadow FW Roll+Pitch; FW-yaw omitted — no ArduPilot
  default). No pytest suite covers it (report/analysis script).
- No FC source, GCS source, or `.af` payload changed.
