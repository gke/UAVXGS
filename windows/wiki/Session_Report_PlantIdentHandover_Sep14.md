# Session Report — In-Flight Plant Identification (identify.c) — HANDOVER

- **Date:** 2026-09-14
- **Author of this report:** Parallel session A (finalised the trace simplification)
- **Addressed to:** Parallel session B, which inherits the identification build
- **Scope:** What we propose, the true current state of the code (correcting stale
  assumptions), the exact plant model to fit, the handover design, and the build rules.

---

## 0. TL;DR for the inheriting session

Your "critic.c scaffold / injected step probe / critic trailer / trace trailer for
the estimates" assumptions are **out of date**. The on-board stimulus, settle gate,
pre-roll, on-board critic (`critic.c`/`critic.h`), and the 24 B critic trailer were
**all removed 2026-09-14** — the trace is now a **passive pilot-stimulus snapshot
recorder** and the GCS does all analysis. That is *compatible* with your proposal —
arguably the right base for it — but you must build against the current tree, not the
Aug-30 design you cited.

The task you inherit: **`identify.c`/`identify.h`, passive-observation plant-parameter
estimation, running in the 500 Hz control loop, with covariance-based confidence and
an excitation gate. No injection, no drives interference — ever.** See §4.

---

## 1. Current state of the tree (what session A finalised just now)

### 1.1 Trace = passive snapshot recorder (2026-09-14)

- **No on-board stimulus.** The FC never injects a rate step. Ch8/Aux2 (`Aux2RC`)
  **OFF→ON** edge while `State == eInFlight` opens a capture immediately; rows are
  written every sampling period while ch8 is held AND in-flight AND the ring isn't full;
  any of ch8-released / leaves-flight / ring-full finalises the header and moves to
  `eTraceHolding`; the next OFF→ON overwrites.
- **The pilot is the stimulus source** (their own stick inputs — any response or none
  is valid data).
- **On-board critic is gone.** `UAVXArmQ/src/critic.{c,h}` were **deleted**.
  `TraceStimulusSetpoint` calls were removed from `ControlRate`/`ControlRateYaw`
  (`control.c`), `#include "critic.h"` removed from `UAVX.h`. Nothing in the FC
  computes rise/overshoot/settle anymore.
- **No 24 B critic trailer.** The flash copy and dump stream carry **header + records
  only** (`TraceCommit` is a single `WriteBlockArmFlash(!{, ...})`). `TRACE_CRITIC_*`
  defines are gone from `trace.h`.
- **GCS still analyses.** `uavx-python/src/critic/` (shared `metrics.py`/`criteria.py`)
  survives; `trace_viewer.py` `rate_step_metrics()` re-derives rise/overshoot/settle
  from the raw `Desired`/`Rate` rows (worst-case `max(desired)` setpoint), with a
  tolerant read of a *historical* trailer for old dumps. `critic.metrics` is your
  validation metric spec for offline checks.
- **Hard rule (Greg), carried from trace:** drives/servos **must** continue to operate
  unaffected by any test. No servo lockup under any circumstances.
- Build state: all active targets built clean this session (SPEEDYBEEF405WING,
  MATEKF405TE, MATEKF411WING, FLYINGRCF4WINGMINI, SKYELECTRONICF405V3). The
  `FLYERF405WING`/`OMNIBUSF4V3` failures are **pre-existing** (missing
  `boards/targets/omnibusf4v3.inc` — nothing to do with trace).

### 1.2 Decimation, ring, header (what you inherit)

- Ring `BBQ[65536]` CCM (`TRACE_RING_SIZE` 64K / 32K on F411), records at
  `TRACE_DATA_BASE = 128`, 16 B each: `u32 tick | f32 Rate | f32 Desired | f32 Out`
  per axis, 3 records per sample tick. 500 Hz for `eTraceRate`/`eTraceIMU` (`periodMs
  2`), 50 Hz for Attitude/AltHold/Actuator (`periodMs 20`).
- v2 header: 32 B base + 21-float gain block (`TRACE_GAIN_*`) + pad = 128 B. Header
  also carries `rateGainCode` — the Ch10 `RateGainScale` pot fixed-point ×1000 at
  capture-open. **A dump is self-describing**: you know the exact gains + pot scale
  that qualified the capture.

## 2. The plant model (the exact structure to fit)

All in `src/emu.c` — this is the *instantiated* model we use in the sim and the emulator;
it is the structure your parameter estimator should predict against. **It is not a
linear first-order regressor** `Rate[k+1]=a·Rate[k]+b·Out[k]` over the full envelope —
it is:

### 2.1 MR plant (`emu.c:452-514`)

```
J_roll = 12/(Mass·ArmLen²);  J_pitch = J_roll/1.5;  J_yaw = J_roll/2.5
InertiaR[a] = 1/J_a            // "ram" = reciprocal inertia

motorInput = DesiredThrottle·TiltThrFFComp·BattThrFFComp + AltHoldThrComp   (clamped 0..1)
MotorLagState += (dT/(MotorTau+dT))·(motorInput − MotorLagState)            // 1st-order lag

Rate[a] -= ( MaxThrust·0.25·Out[a]·ArmLen·InertiaR[a]·MotorLagState
             + 2·Sign(Rate[a])·Rate[a]² ) · dT                 // roll/pitch
Rate[eYaw] -= ( MaxThrust·0.015·Out[eYaw]·ArmLen·InertiaR[eYaw]·MotorLagState
                + 2·Sign(Rate[eYaw])·Rate[eYaw]² ) · dT
```

Parameters you could estimate: authority scale, `MotorTau`, and the quadratic-damping
coefficient (currently the literal `2`), and the yaw drag coefficient. Note the
**`MotorLagState` gating**: torque ∝ actual thrust, so `Out` at zero throttle produces
no torque — your estimator must not assume a static `a·Rate[k] + b·Out[k]` map when
`motorInput` rides through zero. (In a real flight log `RawPW`/throttle is available.)

### 2.2 FW plant (`emu.c:488-519`)

```
Rate[a] -= ( Out[a]·FwCtrlEff[a]·(Airspeed/CruiseSpeed)² + 2·Sign(Rate[a])·Rate[a]² )·dT   // roll/pitch
Rate[eYaw] += ( Out[eYaw]·FwCtrlEff[eYaw]·(Airspeed/CruiseSpeed)² − 2·Sign(Rate[eYaw]−CoordTurnRate)·Rate[eYaw]² )·dT
```

`FwCtrlEff[axis]` is the angular acceleration per unit `Out` at cruise speed
(rad/s²), derived from the sim's validated aero model with `FW_AUTHORITY_SCALE=4`
baked in (`L_ctrl/I = qbar·S·b·C_l·ail_max / I_roll`, per model in `EmuModels[]`,
emu.c:180-310). qbar-scaled live via `(Airspeed/CruiseSpeed)²`. **2026-09-16 fix:**
the legacy `Out[a]·R.Max[a]·InertiaRollPitch` form delivered only ~0.006 rad/s²
per unit Out (terminal roll rate ~3°/s — the emulated Shadow could not bank, the
opening-WP-distance root cause, `Session_Report_EmuFwAuthority_Sep16.md`). The new
form matches the validated sim (Shadow roll ~130 rad/s² full-Out). No motor lag
(`MotorTau = 0`), no thrust coupling, hard `Max*Rate` clamp (physical model
maxima). FW yaw damping anchors at `CoordTurnRate` (coordinated turn) not zero.

### 2.3 Sign convention (critical — do not copy the old emulator bug)

`control.c` rate controllers emit `Out = −conditionOut((PTerm + DTerm)·RateGainScale)`
(`ControlRate` `control.c:478`, `ControlRateYaw` `control.c:490`, damping
`DoRateDampingControl` `control.c:509`). The MR plant then does `Rate −= (…torque…)·dT`.
Your prediction must mirror the **actual** sign chain you fit, and you should fit
**per-axis per-class** (the MR roll/pitch, MR yaw, FW roll/pitch, FW yaw forms differ).

## 3. What already exists that you must not rebuild / must not duplicate

- **`src/critic.{c,h}`** — **deleted** 2026-09-14. Do not resurrect.
- **On-board stimulus / `TraceStimulusSetpoint`** — removed from `control.c`. Do not
  re-add; Greg's directive stands (drive uninterrupted, pilot supplies stimulus).
- **24 B critic trailer (`TRACE_CRITIC_*`)** — removed from `trace.h`/`trace.c`
  (flash = header+records only). If identify later ships outputs, decide a NEW scheme;
  do not re-append a "CRIC" block.
- **`critic.metrics` (GCS side)** — still here, use it as the offline validation spec.
- **`test_pid_sim.py`** — runs the same `emu.c` plant in Python. After you land
  `identify.c`, the natural validation is: feed the recorded `(Out, Rate)` from a real
  dump through your estimator and compare the identified plant against
  `emu.c`-predicted response.

## 4. The handover design — what we propose you build

### 4.1 `identify.c` / `identify.h` — passive observation only

Greg-approved direction (from session B's discussion): a **tracking/self-tuning
predictor** — estimate the per-axis plant parameters continuously; update them as new
data arrives. The clean formulation:

- **Augmented-state estimator.** Your RLS-with-λ is a special case of a Kalman filter:
  parameters as **random-walk states** (small process noise Q on τ/k/authority = slow
  drift; larger Q on the Rate state = fast dynamics), one innovation = measured gyro
  `Rate[a]` vs plant-model prediction, covariance `P` is the confidence. If you do the
  **linear** first-order variant (`Rate[k+1]=a·Rate[k]+b·Out[k]`) the update is an
  ordinary KF and trivially O(n²), n≈2 per axis; the **nonlinear** emu-form requires an
  EKF/UKF or output-error least squares — feasible offline, heavier online. Proposal:
  **run the linear per-axis KF in flight (v1), keep the nonlinear emu-parameter fit
  offline in Python** (`tests/`) as the validation tune, and diffuse the two later.
- **Excitation gate:** a Kalman gain under-pumped by near-zero innovation variance is
  your implicit freeze; still expose an explicit `var(Out)`/`var(Rate)`-based
  confidence flag (GREEN valid / YELLOW frozen) so the GCS never shows noise as truth.
- **No injection, no drives effects, no flash writes in the air** (flash only on
  disarm via the existing config-persist path, or via trace commit — decide later).
- **Export** v1: fold the latest per-axis estimate + covariance diagonals + gate into
  something the GCS already reads. Do not coin a new tag/telemetry frame yet — reuse
  the show-attitude diagnostic or the trace (details below) first. Telemetry/rawlog
  always carries the raw rows the GCS can re-run.

### 4.2 Relationship to the trace mechanism (your question, answered)

> "If it works we do not need to dump traces to the GCS, correct?"

**No — the dump is still needed, but its role changes.** The trace dump is the
**ground-truth archive + the validation corpus + the offline high-fidelity fit source**,
and identify runs on raw `(Out, Rate[, throttle])` rows whether they come live or from a
dump. Specifically:

1. **Live identify** gives you a running smoothed per-axis plant estimate at 500 Hz —
   great for a _status display_ and for catching gross authority mismatch in flight.
2. **The dump remains the tool that makes the estimate trustworthy.** After a capture
   you re-run your estimator offline on the payload bytes, cross-check against
   `critic.metrics`, and compute the **nonlinear emu-parameter fit** that a 2-parameter
   linear KF cannot see (MotorTau, drag coefficient, thrust coupling). You then have
   the `.af`-ready numbers (flight-faithful authority) *before* believing the live
   estimate.
3. So the right architecture is: **live KF = instrument; dump = evidence.** If the live
   number converges and is reproducible across dumps, you can stop *reading* dumps at
   the field — but you still *write* them (a 130–500 ms exchange) as the audit trail.
   Do not remove the dump path.

An additional idea worth inheriting (flag for Greg): carry the **latest identified
per-axis params** as a v2.1 **header-block extension** (analogous to the gain block)
so a dump also remembers what the estimator believed at capture time. Cheap, no new
protocol.

### 4.3 Where it hooks in

- Call `IdentifyProfile()` from `DoControl()` next to `TraceCapture()` (`control.c:693`)
  at the same 500 Hz cadence, gated on `State == eInFlight` (and probably on
  `pTraceType != eTraceNone` or its own enable). It must run *after* the axis
  controllers have written `A[a].Out` and after `Rate[]` has its measured value, i.e.
  at the same point TraceCapture samples.
- Read inputs as `A[a].Out` (plant input) and `Rate[a]` (plant output); throttle/
  `RawPW` for the MR `MotorLagState` gating if you go nonlinear.
- Because it is passive it never touches `A[a].Out`, `Rate[]`, `Desired`, drives, or
  the Ch10 `RateGainScale` pot path. If anything about your design could stall a servo
  or feed signals back into the loop — it is the wrong design.

## 5. Rules you must build under (FC C Conventions — AGENTS.md)

- Classic C, no goto, one exit per function (no bare mid-function returns), no `malloc`,
  no pointer arithmetic beyond the small-loop exemption, no division operators beyond
  the inherent-math exceptions (`1.0f/(x)` guarded), `x/2` → `x*0.5f` etc.
- Bounds-check every array access (or compile-time-known small loops). No assignments
  inside `if()`. ISRs minimal — an RLS/KF update is main-loop work, not an ISR.
- Max 100-line functions; `#define` constants, `const` for typed floats; `//` comments
  explaining WHY. No magic numbers.
- ParamTable is the source of truth for bounds; if you add a param (e.g. identify
  enable/gain) update `params.c` ParamTable AND `uavx-python/src/parameters.py`
  `PARAM_LIMITS` together, and mirror naming (lowercase `p` prefix, `e`-prefixed enums).
- Attribution header on external-derived code (e.g. any textbook KF structure):
  `// Based originally on work by <author>.` + `// Adapted <YYYY-MM-DD> by ...`.
- **Build:** `python3 UAVXArmQ/scripts/fc_build.py` (env `BOARD`), sandbox-friendly;
  GCS compile-check `python3 -m py_compile`. **Never ship logs**; update
  `AGENTS.md` and this wiki report before session end.

## 6. Suggested v1 scope (a realistic first increment)

1. `identify.{c,h}`: per-axis linear KF (`Rate[k+1]=a·Rate[k]+b·Out[k]`), states
   `[Rate, a, b]` or just `[a, b]` with `Rate` measured, random-walk Q on the params,
   gyro-derived R, `P` bookkeeping, excitation gate + confidence (GREEN/YELLOW).
2. Hook at `DoControl()` cadence per §4.3, in-flight only.
3. No new wire tag: surface the estimate in a diagnostic the GCS already prints
   (or as a console line), plus refit offline from a dump in Python.
4. Validation: feed a captured `(Out, Rate)` window into the estimator; confirm the
   identified `a` reproduces the observed settle against `critic.metrics`; confirm the
   identified authority matches the `.af` `R.Kp`-implied DC gain within ±30%.
5. Document results in a new wiki session report and close the loop with Greg.

Ask before you expand scope (nonlinear emu fit, new protocol tag, config bits, or
writing to flash in the air are all out-of-scope without Greg's sign-off).