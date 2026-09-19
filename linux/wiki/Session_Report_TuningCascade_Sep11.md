# Session Report — Tuning Cascade Reduction + Two-View PID Page (Sep 11)

**Date:** 2026-09-11 (design); implementation 2026-09-12.
**Scope:** Pure design/decision session (no FC or GCS code changed yet). Result: the
angle/Q loop is **over-specified** — only 2 of `{P.Kp, P.Max, R.Max}` are independent,
`IntLim` is bounded the same way, and the **rate loop is the only real tuning game**.
Follow-on: build a **two-view PID page** (Rate view / Angle view, flick switch) and mine
the fleet `.af` data for a Kd:Kp ratio before deriving Rate Kd.

**Status (2026-09-12):** design RESOLVED and implemented. FC: the two-function attitude
split was **merged into one unified `DoAngleControl`** and the FW-only
`FWRollControlPitchLimit` clamp (tag 113) was **retired** (`Unused114`/`UNUSED_114`) —
the singularity-free quaternion loop no longer needs a FW-specific roll/pitch angle
cap, and the mean was always AngleMax-identical anyway. GCS: the two-view PID page is
implemented (§5). Build/verify: all 7 FC targets + GCS `py_compile` clean.

---

## 1. The insight: the angle/Q loop is over-specified

### Starting observation (User, Greg)
> "If we know what our max angle is and the max commanded rate then we can compute angle
> Kp directly, can we not?" — then: "But we use Qp not Kp from our quaternion formulation"
> — then: "maybe I meant over specified."

### The math (exact form)
The quaternion angle loop commands (control.c:678-681, inside the unified `DoAngleControl`
`control.c:561`, integral gate `control.c:671`):

```
DesiredRate = 2.0f * Qa[a] * A[a].P.Kp        // Qa = sin(θ/2) for a single-axis error
DesiredRate = 2 * sin(θ/2) * P.Kp
```

So the exact inverse is:

```
P.Kp = RateMax / (2 * sin(AngleMax/2))
```

- **Ecks roll check:** `3.665 rad/s / (2·sin(0.2618)) = 7.08` ≈ tuned `7.0`. ✓
- Linear approximation `RateMax / AngleMax = 7.00` — only ~1% off at half-angle 15°,
  because `sin x ≈ x`, but the `2·sin(θ/2)` form is the *exact* inverse of the code's
  `2.0f·Qa`. Use the exact form if deriving.

### Consequence: over-specified triplet
`{P.Kp, P.Max, R.Max}` carry only **2 degrees of freedom** once we impose the natural
edge condition *full angle error = full rate*. Any of the three is derivable from the
other two. Keeping all three as independent params invites inconsistency — the existing
Ecks pitch/roll mismatch is the live proof:

- **Ecks mismatch symptom:** pitch reuses `Kp = 7` at `AngleMax = 30°` but `RateMax`
  210 → 120 °/s, so pitch rate-saturates at ~17° angle error, well inside the 30° setpoint
  range. Exact-derived pitch Kp would be `2.094/(2·sin15°) = 4.05`, not 7.

### Same game for IntLim (User instinct formalised)
> "I think I-limit is pretty much the same game. I did myself tend to limit Ki to around
> 70% of max angle — don't ask me why — instinct."

In quaternion units the I-term is an *equivalent angle error* (applied like `2·Qa`), so:

```
I_max_rate ≈ 2·sin(IntLim/2)·Kp ≈ 0.7·θmax·Kp ≈ 0.7·R.Max
```

- `IntLim = 0.7·AngleMax` means the integrator alone commands ~70% of full rate
  authority, never all of it.
- Rationale: letting IntLim approach `AngleMax` (= full authority) lets trim + P overlap
  at the edge → I could saturate the rate command → the windup/overshoot the
  `ConditionQuatIntE` sign-dump guards. Capping ~0.7 leaves the P-term ~30% headroom so
  tracking stays P-dominated while I covers steady-state disturbance/trim.
- Defensible rule: **`IntLim ≤ 0.7·AngleMax`** (quaternion-equivalent).

### Rate Kd — the branch we are NOT closing yet
User's first pass: Rate Kp editable, rest derived. Then: "derive Rate Kd perhaps…need to
think more about it. This has ramifications for our tuning studies does it not?" — **YES.**

If Kd follows from Kp, the tuning studies collapse from a **2D (P,D) scan to a 1D scan
along a constant-damping line**:

- Rate plant ≈ `1/(Js)`; PD closure → `ωn = √(P/J)`, **`ζ = D / (2·√(P·J))`**.
- Hold ζ → `D = 2ζ·√(P·J)` — D tracks **√P**. (iNav's class mapping `P/31`, `D/1905`
  is exactly this: D set ∝ P per class.)
- Two forms:
  - **Ratio form** `Kd = k_class·Kp` — needs only a per-*class* constant (MR vs FW differ,
    FW carries aero self-damping). Matches the existing Ch10 master-gain semantics
    (scales correction whole, P:D preserved). Machine-consistent.
  - **Absolute form** `Kd = 2ζ√(Kp·J)` — additionally needs inertia J per airframe, which
    the on-board step identification (autotune stage 2) estimates anyway.
- **Ramifications for the studies:**
  1. P–D trade-off maps become a **one-parameter family** — ask "which ζ / which gain",
     not "which (P, D)".
  2. Critic rise/overshoot/settle then verifies the ratio's assumed ζ; a settle failure
     means *k* is wrong, not that new D is needed.
  3. Inflight autotune becomes **one gain estimate per axis** (Kp), Kd derived — far less
     identification burden.
  4. FW vs MR stops being per-airframe and becomes a **two-row class constant**.
- Proposed next data step (accepted by user, not yet run): mine the fleet
  `/original/` + `user/` `.af` files for the Kd:Kp ratio spread so the decision has data
  behind it.

### Fleet Kd:Kp ratio mining — RESULT (2026-09-11, run via `fleet_dp_mine.py`)

**MR (multi-rotor): ratio is TIGHT — ratio-form derivation is defensible for roll/pitch.**

| axis | n | min | med | **mean** | max |
|---|---|---|---|---|---|
| ROLL | 10 | 0.017 | 0.038 | **0.032** | 0.040 |
| PITCH | 10 | 0.016 | 0.032 | **0.030** | 0.043 |
| YAW | 10 | 0.0006 | 0.0036 | **0.008** | 0.021 |

- Roll/pitch sit in a narrow 0.016–0.043 band (median ≈ 0.032–0.038). One constant
  `k_MR ≈ 0.035` covers the whole tuned MR fleet within ~±2× — acceptable for a
  derivation (unlike P, which spans 0.095–0.60).
- The bookends are telling: the **critic-validated Ecks 220 mm** sits at 0.0235
  (Kp 0.34 / Kd 0.008), mid-band; the stale low-gain copies just above it. **Yaw is a
  different animal** (0.0006–0.021, median 0.0036): yaw rate loop is near-differential,
  tiny Kd — needs its own (much smaller) constant, not the roll/pitch k.

**FW (fixed-wing): BIMODAL — NO single k_class exists.**

| axis | n | min | med | **mean** | max |
|---|---|---|---|---|---|
| ROLL | 8 | 0.000 | 0.000 | **0.006** | 0.045 |
| PITCH | 8 | 0.000 | 0.000 | **0.006** | 0.045 |
| YAW | 8 | 0.000 | 0.000 | **0.002** | 0.011 |

- The whole FW fleet (Shadow, Phoenix, generic wings) runs **Kd = 0** — aero self-damping
  does the work (Phoenix Kp 1.0 / Kd 0.004, ~0). The single exception is the newest
  Shadow tuning (`Shadow_REF5_20260912_070417`, Delta, D/P = 0.045).
- **Verdict: FW must keep Kd editable** (k≈0 by default, where the fleet lives; the one
  0.045 outlier is exactly the tuning-still-in-progress case). Only MR roll/pitch gets a
  derived `k_MR` rate.

**Conclusion for the model:** derived-rate is **MR-roll/pitch-only**: `Kd = 0.035·Kp`
(default deliverable of the two-view page), with yaw and all FW staying fully editable
and the MINED numbers as the starting hint. Studies: MR becomes 1D-constant-ζ; FW stays
2D (P,D) — asymmetry is now data-backed, not guessed.

### Extended survey — legacy airframes + Ki/Kp + IntLimit/AngleMax (2026-09-11, later in session)

User direction: extend the survey to the **legacy (hand-tuned) airframes** — "we may get
a better handle on Angle Ki values and perhaps Rate Kd" — and cross-check iNav + ArduPilot.
Ran via `fleet_survey2.py` over `original/`+`user/`+`generic/` (CUR) and
`backup_angleunits/` (LEGACY, pre-unified **degree-angle** era — angles normalised to
rad; rates are rad/s in both eras) + `backup/_retired_tuned/`.

**Rate Kd/Kp — legacy MR CONFIRMS the current MR constant:**

| cohort | ROLL med | PITCH med | YAW med |
|---|---|---|---|
| CUR/MR   | 0.038 | 0.032 | 0.0036 |
| LEGACY/MR| 0.038 | 0.038 | 0.0014 |

The hand-tuned legacy MR fleet sits on the **same 0.032–0.038** as the critic-validated
current set — two independent tuning generations converge on one k_MR. Legacy FW: D/P≈0
(one 0.020 outlier) — same bimodal story as current FW (D=0, aero self-damping).

**Angle Ki/Kp — a NEW tight constant, ROLL/PITCH ≈ 0.026 (NOT yawable with rate-D/P):**

| cohort | ROLL med | PITCH med | YAW med |
|---|---|---|---|
| CUR/MR   | 0.029 | 0.026 | 0.083 |
| LEGACY/MR| 0.026 | 0.026 | 0.083 |
| CUR/FW   | 0.050 | 0.036 | 0.083 |
| LEGACY/FW| 0.050 | 0.050 | 0.083 |

- Roll/pitch angle `Ki ≈ 0.026·Kp_angle` in BOTH generations — as derivable as rate D/P.
- **YAW is a separate constant (0.083)** — yaw's Qa gain-to-rate authority differs, so yaw
  Ki/Kp clusters ~3× higher. Same "each axis is its own k" lesson as yaw rate-D/P.
- **FW roll/pitch Ki/Kp ≈ 0.05 is DORMANT — the FW angle loop carries NO integral.**
  Since the 2026-09-12 merge the FW loop is the SAME `DoAngleControl` as MR/VTOL
  (`control.c:561`) but the integral gate `ConditionQuatIntE` is keyed on
  `pAFTypeCategory != eCatFw` (`control.c:671`), so FW runs pure angle-P
  `Limit1(2·Qa·Kp, R.Max)` (`control.c:678-681`). So the "long time to reach attitude"
  wind-up-and-overcook risk Greg flagged for conventional FW frames **cannot occur** —
  the FW loop never reads `Ki`/`IntE`. The stored `.af` Ki on FW frames is an inert
  shared-`PIStruct` field.
  **Consequence for the two-view page: disable derived Ki/I-Limit rows (not-applied) on
  FW frames; derived angle-Ki lands only on MR roll/pitch (0.026·Kp_angle) and MR yaw
  (0.083·Kp_angle).** Rationale (Greg): on MR the attitude response is so fast the
  sign-conditioned integral integrates only a few ticks before the error sign flips and
  dumps — it barely acts (hence the 0.01 caps); on a slow FW responder a large-enough
  cap would let `IntE` sit at its limit and over-fire the rate demand through the hold —
  which is exactly why the FW path is P-only.

**IntLimit/AngleMax — the fleet lives at ~1%, NOT 0.7:**

| cohort | ROLL med | PITCH med |
|---|---|---|
| CUR/MR   | 0.95% | 0.95% |
| LEGACY/MR| 0.95% | 0.95% |
| CUR/FW   | 0.95% | 2.9% |
| LEGACY/FW| 0.95% | 0.95% |

- Every hand-tuned airframe (both eras) caps the angle integrator at **~1% of AngleMax**
  (Ecks: IntLimit 0.01 rad vs AngleMax 0.524 rad). The 0.7·AngleMax figure is **never
  approached** — it survives only as the theoretical headroom bound where I numbers one
  would never actually fly.
- **Resolution of the 0.7 question (Greg's view held + data):** 0.7·AngleMax is *correct
  as the SPIN-BOX RANGE MAX* — a safe hard cap. **Confirmed windup-free by construction**
  (Greg's argument, verified in `control.c`): the I-term is amplitude-bounded by `IntLim`
  itself (`Limit1`, misctypes.h:203), the summed demand `rate = 2·Qa·Kp + IntE` is clamped
  to `maxRate` every tick (`control.c:679-681`) so a saturated rate loop cannot pump the
  integral into the demand, and `ConditionQuatIntE` dumps it on error-sign flip
  (`control.c:448`). Only residual effect of a large cap is a bounded, P-aiding release
  transient. **BUT the DEFAULT VALUE is fleet-backed at `≈ 0.01·AngleMax` (~1%):** every
  hand-tuned airframe (both eras) caps the angle integrator there; 0.7 would write ~21°/s
  of I-authority on a 30° vehicle (~10% of max rate) — deliberate trim, not the loop's
  lazy mop-up role. Unit note: `IntE` is added directly in rate units (rad/s), so
  `IntLim` is a rate-contribution bound, not an angle. 0.7·AngleMax = cap; 0.01·AngleMax
  = the default the two-view page derives and writes.

**Derived-AngleKp signability — the equation `RateMax/(2·sin(AngleMax/2))` is CONFIRMED:**

| cohort | ROLL residual | PITCH residual |
|---|---|---|
| CUR/MR   | 4.5% med | 4.5% med |
| LEGACY/MR| 4.5% med | 4.5% med |
| LEGACY/FW| 4.5% (roll) | 1.1–4.5% |

- The hand-tuned MR fleet reproduces the exact quaternion inverse within ~4.5% median —
  tuners converged on the same envelope accidentally. (Fleet outliers are the FW angle-Kp
  "tuning-in-progress" files: Shadow Kp 4.0 vs derived 2.09 = +91%, Quad_Racer 1.8 vs
  8.09 = −78%; FW angle loop is deliberately over/under-driven, staying editable.)

**External cross-check — iNav 7.1.2 + ArduPilot (via `compare_inav.py`/`compare_ardupilot.py`):

- **Neither autopilot has an ANGLE integrator at all** — the I-term lives on the *rate*
  loop: iNav MC rate (P40,I30,D23,FF60 roll/pitch; I=45 yaw; pidSumLimit 500/400),
  ArduPilot Copter rate P0.135/I0.2/D0.0036, angle P-only 4.5. "Angle Ki" + "Angle
  IntLimit" are legacy-UAVX-specific concepts with no external equivalent to sanity-check
  numerically — our 0.026/0.01 figures are self-derived from our own fleet.
- iNav's **rate-I cap = 33% of pidSum** (settings `i_limit`, default 0.33) — the same
  fraction-of-authority windup-clamp philosophy as Greg's 0.7·AngleMax, but applied to the
  rate I-term. Cross-platform precedent that a fractional-of-authority I-clamp is sound.
- **Rate D/P external validation:** ArduPilot Copter `0.0036/0.135 = 0.027` ≈ our MR
  **0.032–0.038** (±15%). Two independent MR rate-loop tunings agree — strong external
  push for `k_MR ≈ 0.035`. iNav maps to ~0.009 (leaner) but iNav carries FF=60 which we
  don't, so the D-sharing differs structurally.
- **FW rate D = 0 in iNav defaults** (`fw_d_*` — which are the LPF-cutoff/horizon slots,
  D is zero) → matches our FW fleet's Kd=0. Aero self-damping does the work on both
  platforms. (ArduPlane FW rate "D" ≈0.23-equivalent is the structurally-non-comparable
  RLL2SRV case settled in the Sep-04 report.)

## 2. Two-view PID page (GCS)

### Decision
"Too late to make a decision but maybe we can build two versions of the PID page so we can
flick in between. Our slider bar seems to be drifting towards just changing the rate."

User-approved split (confirmed via Q&A):

| | **Rate view** (drift target) | **Angle view** (current default) |
|---|---|---|
| `Max Angle` | editable | editable (existing row) |
| `Max Rate (°/s)` | editable | editable (existing row) |
| `Rate Kp` | editable (adj. for FW, maybe not MCs) | editable |
| `Rate Kd` | **editable** (user confirmed Keep Kd editable — it is the stability knob, Ecks 4 Hz) | editable |
| `Q Angle Kp` | **derived read-only** — `RateMax/(2·sin(AngleMax/2))` | editable |
| `Ki Angle` | derived/bounded (pending) | editable |
| `I-Limit` | **derived read-only** — `0.7·AngleMax` | editable |

Mechanics: a flick switch (combo/segmented) at the top of the PID group swaps row
order/content of the *same* grid — **no widget duplication**, so readback/sync/live-write
(`_create_pid_group`, `add_spin`, `_set_widgets_from_raw`, `_raw_float_values`) stay
intact. Derived rows are read-only labels computed from the editable Maxes /
`MAX_ROLL_RATE`-style params (`::360°` conversion deg↔rad on display).

### Rationale
- The cascade reduces to: angle loop fully parametric once constrained (Kp derived,
  IntLim bounded), rate loop is where bandwidth/damping/stability live.
- Evidence already in repo: Ecks 4 Hz → rate P/D tradeoff; emulator limit-cycle → `R.Max`
  clamp; rate-step critic metrics; autotune stage 2 rate-only; Ch10 pot scales rate loops.
- Two-view hedges the open decision (final layout) without committing — cheap, reversible.

## 3. Open decisions / TODO
1. ✅ **Fleet survey — DONE & EXTENDED (2026-09-11):** now covers legacy hand-tuned
   airframes (`backup_angleunits/`, degree-era, +`_retired_tuned/`) and yields FOUR
   fleet-backed constants: MR rate `Kd = 0.035·Kp` (legacy 0.038 ≡ current 0.032),
   MR angle `Ki = 0.026·Kp` (both eras), MR **I-limit value ≈ 0.01·AngleMax** (never the
   0.7 bound), yaw keeps its own (0.0035 rate-D/P, 0.083 angle-Ki/Kp). Derived-AngleKp
   equation validated to ~4.5% med on hand-tuned MR. iNav/ArduPilot cross-check added:
   no angle-I in either (our concept is legacy-UAVX-only; both use rate-I capped at a
   fraction of authority); ArduPilot Copter D/P 0.027 ≈ our MR — external validation.
2. ✅ **Rate Kd derivation direction — RESOLVED:** ratio form `Kd = k_MR·Kp` with
   `k_MR ≈ 0.035` for **MR roll/pitch only**. FW stays editable (D=0 fleet-wide, iNav FW
   D=0 confirms — aero self-damping). Absolute form `2ζ√(Kp·J)` remains the future precise
   path once autotune stage-2 provides J — the critic's ζ/k then verifies the 0.035.
3. ✅ **I-Limit in Rate view — RESOLVED:** derived value `≈ 0.01·AngleMax` (fleet flight
   reality, both eras); the **0.7·AngleMax is the spin-box range max** (hard clamp — safe
   because `ConditionQuatIntE` cancels on sign change, never approached in flight).
4. **Ki Angle in Rate view — RESOLVED:** derived `≈ 0.026·Kp_angle` (roll/pitch MR),
   yaw `0.083·Kp_angle`, FW `0.05·Kp_angle` — all fleet-backed. Value derived; range stores
   the derivation.
5. ✅ **Implement the two-view PID page + the enablement refactors** — DONE (§5).

## 4. Build / verification status
- **FC (2026-09-12):** all 7 targets build clean after the merge + tag-113 retirement
  (UAVXF4V3 232724 B, UAVXF4V4 232460 B, DEVEBOXF4 232836 B, SPEEDYBEEF405WING 234436 B,
  FLYINGRCF4WINGMINI 234572 B, BLUEBERRYF405 234292 B, MATEKF411WING 233980 B).
- **GCS (2026-09-12):** `py_compile` clean on the retirement edits (parameters.py,
  protocol_enums.py, airframes.py, convert_old_defaults.py, migrate_limits.py,
  parameter_window.py) and on the two-view PID page implementation.

## 5. Implementation (2026-09-12)

### 5a. FC: unified `DoAngleControl` + tag-113 retirement
- **Why the merge:** the over-specification insight from §1 means the FW/MR split carried
  no algorithm — both were the same quaternion loop with the same category-conditioned
  details (yaw axis-to-run, desired-quaternion yaw ingredient, integral staging, dive
  override). Keeping two functions invited drift.
- **What changed (`control.c`):** `DoQuaternionAttitudeControl` and `DoFWAttitudeControl`
  deleted; single `static void DoAngleControl(real32 dT)` (`control.c:561`), header
  comment `control.c:547-560` documents the category map; `DoControl` (`control.c:694`)
  is now a `switch (pAFTypeCategory)` with `eCatFw` → FW rate-damped yaw + angle loop,
  `eCatMr`/`eCatVtol` → `CalcTiltThrFF` + `UpdateYawSetpoint` + angle loop, `eCatLand`
  empty. The relative path axes, integral gate (`pAFTypeCategory != eCatFw`,
  `control.c:671`), and dive override gate were preserved 1:1.
- **Tag 113 retired:** the FW angle clamp `FWRollControlPitchLimit` becomes
  `Unused114`/`UNUSED_114` (NULL-targeted row `params.c:270`), removed from the FW
  roll/pitch setpoint line (`control.c:54`), GCS row dropped from the param window, old
  `.af` key loads through the `_LEGACY_PARAM_NAME_TO_TAG` bridge until re-saved. Wire
  format untouched (tag 113 still on the wire, harmless `(0..255)` write/read).
- **FW angle-Ki policy resolved (docs only):** the integral gate is confirmed P-only for
  FW by analysis (`nav.c` `WindCrabHeading` feedforward owns FW steady state, MR yaw
  `YawAngleQKi` is the load-bearing angle-I); record as "verify with flight data", NOT
  "add FW angle-I". See AGENTS.md.

### 5b. GCS: two-view Rate/Angle PID page
- Mechanics per §2: a flick switch at the top of the PID group re-labels the **same**
  grid — no widget duplication, so readback/sync/live-write (`_set_widgets_from_raw`,
  `_raw_float_values`, `param_changed`) are byte-identical across views.
- Angle view (default) — everything editable exactly as today.
- Rate view — hides the Q-gain spins, shows **read-only derived labels**:
  - `QAngleKp = RateMax / (2·sin(AngleMax/2))` (§1 verified equation);
  - `KiAngle = k·Kp_angle` with MR roll/pitch 0.026, MR yaw 0.083, FW 0.05 (§1 fleet);
  - `ILimit = 0.01·AngleMax` (§1 fleet value; spin range max stays `0.7·AngleMax`).
  - `RateMax`/`AngleMax` spins/steps remain editable in both views (they are the
    derivation inputs); `Rate Kd` and yaw stay editable (stability knob).
- FW frames: the derived Ki/I-Limit rows are **disabled** (not-applied — dormant FW Ki,
  see §1); derived `QAngleKp`/Kd rows unaffected.
- Dual-write caution: in Rate view the derived rows are read-only, so there is nothing
  to resync on readback; the editable Maxes do not write-back the derived values.