# Session Report — Cascaded Angle→Rate Control: External Cross-Check (Deepseek) vs Our Stack (Sep 13)

**Date:** 2026-09-13.
**Scope:** Greg asked an independent LLM (Deepseek) its opinion on our cascaded
architecture (outer PI angle→rate, inner PID rate→actuator, with specified max
angles/rates) and whether outer-loop gains can be derived from the maxes alone.
This report records the cross-check, the numerical validation against our own
derivation, and the conclusions we may wish to consider. It is deliberately a
*findings-and-questions* report, not a decision record — the decisions it points
at are still open for Greg. PDF conversion remains a host (Merlin) task.

---

## 1. The architecture under review (as Deepseek summarised it)

```
angle_ref ─►(＋)► PI ─► rate_ref ─►(＋)► PID ─► plant (motor/servo) ─► angle_out
              ▲                       ▲
              │                       │
         angle_fb                  rate_fb (gyro)
```

- Outer loop: PI, angle error in → commanded rate out.
- Inner loop: PID, rate error in → actuator setpoint out.
- Constraints: max angle, max rate.

## 2. Deepseek's key claims (condensed)

1. **Loop bandwidth separation**: inner (rate) loop must be ~5–10× the outer
   (angle) bandwidth, else the loops fight (oscillation).
2. **Two saturation/anti-windup points**: outer output clamp (`rate_ref =
   clamp(pi, ±MAX_RATE)`) and inner output clamp (`cmd = clamp(pid, ±MAX_CMD)`).
   Anti-windup on both; the outer PI will wind up whenever the rate clamps.
3. **Outer plant is kinematically 1/s** (`θ/q = 1/s`) *as long as the inner loop
   tracks its command*. So the outer loop does NOT depend on the airframe — it
   sees an integrator. The inner loop is where all the aero/physics/mass/effectiveness
   mess lives.
4. **No-saturation gain ceiling**: `Kp_outer ≤ MAX_RATE / MAX_ANGLE_ERR`.
   Anything above guarantees saturation on a step of that size. This is the one
   outer-gain quantity *directly* derivable from the maxes.
5. **"Max rate" is a derived quantity, not a design freedom**:
   `MAX_RATE_effective = min(spec, achievable(q̄,mass), usable-human)`.
   FW: achievable is the binding term (720°/s is absurd for a fixed wing). MR:
   usable is the binding term (500–1000°/s achievable, but a human can't use it —
   that is a stick-shaping/expo problem on the *command* path, not a controller gain).
6. **If the signposted rate can't be achieved, reduce it until it is** — an
   unachievable spec guarantees inner saturation → outer windup → overshoot on
   recovery → unpredictable handling.
7. RC-scale docile autonomous FW: the problem collapses — one config, narrow
   speed band, ~2:1 q̄ variation, servos ~10–20 Hz. Suggested conservative
   limits: roll 60–120°/s, pitch 30–60°/s, yaw 20–40°/s. Tune inner empirically
   once. No scheduling needed.

## 3. Numerical validation against our derivation

**Claim 4 is EXACTLY our QKp formula.** We derive
`QKp = RateMax / (2·sin(AngleMax/2))` (quaternion half-angle; the `2·sin(θ/2)≈θ`
small-angle limit gives his `MAX_RATE/MAX_ANGLE_ERR`). That means our QKp is
*already pinned at the maximal non-saturating gain* — feeding exactly RateMax to
the rate reference at full angle error:

```
axis           2·sin(A/2)·QKp    RateMax    |error|
MR roll/pitch       3.4907        3.4907    5.2e-7 rad/s
FW roll             1.5708        1.5708    4.6e-9
FW pitch            1.5708        1.5708    1.2e-8
```

So Deepseek independently confirms: our adopted angle gains are at the
theory-optimal bound, not a hand-picked value. No `.af` change results from the
cross-check. (The implied small-signal ζ = Kp/(2√Ki) is ~3–7 — intentionally
over-damped / non-oscillatory for docile flight; zero overshoot on ident steps
matches.)

## 4. Where the cross-check confirms our stack (no action)

| Deepseek requirement | Our implementation |
|---|---|
| Inner loop faster than outer, 5–10× | Rate loop at 500 Hz control tick, angle loop same tick but gains derived from rate max → rate authority dominates; ident shows clean single/settle steps, no inner/outer fighting |
| Anti-windup on outer PI | `ConditionQuatIntE` (sign-conditioned integral dump, `control.c`) + `ILim` clamps — precisely his "stop integrating when clamped" prescription |
| Outer output clamp = MAX_RATE | `A[a].P.Max` (angle max) and the rate demand fed to `A[a].R.Max`-bounded rate loop |
| Inner output clamp (actuator) | `conditionOut` (±1 fraction-of-authority) at every controller exit |
| Rate feedback quality | Gyro (Mems); no raw differentiation of angle — quaternion attitude does the angle→rate kinematistry |
| Max rate as derived min(spec, achievable, usable) | MR 200°/s (below achievable, at usable); FW 90°/s below capability |
| Outer loop plant = 1/s once inner tracks | Confirms FW sim gate (§Sep-12 §11.2, `control.c:671` P-only) is *theoretically* sound when the inner loop delivers |
| "Don't chase with rate-Kp" for FW walls | Matches our Sep-12 §9.4 reading (Shadow/SkySurfer/SmallSpoileron walls = DC authority, not rate gain) |

## 5. Where it points at things we may wish to consider (open, for Greg)

### 5.1a FW pitch max-rate above Deepseek's "docile" band
Our FW roll/pitch clamp = **90°/s**. Deepseek's RC-docile suggestion is roll
60–120 (we're inside), **pitch 30–60 (we're above)**. Options to consider:

- **(a) Treat 90°/s as a controller ceiling, not a commanded ceiling.** Docile
  autonav commands (15° pitch steps, gentle banks) never reach 90°/s, so the
  clamp is a safety ceiling. No change. (My recommendation — matches his own
  usable-vs-achievable separation; the command layer limits what is demanded.)
- (b) Lower the **stored** FW pitch RateMax to ~60°/s and **re-derive** pitch QKp
  (they are coupled by the formula). This changes loop response fleet-wide and
  contradicts the Sep-12 "gentle robust 90°/s" adoption, which was grounded in
  iNav/ArduPilot practice (focus 4). Do not do lightly.
- (c) Note only in docs; decide when the trace instrument gives measured per-airframe
  achievable-rate truth (the still-blocked dump path — AGENTS TODO, Sep-13 Shadow flight).

### 5.1b Measured: lowering an unachievable rate spec makes the droop WORSE, not better
Greg's follow-up thought: *"if we cannot achieve a rate then we lessen our demand
and that changes our derived angle gains."* The coupling is real — but the
**direction** is the trap. Because our rate max is BOTH the rate clamp AND the QKp
input (they are rigidly coupled by `QKp = RateMax/(2·sin(AngleMax/2))`), lowering
an unachievable rate max *shrinks the angle gain* — and in a P-only hold that is
the gain fighting a DC authority wall, weaker gain = more droop.

Measured on the ident battery, Shadow pitch 15° step, rate max lowered + QKp
re-derived through the same formula:

| pitch RateMax | derived QKp | Shadow pitch final error | verdict |
|---|---|---|---|
| 90°/s (locked) | 6.017 | **5.67°** | ISSUE (fe ≤ 7.5° passes) |
| 45°/s (halved) | 3.01 | **8.23°** | FAIL |
| 30°/s (Deepseek docile-lo) | 2.01 | **9.69°** | FAIL |

So Deepseek's "reduce the max rate until it is achievable" does NOT translate to
lowering the file's rate max in our architecture: the max is a gain input, not a
free clamp. And — critically — our three "cannot achieve" cases are **not
rate-achievement failures at all**: the sim rates are transiently reached with
headroom; what fails is the **sustained-angle hold** (elevator DC authority /
drag-differential yaw at fixed qbar). Lowering the rate spec cannot fix an
authority shortage held at DC; the real lever for those stays **authority**
(propwash/q̄ modelling), which is exactly §4's "don't chase with rate-Kp" and the
Sep-12 §9.4 reading. This measurement *supports* keeping 90°/s as a controller
ceiling and limiting commanded rates at the command layer (5.1a) rather than
re-deriving gains from a lowered spec.

### 5.1c DECIDED 2026-09-13: GCS character slider — HIDE, preserve machinery
**Decision (Greg):** *"OK I agree. Hide the slider but preserve what is useful.
Proceed when you are happy. but document please."* The slider is hidden from the
shipped GCS; the machinery is preserved.

**Why we agreed the slider has no seat in the field workflow** (the referendum's
answer): the Ch10 pot is **fly-ready insurance** (survives n-hours of travel — fly
even with zero pre-measured tuning); at the field, run the trace tests, then reset
the pot to centre (scale 1.0) for the next round; the trace dump yields a better
handle on tuning overall and the test airframe in particular; baking the measured
gain into the `.af` is a one-step change. The slider cannot compete with that:
(1) angle gains are locked-derived (`QKp = RateMax/(2·sin(AngleMax/2))`), so its
angle curves can only re-produce derived values; (2) rate gains have the pot as a
real field instrument, safety-bounded 0.25–4.0; (3) Sep-11 already noticed "our
slider bar seems to be drifting towards just changing the rate"; (4) its
`_get_scale_factors` size-conditioning was empirically wrong (800× model vs <6×
real gain spread, CritiqueRetune Aug-29 audit; the 0.5–2.0 clamp is a band-aid).

**What was hidden (GCS-only, `ui/parameter_window.py`, `_create_setup_group`):**
the character-slider row — `_character_slider` + "Steady"/"Frisky" labels + the
`%` value label + the **Compute**/**Reset** buttons — is now `setVisible(False)`
by default, gated on env `UAVXGS_DEV_TUNE_SLIDER=1` (the same env-flag pattern as
`UAVXGS_VERBOSE_SERIAL`). The widgets are still constructed, so every consumer
stays wired and byte-identical:

- `compute_defaults` / `_get_scale_factors` / `_PARAM_CURVES`(_FW) /
  `_cascade_integrity_caveats` / `reset_to_computed` — intact, reachable in a dev
  session (`UAVXGS_DEV_TUNE_SLIDER=1` restores the full row).
- `.af` `Character` metadata read/write round-trip — untouched (a hidden slide
  position still persists; no `.af` format change).
- The live-edit sim hooks are **not** slider-gated and stay armed:
  `_check_cascade_sanity` (angle/rate spin edits → cascade caveats) and
  `_schedule_sim_check` (debounced robustness check on `_PARAM_CURVES` edits).

**What stayed visible (useful, independent of the fleet curves):**
- **Sim button** (`run_robustness_sim` + the `_sim_btn` state colouring) — the
  robustness report is a genuine tuning-check instrument.
- **`PHYS_*` descriptor inputs** (Physics group) — fleet surveying, sim inputs,
  `.af` filename summaries (e.g. `910g/4M`).
- The **simulator's toolbar slider** — a SEPARATE study instrument
  (`tests/test_pid_sim.py` `apply_slider`/`SLIDER_TESTS`); its ship-wide 2-D
  bracketing study (rate+angle together) is a documented report instrument
  (Sep-12 §11.6). Unaffected.

**Not-coupled (no sync needed):** `airframes/migrate_limits.py` carries its *own*
copy of the reference-aircraft ratios for `.af` limit migration; it does not call
the GCS slider machinery. `tests/test_pid_sim.py` has its *own* `_PARAM_CURVES`.

**Verification:** `python3 -m py_compile ui/parameter_window.py` clean. GCS-only;
no FC, no `.af`, no sim-core change.

**Status roll-forward:** the "GCS character slider" TODO in AGENTS.md is resolved
(2026-09-13); the `_get_scale_factors` "rebuild as per-size-class table" Misc note
is superseded — it only resurfaces if a trace-data-derived per-airframe gain map
(not a mass/arm model) is ever wanted.

### 5.2 The FW angle-Ki question — Greg remains unconvinced
Greg: "I remain unconvinced about dropping Ki for FW; but I think we have that in
the notes." Recording the state honestly:

- The FC gate (`control.c:671`, P-only for FW) is **structural and locked**. FW
  files ship the derived Ki as **dormant**. The sim now gates it too (flight-faithful).
- Deepseek's analysis actually *supports* the P-only choice **in the nominal
  case**: with the inner loop tracking, the outer plant is 1/s, and a P-only
  positional loop on a 1/s plant has zero steady-state error. Ki is only needed
  where the plant drops below 1/s authority — which in our sim is exactly the
  three "walls" (Shadow/SkySurfer/SmallSpoileron DC droop). So Ki on FW is a
  **mask for authority shortage**, not a nominal tracking term.
- Counterpoint (why Greg's instinct is defensible): if the plant *does* sit at
  the authority edge in a real flight regime (slow flight, propwash, gust hold),
  a little Ki is what keeps the outer loop from DC-drooping there. Our FW Ki is
  derived (`0.05·QKp`), so enabling it later is a config/param change, not a
  code change. That is exactly the "verify with flight data, then decide" path
  in AGENTS ("FW angle-Ki policy… Verify-with-flight-data is the open item").
- **Verdict stands**: keep FW P-only and dormant-Ki (both FC and sim), keep the
  revisit item open pending the trace/rawlog instrument delivering measured
  response data. Do NOT re-enable Ki to make ident tests pass (that was never the
  failing metrics, and it violates the FC gate).

### 5.3 MR "usable rate" is a command-path problem (confirms our split)
200°/s MR is achievable; the "controls become very sensitive" observation is the
*usable* term — handled on the command path (RC expo/stick shaping / pot
`RateGainScale` Ch10), not by lowering the controller's rate max. No change; this
cross-check validates the existing separation (controller clamps vs input shaping).

### 5.4 Anti-windup is already where Deepseek says it must be
No new structure needed. If we ever add a rate-command **I** term in the inner
loop (we deliberately have none — see AGENTS "Yaw Rate I-Term"), his inner
anti-windup point would apply; today the inner loop is pure P+D, no windup source
there.

## 6. Summary conclusions for consideration (numbered for easy decision)

1. **Our outer gains sit exactly at the theory ceiling** — no change, confirmed.
2. **The cascade structure already has the required anti-windup + clamps** — no change.
3. **Keep FW 90°/s as a controller ceiling; limit commanded rates at the command
   layer** (recommended, not yet decided).
4. **Keep FW P-only + dormant Ki; keep the revisit item OPEN tied to flight data
   — do not re-enable to satisfy tests.** Greg's unconvinced position is recorded;
   enabling later is a param/derivation choice, not a code change.
5. **MR sensitivity is a stick-shaping issue, not a max-rate issue** — keep the
   current separation.
6. **The trace-dump measurement path is the instrument that settles 3 & 4** —
   still blocked (AGENTS TODO "Trace dump saved 32 zero bytes"); unblocking it
   remains high-value.
7. **GCS character slider HIDDEN (decision §5.1c)** — the field workflow
   (pot insurance → trace → reset pot → bake gain) replaces it; machinery
   preserved dev-gated.

**Status:** report-only for the cross-check itself. The one code change resulting
from the discussion: the GCS character slider is hidden (GCS-only,
`UAVXGS_DEV_TUNE_SLIDER` gate, §5.1c). No FC/sim/`.af` change.
Prev cross-checks of the same class: iNav (Sep-04), ArduPilot (Sep-04).