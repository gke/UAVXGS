# Session Report — AH Comp-Bind Root Cause + Generic .af Base Sync + Free-Flight Dihedral Gap (Aug 28)

## Objective / context
Continuing the test_pid_sim.py characterization suite work (generic-fleet at
slider extremes). Three threads from the previous run needed resolution:
1. Heavy-quad alt-hold rise was on the 2.0 s CONS limit (measured 2.01 s→FAIL).
2. The `ALT_THROTTLE_COMP_LIMIT` slider curve was out of FC bounds.
3. `SmallSpoileron.af` pitch/yaw authority shortfall (physicals suspect) and a
   first pass at `PHYS_*_DAMP` calibration.

## Findings

### 1. AH failure root cause — *missing param in the .af bases*
The real cause of the Quad AH rise 2.01 s FAIL was traced through a drag sweep +
replica sim: `ALT_THROTTLE_COMP_LIMIT` was **not present in any MR `.af`**.
- `apply_slider()` only overrides params that exist in the parsed `.af`, so the
  suite's comp curve silently never bound; the alt-hold sim fell back to its
  `params.get(..., 0.2)` default on the in-memory `ControllerParams`.
- Comp 0.20 → rise 2.01 s (FAIL); comp 0.25 → 1.86 s (PASS). FC clamps the param
  at 0.25 (`PARAM_LIMITS[102] = (0.0, 0.25)`, mirrors params.c `MaxAltHoldThrComp`).
- **Fix (applied):** generic `.af` re-sync — added to all 14 base files
  `ALT_THROTTLE_COMP_LIMIT = 0.25` (0.05..0.25) and `MAX_CLIMB_RATE_MP_S = 5.0`
  (1.0..10.0); to the 5 MR files additionally the params they were missing:
  `ROLL_ANGLE_Q_INT_LIMIT/PITCH_ANGLE_Q_INT_LIMIT = 0.01`, `MAX_ROLL_RATE = 1.396`,
  `MAX_PITCH_RATE = 1.047`, `HORIZON = 3.333` — all values FC-valid so the slider
  envelope now binds on every axis the 4 sims consume.
- Curve corrected to the legal `COMP (0.25, 0.25)` (FC max); the old `(0.32,0.35)`
  was beyond the ParamTable clamp and could never be written by the GCS.
- Duplicate-row guard: Shadow/SkySurfer/Dragon already carried comp (0.1) →
  the sync's add-anyway approach briefly duplicated the row; fixed in place.

**Verified suite results after the sync** (generic fleet, both slider extremes):
- All 5 MR airframes **fully PASS** both extremes (attitude, disturbance, AH, nav).
- MR AH rise times (CONS/AGGR): Hex 1.786/1.68, Oct 1.645/1.527, Quad 1.859/1.763,
  Quad_Medium 1.775/1.671, Quad_Racer 1.571/1.455 s — heavy Quad is the binding
  frame, margin +0.14 s to the 2.0 s limit.
- No AH overshoot regression (comp 0.25 at AGGR still inside the 20 % band).

### 2. "FW nav" fails were actually free-flight turbulence — since resolved
Early extraction mislabelled them as nav; they were the **open-loop
free-flight turbulence** sub-test that follows nav (nav itself PASSes all
frames).  Pre-existing, proven by reverting the +2 FW .af additions and
re-running Delta/Dragon/Elevon/Shadow: identical fails reproduce, and the test
reads no tuned param.  Root-caused in §3 as the missing roll restoring term —
after the fix all four frames pass (see §3 / verification).  Dragon/Shadow's
base-gain Pitch@30° shortfall (passes only under slider override) is a
separate, pre-existing base-value note; their base PITCH_ANGLE_Q_KP predates
this session and is not chased here.

### 3. Free-flight model gap — missing dihedral roll restoring (fixed)
The four "fail" frames failed open-loop turbulence, not nav. The model had
angle-based pitch/yaw static stability but **no roll-angle restoring term** —
only quadratic+linear rate damping and a yaw-rate coupling — so a roll gust
integrated into a random-walk offset / spiral (Delta 29.8° persistent offset,
Shadow 1286° divergence) on frames whose closed-loop sims (and real stick
flying) are fine. That asymmetry — "fly well under manual control, diverge
no-PID" — is exactly the model gap, not an airframe defect. Closed by the C_lβ
composition detailed in Decisions (§4). Shadow specifically is a swept plank:
30° sweep gives real effective dihedral even at ~2° geometric, so its descriptor
carries sweep_deg=30.

### 4. SmallSpoileron physicals — damp anomaly (fixed, one limitation kept)
Table vs identical-geometry twin `Spoileron.af` (same span 1800 mm, chord 280 mm,
AUW 1200 g), damp coefficients:

| param | SmallSpoileron (stored) | Spoileron (twin) | ratio |
|---|---|---|---|
| PHYS_ROLL_DAMP  | 1.4908 | 0.0265 | 56× |
| PHYS_PITCH_DAMP | 2.9817 | 0.5544 | 5.4× |
| PHYS_YAW_DAMP   | 0.6009 | 0.0352 | 17× |

Verified with the real harness (both extremes):
- **yaw@15°** only reaches ~4.7° with stored damp → cures to PASS with the twin
  values (0.2–0.275 s rise) — the stored numbers **are** the anomaly's victim.
- **pitch@30° CONS** is *not* damp-limited: final 25.9° is identical under either
  damp set. The real binding constraint is tail-volume: SmallSpoileron elevator
  arm 0.4 m vs Spoileron 0.7 m → ~30 % less control authority, so the shared CONS
  pitch gain cannot reach 27° (90 %). AGGR passes (0.415 s) at damp-corrected.
- Recommendation tabled: apply the twin damp values (corrects yaw & AGGR pitch);
  CONS pitch@30 stays marginal by design of its shorter tail arm — document, don't
  chase the global CONS pitch curve for one airframe.

## Decisions / rationale
- **Applied & verified**: `.af` base re-sync + comp curve at FC-legal (0.25,0.25).
  Rationale: an absent param silently defeats the envelope (sim default used), so
  bases must carry every consumable param at an FC-valid value; CONS comp pinned
  to the ParamTable max because that is the only setting that clears the heavy-quad
  rise and it is exactly what the FC allows / the retired tuned fleet used.
- **SmallSpoileron damp correction applied** (user-approved): damp set to the
  Spoileron twin's values (ROLL 0.0265 / PITCH 0.5544 / YAW 0.0352).  Verified to
  cure yaw@15° (was 4.7°, now PASS both extremes) and AGGR pitch@30°; 5.4–56×
  stored values were physically impossible for a 1.8 m plank (data-entry-era
  artefact).  **CONS pitch@30° remains a documented, accepted limitation** — it
  is authority-limited (elevator arm 0.40 m vs the twin's 0.70 m → ~30 % less
  tail-volume; final 25.9° identical under both damp sets, so not damp, and the
  shared CONS pitch curve cannot carry one low-authority frame) — not chased.
- **Free-flight roll restoring gap closed** (the "are we missing modelling?"
  thread): `simulate_freeflight` had angle-based pitch/yaw stability but NO
  roll-angle restoring — only rate damping plus a yaw-rate coupling — so
  open-loop gust tests randomly walked off (Delta 29.8°) or spiralled (Shadow
  1286°) on frames whose closed-loop sims and real stick flying were fine.  A
  model gap, not an airframe defect.  Added an explicit geometric-dihedral +
  sweepback `C_lβ` composing term:
    - dihedral panels via Δα = arctan(sinβ·tanΓ), span-weighted, × wing lift
      slope a_w (lifting-line) × taper factor Fλ = (1+2λ)/(3(1+λ));
    - sweepback term K_SWEEP · CL_cruise · tan(Λ) × Fλ (spanwise-lift-leverage;
      zero at CL=0), K_SWEEP = 1.0 per NASA NTRS 19930080953 equivalence
      (~1/3–1/6 the strength of an equal dihedral angle).
    Per-frame descriptor overrides `sweep_deg`/`dihedral_deg`/`anhedral_deg`/
    `anhedral_start` added for the frames with known real geometry: Shadow
    (30° sweep, 2° Γ), Horten (40°, 2°), Arado 555 (45°, +5° inboard 2/3, −5°
    tip anhedral → EDA ≈ 1.7+3.9 = 5.6°).  All other FW frames default to the
    existing flat dihedral_coeff ≈ 3° EDA.
    **Decision (user): descriptor-side only — the PHYS_ spec / .af files are
    NOT augmented**; sweep/panels are a screening-test concern, not a GCS/FC
    data contract.  Fleet passes at every K in {0.2, 0.5, 1.0, 1.5}; the 3° flat
    default is numerically indistinguishable from per-frame K=0.2, and only the
    swept-frame EDAs (~4.5–5.7° at K=1.0) separate.
    **Integrity note:** the user's per-frame geometry did NOT change any pass/fail
    verdict — the fleet passed identically at flat 3°, at the user geometry, and
    at every K tested; the four originally-failing frames (incl. Delta/Dragon/
    Elevon, which carry no user geometry) pass off the newly-added generic
    restoring term alone.  The knowledge only made the screening scale read more
    faithfully.  No test was fit to an outcome.
    **Rationale (fidelity vs tuning accuracy):** adding span/tip·root-chord/
    LE-sweep/tip-height-dihedral PLANAR fields would not materially improve
    tuning accuracy.  Only the free-flight screen reads C_lβ (a wide-band
    pass/fail); the tuning sims consume inertia and authority, which the spec
    already carries.  The dominant model uncertainty is aero-damping/inertia
    heuristics, ≫ any planar-geometry refinement; taper (Fλ) is the only
    remaining cheap win and is exposed as an optional per-frame key.
- **Left untouched (pending user call)**: anything else — the only remaining
  fleet FAIL is the documented SmallSpoileron CONS pitch above.

## Follow-up review (Aug 29) — assumption magnitudes; no change made
- **Dihedral default 3° vs possible 4–5°**: user observed that real effective
  dihedral for this fleet (swept/delta frames) is likely > 3°, closer to 4–5°
  (user-geometry EDAs: Shadow 4.5°, Horten 4.3°, Arado 5.6°; flat planks ~2°).
  A bump of the flat default (3°→4°) was considered.  **Decision: leave at 3°.**
  Rationale: the flat default sits inside the demonstrated insensitivity band
  (2.0–5.7° EDA across every gain K); margins are huge (worst ≈ 3.8° peak /
  1.5 s settle vs 20° / 5 s limits); 4° changes no verdict and 5° would
  over-claim the fleet average (flat-plank floor).  The only benefit would be
  representativeness of the screening read-out, not safety or precision — not
  worth the churn.  Kept as a documented alternative for later readers.
- **Pitch-stability provenance clarified**: `pitch_stability` (Cm_α) derives
  from the user's per-frame **assumed CG position** (SM → Cm_α), not from a
  generic 2–5 % MAC convention.  Scope confirmed: it is consumed **only** in the
  two free-flight routines (`M_stability = pitch_stab_coeff × Pitch`,
  test_pid_sim.py:1885/2157); the closed-loop attitude/tuning sims never read it
  — the same scope guarantee as dihedral.  Reliance acknowledged: it is an
  assumption-driven input, acceptable for the wide-margin screening-only use.
- **Both assumptions share the same character**: open-loop-screen-only, wide
  tolerance, no influence on any tuning or pass/fail outcome.  This is the
  boundary of the current model's fidelity — see the Dutch-roll/spiral follow-up
  (TODO) for the remaining lateral-directional blind spot.

## Verification status
- Full fleet `python3 test_pid_sim.py` → **one** FAIL left: SmallSpoileron
  CONS pitch@30° (accepted, documented). Everything else — MR fleet, all FW
  attitude/disturbance/AH/nav, all free-flight turbulence — PASSes both edges.
- All 14 `.af` re-parsed OK via `parse_af_file()` after sync (no dup/missed rows).
- No C, GCS-source or ParamTable change in this session; no FC build needed.

## Lateral-directional mode analysis — Dutch roll / spiral (Aug 29)
- **Objective / identified blind spot**: the free-flight screen integrates
  *angles* with restoring moments but has no sideslip state (β), so the two
  lateral-directional modes collapse into one roll-settle proxy that cannot
  distinguish an underdamped **Dutch roll** (oscillatory, settles — passes)
  from a real one, nor a slow **spiral divergence** (passe the 5 s window
  while having a short T₁/₂).  Added a complementary linearised diagnostic.
- **Method — 4-state small-perturbation model** (β, p, r, φ) about level
  cruise:
    β̇ = (Yβ/mV)β − r + (g/V)φ
    ṗ = (Lβ/Ixx)β + (Lp/Ixx)p + (Lr/Ixx)r
    ṙ = (Nβ/Izz)β + (Nr/Izz)r
    φ̇ = p
  All derivatives feed from the SAME descriptor data as `simulate_freeflight`:
  L_β from the shared `_effective_dihedral()` C_lβ (extracted refactor, no
  behaviour change), N_β from `yaw_stability`, L_r from `dihedral_coeff`,
  damping as an **equivalent-linear** term at a representative oscillation
  amplitude `LIN_EQ_RATE = 0.25 rad/s` (the sim's quadratic damping is
  amplitude-dependent, so linearising at a typical |rate| keeps the mode model
  consistent with the non-linear sim's operating point rather than
  artificially damping-free).  Fin sideforce (Y_β) and yaw damping (N_r,fin)
  estimated from `rudder_area`/`rudder_arm` with fin lift-slope
  `A_V_FIN = 2.5` (/rad, typical RC fin AR≈1–2; documented assumption, same
  class as the pitch-stability CG input).  N_p (roll→yaw) deliberately 0 —
  the non-linear sim has no such coupling either.
- **Verdict gates** (wide-margin, screen-only — no tuning feedback):
    Dutch roll: FAIL if damping ratio ζ < 0.05
    Spiral:     FAIL only when actually unstable AND T₁/₂ < 8 s
    Roll subsidence reported as τ (informational).
- **Numerics — why closed-form**: Durand–Kerner iteration *stalls into
  non-root fixed points whenever all four roots are real* — the common case
  for this fleet — and Python's `**(1/3)` returns the complex principal cube
  root of a negative real (breaking Cardano's real-root selection).  Both are
  reasons the fleet-wide solver is the **closed-form Ferrari solution**
  (depression → resolvent cubic → two quadratics) with a sign-preserving
  `_cbrt`.  Validated: `det(A − λI) ≤ 7e-12` for every λ across the fleet and
  `a₄ = det(A)` to machine precision.
- **Fleet results (no gate trips — screen consistent with all-PASS free-flight)**:
  formula             DR ζ     T(DR)   spiral T½  roll τ   EDA
  SkySurfer_Bixler    0.40    2.0 s    0.6 s      0.8 s    2.9°
  Shadow              0.49    1.5 s    0.4 s      0.5 s    4.5°
  Shadow2 (plank)     0.47    1.9 s    0.4 s      0.5 s    2.0°
  Horten              0.45    1.5 s    0.8 s      1.1 s    4.1°
  Arado 555           0.46    1.7 s    1.8 s      2.6 s    6.3°
  Delta               0.43    2.0 s    0.5 s      0.7 s    2.9°
  Dragon              0.43    1.9 s    1.1 s      1.6 s    2.9°
  Elevon              0.34    2.1 s    1.7 s      2.4 s    2.9°
  RudderElevator      0.41    2.2 s    2.4 s      3.5 s    2.9°
  Radian              0.46    2.2 s    1.2 s      1.8 s    2.9°
  Spoileron           0.35    2.3 s    3.8 s      5.5 s    2.9°
  SmallSpoileron      0.25    1.7 s    3.6 s      5.2 s    2.9°
  (original/Phoenix   0.32    1.6 s    0.6 s      0.7 s    2.9°)
  All Dutch rolls well-damped, all spirals stable; general trend — swept/high-EDA
  frames gain DR damping but the low-EDA planks (Spoileron/SmallSpoileron)
  show the weakest spiral/roll-subsidence (longest T½/τ), the fleet's only
  lateral-directional concern and consistent with their free-flight margins.
- **Decision (user): proceed with implementation.  All descriptors /
  .af data contracts untouched** — the analysis reads the existing surface
  descriptors only (`A_V_FIN`/`LIN_EQ_RATE` are solver assumptions, not
  parameters).  `< 40 lines of new solver`, no GCS/FC change, no FC build.
- **Verification**: `python3 -m py_compile test_pid_sim.py` clean; full fleet
  suite → still exactly ONE FAIL (the accepted SmallSpoileron CONS pitch@30°);
  worst eigen residual 7e-12; machine-verified a₄ = det(A).

## TODO (updated)
- ~~Dutch-roll / spiral recovery diagnosis~~ → **implemented Aug 29** (above):
  linearised 4-state lateral modes with closed-form quartic root finder; gates
  dormant for the current fleet pending a frame or re-sweep that trips them.
  From here, only tuning-worthy follow-up: nothing in-fleet needs it; revisit
  only if a new swept/low-EDA frame joins the fleet.