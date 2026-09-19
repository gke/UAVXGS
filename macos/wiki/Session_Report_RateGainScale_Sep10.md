# Ch10 Potentiometer → Master Rate-Loop Gain (2026-09-10)

## Change
Ch10 — the dormant `Map[eTransitionRC]` slot, tag 93 `RxAux5Ch`, default ch10 —
is now a **potentiometer-driven master scale for the RATE loops**. The angle
loop is untouched (its tuning is trusted). All three rate controllers scale
their output by `RateGainScale` before `conditionOut`.

### FC (`UAVXArmQ/src/`)
- `tune.c:24` / `tune.h`: new `real32 RateGainScale = 1.0f;` (next to the dead
  `TuningScale`, deliberately NOT reusing it).
- `rc.c:1069` (`UpdateControls`, inside `if (RCNewValues)`):
  ```c
  RateGainScale = ActiveCh(eTransitionRC)
          ? powf(4.0f, 2.0f * RC[eTransitionRC] - 1.0f) : 1.0f;
  ```
  Decoded once per RC frame, like the other channels; default 1.0 when the
  channel is dead/unwired.
- `control.c` — output scaled (P+D together, whole correction):
  - `ControlRate()` line 479: `A[a].Out = -conditionOut((R->PTerm + R->DTerm) * RateGainScale);`
  - `ControlRateYaw()` line 492: same form.
  - `DoRateDampingControl()` line 511: `A[a].Out = -conditionOut((R->PTerm + Stick * R->Max * R->Kp) * RateGainScale);` (stick term included, so the FW rate-mode/auto damping scales uniformly).
- `params.c:248`: tag-93 comment documents the new role.

### GCS (`uavx-python/src/`)
- `parameter_window.py:635`: RC-map spin label "Transition" → **"RateGain"**.
- `eTransitionRC` enum name untouched (`protocol_enums.py:138`) — wire format
  and protocol unchanged.

## Mapping (user-specified anchors)
| pot | scale |
|-----|-------|
| 0.0 (min) | 0.25 |
| 0.5 (centre) | 1.0 |
| 1.0 (max) | 4.0 |

Natural curve through the three anchors: **`scale = 4^(2·pot−1)`**
(= `2^(4·pot−2)`): 0.25→4⁻¹, 0.5→4⁰, 1.0→4¹. Smooth at centre, symmetric
multiplicative (equal gain factor per pot arc). Midpoints: 0.75→2.0, 0.25→0.5.

## Rationale / discourse
- **Why rate loops only.** The rate-loop gains are what get field-tweaked on a
  new airframe; the angle loop is proven tuning and the pilot trusts it. A
  master knob on the rate loops lets the first flights bracket the gain without
  flashing/loading gains repeatedly.
- **Why scale P and D together.** Multiplying the whole correction
  `PTerm + DTerm` preserves the P:D ratio → the rate-loop damping shape is
  unchanged and the effective loop gain scales (bandwidth up/down, same relative
  damping). Scaling P-only would re-balance D against P and destabilise at 4×.
- **Why an exponential curve.** A linear map through (0.25 → 1.0 → 4.0) has a
  slope discontinuity at the centre and asymmetric per-arc authority; the power
  curve is the natural one-parameter curve through three points that keeps
  centre=1.0 exactly, is smooth at centre, and gives equal fractional-change
  authority per pot arc (0.25→0.5 is a 2× gain step, 0.5→1.0 is a 4× step, but
  each is a full half-travel arc — which is what a pot-arc intuitively means).
- **Why a NEW variable, not the dead `TuningScale`.** The inflight-autotune plan
  (AGENTS TODO) reserves `TuningScale` as a paced ~0.5 s ramp for gain applies —
  an orthogonal semantic. Reusing it would double-apply the autotune ramp with
  this pot later.
- **Why once-per-RC-frame, and defaulting to 1.0.**
  - Once per frame (RCNewValues) so a clean RC wander doesn't rescale at 500 Hz
    control rate; 50 Hz RC is far below the loop rate and a pot is DC-like.
  - `!ActiveCh(eTransitionRC)` → 1.0: an unwired/dead channel (default
    `DiscoveredRCChannels` < `Map[eTransitionRC]`) is a no-op — identical to the
    pre-change behaviour, so boards that never assign ch10 are bit-identical.
  - Failsafe presets (`FS[Map[c]].Raw = 1500`, `rc.c:826`) put a failed-sense
    channel at pot 0.5 → scale 1.0: benign.
- **Why apply inside the controllers before `conditionOut`.** The output is
  clamped by `conditionOut` (`Limit1(v, 1.0)`); scaling before the clamp means
  the commanded correction is scaled and THEN authority-limited — the only
  honest way for a "gain" (a 4× pot pulls harder into the ±1 authority limit,
  which is exactly what more gain should do). Scaling after the clamp would
  merely reduce a clamped command.
- **Beneficial side effects.** `A[a].Out` is what the Trace viewer records and
  what the emu plant consumes (`emu.c:436,440,487,502,524` use `A[a].Out`
  directly). The scaled demand is therefore visible in Trace captures and in
  emulated flights with zero extra plumbing — a bench way to verify the curve
  (drive ch10 in the emulator, watch `Out` scale).
- **Applies to MR and FW alike** — all three rate functions
  (`ControlRate` MR roll/pitch, `ControlRateYaw` MR yaw, `DoRateDampingControl`
  FW roll/pitch + FW yaw via the conditional call path). One knob, both
  categories.
- **Slot is free.** `eTransitionRC` is consumed only by
  `VTOLMode = Select2(eTransitionRC) && (pAFType == eVTOLAF) && !F.PassThru`
  (`rc.c:1065`), and no `eVTOLAF` airframe flies, so sharing the channel is
  harmless. Returns to VTOL-transition duty the moment a VTOL airframe appears
  (that path is unchanged).

Alternatives considered/rejected:
- **Reuse `TuningScale` as the carrier** — rejected (autotune ramp semantic
  collision, above).
- **Scale by a linear pot map** — rejected (slope discontinuity at centre,
  asymmetric authority).
- **Scale only the P term** — rejected (alters D:P balance, unstable at 4×).
- **Scale the RC `Stick` term separately in `DoRateDampingControl`** —
  rejected: the stick feed-forward is part of the rate correction; scaling the
  whole thing keeps manual and auto authority consistent (per the DriveSigns
  convention that manual and autonomous drives must share sense/scale).

## Verification
- All 7 FC targets build clean via `scripts/fc_build.py`
  (UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI,
  BLUEBERRYF405, MATEKF411WING). Artifact:
  `obj/SPEEDYBEEF405WING/SPEEDYBEEF405WINGQ_r0.bin` (233796 bytes).
- GCS `python3 -m py_compile src/ui/parameter_window.py src/protocol_enums.py`
  — OK.
- No `.af` round-trip change (tag 93 wire format untouched) and no param
  limits/config magic change (tag 93 bounds 0..15 unchanged; it is a channel
  index, not a gain).

## Next for the user
- Rebuild+reflash SPEEDYBEEF405WING; assign ch10 to a potentiometer on the Tx
  model that flies the airframe you want the master gain on.
- For first flights: start pot 0.5 (scale 1.0) and sweep; expect ~0.25–1.0 for
  a nervous first flight, 1.0–4.0 to feel authority up to the ±1 clamp.
- Bench-verification option: run the emulator with ch10 exercised or drive
  `RateGainScale` via a Trace capture — `Out` will show the scaled demand.