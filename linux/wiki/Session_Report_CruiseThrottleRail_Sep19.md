# Session Report — Cruise Throttle Rail: Ungated `TrackCruiseThrottle` Bottoms Out the Feedforward

**Date:** 2026-09-19
**Build/verify status:** FC clean — all 9 targets rebuilt via
`python3 scripts/fc_build.py BOARDS_ALL`, 0 errors (only the pre-existing
"LOAD segment with RWX permissions" linker note); fresh `_r0.bin` for every
board (SPEEDYBEEF405WING 236252 B, MATEKF411WING 235788 B, UAVXF4V3,
UAVXF4V4, DEVEBOXF4, FLYINGRCF4WINGMINI, BLUEBERRYF405, MATEKF405TE,
DISCOVERYF4). Sizes +96 B vs the 2026-09-18 build — the health-gate constants
(control.c) and the emu FF seed block. GCS unchanged this session (FC-side
only). Emu verification re-run pending (Greg on the bench).

---

## Problem (why this session)

`20260918_163941.rawlog`, the 2026-09-18 emulated quad 4-WP mission, showed an
MR that could **never climb under navigation**: the WP-1 leg ran 62.9 → 0 m at
−3.8 m/s, altitude stuck at 0, while `Config.CruiseThrottleFF` drained
0.278 → 0.200 (about −0.02 every ~4 s) and **pinned at the `cThrMinAltHoldStick`
0.20 floor**.

Consequence chain:

```
FF floor (0.20)  ⇒  max nav throttle = FF(0.20) + AltHoldThrComp(≤ 0.25)
                 = 0.45  <  0.549 (quad hover fraction)
        ⇒  nav can never generate positive climb
        ⇒  WP altitude window never closes
        ⇒  mission stalls with altitude pinned at 0
```

The pattern contradicts the design intent of `TrackCruiseThrottle`: a stable
hover should hold the baseline near the true hover demand (quad model hover =
Mass·g/MaxThrust = 0.8·9.80665/14.3 = **0.549**), not walk it to 0.20.

---

## Root cause analysis (and why the lead file at line 229 was wrong)

The AGENTS.md "lead" (2026-09-18) suspected a *state/RC-source* fault: `eLanded`
takeoff gating on `StickThrottle >= pIdleThrottle` and the MR `eInFlight →
eLanding` exit on `StickThrottle < cRcThresStartStick`. That lead was **not
the cause** — the aircraft was genuinely in `eInFlight`/`eTransiting` with GPS,
and alt was merely never commanded up (nav throttle too weak). The real fault
was in the cruise-throttle *learner*.

### The drain mechanism

`TrackCruiseThrottle` (r15+ v2) learned whenever
`F.HoldingAlt && !F.ThrottleMoving`, and `ThrottleMoving` is **delta-based**
(`CheckThrottleMoved`, rc.c:928-939, window 0.02). Two consequences:

1. A **held** manual-throttle stick reads "not moving" — so during a manual
   climb the learner is *active*, not gated off. (The original-intent caveat
   "prevents tracking when stick off-center" was false: a centred-and-held
   stick is indistinguishable from a hands-off neutral.)
2. During that manual climb `AltHoldThrComp` **saturates at its −0.20 limit**.
   A clipped correction is NOT the equal of the loop's desire — it is
   "at-least-limit". Integrating a pinned-at-rail value walks the baseline to
   the rail (0.20 floor), which is exactly what the q every ~4 s shows.

The legacy reference (trusted preserve `/media/doomsday/UAVXArm32F4Preserve`,
`control.c:134-151`) was never exposed to this: it gated learning on
`F.Hovering = (fabs(ROCTrack) < AltitudeHoldROCWindow) &&
(DesiredThrottle > THR_MIN_ALT_HOLD_STICK)` plus `!F.Emulation` — the r15+
rewrite dropped both the hover-quality gate and the emulation exclusion while
keeping the similarly-named `ThrottleMoving` gate that does not actually
mean "hovering".

---

## Fix (health-gated v3)

`control.c` `TrackCruiseThrottle`:

```c
void TrackCruiseThrottle(real32 dT) {
    if ((State == eInFlight) && !F.Emulation
            && (Abs(ROCTrack) < cCruiseTrackRocMPS)
            && (Abs(AltHoldThrComp) < pMaxAltHoldThrComp
                    * cCruiseTrackCompFrac)) {
        Config.CruiseThrottleFF += Limit1(AltHoldThrComp,
                               cCruiseTrackingRate * dT);
        Config.CruiseThrottleFF = Limit(Config.CruiseThrottleFF,
                             cThrMinAltHoldStick,
                             cThrMaxAltHoldStick);
        ConfigChanged = true;  // RAM-only; flash written by RefreshConfig
                                       // when out of flight
    }
}
```

New constants beside `cCruiseTrackingRate` (0.02f):

- `cCruiseTrackCompFrac = 0.9f` — learn only while `AltHoldThrComp` is under
  90% of its authority limit (deadband margin against the rail).
- `cCruiseTrackRocMPS = 1.0f` — learn only while actually hovering
  (|ROCTrack| < 1 m/s).

Learns only when genuinely hovering: `eInFlight`, **not emulating**, ROC inside
the settled window, correction not pinned at its limit. This restores the
legacy `F.Hovering` semantics while keeping the v2 direct-`AltHoldThrComp`
tracking and the 2 %/s rate (`CRUISE_TRACKING_RATE`).

### Plus an emulator seed (the recovery problem)

The gated tracker alone **cannot recover** an already-floored persisted value:
with FF pinned at 0.20, hover is impossible, so `AltHoldThrComp` stays pinned
at its limit, so the gate blocks learning forever — a deadlock. Options
considered:

- **Re-seed from model at emulation boot** (ADOPTED): `InitEmulation` sets
  `Config.CruiseThrottleFF` from the model — `md->Mass · cGravityMpsS ·
  (1/md->MaxThrust)` (0.549 quad), or `md->CruiseThrottle` for FW. RAM-only:
  `ConfigChanged` deliberately NOT set, no flash write from emu boot. This
  restores the "50% in emulation" statement that AGENTS.md had documented but
  `InitEmulation` had never actually implemented.
- **Gate `!F.Emulation`** (also ADOPTED inside the tracker): the emulator seed
  is the source of truth for bench runs; a bench flight must not re-corrupt
  the persisted cruise throttle by tracking the bench's (model-perfect) hover.
- **Bump the `.af` cruise baseline** (REJECTED): the `.af` slot is `UNUSED_20`
  (NULL-targeted) and dead on the FC; resurrecting it re-opens the legacy
  scaling mess Tuneeval officially retired. Cold config seed remains 0.55 in
  `UseDefaultParametersEx`.
- **Raise `cThrMinAltHoldStick`** (REJECTED): raising the floor is a band-aid
  on the learner's failure, masks the real bug, and inflates feedforward for
  light frames.

---

## Verification / next steps

- **Build:** clean, all targets, sizes as listed above.
- **Greg on the bench:** reflash SPEEDYBEEF405WING, re-run the emu quad
  mission. Expect:
  - `Config.CruiseThrottleFF` to seed at 0.549 (tag-57 / GCS identify or
    tag-13 `cruise_throttle` field),
  - nav to close the WP distance / climb under nav (was pinned at 0),
  - and NO decay during the flight (gate prevents rail-walking).
- **Open item (carry):** after altitude is confirmed fixed, re-assess the
  RTH switch reliability item (settle/edge logic, `eReturningHome` never set)
  — deliberately deferred until the altitude/FF state is proven.

---

## Cross-references

- AGENTS.md "Cruise Throttle as Feedforward" and "Cruise Throttle Feedforward
  v2 (r15+)" — updated to health-gated v3.
- AGENTS.md TODO "Multirotor does NOT climb under emulation" — updated to
  ROOT CAUSE FOUND & FIXED (re-verify pending).
- Legacy gate: `/media/doomsday/UAVXArm32F4Preserve/src/control.c:134-151`.
- Dump analysed: `20260918_163941.rawlog` (decoder `core/frame_decoder.py`).