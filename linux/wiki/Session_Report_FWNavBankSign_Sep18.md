# Fixed-Wing WP Navigation — Inverted Bank-to-Turn Sign

**Date:** 2026-09-18
**FC:** `UAVXArmQ/src/nav.c` (`Navigate`)
**Status:** Fixed, all 7 targets build clean (sizes unchanged — pure sign flip). Awaiting reflash + re-fly.

---

## 1. Symptom

FW (Shadow, 4 uploaded WPs) lifts off, enters `eTransiting`, and then flies
around without ever acquiring a waypoint. On the raw capture
`20260918_095428.rawlog` (446 732 B, 4852 frames, span 157.6 s):

- `curr_wp` stayed **1** for the entire flight.
- `cross_track_error` grew **monotonically 0 → +548 m**.
- `nav_state` 5 (`eTransiting`) from 46.1 s until 152.8 s, then `ePIC`.
- The aircraft banked within limits (`|Angle[Roll]|` reached ~18°) and never
  tumbled — i.e. the attitude loop and roll authority were healthy. It simply
  steered the wrong way and circled.

## 2. Heading references were self-consistent (ruling out mag/estimator)

`Heading` (tag-13), `gps_heading` (tag-14 offset 33, `Make2Pi(GPS.heading)`)
and `mag_heading` all agreed to a few degrees (constant offset = declination),
and `Heading` matched the GPS course derived from position deltas. The Sep-18
mag true-attitude fix therefore did not regress or cause this.

Note: `north_pos_e` / `east_pos_e` in tag-14 are **position errors**
(`Nav.C[].PosE = DesPos − Pos`), not absolute positions — this resolves the
earlier apparent ~180° confusion. `Nav.WPBearing = atan2(PosE_East,
PosE_North)` is the true bearing *to* the WP.

## 3. Root cause — inverted heading error in the FW bank command

`nav.c` `Navigate()`, FW branch:

```c
Nav.DesiredHeading = WindCrabHeading(Nav.WPBearing);
Turn = MakePi(Heading - Nav.DesiredHeading);        // <-- inverted
C = (Turn * Nav.MaxBankAngle) / Nav.HeadingTurnout;
A[eRoll].DesiredNavCorr = Limit1(C, Nav.MaxBankAngle);
```

`A[eRoll].NavCorr` slews to `DesiredNavCorr` (`control.c:70`) and feeds
`A[eRoll].P.Desired = stick*P.Max + NavCorr` (`control.c:598`). This is the
only consumer of `NavCorr`; `DesiredNavCorr` has no other FW setter.

**Verified roll convention (from the flight itself):** positive roll angle
increases heading (right turn), negative roll decreases it. Stick-neutral
segments:

| t (s) | `NavCorr` | heading | WP bearing | shortest turn needed | commanded |
|---|---|---|---|---|---|
| 57.9–60.0 | −20° | 16.7° → 11.8° (left) | ~188° | +171.7° (right) | **left — wrong** |
| 62.2–64.4 | +20° | 1.9° → 11.4° (right) | ~194° | −168.7° (left) | **right — wrong** |

Both cases command the **long way round**; the sign of `MakePi(Heading −
DesiredHeading)` is the negation of the required bank. `MinimumTurn` (nav.c:165)
and `NavYaw` already use `MakePi(Desired − Heading)` for yaw, and Greg confirmed
the manual sense: right stick rolls right and the compass heading increases.

## 4. Fix

```c
Turn = MakePi(Nav.DesiredHeading - Heading);
```

One line. Positive `Turn` now = heading must increase = bank RIGHT = positive
roll, matching the yaw convention and the airframe's measured roll sense.

## 5. Why the legacy reference is *not* a contradiction

The trusted legacy preserve
`/media/doomsday/UAVXArm32F4Preserve/src/nav.c:289` and `outputs.c:132`
(`Rl = Limit1(A[Roll].Out, ...)`) are byte-identical here. So legacy FW WP-nav
either was never actually validated, or relied on an opposite aileron/servo
sense. This is a **fork-forward correction, not a regression to bisect** — the
fork's roll convention is confirmed correct by the stick.

## 6. Consistency with the emulated "opening WP distance"

This also explains why the emulated 4-WP bench mission kept opening the WP
distance even after the 2026-09-16 FW plant authority fix (`Session_Report_
EmuFwAuthority_Sep16.md`): the nav was steering the long way round, so more
plant authority just made it circle faster. Re-run the emu mission to confirm
the distance now closes.

## 7. Verification

- All 7 targets build clean (`python3 UAVXArmQ/scripts/fc_build.py`), bin sizes
  unchanged (sign flip only).
- Not yet flight-verified. **Greg: reflash `SPEEDYBEEF405WING` and re-fly the
  WP mission — expect `curr_wp` to advance and cross-track to close.**
