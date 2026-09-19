# Legacy Scaling Map

**Status:** living reference — keep this in sync whenever a param gains/loses a
`LEGACY_TAGS` membership or a `PARAM_SCALES` value in
`uavx-python/src/parameters.py`. Do **not** extend or remove entries on a whim;
they are the only bridge between the historical integer-encoded UAVX parameter
space and today's raw `float32` values.

**Origin (2026-08-28 session):** extracted and cross-verified from
`UAVXArmQ/originalparams.c` (pristine U/V3.15-era UAVX source, Greg Egan) and
confirmed identical in the classic-era `Retired/UAVX Collection/UAVXArm32F4RB/src/params.c`.
The per-param "scalings" are the **apply-time multipliers** the legacy firmware
used to turn the integer `P(...)` parameter into the physical raw float, e.g.
`Nav.PosKp = (real32) P(NavPosKp) * 0.0165f;  //20 -> 0.33f;`.

**Why this must never be lost:** every legacy airframe baseline, SimFlight gain
table and old tuning note is expressed in the old integer encoding (e.g. NavPosKp
"20"). The current FC stores raw floats only. `PARAM_SCALES` recovers the old
encoding from a raw value; `PARAM_DISPLAY_MULT` converts raw to today's display.
Without the map the "20" ↔ `0.33f` relationship is unrecoverable.

## Conversions

- **Unified display (live paths, `.af`, FC):** `display = raw * PARAM_DISPLAY_MULT[idx]`
- **Legacy count (dormant Legacy checkbox, off by default):** `legacy = raw / PARAM_SCALES[idx]`
- **Raw from legacy count:** `raw = legacy * PARAM_SCALES[idx]`

Note `PARAM_SCALES[idx] == 1` ⇒ legacy view shows the raw value unchanged; the
entry is tagged only so it participates in the same toggle.

## Master table (25 legacy-tagged indices)

Sources: `LEGACY_TAGS`/`PARAM_SCALES`/`PARAM_DISPLAY_MULT`/`PARAM_DEFAULTS` in
`parameters.py`; armQ ParamTable `.scale` (`params.c`) is the FC-side mirror.
`orig` column = motivating `originalparams.c` multiplier (raw lender per counted
integer step); `legacy default` = `FC default raw / scale`.

| idx | Param | Type | mult | scale | FC dflt (raw) | display (raw) | legacy dflt | orig (originalparams.c) |
|----:|-------|------|-----:|------:|--------------:|--------------:|------------:|--------------------------|
|  0  | RollRateKp | FLOAT | 1 | 0.005 | 0.3 | 0.3 | 60 | `ScaleRateKp=0.005f` |
|  1  | AltPosKi | FLOAT | 1 | 0.00046 | 0.001 | 0.001 | 2.174 | `*0.00046f` (10→0.0046) |
|  4  | RollAngleQIntLimit | FLOAT | 1 | 0.015 | 0.01 | 0.01 | 0.667 | `DEG_RAD*ScaleAngleIL`, `ScaleAngleIL=0.015f` |
|  5  | PitchRateKp | FLOAT | 1 | 0.005 | 0.45 | 0.45 | 90 | `ScaleRateKp` |
|  6  | AltPosKp | FLOAT | 1 | 0.0183 | 0.35 | 0.35 | 19.13 | `*0.0183f` |
|  9  | PitchAngleQIntLimit | FLOAT | 1 | 0.015 | 0.01 | 0.01 | 0.667 | `DEG_RAD*ScaleAngleIL` |
| 10  | YawRateKp | FLOAT | 1 | 0.005 | 0.75 | 0.75 | 150 | `ScaleRateYawKp=0.005f` |
| 11  | RollRateKd | FLOAT | 1 | 0.0001 | 0.015 | 0.015 | 150 | `ScaleRateKd=0.0001f` |
| 18  | RollCamKp | FLOAT | 1 | 0.1 | 1.0 | 1.0 | 10 | `Cam.RollKp=P(RollCamKp)*0.1f` |
| 23  | RollAngleQKi | FLOAT | 1 | 0.05 | 0.0125 | 0.0125 | 0.25 | `ScaleAngleKi=0.05f` |
| 24  | PitchAngleQKi | FLOAT | 1 | 0.05 | 0.0125 | 0.0125 | 0.25 | `ScaleAngleKi` |
| 25  | PitchCamKp | FLOAT | 1 | 0.1 | 1.0 | 1.0 | 10 | `Cam.PitchKp=P(PitchCamKp)*0.1f` |
| 27  | PitchRateKd | FLOAT | 1 | 0.0001 | 0.0225 | 0.0225 | 225 | `ScaleRateKd` |
| 28  | NavVelKp | FLOAT | 1 | 0.06 | 0.5 | 0.5 | 8.333 | `Nav.VelKp=P(NavVelKp)*0.06f` |
| 29  | AltVelKd | FLOAT | 1 | 0.001 | 0.16 | 0.16 | 160 | `Alt.R.Kd` (dir~`*0.001f`) |
| 31  | MadgwickKpMag | FLOAT | 100 | 0.01 | 0.005 | 50 | 0.5 | `KpMag=P(MadgwickKpMag)*0.01f` |
| 40  | NavPosIntLimit | FLOAT | 1 | 1 | 5 | 5 | 5 | `legacy NavPosIntLimit` (m/s, count) |
| 48  | NavCrossTrackKp | FLOAT | 100 | 0.01 | 0.0004 | 4 | 0.04 | `Nav.CrossTrackKp=P(...)*0.01f` |
| 56  | NavPosKp | FLOAT | 1 | 0.0165 | 0.25 | 0.25 | 15.15 | `*0.0165f` (20→0.33) |
| 60  | NavPosKi | FLOAT | 1 | 0.004 | 0.02 | 0.02 | 5 | `*0.004f` (5→0.02) |
| 66  | UnusedAltVelIntLimit | FLOAT | 1 | 0.05 | 0.5 | 0.5 | 10 | `Alt.R.IntLim=P(...)*0.05f` |
| 90  | YawRateKd | FLOAT | 1 | 2.5e-05 | 0.0375 | 0.0375 | 1500 | `ScaleRateYawKd=0.000025f` |
| 97  | YawAngleQKi | FLOAT | 1 | 0.05 | 0.0125 | 0.0125 | 0.25 | `ScaleAngleYawKi=0.05f` |
| 98  | YawAngleQIntLimit | FLOAT | 1 | 0.000872665 | 0.03 | 0.03 | 34.38 | `DEG_RAD*ScaleAngleYawIL`, 0.05f |
| 99  | AltPosIntLimit | FLOAT | 1 | 0.05 | 0.5 | 0.5 | 10 | `Alt.P.IntLim=P(...)*0.05f` |
| 101 | AltROCKi | FLOAT | 1 | 0.00027 | 0.001 | 0.001 | 3.704 | `Alt.R.Ki=P(AltVelKi)*0.00027f` |

## Worked examples (old encoding ↔ raw)

- NavPosKp legacy "20" ↔ raw `20*0.0165 = 0.33f` (classic comment `//20 -> 0.33f;`).
  The FC default raw 0.25 ↔ legacy "15.15" (tuned airframes sit ≈13–20).
- NavPosKi legacy "5" ↔ raw `5*0.004 = 0.02f` (classic comment `// 5 -> 0.02f;`).
- YawRateKd legacy "1500" ↔ raw `1500*0.000025 = 0.0375f`.
- MadgwickKpMag: raw 0.005 ↔ `0.005*100 = 50` (percent-style display, mult 100) and
  legacy `0.005/0.01 = 0.5`. Both views coincide numerically because
  `mult = 1/scale` = 100 for this pair — same for NavCrossTrackKp (raw 0.0004:
  display 4, legacy 0.04 — NOT equal: mult 100, scale 0.01, rad/percent ratio differs).
  (For tag 31 the two agree by coincidence; for tag 48 they don't — do not assume.)

## Known idiom / asymmetries (documented, do not "fix")

1. **Angle IntLimits:** original expression folded a `DegreesToRadians(...)` factor
   into the legacy value for Yaw only — tag 98's scale is `DEG_RAD*0.05` (recovers the
   integer count), while tags 4/9 use the bare `0.015` (their legacy view shows
   `DEG_RAD*count`, not the integer). This asymmetry matches `originalparams.c`
   lines 174/188 (`DegreesToRadians(P(...))*ScaleAngleIL`) vs 214
   (`DegreesToRadians(P(...)*ScaleAngleYawIL)`). Preserve as-is.
2. **Tags 31/48** carry `display_mult=100` while being legacy-tagged; the Legacy
   checkbox only alters the lhs of a spinbox in legacy mode — it does **not** touch
   write/`.af`/FC paths (those always use `PARAM_DISPLAY_MULT`).
3. **NavPosIntLimit (tag 40)** is tagged with scale 1 (pure m/s count in both views).

## Scope

- Kept dormant by design (2026-08-28): GCS `_legacy_mode` checkbox off by default.
- Do **not** remove; do **not** extend. The FC ParamTable `.scale` column mirrors
  these values — edit only `min`/`max` limits, never `.scale`.
- The entry for `MadgwickKpMag` above uses default raw 0.005; armQ may carry a
  different tuned default — the *scale* is the invariant, not the default.

## Source pointers

- `UAVXArmQ/originalparams.c` — apply-time legacy multipliers (lines 143–235).
- `Retired/UAVX Collection/UAVXArm32F4RB/src/params.c` — identical multipliers (2015-era pristine).
- `Retired/UAVX Collection/UAVXGS/Parameters.cs` — stub form only; the original
  C# GUI's display-scale metadata (e.g. `15 * 0.0165`) was not recovered from this
  checkout. Where a GCS scale relies on GUI-side (non `originalparams.c`)
  information, that source is currently unverifiable — flagged for future backfill.
- `UAVXArmQ/src/params.c` — current ParamTable `.scale` = FC mirror of this table.