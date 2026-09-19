# Session Report — Dive Mode Flare-Lead (Aug 28)

## Objective
Close the GPS-lost baro-floor hazard in the dive mode: when the timed pitch
profile overshoots (real sink > the 12 m/s model, GPS gone so Doppler
supervision has frozen), the old last-resort baro floor forced the flare *at*
`pDiveRecoverAlt` — but a flare bleeds sink while the pitch ramps to level, so
the aircraft continued 15–30 m past the floor and hit terrain.

User decision (this session): implement a GPS-free baro-ROC-derived **flare
lead** so the ramp *ends at* the floor; keep the existing auto spiral-land on
GPS loss (already implemented, unchanged).

## Change
`UAVXArmQ/src/dive.c` only.

### 1. New helper `DiveFlareLead()` (dive.c:107–121)
```c
static real32 DiveFlareLead(void) {
	return Max(-KFROC, cDiveExpectedSink) * (cDiveFlareS * 0.5f);
}
```
- Bleed model: sink decays ~linearly to zero over the `cDiveFlareS = 5 s`
  flare (pitch ramps cruise→level at fixed 0.55 throttle), integral ≈
  `s·T/2` (triangle).
- Sink source: `-KFROC` — the baro+accel KF vertical velocity that already
  *authorises* the spiral descent (dive.c vertical authority), so the same
  trust model extends to the flare. GPS-free by construction (baro+accel).
- Bias LONG, never short: `Max(·, cDiveExpectedSink)` floors the assumed sink
  at 12 m/s, giving a **30 m minimum lead** even if KFROC reads thin/lagged.
  Rationale: over-lead is survivable (flare ends above the floor → recover
  phase climbs back to `DiveTargetAlt`), under-lead is a crash.
- 30 s hard profile ceiling (`cDiveMaxProfileS`) still bounds the whole dive.

### 2. Baro-floor flare gate (dive.c:228–234)
Replaced `RawBaroAltitude < pDiveRecoverAlt` with
`RawBaroAltitude < pDiveRecoverAlt + DiveFlareLead()`. Abort path (switch
release) untouched — pilot-commanded flare instantly is still correct.

## Rationale / discourse
- **Why KFROC and not GPS.velD:** the whole point of this gate is the
  GPS-lost fire-and-forget case; Doppler has frozen (`DiveGPSLost` latch).
  KFROC is the only trusted GPS-free sink source, and it is already the
  authoritative vertical signal in the approved spiral-descent fallback.
  Using it keeps one trust model for the entire landing authority.
- **Why a fixed 30 m floor:** a baro-lagged KFROC during a 45° powered dive
  reads low; trusting it would *shrink* the lead exactly when it matters.
  `cDiveExpectedSink·T/2 = 30 m` is the bleed at the model's own assumed
  cruise sink, so the gate can never fly the flare later than the schedule
  itself assumed. Symmetric failure analysis: too-early flare costs nothing
  (level off above floor, throttle up, recover), too-late flare is unrecovered.
- **Rejected:** hooking the lead into `SuperviseDiveProfile` (Doppler branch)
  — that path is already rescaled and is GPS-only; the baro gate exists
  precisely when supervision cannot run. Rejected adding it to the abort
  path — pilot release is an emergency takeover, holding the flare early
  would steal altitude authority from the pilot.
- **No new params** — bounds/scale work from this morning's sweep stands
  untouched; no GCS change required.

## Verification
- `python3 UAVXArmQ/scripts/fc_build.py` — clean compile+link of both
  `SPEEDYBEEF405WING` and `FLYINGRCF4WINGMINI` targets.
- `dive.o` rebuilt and relinked; `SPEEDYBEEF405WINGQ_r0.bin` (228556 B)
  and `FLYINGRCF4WINGMINIQ_r0.bin` (228788 B) regenerated 16:47.
- No GCS source touched → no `py_compile` required.

## Next / open
- Field tune: verify the 30 m fixed floor against actual cruise sink on the
  bench/sim (only the MR class is gated into dive, rc.c:1020).
- Unresolved interactions for commissioning (AGENTS TODO): `DiveRecoverAlt`
  vs `VRSROC` pre-flare recovery and `MaxDescentRateDmpS`/`DescentDelayS` —
  still to be commissioned as one system, not in isolation.