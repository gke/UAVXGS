# Session Report — FW Climb Throttle Ceiling: Elevon-Class Frames Dead (2026-09-24)

## Summary of Change
Five generic FW airframes shipped `FW_CLIMB_THROTTLE = 0` (tag 21,
`pFWClimbThrottleFrac`), which the mixer treats as a **hard zero-throttle ceiling**
in every non-PassThru (alt-hold / navigation) mode — so those five could never
climb under altitude hold. Set them to **0.7** and added the missing `[LIMITS]`
band (fleet-uniform). Also deleted all dated-attribution `.af` churn files.

| Frame | `FW_CLIMB_THROTTLE` before | after | `[LIMITS]` band |
|---|---|---|---|
| Elevon | 0 | **0.7** | 0.35, 1 |
| Delta | 0 | **0.7** | 0.35, 1 |
| Radian | 0 | **0.7** | 0.275, 1 |
| RudderElevator | 0 | **0.7** | 0.275, 1 |
| Spoileron | 0 | **0.7** | 0.35, 1 |

Already-correct frames untouched: Shadow 0.7, Dragon 0.7, SmallSpoileron 0.7,
SkySurfer_Bixler 0.55 (eAileronAF trainer, different class), MR frames 0 (no FW
ceiling applied to MR).

## Why — the mechanism (audio review, 2026-09-24)
`DoMix`/`UpdateDrives` caps the automatic throttle:
`TempThrottle = Limit(DesiredThrottle + AltHoldThrComp, 0,
cOutMaximum * pFWClimbThrottleFrac)` (`mixer.c:191-201`; legacy-identical in the
trusted preserve `UAVXArm32F4Preserve/src/mixer.c:279-281`). `pFWClimbThrottleFrac`
defaults to 0 (`params.c:176`), so an unset value is a *hard zero-throttle* clamp,
not "no limit". Five frames had never been configured:
- FC default is 0; the five `.af` files carry the untouched default while the four
  value-holders were evidently set when FW alt-hold/nav was actually flown.
- The files are **not corrupt**: the value round-trips, the GCS parses them, and the
  2026-09-14 fleet audit confirmed load→save identity. "0" is a legal, stale value.
- The emulator (`emu.c:472-475`) applies **no ceiling** — `dThrottle` runs uncapped
  `Desired+comp` — so emulated FW hold worked without the parameter while the
  hardware throttle was pinned dead (same emu-vs-mixer divergence family as the
  Sep-18 WP-turn and Sep-19 MR-climb findings).

## Proposed values — rationale (options considered)
1. **Uniform 0.7 (adopted)** — grounded in the value-holders: Elevon↔Shadow/Dragon
   (all `eElevonAF`, 0.7), Spoileron↔SmallSpoileron (both
   `eAileronSpoilerFlapsAF`, 0.7). Headroom: `UNUSED_20` cruise seed 0.45–0.5 →
   ceiling 0.7 leaves 0.2–0.25 = the full `ALT_THROTTLE_COMP_LIMIT` (0.25)
   — in sim terms `pos_limit = min(0.25, 0.7−0.45) = 0.25`, ~4.5–5 s climb.
   Field-converged `Config.CruiseThrottleFF` erodes headroom after
   `TrackCruiseThrottle`, so a larger ceiling is safer, not looser.
2. **Rejected: 0.55 (SkySurfer-class).** SkySurfer (0.55, UNUSED_20 0.2) sits at the
   hairline — sim rise 5.92 s vs 6.0 s criterion, and that is before FF
   convergence eats the margin. Applying 0.55 to cruise-0.45 frames leaves
   `pos_limit = min(0.25, 0.10) = 0.10` → reproduces the SkySurfer hairline.
3. **Parked: re-bias semantics to increment-over-cruise.** A ceiling is the legacy
   semantics (preserve is byte-identical); changing the meaning affects every frame
   and the FC, and is a separate design decision. Note the 0.55-cohort hairline
   shows the ceiling semantics are fragile at low values — revisit if traced.
4. **Bands:** class-matched — plank/spoileron (Elevon/Delta/Spoileron) `0.35, 1`
   (= Dragon/SmallSpoileron), conventional (Radian/RudderElevator) `0.275, 1`
   (= Shadow/SkySurfer). Value-holding frames all carry a band; the five lacked one.

## What Changed
### Deleted (44 dated `.af` files, active trees only)
- `uavx-python/src/airframes/user/*` — 28 dated auto-save frames (Ecks/Shadow/
  SkySurfer_Bixler `*_2026*.af`). NOTE: several differed materially from `generic/`
  (e.g. `SkySurfer_Bixler_20260923_170909.af` = eAileronAF with different PHYS) —
  they were per-session GCS auto-saves, recoverable from git history
  (`git ls-files`); deletion matches the 2026-09-14 "no user saves" precedent.
- `uavx-python/src/Params_2026*.af` + `{linux,macos,windows}/src/Params_2026*.af`
  — 16 dated param dumps.
- Left intact: `airframes_backup_20260912_211844/` safety archive and the
  `gitUAVXGS` local mirror's pre-existing copies (re-sync on next update flow).

### Edited (value + band, canonical + 3 kits + gitUAVXGS = 5 roots)
- `Elevon.af`, `Delta.af`, `Radian.af`, `RudderElevator.af`, `Spoileron.af`:
  `FW_CLIMB_THROTTLE = 0` → `0.7`; appended `FW_CLIMB_THROTTLE = <band>` to
  `[LIMITS]`. Mirrors patched with a one-occurrence assert per file.

## Verification
- Sim (`src/tests/test_pid_sim.py` AH 5 m step, `AH_CRITERIA_FW` rise 6 s / settle
  8 s): Elevon **FAIL→PASS**, Delta/Radian/RudderElevator/Spoileron all **PASS** —
  rise 5.69–5.71 s, settle 7.48–7.52 s, fe < 0.04 m. (Pre-change Elevon: Final
  0.00 m, rise 8 s, fe 5 m — could not climb.)
- The sim enforces the mixer ceiling (`simulate_alt_hold_fw` `pos_limit`, 1508-1511)
  and correctly reproduced the hardware failure; **the emulator does not** (no
  ceiling) — the emu-vs-hardware divergence remains an open item for the FW alt
  path (see notes below).
- No FC source or GCS source changed → no FC build / `py_compile` required.

## Notes / open follow-ups
- **Descent leg missing from the sim AH test**: the 5 m step is climb-only; the
  sim's negative-comp cap (`−min(thr_lim, cruise)`) is never exercised. A descent
  leg would expose the FW descent margin (glide-only) — candidate sim addition.
- **Sim uses the static `UNUSED_20` seed, not the converged `CruiseThrottleFF`** —
  the PASS margin (5.7 vs 6.0 s) is comfortable but does not model FF convergence
  eroding climb headroom on longer flights.
- Emu FW branch should enforce the same `pFWClimbThrottleFrac` ceiling so emulated
  hold is flight-faithful (also the Sep-18/19 emu-vs-hardware family).
- Greg: next bench/flight load of any of the five frames should report climb
  authority now available; verify per-frame `FW_CLIMB_THROTTLE` reads back 0.7 on
  the PID/setup page.

---

# Follow-up Implementation (2026-09-24): Sim Descent Leg + Model Cruise FF + Emu Ceiling + Per-Model Glide Sink

## Item 1 — sim AH descent leg (`test_pid_sim.py`)
`simulate_alt_hold_mr` / `simulate_alt_hold_fw` gained a `descend=True` mode
(same criteria/view): starts at `+step_m`, commands `0 m`, and scores the
**descent-progress** series (`step_m − alt`, still a 0→step_m rise) so the
shared rise/settling/overshoot metrics apply unchanged. This exercises the
NEGATIVE compensation authority — the throttle-floor bound
`−min(thr_lim, cruise_thr)` — the counterpart of the +climb ceiling. A new
"Altitude Hold (5m descent)" block is added to the AH section in
`run_tests_for_af` right after the climb leg.

Result: **all 14 generic frames PASS the descent leg** (6/6 sub-verdicts), MR
rise ≈1.9 s / settle ≤2.4 s, FW rise 5.69–5.92 s / settle ≤7.5 s.

## Item 2 — sim cruise throttle from the model (not the static seed)
The AH sim previously read the legacy `UNUSED_20` (=0.5 nominal) as its
cruise baseline while the emu/flight baseline is the **per-model**
`Config.CruiseThrottleFF`. New `FW_CRUISE_THR[]` in `test_pid_sim.py` mirrors
`emu.c` `EmuModels[].CruiseThrottle` per `FW_MODEL_IDX` (Elevon 0.45,
Aileron 0.35, Delta 0.50, Spoileron 0.35, V-Tail 0.40, RudderElevator 0.35,
VTOL 0.50); `simulate_alt_hold_fw` takes `cruise_thr=None` → model value when
given, else legacy fallback. **Consequence turned up a real seed scatter:**
`generic/Shadow.af` had `UNUSED_20 = 0` (legacy), which under the OLD code gave
zero descent authority (`−min(0.1, 0.0) = 0`) — the descent leg FAILED until the
model cruise (0.45) was threaded in. This is exactly the divergence item 2 was
meant to kill: the static seed is not a trustworthy FF baseline.

Verification: all 14 generic frames PASS climb + descent with model cruise FF.
`py_compile` clean; `gitUAVXGS` source mirror synced.

## Item 3 — emu FW branch enforces the mixer ceiling + per-model glide sink (`UAVXArmQ`)
- **Ceiling mirror:** the emu `eCatFw` branch (`emu.c` `DoEmulation`) now mirrors
  `mixer.c` `DoMotors`' automatic-throttle shaping EXACTLY: `Temp = F.PassThru ?
  DesiredThrottle : (ThrottleSuppressed ? 0 : DesiredThrottle + AltHoldThrComp)`,
  then (auto only) `+= pFWPitchThrottleFFFrac · Abs(Pl)` clamped to
  `[0, cOutMaximum · pFWClimbThrottleFrac]`, then `Limit(Temp·cOutMaximum, 0,
  cOutMaximum)` → `dThrottle`. Previously the emu fed the RAW
  `DesiredThrottle + AltHoldThrComp` uncapped, so emulated FW over-reported
  climb authority vs what the FC would ever command (same emu-vs-mixer family as
  Sep-18/19). Verified all symbols (`ThrottleSuppressed` soar.h:54, `Rl/Pl/Yl/Sl`
  outputs.h:93, `cOutMaximum` params.h:35, `pFWClimbThrottleFrac`/
  `pFWPitchThrottleFFFrac` control.h:107-108) are visible through `UAVX.h`.
- **Per-model glide sink (Greg's "fold into item 3"):** replaced the FW-fixed
  `cExpThermalSinkMps` (=−0.7 m/s for ALL models) with a new `EmuModel.GlideSinkMps`
  field = `CruiseSpeed · DragCoeff / LiftCoeff` (steady-glide sink). Values:
  Elevon 1.82 (13·0.07/0.5), Delta 2.18 (14·0.07/0.45), Aileron 1.00 (10·0.06/0.6),
  Spoiler 0.79 (10·0.055/0.7), V-Tail 1.20 (11·0.06/0.55), RudderElevator 1.18
  (10·0.065/0.55). The shared `cExpThermalSinkMps` is left untouched — it also
  feeds real-flight soar logic (`soar.c:71`); the emu uses
  `−md->GlideSinkMps` (stored positive, negated at use). MR/Land/VTOL carry 0
  (unused — only `eCatFw` reads the sink; MR sink is thrust-based, confirmed
  no other consumer).

## Build / verification status
- **FC:** all 7 targets build clean (`python3 scripts/fc_build.py`, all `=== <T> OK ===`).
- **GCS:** `test_pid_sim.py` `py_compile` clean ×2 (source + gitUAVXGS mirror);
  full 14-frame generic fleet AH climb + descent PASS.
- **Open:** Greg to reflash a FW frame and run an emu bench mission to confirm
  climb authority in the emulated plant now matches the mixer ceiling (the
  Sep-18/19 emu-vs-hardware family resolution).