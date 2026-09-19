# Session Report — RC switch decode generics + wide-but-sane gain brackets

Date: 2026-09-09
Session: Two linked cleanups in support of the Shadow field-retune: (a) the
RC multi-position switch decoding was re-expressed as two reusable generics
(`Select2`/`Select3`) with documented zone conventions, and (b) the PID/gain
param ceilings were widened (~10× class ceiling, mirrored FC↔GCS) so a pilot
can type retuned gains directly instead of being clamped to the old narrow
window. In the process the `test_param_limits.py` FC↔GCS bounds regression test
was repaired (it had been silently broken) and six latent `.af` `[LIMITS]`
defects it then exposed were fixed.

## Change summary

### (a) RC switch decode generics — `rc.c` / `params.h`

The old decode of 3-position switches was an ad-hoc
`Limit((uint8)(RC * 3.0f), eSwLow, eSwHigh)` snap; the zones were magic and
every call site re-derived the pattern with slightly different commentary.
Replaced by two static helpers in `rc.c` (`Select2` at rc.c:936, `Select3` at
rc.c:942) plus named zone constants in `params.h`:

- `cSwZoneLow = 1.0f/3.0f`, `cSwZoneHigh = 2.0f/3.0f` (params.h:55-56) — the
  3-position boundaries, carrying a comment that names each zone's meaning
  (`eSwLow / eAngleMode / ePIC`, `eSwMiddle / eHorizonMode / PH`,
  `eSwHigh / eRateMode / RTH`).
- `Select2(r)`: boolean — true when `ActiveCh(r) && RC[r] >= cRcNeutral`; two
  equal halves. Returns `false` on an inactive channel.
- `Select3(r)`: int 0/1/2 — three equal thirds, index suitable for `switch()`
  or array indexing; returns `0` on an inactive channel.
- Both reset to the **safest default** on a dead channel (position 0 / false).

Converted call sites:
- `NavSwState = Select3(eNavModeRC)` (rc.c:1006) — index maps directly onto
  `eSwLow/eSwMiddle/eSwHigh`.
- `AttitudeMode = Select3(eAttitudeModeRC)` when in `ePIC` and NavSw is low
  (rc.c:1047) — index maps directly onto `eAngleMode/eHorizonMode/eRateMode`.
- `VTOLMode = Select2(eTransitionRC) && (pAFType == eVTOLAF) && !F.PassThru`
  (rc.c:1055) — the transition channel is a 2-position switch.

A **RC CHANNEL CONVENTIONS** doc block (rc.c:959-981) now states the whole
policy in one place: all `RC[]` are 0.0–1.0 (sticks ±1.0); 3-position switches
use `Select3`; 2-position switches use **explicit `>threshold`** so an
unintentional near-center throw can never trip them — PassThru > 0.7, Dive >
0.7, VTOL transition > 0.5, Trace ch8 ≥ 0.5; the Arming/NavQual channel is a
**pot**, not a `Select3` switch, because a pot needs hysteresis-tolerant
thresholds (0.1 / 0.4 / 0.6). Stale/"gke does not seem correct"/"COMPLICATED"
comments removed along the way.

### (b) Wide-but-sane gain ceilings — FC `params.c` and GCS `parameters.py`

The PID/gain class ceilings (the FC hard clamp in `ProcessParamsWrite` /
`ReadParametersFromFLASH` via `ClampParamValue`, params.c:294) were ~3× the
typical gain, so a field retune that wanted e.g. a rate-P of 2.0 could not
even be typed. Widened to ~10× typical in the shared `ParamClass[]` table
(params.c:111-135):

| class | old ceiling | new ceiling |
|---|---|---|
| `eClassGainRateP` | 0 – 3.0 | 0 – 30 |
| `eClassGainRateD` | 0 – 0.2 | 0 – 2.0 |
| `eClassGainRateI` | 0 – 0.2 | 0 – 2.0 |
| `eClassGainAngleQ` | 0 – 20 | 0 – 200 |
| `eClassGainAngleI` | 0 – 25 | 0 – 250 |

Rationale comment added at params.c:106-109: ceilings are deliberately wide so a
tuning pilot can exceed the default window; tight per-frame guidance lives in
the GCS `.af [LIMITS]`, not in the class table; the ceiling still catches
fat-finger 100× slips.

Mirrored in GCS `PARAM_CLASS_BOUNDS` (parameters.py:427-431). `PARAM_LIMITS`
is derived from the class tables (parameters.py:588-592), so it follows
automatically. Spinbox precedence: `.af [LIMITS]` first (`_param_limits`,
parameter_window.py:887-891), else `PARAM_LIMITS`. `_min_decimals`
(parameter_window.py:893-899) keeps the 18 PID/gain tags at ≥4 decimal places
regardless of the wider span, so values type and round-trip cleanly; the
`singleStep` affects only arrow-clicks, and a typist enters values directly.

Shadow.af `[LIMITS]` was widened consistently (rateP ≤ 10, angleQ ≤ 50,
angles ≤ 60°, etc.), and `YAW_RATE_KD` capped at tag-90's explicit 0.05 — see
Verification for why.

### (c) `test_param_limits.py` repair

`uavx-python/src/tests/test_param_limits.py` is the regression net that parses
FC `params.c` `ParamTable`/`ParamClass[]` and asserts GCS `PARAM_LIMITS` and
`PARAM_CLASS_BOUNDS` match byte-for-byte, then validates every shipped `.af`
value/`[LIMITS]`. It had been **silently broken**: `parse_param_table` crashed
on four FC bound tokens it did not resolve, so it never reached its
assertions:

- `cGyroLpfSelMax` / `cAccLpfSelMax` — `static const int32` in
  `sensors/mpu6xxx.h` (GyroLPFSel tag 47, AccLPFSel tag 89).
- `cBatteryCapacityMahMin` / `cBatteryCapacityMahMax` — `static const` in
  `params.h` (BatteryCapacity tag 53).
- `eTraceNone … eTraceEnd` — `TraceTypes` enum in `trace.h` (TraceType
  tag 125).

All added to the test's frozen-value `ENUM_VALUES` dict. With the resolvers in
place the test runs to completion — and immediately exposed six latent `.af`
`[LIMITS]` defects (below), which were also fixed.

### (d) `.af [LIMITS]` latent defects found by the repaired test

1. **`Shadow_*.af` and `Arado_555.af`/`Horten.af` `FW_MAX_CLIMB_ANGLE`**
   (13 files): limits hi `1.047` truncated below the stored class-ceiling
   value `1.0471976`. The GCS `QDoubleSpinBox::setRange()` would have clamped
   the value **down** to `1.047` on every load→save, silently shrinking the 60°
   limit. Hi corrected to `1.047198` in all 13.
2. **Dated `Shadow_*.af` `UNUSED_20`** (6 files): value `0.0` sits below the
   stale limits lo `0.1` —
   [the exact "clamp-up" trap](Session_Report_AFTypeLoadRevert_Sep07.md)
   the invariant was written for. `UNUSED_20` is now a NULL-targeted
   `(0, 255)` slot, so limits widened to `0, 255`.
3. **`Shadow.af` `YAW_RATE_KD`**: widened limit `(0.0001, 0.5)` overshot FC
   tag-90's *explicit* class bound `(0.0, 0.05)` (YawRateKd is
   `eClassExplicit`). A `.af` must never advertise a range the FC cannot
   accept. Capped to `(0.0001, 0.05)`.

## Rationale and logical discourse

**Why `Select2`/`Select3` and not a per-site threshold?** The multi-position
decode was repeated inline with magic numbers (`RC * 3.0f`, `uint8` casts) and
each site carried slightly different (and partly stale) commentary. Two
generics with named zone constants make the convention (equal thirds / equal
halves, dead-channel → safest default) a single, testable statement, and give
the doc block a concrete API to reference. The int return of `Select3` maps
straight onto the existing enums because their ordinals already line up:
`eAngleMode=0/eHorizonMode=1/eRateMode=2` and `eSwLow=0/eSwMiddle=1/eSwHigh=2`
— so `switch (Select3(ch))` needs no re-mapping.

**Why explicit thresholds for 2-position switches instead of `Select2`?**
Deliberate, and documented: DAC067-mode switches like PassThru (0.7) and Dive
(0.7) are safety functions that must require a *definite throw*. `Select2`'s
`>= 0.5` half boundary would trip on nothing, but the philosophy stated in the
doc block is "a safe definite throw is required" — a pilot may detent a switch
only part-way, and the conservative habit is to demand `>0.7` for anything
that disengages a protection. `Select2` is reserved for true two-position
functions (VTOL transition, `eTransitionRC`).

**Why is Arming/NavQual a pot and not `Select3`?** A pot under a pilot's thumb
has no deadband and jitters around a boundary; `Select3`'s fixed zones would
read a value parked at 0.34 as position 1 (armed/AltHold) on a glitch. The
existing threshold scheme (0.1/0.4/0.6) is hysteresis-tolerant — crossing a
threshold is a deliberate motion, not a zone membership. The doc block now
says this explicitly so nobody "simplifies" the pot to a `Select3` later.

**Wide brackets rather than full bypass or a toggle.** Three options were
considered for the field-retune need:
1. **Full bypass** of the class clamp. Rejected: a fat-finger 100× slip would
   reach the flight controller unguarded — the exact thing the clamp exists
   for, and an in-flight verify mismatch is the *bad* outcome (a committed
   garbage gain that the FC accepted).
2. **A config toggle** to relax/restore the clamp. Rejected: extra param,
   extra UI, and a pilot tuning in the field would have to remember to restore
   it; the failure mode of a forgotten ON is an unguarded write.
3. **Wide-but-sane brackets (~10× typical)**. Adopted: keeps a real guard
   (still catches 100× slips) while opening the entire plausible gain space
   for typing. Tight per-frame guidance moves where it belongs — the `.af
   [LIMITS]` block — and the class table explicitly outsources to it.

**Why also widen the FC `ParamTable` FLOAT literal `(lo,hi)` columns?** The
GCS/FC agreement test asserts `PARAM_LIMITS[tag] == FLOAT(literal lo, hi)` for
every tag, and the AGENTS convention makes the ParamTable the source of truth
for bounds. Widening only `ParamClass[]` while leaving the literal columns at
the old values would have (a) failed the test and (b) left the GCS display
metadata advertising a narrow window that no longer matches the actual FC
clamp. The five gain-class FLOAT entries (tags 0, 2, 5, 7, 10, 11, 23, 24, 27,
96, 97, 108) were therefore widened to match the class ceilings exactly.

**`test_param_limits.py` had been validating nothing.** The four
unresolvable-token crashes are worth recording as a class of bug: the frozen
`ENUM_VALUES` dict was a snapshot of FC enum/consts that silently went stale
as new params (Trace) and enum-typed bounds (`static const`, not `#define`)
were added. Any future param with a named non-`#define` bound will re-break
it. The test's crashing on `cGyroLpfSelMax` had nothing to do with the gain
widening — it would have failed on any run, gain change or not. Worth a
follow-up to make `resolve_number` scrape the actual headers instead of a
hand-maintained dict (parked; the dict is fine for now).

**The `.af` defects were pre-existing and invisible.** Because the test never
ran past the token-crash, 13+6 files shipped with limits that would silently
bias stored values on load→save (FW_MAX_CLIMB_ANGLE clamped down to the
truncated hi; UNUSED_20 clamped up to the stale lo). The Shadow 2026-09-06
clamp-up report was the *same class* of bug caught in one file; the repaired
test now makes this class exhaustive across the fleet. None of the fixed files
would have behaved catastrophically (both were ≤0.1% value perturbation, and
UNUSED_20 is a dead weathervane) — but each was a silent datum rewrite on the
next save.

**The `YAW_RATE_KD` cap is a different invariant.** Unlike the class-bound
gains, tag 90 is `eClassExplicit` — its literal `(0, 0.05)` *is* the FC clamp.
The `.af [LIMITS]` may be tighter than the FC, never looser. Widening a
class-bound gain's `.af` range is fine (the class ceiling still caps it
harder); widening an explicit param's `.af` range past the entry is a lie. The
test enforces exactly this asymmetry and we kept it.

## Verification

- FC build: `python3 scripts/fc_build.py` — **all 7 targets** compile clean
  (UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI,
  BLUEBERRYF405, MATEKF411WING).
- GCS: `python3 -m py_compile parameters.py airframes/airframes.py
  tests/test_param_limits.py` — clean.
- **`python3 tests/test_param_limits.py`** — now runs to completion:
  `OK: all 128 tags + 23 classes match FC params.c` and
  `OK: 39 .af files validated ([LIMITS] within class bounds, values in range)`.
- `Select2`/`Select3` semantics, ELRS/button-cluster mapping, and gain-bracket
  decision confirmed with Greg during the session.

## Files changed

- `UAVXArmQ/src/params.h` — `cSwZoneLow`/`cSwZoneHigh` (with zone-meaning
  comment).
- `UAVXArmQ/src/rc.c` — `Select2`/`Select3` (rc.c:936/942), RC CHANNEL
  CONVENTIONS doc block (rc.c:959), NavMode/AttitudeMode decode via `Select3`,
  VTOLMode via `Select2`, stale comments removed.
- `UAVXArmQ/src/params.c` — `ParamClass[]` gain ceilings widened (~10×) with
  rationale comment (params.c:111-135); five gain-class `FLOAT` literal
  `(lo,hi)` columns widened to match (tags 0/2/5/7/10/11/23/24/27/96/97/108).
- `UAVXGS/uavx-python/src/parameters.py` — `PARAM_CLASS_BOUNDS` mirrored
  (:427-431); `PARAM_LIMITS` auto-derives.
- `UAVXGS/uavx-python/src/ui/parameter_window.py` — no functional change this
  session (`_param_limits`/`_min_decimals`/`_apply_airframe_limits` already
  handle the wider ranges).
- `UAVXGS/uavx-python/src/tests/test_param_limits.py` — `ENUM_VALUES` gaps
  filled (`cGyroLpfSelMax`/`cAccLpfSelMax`, `cBatteryCapacityMahMin/Max`,
  `eTraceNone…eTraceEnd`) so the test runs.
- `UAVXGS/uavx-python/src/airframes/user/{Shadow.af, Shadow_2026*.af}`
  (widened gains; `YAW_RATE_KD` capped to 0.05), plus `Arado_555.af` /
  `Horten.af` / dated `Shadow_*` — `FW_MAX_CLIMB_ANGLE` hi corrected to
  class ceiling; dated `Shadow_*` `UNUSED_20` limits → `(0, 255)`.

## RC decode into rc.c — files changed

- `UAVXArmQ/src/rc.h` — `SetNavModeRC(uint8)` declaration + `extern boolean
  Aux2High` (Decoded RC switch outputs block).
- `UAVXArmQ/src/rc.c` — `Aux2High` global; `Aux2High = Select2(eTraceRC)` in
  `UpdateControls`; `SetNavModeRC` added.
- `UAVXArmQ/src/alarms.c` — `FailPreflight` now `(NavSwState == eSwLow)`.
- `UAVXArmQ/src/trace.c` — consumes rc.c `Aux2High`; local decode removed.
- `UAVXArmQ/src/telemetry/telem.c` — `miscSetNavMode` → `SetNavModeRC()`.
- `AGENTS.md` (UAVXGS + gitUAVXGS mirrors) — failsafe section corrected to
  match rc.c (SBus-flag path uses receiver frame data, not `FS[c].Raw`;
  timeout path injects `FS[c].Raw`; presets are `Map[]`-indexed so the
  2026-09-02 reorder relocated them automatically).

## RC decoding moved into rc.c — audit + enforcement

Continuing the alpha rule ("all decoding of stick values is to be in rc.c where
Flags are set for use by other parts of the code"), a whole-tree grep found the
last three `RC[e...]` decode/write sites **outside** rc.c and moved them in.

**Sites eliminated:**
- `alarms.c` `FailPreflight()` (rc.c:216 before): rejected arming when
  `RC[eNavModeRC] <= FromPercent(20)`. Now `(NavSwState == eSwLow)` — rc.c's
  decoded Nav-switch position. Slightly different threshold semantics (old 20%
  vs zone boundary 1/3) but semantically identical intent: "NavMode in the
  low/PIC position".
- `trace.c` `TraceCapture()` (trace.c:186 before): `Aux2High =
  (RC[eTraceRC] >= 0.5f)`. Now a rc.c-owned global `Aux2High`, set in
  `UpdateControls()` as `Aux2High = Select2(eTraceRC)` right beside the
  `PW[Aux2C]` block.
- `telem.c` `miscSetNavMode()` (telem.c:1059 before): a raw `RC[eNavModeRC] =
  UAVXPacket[5]*0.5f` write. Now routed through the new `SetNavModeRC(uint8)`
  accessor in rc.c, which validates `v <= 2` and writes
  `RC[eNavModeRC] = (real32)v * 0.5f` (the same re-scale the raw write did).

**New rc.h exported decodes** (keeping the `NavSwState` precedent — a plain
rc.c-owned global, NOT a packed Flags bit, so the telemetry union layout is
untouched):
- `extern boolean Aux2High;` — decoded Trace/Aux2 test-switch high state.
- `void SetNavModeRC(uint8 v);` — telemetry-set NavMode channel (0..2 → 0.0/0.5/1.0).

**Why a global, not a new F bit:** the Flags union is `packed` over a byte
array that the GCS maps by bit index (main.h:101, `flag_defs`); adding a field
would shift the telemetry contract. `NavSwState` (auto.h:142) is the existing
precedent for "decoded value exported from rc.c".

**Failsafe ordering verified (2026-09-02 reorder still safe):** the AGENTS.md
failsafe section had drifted from the code (it claimed the SBus-flag path uses
`FS[c].Raw`; actually only the timeout path does). Verified in rc.c and the
doc corrected:
- FS presets are **indexed via `Map[]`** (`FS[Map[c]].Raw = ...` in `InitRC`) —
  so the 2026-09-02 NavMode 4→6 / Arming 6→4 swap automatically relocated both
  the live decode AND the failsafe presets; the sub-640 action block keys only
  on `RCFailsafe || RCSignalLost` + `pFailsafeAction`, fully channel-independent.
- The two failsafe paths both end in a single flag: `sbusDecode()` sets
  `RCFailsafe`/`RCSignalLost` from frame flags (rc.c:325-326) and MapRC maps
  with the **received frame's own channel data** (`RCInp[c].Raw`); only the
  timeout path (`GenerateFailsafePacket`) injects `FS[c].Raw` (rc.c:221-236).

**Verification:** `python3 scripts/fc_build.py` — all 7 targets compile clean.
GCS unchanged (no Python edits this pass).

## Remaining

1. **Propagation note (user):** widening the gain class *ceilings* (`ParamClass[]`)
   does **not** touch `ParamTableCRC` (params.c:70 covers `ParamTable[]` only),
   so an already-flashed FC keeps its stored config — no defaults reload, no cal
   clear. Old narrow `.af` values load fine within the new windows. But the
   **FLOAT literal `(lo,hi)` columns *are* in `ParamTable[]`**, so `ParamTableCRC`
   changes on this flash and the FC will do the table-changed path (defaults
   reload, `Config.BootDiag=3`, **cal preserved**) on first boot — expected,
   benign, standard CRC-preserve-cal flow.
2. Still open from the Shadow tuning session: the flight issues that motivated
   the wide brackets — auto-mode over-gain, reversed manual controls, SensePW
   no effect, angle mode not engaging (possible committed quad-gain load under
   Shadow's name). The widened brackets now let those be fixed by typing.
3. Parked: make `test_param_limits.py` `resolve_number` scrape the FC headers
   instead of the hand-maintained `ENUM_VALUES` dict, so future FC enums can't
   silently re-break it.