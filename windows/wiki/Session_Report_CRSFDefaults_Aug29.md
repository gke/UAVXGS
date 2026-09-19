# Session Report — CRSF/Config-Bit Defaults, Ken_450_1165 Promotion, parameter_window Repair — Aug 29

## TL;DR

- **`parameter_window.py` corruption false alarm resolved.** The previously-reported
  double-`__init__` / duplicated `_load_only_dirs` junk no longer exists in any copy.
  An exhaustive filesystem sweep found `gitUAVXGS` (210300 B) and `gitHub/UAVXGS`
  (210300 B) are a **stale pre-feature mirror**; `UAVXGS.BAK` (74443 B) is an ancient
  snapshot. The in-tree canonical `UAVXGS/uavx-python/src/ui/parameter_window.py`
  (210738 B) is **clean** (single `__init__`, one `_restore_airframe_selection`),
  carries ALL recent feature work, `py_compile` OK, and the three kits byte-match it.
  Recommendation: keep the canonical file; do NOT restore from the mirrors.
- **Ken_450_1165 promoted to GCS default airframe**: pointer now `proposed/Ken_450_1165.af`
  (was generic `Ecks_800g_Moderate`); `_proposed_dir` is a first-class dir in both
  `ParameterWindow` and the `AfLoadDialog` (new "Proposed (Read-Only)" filter).
- **CRSF everywhere**: FC `RxType` param default `eCPPMRx -> eCRSFRx`; all `.af` files
  already carry `RX_TYPE = eCRSFRx` from the earlier sweep.
- **Config bits everywhere** (use mag, batt comp, beep wp, fast start, have gps):
  `DEFAULT_CONFIG1` unchanged (already Mag|RTHDescend|AltHoldAlarm); `DEFAULT_CONFIG2`
  and the tag-73 `Config2Bits` default expanded to
  `BatteryComp|FastStart|GPS|NavBeep` (0x4B); GCS `load_default_params` matches;
  `Config1Bits.DEFAULT` / `Config2Bits.DEFAULT` in `protocol_enums.py` synced.
- **Latent bug fixed**: `_ensure_config2_fast_start()` was CLEARING bit 0 (Batt Comp)
  on every airframe load (`(config2 & ~1) | 2`), silently undoing the batt-comp-everywhere
  decision. Now it only SETS Fast Start (bit 1) and preserves all other bits.
- Verification: GCS full `py_compile` PASS; FC `BLUEBERRYF405` and `SPEEDYBEEF405WING`
  builds OK; `test_pid_sim proposed/Ken_450_1165.af` exit 0, **0 FAILs**; `.af`
  round-trips RX_TYPE=4 / CONFIG1=6 / CONFIG2=75 correctly.

## What was found (parameter_window repair path)

1. Prior sessions flagged corruption in `ui/parameter_window.py` (double `__init__`,
   13 duplicated `_load_only_dirs`, 3913-char `...#···` separator). On this session's
   `find`-based sweep of the entire home tree, 14 candidate files surfaced.
2. Diffs proved:
   - `gitUAVXGS/uavx-python/.../parameter_window.py` = `gitHub/UAVXGS/windows` = 210300 B:
     clean structurally (single `__init__`@716, zero `_load_only_dirs`, zero long-hash lines)
     but **lacks the recent feature set** (`ONE_DECIMAL_LIMIT_TAGS`, `NAV_POS_INT_LIM`
     I-Limit row, `MAX_*_RATE_MP_S` renames, Use Mag label, eUsingMag CONFIG1 default).
   - canonical `UAVXGS/uavx-python` = all 3 kits = 210738 B: identical clean structure
     PLUS the feature set. The 438-byte delta is exactly the recent-work diff.
3. Conclusion: the "corruption" state no longer exists in any reachable copy. Either it
   was repaired in a prior session or the earlier report mis-described a transient state.
   The canonical in-tree file is the only sensible source; mirrors are stale and would
   REGRESS recent work if copied over canonical.

## What changed

### GCS — `uavx-python/src/ui/parameter_window.py`
- `AfLoadDialog.__init__`: added `self._proposed_dir` (airframes/proposed).
- `AfLoadDialog` filter combo: added `"Proposed (Read-Only)"` item.
- `AfLoadDialog._populate_list`: `collect(self._proposed_dir, True)` for
  "All" / "Proposed (Read-Only)" (read-only, like original/generic).
- `ParameterWindow.__init__`: added `self._proposed_dir`.
- `ParameterWindow._restore_airframe_selection`: default `default_name = "Ken_450_1165"`
  resolved from `self._proposed_dir` (was `"Ecks_800g_Moderate"` from generic). Persisted
  through the existing `QSettings("UAVX","Groundstation")/airframe_path` mechanism, so an
  existing saved path still wins.
- `load_default_params`: CONFIG2 default now
  `eUseBatteryComp | eUseFastStart | eUseGPS | eUseNavBeep` (=75, matches FC tag-73
  default and the .af sweep). CONFIG1 unchanged (EnforceDriveSymmetry|UsingMag).
- `_ensure_config2_fast_start`: `corrected = config2 | 2` (set FastStart bit 1 only)
  replacing `(config2 & ~1) | 2` (which cleared Batt Comp bit 0 — a silent undo of the
  batt-comp-everywhere decision on every `_load_airframe`).

### GCS — `uavx-python/src/protocol_enums.py`
- `Config1Bits.DEFAULT` = `eUseRTHDescend | eUseAltHoldAlarm | eUsingMag` (mirrors FC
  `DEFAULT_CONFIG1`; was stale `eEmulationEnable | eEnforceDriveSymmetry`).
- `Config2Bits.DEFAULT` = `eUseBatteryComp | eUseFastStart | eUseGPS | eUseNavBeep`
  (mirrors FC `DEFAULT_CONFIG2`; was stale `eUseFastStart`).
- Note: these `.DEFAULT` attributes are currently not referenced by GCS runtime (only
  deployed in `load_default_params` inline) — kept in sync so a later consumer can't
  inherit stale values.

### FC — `UAVXArmQ/src/`
- `params.c:29` `DEFAULT_CONFIG2` =(UseBatteryCompMask | UseFastStartMask | UseGPSMask | UseNavBeepMask)
  (was UseBatteryCompMask only; `DEFAULT_CONFIG1` untouched).
- `params.c:154` `pRxType` default `eCPPMRx -> eCRSFRx` (param tag 14).
- `params.c:214` `pConfig2Bits` default = batterycomp|faststart|gps|navbeep (tag 73, was
  batterycomp|gps). Inline comment updated.
- `boards/targets/omnibusf4nxt.inc:144` stale comment "pRxType always FutabaSBus — forced
  in params.c" corrected to note the CRSF default (no such forcing existed; truth is the
  ParamTable default).

## Rationale / decision log

- **Don't restore from mirror**: `gitUAVXGS`/`gitHub` copies are a frozen pre-feature
  snapshot (210300 B vs 210738 B). Copying them over canonical would silently delete the
  recent config/limits work — strictly worse than the alleged (and now absent) corruption.
- **Default airframe = proposed/Ken_450_1165.af**: matches the accepted design that
  `proposed/` holds read-only, fully-populated, validated defaults and is the natural
  landing-airframe for the retuned fleet. Kept the QSettings override so users' own saved
  selections are never clobbered.
- **`_ensure_config2_fast_start` was an active contradiction**: the sweep added Batt Comp
  (bit 0) fleet-wide; the guard then stripped it at every `_load_airframe`. Clearing bit 0
  was a legacy holdover with no remaining rationale once batt comp is a default. Chose
  set-only semantics to honour both Fast-Start and Batt-Comp defaults. Rejected removing
  the guard entirely (still useful to force Fast Start on files that lack it).
- **FC __default__ values, not magic bump**: RxType/Config defaults live in the ParamTable;
  changing defaults changes `ParamTableCRC`, which per LoadParameters policy preserves cal
  (layout identical). No magic bump, no cal clear expected.

## Verification

- GCS: `python3 -m py_compile` on every `.py` in `uavx-python/src` — ALL OK.
- GCS .af round-trip: `proposed/Ken_450_1165.af` parse -> RX_TYPE=4 (eCRSFRx),
  CONFIG1_BITS=6 (Mag|RTHDescend), CONFIG2_BITS=75 (0x4B), 123 params, gains intact.
- FC: `fc_build.py` (default BLUEBERRYF405) and `BOARD=SPEEDYBEEF405WING` — both link,
  .bin generated (228500 / 228556 B).
- Sim: `python3 -m tests.test_pid_sim proposed/Ken_450_1165.af` — exit 0, zero FAIL
  lines; final Alt-Hold 5m and Nav 10m blocks PASS (AH rise 1.862 s, overshoot 0.4%).

## Follow-ups

- Kits (`linux|macos|windows`) were verified BYTE-IDENTICAL to canonical for
  parameter_window.py — no kit refresh needed for this change.
- Next airborne step: flash `SPEEDYBEEF405WINGQ_r0.bin`, connect CRSF Rx on `RX1`(PA10),
  confirm stick channels + Tx telemetry with the new defaults and no config re-write.
- Tuning campaign for the remaining original airframes (per TODO: current vs proposed
  tuning table, yaw-rate KP/KI + alt-ROC KP targets) remains open — validate via
  `python3 -m test_pid_sim <file>` continuing from clean `original/` baselines.

## Files touched

- `UAVXGS/uavx-python/src/ui/parameter_window.py`
- `UAVXGS/uavx-python/src/protocol_enums.py`
- `UAVXArmQ/src/params.c`
- `UAVXArmQ/src/boards/targets/omnibusf4nxt.inc`