# Session Report — Unused Param Slots (Phase 3) + Cal-style Config Fields

**Date:** 2026-09-07
**Build/verify status:** FC clean — all 7 targets rebuilt via
`python3 scripts/fc_build.py`, 0 errors (only the pre-existing "LOAD segment
with RWX permissions" linker note); fresh `_r0.bin` for every board
(SPEEDYBEEF405WING, MATEKF411WING, UAVXF4V3, UAVXF4V4, DEVEBOXF4,
FLYINGRCF4WINGMINI, BLUEBERRYF405). GCS `py_compile` clean on all changed
files.

---

## Problem (why this session)

The 128-param wire space still carried **FC-owned and dead values** that the
GCS had no business exposing as R/W parameters:

| tag | old slot | why dead / FC-owned |
|---|---|---|
| 19 | `EST_CRUISE_THR` | FC-runtime value; `TrackCruiseThrottle()` owns the converged hover baseline (`Config.CruiseThrottleFF`) |
| 72 | `KFAccUBiasVar` | KF noise matrix entry, tracked live inside `state.c` |
| 104 | `BootDiag` | boot-outcome diagnostic — read-only, never settable |
| 110 | `KFBaroVar` | KF noise matrix entry |
| 111 | `KFAccUVar` | KF noise matrix entry |

Greg's directives: *"Remove the dead stuff. Anything that is not R/O must be
removed from parameters. Their entries can be marked Unused in a manner used
previously. No changes to the ParamTag enum sequence!"* and *"CruiseThrottleFF
and BootDiag should not be within the main parameters block of the Config but
separate as for other calibration values in the Config Struct."*

---

## Design decisions (and the reasoning)

- **Unused weathervane naming — `Unused<N+1>` (tag N).** The tag is NOT
  renumbered and the enum sequence is untouched, so the wire format is byte-
  identical and old `.af` files stay numerically aligned. The GCS key mirrors
  the tag (`UNUSED_20` ← tag 19, `UNUSED_73` ← 72, `UNUSED_105` ← 104,
  `UNUSED_111` ← 110, `UNUSED_112` ← 111).
- **NULL-targeted rows.** Each Unused row is `FLOAT(NULL, eClassExplicit, 1,
  0, 255, 0, NULL)` / `U8(NULL, eClassExplicit, 0, 255, 0, NULL)`. The FC-side
  `e->target` is NULL so `ReadParametersFromFLASH`/`UseDefaultParametersEx`
  skip unpacking into a real variable; `Config.ParamData[i]` still hosts the
  (0..255) default so `WriteParametersToFLASH` clamps/writes it harmlessly.
  `send_params_typed` only writes **dirty** indices, and the row is no longer
  in the GCS grid list, so the GCS never dirties tag 19.
- **Cal-style Config fields (the "as for other calibration values"
  directive).** `Config.CruiseThrottleFF` (real32) and `Config.BootDiag`
  (uint8) live in `ConfigStruct` beside `RateCal` — NOT in `ParamData[]`, NOT
  behind a ParamTable row, and **the `p`-prefixed globals are deleted**
  (`pCruiseThrFF`, `pBootDiag`). This mirrors `RateCal`/`IMUCal`/`MagCal`: FC-
  owned, persisted via the same `RefreshConfig()` Config-sector write.
- **CONFIG_MAGIC bump `0xFEEDBEE4 → 0xFEEDBEE5`.** Adding bytes to
  `ConfigStruct` changes field offsets, so a magic bump is mandatory; first
  boot after reflash clears calibration (`UseDefaultParametersClearCal`) and
  wipes `AirframeName`. **USER: recalibrate after this flash.**
- **`Config.BootDiag` write ORDER (the subtle part).** The valid-flash path
  reads the existing block (which carries the *previous* boot's code) and then
  sets `BootDiag = 1` AFTER the load so it reflects *this* boot. The
  reset paths (5/2/3/4) set `BootDiag` **BEFORE** calling
  `UseDefaultParametersEx()`, because that function calls `RefreshConfig()`
  which persists the config block (incl. the fresh code). Setting it after
  those calls would have lost it. `BootDiag` is NOT seeded inside
  `UseDefaultParametersEx` (that would clobber the branch code it processes).
- **`Config.CruiseThrottleFF = 0.55f` seeded in `UseDefaultParametersEx`**
  right after the ParamData loop, before `ConfigChanged = true` +
  `RefreshConfig()` — so a cold/flash-cleared FC gets the operational 0.55
  hover baseline immediately (matches the 0.55 FC default; the emulator uses
  0.5). `TrackCruiseThrottle()` converges it at runtime and no longer also
  writes `Config.ParamData[19].f`; `ConfigChanged = true` still marks the
  config dirty so the next out-of-flight `RefreshConfig` persists it.
- **`_LEGACY_PARAM_NAME_TO_TAG` transitional alias bridge (airframes.py).**
  Old `.af` files still contain `EST_CRUISE_THR` / `KF_ACC_U_BIAS_VAR` /
  `KF_BARO_VAR` / `KF_ACC_U_VAR` keys. `parse_af` falls back to this dict for
  unknown keys so pre-rename files keep loading until re-saved as
  `UNUSED_NN`. This is a **new, deliberate, temporary bridge** — it is NOT an
  extension of `PARAM_SCALES`/`LEGACY_TAGS` (those remain frozen and dormant),
  which stay the only historical integer-encoding bridge. (Why a bridge and
  not a silently-dropped key: a load that silently DROPs the cruise baseline
  would bake the FC default 0.55 over the .af's tuned 0.5 — a small but
  invisible behaviour change on every legacy load.)
- The bridge maps to the raw tag, so a loaded legacy value lands in the right
  ParamData slot and the GCS saves it out as `UNUSED_20` on the next save
  (re-save permanently migrates the file).

---

## FC changes (UAVXArmQ/src/)

- `nvmem.h`
  - `CONFIG_MAGIC` → `0xFEEDBEE5` with a bump-reason comment (nvmem.h:31).
  - `ConfigStruct`: `real32 CruiseThrottleFF;` + `uint8 BootDiag;` added after
    `RateCalStruct RateCal;` (nvmem.h:61-62).
- `params.c`
  - Removed the `uint8 pBootDiag` global.
  - ParamTable rows 19/72/104/110/111 → NULL-targeted Unused (params.c:169,
    223, 255, 261, 262), each with a `— was <name>` comment.
  - `Config.CruiseThrottleFF = 0.55f` seed in `UseDefaultParametersEx`
    (params.c:741).
  - `Config.BootDiag` written in `LoadParameters` reset branches BEFORE the
    `UseDefaultParameters*()` call (params.c:797, 803, 809, 818) and `= 1`
    after a valid load (params.c:834), with updated comments.
  - `ApplyParameters()` comment notes cruise throttle is no longer overridden
    at commit/condition time.
- `control.c` / `control.h`
  - `pCruiseThrFF` global (control.c:46) and extern (control.h:100) deleted.
  - `DesiredThrottle` feedforward reads `Config.CruiseThrottleFF` (control.c:
    145, 152).
  - `TrackCruiseThrottle()` writes `Config.CruiseThrottleFF` (was
    `Config.ParamData[19].f`), keeps `ConfigChanged = true` (control.c:166-177).
- `dive.c:338`, `emu.c:414` (FW branch), `telemetry/telem.c:377` —
  `pCruiseThrFF` → `Config.CruiseThrottleFF`.
- `grep pCruiseThrFF|pBootDiag src/` → zero matches after the sweep.

## GCS changes (uavx-python/src/)

- `protocol_enums.py` — `UNUSED_20 = 19`, `UNUSED_73 = 72`, `UNUSED_111 =
  110`, `UNUSED_112 = 111` (each with a `was ...` comment); 104 already
  `UNUSED_105`.
- `parameters.py` — `PARAMETER_DEFS` 19/72/104/110/111 renamed to
  `Unused20/73/105/111/112`; `PARAM_CLASS_OF` 19/72/110/111 →
  `eClassExplicit`; new `PARAM_EXPLICIT_LIMITS` `(0.0, 255.0)` for 19/72/110/
  111; `PARAM_DISPLAY_MULT[19]`: 100.0 → 1.0 (raw unit, matches the raw 0.55
  .af values).
- `ui/parameter_window.py` — `EST_CRUISE_THR` row removed from the grid list
  (previously ~:678). The hidden all-128 spinboxes and `_raw_float_values`
  iterate the table, so a removed row is simply not surfaced.
- `airframes/airframes.py` — `_LEGACY_PARAM_NAME_TO_TAG` dict + `parse_af`
  fallback for pre-rename keys.
- `tests/test_pid_sim.py:1343` — reads `params.get("UNUSED_20", 0.5)`.

## Verification status

- **FC:** `python3 scripts/fc_build.py` — all 7 targets, 0 errors; fresh
  `_r0.bin` at ~10:12-10:14 has the ConfigStruct change.
- **GCS:** `py_compile` clean on `protocol_enums.py`, `parameters.py`,
  `airframes.py`, `parameter_window.py`, `test_pid_sim.py`.
- **Cross-repo bounds agreement:** parsed the FC ParamTable for all 128 rows
  and compared against GCS `PARAM_LIMITS` — the 5 changed rows are exactly
  `(0,255)` on both sides, confirmed no mismatch on any tag.
- **`.af` bridge exercised:** regenerated `af_params.csv` +
  `airframes_all_params.csv`; the old `EST_CRUISE_THR` values (0.54/0.31/0.30/
  0.5/0.55/0.59…) now appear under the `UNUSED_20` column — a live proof the
  alias bridge migrates legacy files.
- **Note:** `tests/test_param_limits.py` fails on pre-existing gaps in its
  frozen `ENUM_VALUES` (unresolved c-constants like `cGyroLpfSelMax`,
  `cBatteryCapacityMahMin` and trace.h enums) — unrelated to this change
  (row 47 etc. untouched and failing identically before).

## USER action required

1. **Recalibrate after reflash** — the CONFIG_MAGIC bump clears cal on first
   boot (gyro erecting + mag), and `AirframeName` is wiped (re-enter the name
   in the GCS and commit).
2. Verify an old `.af` (e.g. an Ecks file with `EST_CRUISE_THR`) still loads
   and round-trips: it should display without the protected-param dialog and
   re-save as `UNUSED_20`.
3. Confirm tag-71 readback / write of tag 19 is never offered in the param
   window grid.
4. Optionally delete the stale `.af`/CSV files that still carry pre-rename
   keys once migration is proven.

## Cross-references

- AGENTS.md "Parameter System (Unified Float)" → Unused weathervane naming +
  cal-style Config fields.
- AGENTS.md "Cruise Throttle as Feedforward" → `Config.CruiseThrottleFF`
  (default 0.55), `.af` baseline 0.5, bridge note.
- Prior phases: `Session_Report_ParamFlashVerify_Sep06.md` (verified writes),
  AGENTS.md "Legacy Scaling Map" (why `PARAM_SCALES`/`LEGACY_TAGS` stay
  frozen/dormant).