# Session Report — FC⇄GCS Parameter Bounds Sync Sweep (Aug 28)

## Objective
After the recent parameter-system changes (unified float, angle-unit conversion, VRSROC/DiveRecoverAlt/VRS commissioning, FW-limit retunes), verify that
parameter min/max are still in sync between:
1. the FC `ParamTable` (`UAVXArmQ/src/params.c`) — the source of truth for bounds, and
2. the GCS `PARAM_LIMITS` (`uavx-python/src/parameters.py`), and
3. every `.af` airframe file (`UAVXGS/windows/src/airframes/**`).

## Method
### FC side
Parsed the 128-row `ParamTable` (index == tag) directly from `params.c`:
- Macro rows `FLOAT(&p..., class, scale, lo, hi, default, "comment")` and
  `U8(&p..., class, lo, hi, default, "comment")`; gap rows use `NULL` targets.
- Effective bounds per tag = per-row `(lo, hi)` when `eClassExplicit`, else
  `(lo,hi)` intersected with the `ParamClass[cls]` family clamp
  (`ClampParamValue`). `ParamClass` table read from `params.c`.
- Symbolic enum bounds resolved to numerics by hand from source:
  - `params.h:218-221` FailsafeActions (`eFsRth=0, eFsLand=1, eFsMotorsOff=2`)
  - `params.h:188-196` RFs (`eMaxSonarcm=0 … eNoRF=5`), `params.h:267` AFTypes
    (`eAFUnknown=26`)
  - `params.h` RCControls / RxTypes (`eCPPMRx=0 … eUnknownRx=5`),
    `eTxArming=0/eSwitchArming=1`, `eLandNoStop=0…eLandDescentRateAndAccU=4`
    (auto.h:117), `eUnknownReset=0…eBrownoutReset=7`
  - `imu.h:24-27` IMUFilterTypes (`eLP2Filt=0 … IMU_FILT_TYPE_MAX=eF4=5`)
  - `telem.h:25` inflightLogs (`eLogUAVX=0 … eLogYaw=4`), `telem.h:97-105`
    TelemetryTypes (`eUAVXDJTTelemetry=0 … eU8Telemetry=7`)
  - `sensors/mpu6xxx.h:83-84` `cGyroLpfSelMax=7`, `cAccLpfSelMax=5`
  - `BatteryCapacity` `cBatteryCapacityMahMin=1500 … MahMax=10000`
  - `params.c:28-47` Config masks: `DEFAULT_CONFIG1 = RTHDescend|Mag|AltHoldAlarm
    = 0x02|0x04|0x10 = 22`; `CONFIG_BITS_MAX=255`
- GCS side mirrored from `parameters.py` `PARAM_LIMITS`, `PARAM_DEFAULTS` (converted
  to raw via `PARAM_DISPLAY_MULT`), `PARAMETER_DEFS` names.
- Full comparison written to `/tmp/opencode/param_sweep.csv`.

### .af sweep
Used the GCS's own `airframes.parse_af_file()` (which resolves enum tokens, incl.
`s_pop` split of bitmask `CONFIG*_BITS = eA|eB|eC`), checking every parsed raw value
against the GCS limits (which we just proved equal the FC bounds). 72 files across
`generic/`, `user/`, `backup/_retired_tuned/`, `backup_angleunits/`, `original/`.

## Results
### 1. FC ParamTable vs GCS PARAM_LIMITS — fully in sync (128/128)
Zero `BOUND-MISMATCH`, zero `UNRESOLVED-BOUND`, zero `GCS-NO-LIMIT`. Every per-tag
raw `(lo,hi)` matches the GCS exactly, including the recently retuned
`FWRollControlPitchLimit(113)`, `VRSROC(103)`, `DiveRecoverAlt(116)`,
`NavPosKp(56)`/`NavVelKp(28)`, `AltROCKp(119→120 range)` and the enums above.

The only flagged category was `DEF-MISMATCH` (defaults). These are **by design, not
a defect**: FC `ParamTable.default_val` is the factory-reset baseline, while GCS
`PARAM_DEFAULTS` come from the airframe-tuned world. The
params.h:318 comment ("defaults now use ParamTable.default_val") governs the
default path; limits, not defaults, are the interface contract.

### 2. Live airframe files — fully in-bounds (0 issues)
`generic/` (14 files) and `user/` (21 files) all values inside limits.
No action needed.

### 3. Archived/historical trees — expected legacy-unit drift (isolated)
37 flagged values confined to `backup/_retired_tuned/`, `backup_angleunits/`,
`original/` — pre-conversion snapshots, never loaded in normal use. Patterns:
- `FWRollControlPitchLimit=45` — old **degrees** written raw into the 
  radians slot (range now 45°..60° = 0.785..1.047 rad). `backup_angleunits/`
  files likewise store angles (FWMaxClimbAngle, NavMaxAngle, MaxPitch/MaxRoll,
  NavHeadingTurnout, FWBoardPitchAngle) as degrees.
- `Horizon=100 / 3.33 / 2.5` — old percent display encoding vs new fractional.
- `LowVoltThres=3 / 3.4` — old cell-count×voltage encoding vs unified volts.
- `VRSROC=-5` — pre-commissioning raw value; FC range tightened to (−3, 0).
- `Unused120=15` — leftover landed in the now-unused tag 120 range (0, 0.5).
- Config bitmask token `eDisableLEDsInFlight` (×5, archives only) — a stale
  legacy display token not present in FC symbols or GCS `Config1Bits`; the
  GCS loader cannot parse it (parser gap on unknown bit tokens in OR-expressions),
  the FC Mag bit is its descendant.

## Decisions / rationale
- **No code or bound changes made.** The sweep was verification-only; nothing
  needed re-syncing. `ParamTable`/`PARAM_LIMITS` already agree on every tag.
- **Archives left untouched** — they are deliberate historical snapshots of the
  two unit-conversion eras; editing them would destroy provenance. They must not
  be treated as loadable sets.
- **GCS legacy-token parser gap** (`eDisableLEDsInFlight`) consciously NOT fixed:
  it only ever triggers on archived files, and "fixing" it would silently
  resurrect a stale semantics (Mag bit) into a currently-valid config write path.
  Left as a latent, documented limitation.
- Default mismatches (Section 1) recorded in `param_sweep.csv` for audit but not
  "corrected": parity with .af-tuned GCS defaults is the load path; FC factory
  reset values are intentionally conservative.

## Verification status
- 128/128 FC⇄GCS bound agreement (tags 0..127), symbolic enums hand-verified.
- 8105 param values in 72 `.af` files checked; 0 issues in live trees.
- GCS `py_compile` not required (no source edited). No FC rebuild required
  (no C change).

## Cross-references
- AGENTS.md "Parameter System (Unified Float)" and "ParamTable is the source of
  truth for bounds" mandates — both confirmed holding.
- `wiki/Legacy_Scaling_Map.md` — the legacy (pre-unified) scaling bridge this
  sweep's archive drift traces back from.