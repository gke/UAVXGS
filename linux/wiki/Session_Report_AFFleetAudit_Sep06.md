# Session Report — Fleet `.af` Audit & Tuning Check — Sep06

## Summary
Per Greg's direction ("all of the `.af` files need checking for consistency of
formatting and content … run a tuning check on all of them except the originals;
check the format etc of the originals but do not tune"), a fleet-wide format/content
audit and PID-critique sweep was run over all 71 `.af` files.

**Result at a glance — only 5 files are clean/current; the rest carry a legacy
format legacy:**

- **5 clean** (new, 128-param unified float, no stale names): the `user/Ecks_220mm_*`
  files exported by the current GCS (`REFERENCE`, `REFERENCE_20260905_103129`,
  `REFERENCE_20260905_132218`, `20260905_160123`).
- **0 tuning-critical content loss in the modern files** — the `user/Ecks_220mm_REFERENCE*`
  family passes every PID criterion except a **marginal Alt-Hold rise** (`2.619 s` vs `2.5 s`).
- **~53 legacy-format files** (all `original/`, `proposed/`, `backup/`,
  `backup_angleunits/`, and the older `user/` + `generic/` family) drop **up to 6
  current tags** on load because they carry **stale pre-rename enum names**
  (`TILT_THROTTLE_FF`, `ALT_ROC_INT_LIMIT`, `UNUSED_122/123/125`). One of those,
  tag 122 `RUDDER_MOTOR_FF`**, is **live** in the current firmware; the rest are
  dead/unused slots — **content is silently lost, not misread**.
- **2 unparseable files** (`backup/_retired_tuned/Ecks_220mm_Tuned*.af`): a
  `CONFIG1_BITS` token `eDisableLEDsInFlight|eUsingMag` the parser cannot decode.
- **Generic templates PASS at both slider extremes** (12/15); the 3 that fail are
  **FW/Spoileron-family** (gust-settle margin) + `SmallSpoileron` (rise time) —
  known low-authority FW tails, see below.
- **Tuning sweep verdicts** (non-original, parseable): `generic` 12 PASS / 3 FAIL,
  `proposed` 7 PASS / 2 FAIL, `user` 0 PASS / 10 FAIL, archives are expected-FAIL.

## Method (tooling added)
Two report tools were added to the GCS tree:

- `uavx-python/src/airframes/audit_af_fleet.py` — per-file audit: parse via the
  canonical `airframes.parse_af_file`, counts params/phys/missing tags, flags
  unknown/legacy enum names and `[LIMITS]`-block structural quirks.
- `uavx-python/src/tests/fleet_tuning_check.py` — wraps `test_pid_sim.run_tests_for_af`
  (the fleet critique) for every non-original file; `original/` is format-only (never
  critiqued), truly unparseable files report ERROR. Generic templates run at **both**
  slider extremes (0 %/100 %) mirroring `test_pid_sim.main()`. Outputs a CSV too.

Both compile clean (`python3 -m py_compile`); no GCS behaviour changed.

## Audit findings (format/content consistency)

### The one real content hazard: stale enum names
Legacy-era `.af` exports used the **old param names** `TILT_THROTTLE_FF`,
`ALT_ROC_INT_LIMIT`, `UNUSED_122`, `UNUSED_123`, `UNUSED_125`, `UNUSED_126`.
`airframes.parse_af_file` maps names → tags via the **current** `ParamIndex`, so
unknown names **silently drop** their value rather than erroring. Per-file loss:

| current tag | current meaning | what the old file carries | loss |
|---|---|---|---|
| 62 | `NAV_POS_INT_LIM` | `TILT_THROTTLE_FF` (dead param) | value dropped |
| 121 | `MAX_CLIMB_RATE_MP_S` | `ALT_ROC_INT_LIM` (dead) | value dropped |
| 122 | `RUDDER_MOTOR_FF` | `UNUSED_122` | **LIVE param!** dropped |
| 123 | `SPIRAL_DESCENT_BAND_M` | `UNUSED_123` | dropped |
| 125 | `TRACE_TYPE` | `UNUSED_125` | dropped |
| 124/126/127 | `UNUSED_124/126`, `POWER_RESET_CAUSE` | `UNUSED_125/126` | dropped |

The dangerous case is **tag 122 = `RUDDER_MOTOR_FF`**: it is a real mixer feed in
the current firmware, yet every legacy `.af` omits it (and old files also mislabel
their line). Position checking confirmed the stale names really sit in the current
tags' slots (line 169.. → tags 122-125 in `user/Arado_555.af`).

**Recommendation:** audit produces a migration table; do NOT auto-rewrite legacy files
with exact renames — `UNUSED_122→RUDDER_MOTOR_FF` etc. are ahead-of-assumption and
cannot be verified for every airframe without FC round-trip. Keep the audit as a
warning, and rewrite only files that are actually re-saved/loaded through the GCS
(live user/ files) — the legacy chain is otherwise harmless because the affected
slots are all zero/unused at the FC.

### Unparseable files
`backup/_retired_tuned/Ecks_220mm_Tuned.af` and `_Tuned_Tuned.af` carry
`CONFIG1_BITS = eDisableLEDsInFlight|eUsingMag`. The enum name `eDisableLEDsInFlight`
does not exist in `Config1Bits` (current GCS) — the parser falls back to a raw
`float()` conversion attempt and fails. These are retired copies that cannot be
loaded; flag for cleanup.

### Generic templates — sparse by design
`generic/*.af` carry only the tuning-relevant subset (48-57 params); the character-slider
curves fill the rest. The audit flags them as "missing tags" but that is **by design**,
not a defect.

### Original/ — format-only, no tuning
`original/*.af` (9 files) are the fixed historical baselines. They were format-checked
only (all legacy-format with the stale-name gap above) and were **not** critiqued.

## Tuning sweep results (non-original, parseable files)

| scope | PASS | FAIL | ERROR | note |
|---|---|---|---|---|
| generic/ (slider extremes) | 12 | 3 | 0 | FW/Spoileron tails fail |
| proposed/ | 7 | 2 | 0 | `Ken_LadyBug` + `WLToys_LadyBug` fail |
| user/ | 0 | 10 | 0 | Ecks REFERENCE family marginal AH; others stale/experimental |
| backup/_retired_tuned/ | 0 | 11 | 2 | all expected-FAIL (retired + unparseable) |
| backup_angleunits/ | 0 | 15 | 0 | expected-FAIL (angle-units era, pre-unified) |
| **non-original total** | **19** | **41** | **2** | |

### What is genuinely tuning-relevant
The only files that matter for flight are the **live `user/` Ecks files**:

- `user/Ecks_220mm_REFERENCE.af` — passes all PID axes and disturbance tests;
  **single FAIL is Alt-Hold rise `2.619 s` vs `2.5 s`** (marginal, T/W-limited)
- `user/Ecks_220mm_REFERENCE_20260905_103129.af` — identical
- `user/Ecks_220mm_REFERENCE_20260905_132218.af` — same + an **Integrator utilization
  `1.0` vs `max 1.0`** boundary (float-32 round-trip noise on `IntLim 0.0100005` vs
  `0.01`); effectively PASS
- `user/Ecks_220mm_20260905_160123.af` — **stale low-gain file** (`rateKp=0.1`,
  `angleKp=7`); fails hard (overshoot 432-606 %, no settle). **Do not fly this file** —
  matches the 2026-09-04 incident-profile (low flashed gains)
- `user/Ecks_220mm_REFERENCE_20260905_103108.af` — **another stale low-gain save**
  (`rateKp=0.002`, `angleKp=1.8`); fails hard. Same warning.

The two `original/Ecks_220mm.af` / `proposed/Ecks_220mm.af` files are the legacy/original
baseline and proposed template; not flight files.

### Generic FW tails (known low-authority)
`generic/SmallSpoileron` (rise 8 s), `generic/Spoileron`/`Elevon` (gust-settle ~1.7 s
vs 1.5 s) — consistent with the documented FW tail/tuning caveats; base templates are
meant to be slider-tuned, and only the extremes are scored here.

### Proposed
`proposed/Ken_LadyBug` (rate headroom 0.154, rise 6.9 s) and `proposed/WLToys_LadyBug`
(rise 4.25 s) exceed the MR criteria. The other 7 proposed templates PASS.

## Build / verification status
- FC: no FC edits (GCS-side tooling only). No FC target rebuild needed.
- GCS: new `audit_af_fleet.py`, `fleet_tuning_check.py` py_compile clean.
- Full sweep outputs: `/tmp/fleet_tuning.txt` (console), `/tmp/fleet_tuning.csv` (CSV).

## Notes / next steps
- Clean up: delete the 2 unparseable retired `Ecks_220mm_Tuned*.af`, and the stale
  low-gain `user/Ecks_220mm_20260905_160123.af` + `_REFERENCE_20260905_103108.af`
  (unless Greg wants them kept as bad-example references).
- Re-save any legacy `user/` airframe through the GCS when it is next modified so the
  stale-name gap closes naturally for that file.
- The `RUDDER_MOTOR_FF` (tag 122) silent-drop in legacy files is the only true data
  hazard; confirm whether any legacy airframe needs a nonzero value there before
  migration.