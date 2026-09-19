# Legacy x100 parameter mis-scaling — fleet correction (Sep 17)

**Date:** 2026-09-17
**Status:** FLEET CORRECTED (data) + build/parse verified; emu fake-mag
consistency still OPEN
**Scope:** `uavx-python/src/airframes/**/*.af` across canonical + 3 kits +
`gitUAVXGS`; new idempotent tool `airframes/fix_legacy_scaling.py`; GCS load
menu reviewed (not changed).

---

## 1. Trigger

While diagnosing the emulated fixed-wing 4-WP orbit (yaw held at
`rate_yaw = +0.17..0.2 rad/s` yet `Angle[eYaw]`/`heading`/`mag_heading` frozen),
the leading suspect was a **legacy x100 parameter mis-scale**: the airborne
`.af` carried raw `MADGWICK_KP_MAG = 0.5` instead of the FC default `0.005`.

Prof's ruling (2026-09-17): *"The only x100 scalings should be to display
percentages otherwise they are almost certainly incorrect."* This turned the
one suspect into a fleet-wide audit.

---

## 2. Finding — `ParamTable .scale` is display metadata only

The `FLOAT(...)` macro's `.scale` column (`params.c:140-141`) is **never applied
at runtime**; the FC stores the `.af` float verbatim in `ParamData[i].f` and
consumes it directly. `pKpMag` is used only at `inertial.c:408-410`
(`gz += (mx*wy - my*wx) * pKpMag`), with no multiply/divide. Therefore a raw
`0.5` in an `.af` **is** a 100x Mag fusion gain at runtime.

The GCS display multiplier for tag 31 is `100.0` (`parameters.py`), so the FC
default raw `0.005` *displays* as `0.5`. During the uint8 -> raw-float
migration, old saves baked the **display value** into the **raw slot** — the
display/raw coincidence is exactly why it went unnoticed.

Same class, same tag family: tag 109 `YAW_RATE_INT_LIM`, raw default `0.0003`
(display `0.03`), stored as `0.03` in the same files.

## 3. Finding — the x100 signature is exactly two current-fleet tags

Detection signature: `stored_raw == fc_default_raw × PARAM_DISPLAY_MULT`.
Verified anchors (parser checked against `params.c`): `fcd[31]=0.005`,
`fcd[22]=0.05`, `fcd[96]=0.75`, `fcd[48]=0.0008`.

* **tag 31 `MADGWICK_KP_MAG`** — 40 files stored `0.5` (=100x); 20 newer saves
  correct `0.005`; 6 deliberate `0`.
* **tag 109 `YAW_RATE_INT_LIM`** — the **same 40 files** stored `0.03`; same
  20 correct `0.0003`; same 6 deliberate `0`.
* The degree-era angle tags 67/68/74/76/78 (`×57.2958`) trigger **only** in
  `original/backup/proposed` legacy dirs, not the current fleet.

The identical 40/20/6 split across the two tags confirms a single save
generation (the pre-unified-raw era) carried both.

## 4. Fix — `airframes/fix_legacy_scaling.py`

Surgical + idempotent, modelled on `adopt_angle_params.py`:

* rewrites a base line **only** when its value exactly equals the known-wrong
  legacy value (`0.5` / `0.03`); correct (`0.005`/`0.0003`) and deliberate `0`
  are left byte-identical;
* a `[LIMITS]` band that no longer brackets the corrected base is widened
  (far edge kept, inner edge pulled in) — `0.25,0.55 -> 0.005,0.55` and
  `0.015,0.06 -> 0.0003,0.06` — so the GCS spinbox clamp never clips the
  authored base;
* root set mirrors the established batch tools: canonical + `linux`/`macos`/
  `windows` kits + `gitUAVXGS/uavx-python` + `gitUAVXGS/windows`;
* dirs `generic,user,proposed,original`.

**Applied:** 240 file instances changed (450 base + 46 band rewrites).
Safety backup: `/tmp/opencode/af_backup_20260917/af_trees.tgz`.

**Deliberately NOT changed:**
* the 6 `Shadow_*` + `Rok_Quad` files storing `MADGWICK_KP_MAG = 0` — an
  explicit mag-off value, not an x100 artefact;
* `airframes/backup_angleunits/` — a pre-angle-unit-migration backup whose
  whole purpose is to preserve the old bytes.

**Why a data fix, not an FC rescale:** `.af` stores FC-native raw by
convention (`PARAM_DISPLAY_MULT` is the only conversion bridge); adding a
runtime divide would silently change the meaning of every stored value and the
wire contract. The bug is in the saved data, so it is corrected there.

## 5. Verification

* Residual `= 0.5` / `= 0.03` in the four dirs across all roots: **0**.
* Canonical distribution: `MADGWICK_KP_MAG` 0.005 x72, 0 x8;
  `YAW_RATE_INT_LIM` 0.0003 x78, 0 x6.
* `parse_af_file` round-trips `SkySurfer_Bixler.af`, `Shadow.af`,
  `Shadow_20260914_154257.af` to 0.005 / 0.0003.
* Re-run of the tool: **0 files changed** (idempotent).
* `python3 -m py_compile airframes/fix_legacy_scaling.py` clean.

## 6. Load-menu class sorting — review (no change)

Prof noted the load menus do not sort `.af` into correct MC/FW/Land classes.
Review of `AfLoadDialog` (`ui/parameter_window.py:138`) shows it **already**
groups with `class_order` and `=== MR/FW/VTOL/Land/Sensor Airframes ===`
headers, in canonical + all 3 kits + `gitUAVXGS`. The classifier chain
(`_category_for_af_file -> category_of`) was exercised headlessly over every
`generic/` + `user/` file and resolves correctly (Delta/Dragon/Elevon/... FW;
Quad/Hex/Oct/... MR).

Two ways a user can still *see* wrong grouping:
1. running a pre-sort copy (`UAVXGS.BAK`, and `gitHub/UAVXGS/linux` has no
   `src/` at all) — stale build;
2. a file whose stored `AF_TYPE` is itself wrong (many `user/Shadow_*.af` carry
   `eQuadXAF`, so they legitimately group under **MR**; these are the known
   stale quad-content saves / the pre-2026-09-07 AF_TYPE-revert casualties).
Also, canonical `AfLoadDialog` collects `user`/`proposed`/`generic` but **not**
`original/`.

**Open for Prof:** which of the above did you observe, or should the dialog
also list `original/`?

## 7. Still open (unchanged this session)

Emu fake-mag consistency for the FC `Mag[]` consumers (ExtMag clear on
SpeedyBee: `SensorQuadrant=2`, no X/Z flip) — the 100x gain is now removed from
the fleet, so a corrected-gain emu re-run is the next validation.

## 8. Files

* `uavx-python/src/airframes/fix_legacy_scaling.py` (new).
* `uavx-python/src/airframes/{generic,user,proposed,original}/*.af` and kit +
  `gitUAVXGS` mirrors (corrected).
* `UAVXArmQ/src/params.c:140-141,185`; `UAVXArmQ/src/inertial.c:408-410`.
* `uavx-python/src/parameters.py` (tag 31/109 display mult + `PARAM_LIMITS`).

## 9. Generic `.af` fleet normalization to canonical 128-param files (2026-09-17, afternoon)

**Request:** Greg "ensure the generic files are correct please" after the ×100
fleet fix, then chose **full normalize to canonical 128-param files** (not
header/alias-only, not leave-as-is).

**State found:** all 14 `generic/*.af` had correct `AF_TYPE`, the locked
angle/rate maxes + derived Q gains, and props-bits; but coverage was wildly
inconsistent (49..123 params), 9/14 carried a `# Unknown` header, partial files
relied on implicit FC-default fallbacks (differing per airframe → would
confound cross-frame studies), and the near-full files still carried legacy
alias keys (`EST_CRUISE_THR`, `KF_*`, `FW_ROLL_CONTROL_PITCH_LIMIT`).
`Shadow.af` uniquely had an empty `[LIMITS]` block.

**Adopted solution — `airframes/normalize_generic.py` (new, idempotent):**
every file is normalised to the full 128-param set in canonical key names via
`format_af`, where each omitted tag is **filled from the FC `ParamTable`
`default_val` column** (`params.c` `UseDefaultParametersEx`, `params.c:737-746`)
— making the implicit FC fallback explicit and uniform. Present values,
`PHYS_*`, and `Character` metadata are preserved byte-for-byte; legacy alias
keys are resolved through the existing `_LEGACY_PARAM_NAME_TO_TAG` bridge and
re-emitted canonically. The `# <name>` header is set to the airframe's proper
name (cosmetic — `parse_af`'s `read_name` is only logged). `Shadow.af`'s
missing `[LIMITS]` block is synthesised as the FW-fleet span (min/max of the
other 8 FW files' bands per tag) **unioned with Shadow's own stored values** so
every band contains its value; recomputed deterministically each run.

**Default resolution notes:** the GCS `PARAM_DEFAULTS` table is **stale and
display-unit** (it even carries the retired ×100 values for
`MADGWICK_KP_MAG`/`YAW_RATE_INT_LIM`) and was deliberately NOT used — the FC
ParamTable is the single source. Symbolic defaults resolve via the GCS enum
map + `_parse_value`; `DEFAULT_CONFIG1`/`DEFAULT_CONFIG2` via the
`Config1Bits.DEFAULT`/`Config2Bits.DEFAULT` IntFlag mirrors; four FC-only
symbols hard-coded numerically (`eLogUAVX`=0 tag 13, `eLandContactSw`=1 tag
100, `eTraceRate`=1 tag 125, `eUnknownReset`=0 tag 127); one arithmetic
default evaluated (`0.05 × 20` = 1.0, tag 66).

**Net effect per file:** partial MR frames gain 79 tags
(`IMU_FILT_TYPE`, `RX_*_CH` map, failsafe, filters, `MADGWICK_KP_MAG=0.005`,
`TRACE_TYPE=1` [= FC default `eTraceRate`, so parity with a fresh FC], …);
partial FW frames gain 71; near-full Dragon/SkySurfer/SmallSpoileron gain 5;
Shadow gains only its `[LIMITS]` block. A generic load now deterministically
sets the complete 128-param image instead of leaving stray FC state in place.

**Verification:** all 6 roots (canonical + linux/macos/windows kits +
`gitUAVXGS/uavx-python` + `gitUAVXGS/windows`) byte-identical for all 14 files;
128/128 params everywhere; every `[LIMITS]` band contains its value; locked
angle/rate maxes + derived Q gains verified unchanged on every root; re-run =
0 changes (idempotent); `fix_legacy_scaling.py` still reports 0 (no residual
mis-scales); `airframes_all_params.csv` regenerated (23 frames, 0 user) and
synced. Backup `/tmp/opencode/af_generic_backup_20260917.tgz` (84 files, 6
roots) taken before apply.

**Stale tool flagged:** `airframes/reformat_af.py` is unsafe (it calls
`format_af(name, values)` without metadata → silently drops every `PHYS_*`,
`# Character`, and `[LIMITS]` block) and `audit_af_fleet.py` is stale (flags
legit alias keys/absent-tag patterns → 60/60 noise).

## 10. md2pdf runs once, before the kit sync (2026-09-17)

**Request (Greg):** "can the script do the md2pdf update before updating the
toolkit?" + "going through each of the toolkits running md2pdf for each toolkit
is tedious."

**Change:** `scripts/sync_kits.sh` now runs `scripts/md2pdf.sh` **once** over
the canonical wikis (`$SRC_DIR/wiki`, plus the UAVXArmQ wiki at
`$HOME/Documents/Flight/gitHub/UAVXArmQ/wiki` when present) at the top of the
run, *before* the per-kit loop. The existing wiki rsync then carries the fresh
PDFs into every kit, so there is no per-toolkit pandoc pass. If `md2pdf.sh` or
pandoc is absent it warns and continues (PDF refresh is non-fatal; the source
sync still completes).

`scripts/update_all.sh` Step 3 no longer invokes the per-toolkit
`build_docs.sh`; it is now just an informational echo (PDFs handled in Step 2).
`build_docs.sh` is left on disk as a legacy standalone utility.

**Rationale:** `build_docs.sh` loops `for kit in linux macos windows` and
re-runs pandoc/xelatex over each kit's `wiki/docs` — the exact repetition Greg
flagged. `md2pdf.sh` is the newer incremental, recursive, emoji-sanitising tool
and already accepts multiple wiki roots in one invocation, so one canonical run
is both correct and cheaper.

**Verification:** `bash -n` clean on both scripts. Sandbox has neither pandoc
nor `rsync`, so the run was exercised only as far as the pandoc-missing warning
(continue-on-warning path works); the rsync flow is host-Merlin-only and
unchanged in form. Kit `generic/*.af` were propagated directly by
`normalize_generic.py` (independent of rsync) — all 3 kits confirmed 14 files ×
128 params, with `airframes_all_params.csv` + `normalize_generic.py` present.
