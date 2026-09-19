# Session Report — AF-Type Combo Exhaustive Fix & No-Silent-Fallback (Sep-14)

## Summary
Fixed the bench-observed `.af` misclassification: FW airframes loading as "X Quadcopter",
Elevon models appearing as "Delta", and corrupted values being baked back into saved `.af`
files. The GCS AF_TYPE combo only ever contained the 10 `ACTIVE_AIRFRAMES`, and every
missed `findData()` called out to a **value-as-positional-index** fallback that selected the
WRONG airframe. The same fallback also existed in value collectors (`_raw_float_values`,
`_pending_live_float`, `_get_param_value`, `send_params_typed`), so the wrong selection was
silently written to `.af` saves and FC live-writes.

Per Greg's directive ("We definitely do not want to launch with some critical parameter
having been silently clobbered"), ALL index-as-value fallbacks were removed, including the
"keep current selection" variant that left a stale airframe displayed and would re-corrupt
on the next save.

## Root-Cause Analysis
- The two AF combos (general grid `GENERAL_PARAMS`, physics/setup box)
  were built from `ACTIVE_AIRFRAMES` = 10 name-sorted entries (Aileron…X Quad), i.e. a
  SUBSET of `AirframeType`. `REDACTED_AIRFRAMES` (Aileron, Spoilerons, AileronVTail,
  RudderElevator, DifferentialTwin) were present, but anything outside those 10 was unrepresentable.
- `findData(raw_value)` then returned −1 for e.g. `eAileronAF`(15) and the set-combo code
  fell back to `setCurrentIndex(15)` → Qt clamps to index 9 = **X Quadcopter**. A re-save
  (`_raw_float_values`, which reads `itemData`) then baked `eQuadXAF` back into the file —
  the file on disk was corrupt *after* being misdisplayed, so re-loading the corrupted file
  honestly reported the wrong-but-internally-consistent answer.
- `eElevonAF`(13) WAS in ACTIVE_AIRFRAMES (position 2), so the Elevon→Delta examples
  observed in the field came from files already corrupted in earlier sessions — once a `.af`
  stored `eDeltaAF`, every downstream consumer had no way to know it was wrong.

## Fix Design Decision (no silent fallback)
Two candidate designs were considered:

1. **Keep current selection + log** (the natural minimal fix). REJECTED — Greg's principle:
   a stale airframe stays displayed, and the next save silently writes the stale `itemData`
   → corruption relocated, not eliminated.
2. **Append-on-miss carrying the TRUE raw value** (adopted). If `findData` misses, the
   helper appends a marked `"? Unknown (N)"` item with `itemData == N` and selects it.
   Display is loud (clearly marked), and any save/FC-write round-trips the actual value —
   a corrupt/future value can never silently turn into a different value.

Because both the general and setup combos route every set through the same
`_resolve_combo_index`, any appended item appears in both; the setup⇄advanced sync and the
protected-param "No"-revert can no longer diverge.

## Changes
### `protocol_enums.py`
- Added `ALL_AIRFRAMES` — exhaustive, name-sorted list of all `AirframeType` members
  except `eAFUnknown` (26 entries). Combo items use `AIRFRAME_NAMES[af]` + `af.value`.

### `ui/parameter_window.py`
- Both AF combos built from `ALL_AIRFRAMES` (was `ACTIVE_AIRFRAMES`).
- New `_resolve_combo_index(widget, value)` — findData-only, append-on-miss with a
  `"? Unknown (N)"` item carrying the true value; logs; returns a valid index.
- All four combo-set sites use it unconditionally (dead `if cb_idx >= 0 else keep-current`
  blocks removed): `_set_widgets_from_raw` (load), `update_params_from_typed`,
  `update_params_from_packet`, readback-verify.
- Setup⇄advanced sync (`_setup_sync_to_advanced`, `sync_setup_from_advanced`),
  `param_changed`/`combo_changed` mirror-sync, and protected-param revert all route through
  `_resolve_combo_index`.
- Value collectors raise `ValueError` on a data-less combo selection instead of writing
  `float(currentIndex())`: `_raw_float_values`, `_pending_live_float`, `_get_param_value`,
  `_latest_widget_floats`.
- `load_default_params` index-0 startup zero-state left as-is (overwritten by FC readback).

### `ui/main_window.py`
- `send_params_typed` combo branch raises instead of `float(currentIndex())`.

### Unchanged (documented dormant / correct)
- `airframes.py` `export_af_from_widgets` / `import_to_widgets` (dead code; save path is
  `_raw_float_values`).
- `fw_group.setVisible(eElevonAF<=int(data)<=eVTOL2AF)` (data-based).
- Line-3481 PID-view string combo (`findData("angle")` on a text-only combo, non-param).

## Fleet Sweep
- All 14 `generic/*.af` verified internally consistent; load→save round-trip with zero drift.
- Prior corrupted `user/*.af` are unrecoverable-in-place; ALL 28 deleted per Greg's
  directive, source + 3 kits + gitUAVXGS. Safety copy added to
  `airframes_backup_20260912_211844/user/` (existing backup held `generic/` only).
- `airframes_all_params.csv` regenerated (23 frames, 0 user); `physical_specs_all.csv`
  pruned of `user/` rows.

## Verification
- `python3 -m py_compile` clean: `protocol_enums.py`, `ui/parameter_window.py`,
  `ui/main_window.py` ×5 trees (source + linux/macos/windows kits + gitUAVXGS mirror).
- Headless resolver regression: all 26 enum values resolve; missed value appends once with
  true `itemData`; repeat miss reuses the appended item (no duplicates); save path reads the
  true value, never the positional index.
- Kits (linux/macos/windows) + gitUAVXGS mirror synced for all changed files.

## Outstanding
- Issue A (disarm still "flying") and Issue B (trace combo None vs FC Rate) remain open
  separately — the empty 128-B trace dump and disarm-state work are tracked in AGENTS.md
  TODO entries. This session's fix addresses the `.af` misclassification only.
- Greg to re-open a generic Shadow load in the GCS: combo must show "Elevon", no dialog,
  save must round-trip `eElevonAF`.