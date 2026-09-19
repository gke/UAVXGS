# Session Report — RC Switch Standardisation (iNav convention)

Date: 2026-09-02
Session: RC channel/switch mapping standardisation + `.af` channel-map save-bug fix

## Change summary

The default RC channel map (the `Map[]` default table in the FC ParamTable and
the GCS `RC_MAP_PARAMS` UI table) was swapped so UAVX now uses the **iNav
canonical layout** — the convention the user's Tx is already programmed for on
his other aircraft:

| Channel | Function | FC logical | Old default | New default |
|---|---|---|---|---|
| 4 | Arming (was NavQual) | `eNavQualificationRC` | 6 | **4** |
| 5 | AttMode | `eAttitudeModeRC` | 5 | 5 |
| 6 | NavMode (Cruise/Hold/RTH) | `eNavModeRC` | 4 | **6** |
| 7 | PassThru | `ePassThruRC` | 7 | 7 |

### Root cause of the "stuck at eReady / won't arm" report
Not a firmware regression. The user's Tx is programmed with **iNav AUX
assignments**, so the radio channel that UAVX maps to `eNavQualificationRC`
was the old default **ch6** (radio 0-based). At the switch position the pilot
expected to arm, that channel was not driven `>10%`, so `ActiveAndTriggered
(0.1f, eNavQualificationRC)` stayed false → `TxSwitchArmed` false → `Armed()`
false → stuck at `eReady`. The `RCControls` enum order is byte-identical to the
`UAVXArm32F4` reference fork, so nothing else had drifted.

### Files changed
- `UAVXArmQ/src/params.c`
  - `Map[eNavModeRC]` default 4 → 6 (was `RxGearCh`), comment "iNav-standard:
    ch6 NavMode (Cruise/Hold/RTH)".
  - `Map[eArmingRC]` (renamed from `eNavQualificationRC`) default 6 → 4,
    comment "iNav-standard: ch4 Arming (pot: >10% arm, >40% AltHold, >60%
    WPNav)".
- `UAVXArmQ/src/params.h` — `RCControls` enum: `eNavQualificationRC` →
  `eArmingRC` (same value 6, wire format unchanged).
- `UAVXArmQ/src/rc.c` — tag/usage references `eArmingRC`; failsafe comment
  update.
- `UAVXGS/uavx-python/src/ui/parameter_window.py`
  - `RC_MAP_PARAMS`: NavMode (RxGearCh) default 6, Arming (RxAux2Ch) default 4;
    "NavQual" label → "Arming". Order (by param index) otherwise unchanged.
  - `_raw_float_values`: added `QSpinBox` branch → `float(widget.value())`;
    previously the RC-map spins (a `QSpinBox`) fell through to `0.0`, so **every
    `.af` save wrote all `RX_*_CH = 0`**.
  - `_set_widgets_from_raw`: added `QSpinBox` branch so bulk FC readback and
    `.af` load populate the channel-map spins from stored values.
- `UAVXGS/uavx-python/src/protocol_enums.py` — `RCControl.eNavQualificationRC`
  → `eArmingRC`; dead `RC_MAP_EXPECTED` list updated to compile.

## Rationale and logical discourse

**Which convention wins?** The user flies more iNav aircraft than UAVX and wants
a single Tx model he can carry across every FC. Two candidate layouts were
considered:
1. **Keep UAVX enum order** (NavMode=ch4, NavQual=ch6). Pros: no code change.
   Cons: keeps the clash with the pilot's existing iNav muscle-memory/Tx wire-up.
2. **iNav convention** (Arming=ch4=NavQual, AttMode=ch5, NavMode=ch6). Pros:
   matches the user's other aircraft and the Tx he's already flying; his Tx is
   the harder thing to reprogram on the bench. Cons: requires the swap.

Adopted **iNav convention**, per the user's confirmation ("Yes it is a swap of
4 and 6"). The enum order in `params.h`/`protocol_enums.py` was **left
untouched** so the wire format and the `RCControl` int-to-name mapping are
byte-identical — only the *default* channel numbers changed. This keeps all
`RC[e*RC]` indexed code valid with no logic edits.

**Why `Map[]`-indirect code needs no changes.** Every functional read goes
through `Map[...]` or the `e*RC` enum, not through raw channel numbers:
- `InitRC` FS presets `FS[Map[eNavQualificationRC]].Raw=2000` and
  `FS[Map[eNavModeRC]].Raw=2000` (rc.c:836-837) follow the new default
  automatically.
- `MapRC`/`ActiveCh`/`ActiveAndTriggered` (rc.c:856-927) operate on the enum
  indices, so the swap is transparent.
- Unassigned logicals are false by default (`RC[]` inits 0, `ActiveCh` checks
  `DiscoveredRCChannels > Map[c]`), so e.g. the trace trigger
  (`RC[eAux2RC] >= 0.5f`) stays inactive until ch8 is assigned.
- The `emu.c` plant model never touches RC channels, so emulation is unaffected.

**Propagation to existing FCs without GCS re-flash of params.** Changing a
`Map[]` default changes `ParamTableCRC`. On next boot, `LoadParameters` sees a
table CRC mismatch with identical layout, so it **preserves calibration** and
reloads defaults (`pBootDiag=3`). The new map arrives on an already-flashed FC
the first time it boots the new firmware. No magic bump, no cal loss.

**`.af` save bug — the real reason maps never persisted.** The RC-map spins in
the Radio group are `QSpinBox`, and `_raw_float_values` only handled
`QDoubleSpinBox`/`QComboBox`, falling through to `0.0`. Hence every saved `.af`
stored `RX_*_CH = 0`, which is why the user's recent saves all show zero maps.
Fixed by returning `float(widget.value())`; mirrored the `QSpinBox` branch into
`_set_widgets_from_raw` so readback/load round-trips it. This means a saved `.af`
now actually carries the (new) channel map instead of silently zeroing it.

**Label rename.** "NavQual" → "Arming" in `RC_MAP_PARAMS` to match the user's
iNav mental model. The FC-side `eNavQualificationRC` enum identifier was also
renamed to **`eArmingRC`** (params.h enum order/value unchanged, so wire
format untouched): updated `params.h`, `rc.c`, `params.c`, and the GCS
`RCControl` enum + dead `RC_MAP_EXPECTED` list in `protocol_enums.py`.

## Verification
- FC build: `python3 scripts/fc_build.py` — **all 6 targets** compile clean
  (UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING [233748 B], FLYINGRCF4WINGMINI,
  BLUEBERRYF405).
- GCS: `python3 -m py_compile ui/parameter_window.py ui/main_window.py
  parameters.py protocol_enums.py protocol_constants.py` — clean.

## Remaining (user)
1. **Reprogram the Tx model(s)** to the new standard (arm switch >10% on ch4).
2. Re-test arming with the arm switch pulled above the 10% threshold on ch4.
3. Re-verify telemetry/trace/dive keep working on the newly assigned channels
   (ch8 still Trace, ch11 Dive, etc. — unchanged).
