# Servo Sense → GCS Fixed Wing Checkboxes (2026-09-05)

## Change
GCS Param page, Fixed Wing group box: replaced the raw numeric
"Servo Sense" spinbox (tag 51, 0..127 bitmask) with **seven labelled
checkboxes**, one per FC servo output, in the FC bit layout order:

| bit | checkbox label | FC output (`SM[m]`) | PWSense when checked |
|-----|----------------|---------------------|----------------------|
| 0 | Right Aileron | `RightAileronC` | -1.0 (reversed) |
| 1 | Left Aileron | `LeftAileronC` | -1.0 |
| 2 | Elevator | `ElevatorC` | -1.0 |
| 3 | Rudder | `RudderC` | -1.0 |
| 4 | Spoiler | `SpoilerC` | -1.0 |
| 5 | Cam Roll | `CamRollC` | -1.0 |
| 6 | Cam Pitch | `Aux1CamPitchC` | -1.0 |

File: `uavx-python/src/ui/parameter_window.py` (GCS only — no FC change).

## Rationale / discourse
The FC already exposes exactly this bit vector: `pServoSense` (tag 51,
`U8(&pServoSense, eClassExplicit, 0, 127, 0, "bitmask")` in params.c:202) is
consumed by `InitServoSense()` (mixer.c:441-453), which walk
`SM[] = {RightAileronC, LeftAileronC, ElevatorC, RudderC, SpoilerC, CamRollC,
Aux1CamPitchC}` and derive `PWSense[SM[m]] = -1.0f` per set bit. So:

- **Reused the existing param** — no FC protocol change, no new tag, the
  `.af` and tag-71 readback/tag-17 write paths work unchanged.
- **Checked = reversed** matches the FC bit semantics directly (bit SET →
  `PWSense = -1.0`).
- **Generic spinbox removed from General/Camera group** — a raw `0..127`
  number tells a pilot nothing; the checkbox labels are self-documenting.
  Param 51 is now backed by the existing hidden-spinbox fallback
  (range 0..255, value 0), satisfying the generic write/readback loop.
- **`_display_mult(51) == 1.0` in BOTH normal and legacy mode**
  (PARAM_DISPLAY_MULT=1.0, PARAM_SCALES=1.0, not in LEGACY_TAGS) so the
  spinbox mirrors the raw bitmask exactly — no conversion hazard.
- **Write path via the existing machinery**: `servo_sense_bit_changed()`
  pushes the new bitmask into the hidden spinbox; its
  `valueChanged → param_changed` connection marks dirty, fires the
  protected-param confirm (tag 51 is in `_PROTECTED_PARAMS`) and queues the
  tag-17 write. Declined confirm reverts the spin; `update_config_display()`
  is re-run by the handler so the checkboxes re-sync (the standard
  `update_config_display()` tail of `param_changed` doesn't run on the
  decline path).
- **Readback sync**: `update_config_display()` now also reads the tag-51
  spinbox and drives `servo_sense_checks`, so connect/download, tag-17 echo
  ACK and `.af` load all check the checkboxes from the FC value.

Alternatives considered:
- A dedicated `_config_values`-style cache for tag 51 — rejected (the
  spinbox already is the source of truth; a second cache would need syncing
  at every readback site).
- Representing sense per physical driver index (0..9) instead of per
  `SM[]` designation — rejected: `PWSense[]` appliers in mixer.c index by
  the *designation* constants (RightAileronC etc.), not by raw driver
  number, so the SM[] order is the only faithful mapping.

Note on apply timing: `PWSense[]` is derived only in `InitServoSense()`
(boot, outputs.c:257, and UseDefaultParametersEx, params.c:667) — a live
tag-17 write stores the new `pServoSense` but does NOT re-derive PWSense
mid-session. Checkbox tooltips say "Applies at FC boot/config commit".
This is the safe behaviour (reversing a servo live could fly an aircraft
into the ground); a commit (tag 72, flash+restart) applies it. No FC
change was needed.

## Verification
- `python3 -m py_compile src/ui/parameter_window.py` — OK.
- FC unchanged this session: PWSense mapping verified against
  mixer.c `SM[]`/InitServoSense and mixer.h (6-target build unaffected).
- No FC rebuild required (GCS-only change), but next FC build/session
  should re-confirm 6 targets still clean (should be, no C touched).