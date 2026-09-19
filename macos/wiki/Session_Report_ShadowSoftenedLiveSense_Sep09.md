# Session Report — Shadow.af Softened + Live SERVO_SENSE — Sep09

## Scope
1. **`generic/Shadow.af`**: rate gains down ×50, rate Kd zeroed, RC map made sequential 0–11.
2. **FC**: `PWSense[]` now recomputed immediately on a live tag-17 write of `SERVO_SENSE` (tag 51), not deferred to boot/commit.
3. **GCS**: tooltips updated to reflect live application.

## Changes

### `generic/Shadow.af` (source `uavx-python/src/airframes/generic/Shadow.af`)
Gains reduced by factor-of-50 on the flight-controller rate P gains, Kd zeroed:

| key | before | after |
|---|---|---|
| `ROLL_RATE_KP` | 0.53476 | 0.0106952 (÷50) |
| `PITCH_RATE_KP` | 0.33422 | 0.0066844 (÷50) |
| `YAW_RATE_KP` | 0.33422 | 0.0066844 (÷50) |
| `ROLL_RATE_KD` | 0.0168 | 0.0 |
| `PITCH_RATE_KD` | 0 | 0 |
| `YAW_RATE_KD` | 0.00112 | 0.0 |

RC map made sequential:

| RC param | before | after | logical |
|---|---|---|---|
| `RX_THROTTLE_CH` | 0 | 0 | Throttle |
| `RX_ROLL_CH` | 1 | 1 | Roll |
| `RX_PITCH_CH` | 2 | 2 | Pitch |
| `RX_YAW_CH` | 3 | 3 | Yaw |
| `RX_GEAR_CH` | 4 | 4 | NavMode |
| `RX_AUX1_CH` | 5 | 5 | AttMode |
| `RX_AUX2_CH` | 6 | 6 | Arming |
| `RX_AUX3_CH` | 10 | **7** | CamPitch |
| `RX_AUX4_CH` | 8 | 8 | Trace |
| `RX_AUX5_CH` | 9 | 9 | Aux5 |
| `RX_AUX6_CH` | 7 | **10** | Aux6 |
| `RX_AUX7_CH` | 11 | 11 | Aux7 |

Rationale: the file's RC map was scrambled (AUX3=10, AUX6=7). 0–11 sequential
matching the widely-used canonical ordering. `user/Shadow.af` already had a
sequential map and was deliberately left untouched.

`[LIMITS]` block updated to contain the reduced values (rate P lo 0.2674→0.005 /
0.1671→0.003, rate D lo 0.0084→0 / 0.00056→0). Rationale: the ParamTable class
ceiling (`eClassGainRateP` 0–30, `eClassGainRateD` 0–2.0) allows these values;
the per-file `[LIMITS]` window is the only constraint, so it had to come down or a
reload would widen/be clamped. FC `param.c` ParamTable entries unchanged (class
bounds already contain the values — no table edit needed, no `ParamTableCRC` bump).

`SERVO_SENSE = 9` and its meaning for the mirrored-wing convention was NOT changed
— this session is purely gains + mapping + live-sense, not the sense value itself.

### FC — `UAVXArmQ/src/telemetry/telem.c` `ProcessParamsWrite()`
After writing the target + `Config.ParamData[tag]` for **any** tag, if the written
tag is `ServoSense` (51) call `InitServoSense()` immediately:

```c
Config.ParamData[UAVXPacket[3]].f = u.f;
ConfigChanged = true;

// Recompute the derived PWSense[] servo-sense table now (not just
// at boot/commit) so a live tag-17 SERVO_SENSE write takes effect
// immediately on the control surfaces.
if (UAVXPacket[3] == ServoSense)
    InitServoSense();
```

Previously `PWSense[]` was rebuilt only by `InitServoSense()` reached via
`ApplyParameters()` at boot/tag-72-commit. A live write updated `pServoSense` +
`ParamData[51]` and echoed the ACK but had zero effect on the servos until reboot.
This is why Greg's "changing servo sense changes nothing on the surface" symptom
occurred in the first place.

Design decisions / options considered:
- **Recompute in `ProcessParamsWrite` directly** (chosen): the write handler already
  owns the target write + clamp; `InitServoSense()` is tiny (7 × iota of float
  assignments) and iterates only `MAX_PWM_OUTPUTS`, so calling it inline is
  real-time safe (no heavy math, no flash). No ordering hazard: it reads
  `pServoSense`, which `ProcessParamsWrite` has already updated.
- **Gate on ground state** (rejected): `ApplyParameters()` gates its derived-value
  block on `ePreflight/eReady/eMonitorInstruments`. For SERVO_SENSE there is no
  reason to defer — reversing a surface's sense in-flight is the pilot's explicit
  command on a live write; gating it would reintroduce the "change does nothing"
  confusion. The GCS already live-writes every tag-17 edit; this makes SERVO_SENSE
  behave like the rest.
- **Make tag 51 boot-required** (rejected): that would force a commit+reboot for a
  simple sense toggle. Overkill; live recompute is cheaper and immediate.

`ServoSense` enum value (params.h `enum Params`) is 51 — verified.

### GCS — `uavx-python/src/ui/parameter_window.py`
- Section header comment block (lines ~3231): "Applies at FC boot/commit ... not on
  a live param write" → now documents live recompute in `ProcessParamsWrite`.
- Per-checkbox tooltip (line ~3251): "Applies at FC boot/config commit." →
  "Applies immediately (live param write)."

No functional GCS change; the tag-17 debounced live-write path was already correct.

## Build / verification
- FC: `python3 UAVXArmQ/scripts/fc_build.py` — all 7 targets OK
  (`UAVXF4V3`, `UAVXF4V4`, `DEVEBOXF4`, `SPEEDYBEEF405WING`, `FLYINGRCF4WINGMINI`,
  `BLUEBERRYF405`, `MATEKF411WING`).
- GCS: `python3 -m py_compile uavx-python/src/ui/parameter_window.py` — clean.
- The flash artifact to load remains the default
  `obj/SPEEDYBEEF405WING/SPEEDYBEEF405WINGQ_r0.bin` (regenerated this session).

## Test prescription (Greg)
1. Load `generic/Shadow.af` into the GCS and confirm the RC map reads 0–11 in order
   and the rate gains show the ÷50 values.
2. Arm on the bench, flip a SERVO_SENSE checkbox — the surface must now reverse
   immediately **without** a reboot (that's the whole point of the telem.c change).
3. Bench rate-stability sanity: with rate P ~0.01 / Kd 0 the attitude loop
   (`ROLL/PITCH_ANGLE_Q_KP 7.8 / 1`) is the remaining authority — expect a soft,
   sluggish surface response. This is intentional (diagnostic / re-tune starting
   point).