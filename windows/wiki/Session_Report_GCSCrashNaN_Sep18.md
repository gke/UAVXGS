# Session Report — GCS NaN Actuator Crash + Mock Battery Reset (Sep 18 2026)

## 1. Context / symptoms

During an emulator flight the pilot reported "GCS crashed with a quick jerk of
the servos." The FC had emitted **NaN** in a tag-13 FLIGHT packet; the GCS
stopped logging at exactly that packet and aborted.

## 2. Evidence chain

- `uavx-python/src/log.txt` covers 10:51:07.012 → 10:51:19.515 (one 12.5 s
  session, 125 FLIGHT lines). Its **last** line is a FLIGHT packet at
  `10:51:19.515` with the following non-finite fields:

  ```
  acc_confidence=nan acc_du=nan acc_fb=nan acc_lr=nan alt_comp=nan
  altitude=nan roc=nan rate_pitch=nan rate_roll=nan
  pwm=nan,nan,nan,nan,0.5,0.5,0.0,1.0,0.5,0.5068359375
  ```

  Attitude (`q0..q3`, `angle_*`) and GPS fields were finite; the NaN set is
  exactly the emulator-derived accel/rate/actuator values.
- `~/Documents/UAVXGS/20260918_104847.rawlog` decoded with the shared
  `FrameDecoder` shows **no** NaN frames and its last decoded frame is a finite
  FLIGHT packet. This is consistent with the raw log losing its final buffered
  ~0.9 s when the GCS aborted: `RawLogWriter` writes through a buffered file
  object and a PyQt `qFatal()` abort does not flush it. So the log file, not the
  rawlog, is the authoritative tail.
- `RC_FLIGHT`/`FLIGHT` lines logged normally up to and including the NaN packet,
  so `process_packet` completed that packet and the abort happened **after** it.

## 3. Root cause

PyQt5 aborts the process (`qFatal`) on an unhandled Python exception raised
inside a Qt slot. The parameter window runs a 50 ms `QTimer`
(`parameter_window.py:817-819`) whose slot is `update_rc_display`, which calls
`_update_motor_display`. That function converted the actuator value with:

```
pwm_us = (flight_data.pwm[i] + 1.0) * 1000.0
scaled_val = max(0, min(1000, int(pwm_us - 1000)))   # int(nan) -> ValueError
...
label = f"{round(pwm_us)}" if show_us else f"{pct}%"  # round(nan) -> ValueError
```

`NaN + 1.0` is `NaN`, and both `int(NaN)` and `round(NaN)` raise
`ValueError: cannot convert float NaN to integer`. Because this runs in a timer
slot, the exception was unhandled -> GCS abort, within 50 ms of the NaN packet
being stored in `self.flight_data`. The `data_manager` observer path is wrapped
in `try/except`, so it swallowed the same error; the timer did not.

The console-dump block in `main_window.process_packet` had the same latent
hazard (`round((v + 1.0) * 1000)` for pwm, `round(rc[i])` for RC) but only runs
behind the 0.8 s rate gate, so it did not fire this time.

## 4. Fix (GCS only)

`ui/parameter_window.py` — `_update_motor_display`:
- Read `raw = flight_data.pwm[i]`; if `not math.isfinite(raw)`, render that bar
  as invalid (`---`, bar 0, red label) and `continue` — never reach `int()/round()`.
- Track `has_nan` across the frame; log a single `Error` trace per contiguous
  NaN episode via `_motor_nan_warned` (initialised in `__init__`), reset when a
  fully finite frame arrives. This keeps the FC anomaly loud without 20 Hz spam.

`ui/main_window.py` — console dump:
- Guard the pwm and RC `round()` calls with `math.isfinite`, printing `nan`
  rather than raising.

Rationale / rejected alternatives:
- We did **not** sanitize NaN out of `process_packet` at the boundary
  (silently replacing with 0), because that would hide a genuine FC fault; the
  GCS should display "invalid" and report it. The defensive guards are local to
  the conversions that would otherwise abort.
- We did **not** add a blanket `try/except` around `process_packet` because it
  would mask the class of slot bugs rather than fix the specific unsafe
  conversion.

## 5. Mock battery reset on flight-mode exit

`MockBattery()` (`batt.c`) integrates `BatteryChargeUsedmAH`, so in emulation a
second flight starts on a partially-drained pack. Added:

```c
void MockBatteryReset(void) {
    if (F.Emulation) {
        BatteryChargeUsedmAH = 0.0f;
        BatteryCurrent = 0.0f;
        BatteryTimeRemainingSecs = 0;
        BatteryVolts = StartupVolts;
    }
} // MockBatteryReset
```

Declared in `batt.h`. Called at **both** transitions out of `eInFlight` in
`uavxarm-v3-gke.c`:
- throttle-low / disarm -> `eLanding`
- `UpsideDownMulticopter()` -> `eShutdown`

Emulation-gated so real hardware battery accounting is untouched. A dedicated
reset (rather than re-calling `InitBattery`) avoids the 1 s blocking ADC settle
loop and only clears the mock state.

## 6. Build / verification

- FC: all 7 targets build clean (`UAVXF4V3`, `UAVXF4V4`, `DEVEBOXF4`,
  `SPEEDYBEEF405WING`, `FLYINGRCF4WINGMINI`, `BLUEBERRYF405`, `MATEKF411WING`).
  `SPEEDYBEEF405WINGQ_r0.bin` = 236156 B.
- GCS: `py_compile` clean for the two source files plus all 5 mirrors
  (`linux/src`, `macos/src`, `windows/src`, `gitUAVXGS/uavx-python/src`,
  `gitUAVXGS/windows/src`).

## 7. Open items

- The **FC** produced NaN (accel/rate/actuator) in the emulator. The GCS now
  tolerates it, but the FC NaN source is a separate investigation (likely the
  emulator plant/estimator on an edge state). Capture a raw log at the event
  and trace `emu.c` / inertial synthesis.
- Greg: the terminal traceback (if captured) would confirm the exact slot; the
  fix above addresses the identified `_update_motor_display` abort regardless.
