# Session Report — Mag Config-Register Instrumentation (bench debug aid)

**Date:** 2026-08-31
**Components:** `UAVXArmQ/src/sensors/hmc5xxx_mag.{c,h}`, `UAVXArmQ/src/telemetry/telem.c`,
`UAVXGS/uavx-python/src/packet_parser.py`, `UAVXGS/uavx-python/src/ui/calibration_window.py`
**Build/verify:** `fc_build.py` SPEEDYBEEF405WING ✓ + BLUEBERRYF405 ✓ (shell env had BOARD set);
`python3 -m py_compile` on both edited GCS modules ✓

## Motivation

Bench workflow needs to see the **actual magnetometer configuration as read back
from the chip** to cross-check that the FC's programmed config actually took (bus
glitches, mis-wired SDA/SCL, wrong board pinout). The Calibration (mag) tab already
shows the live per-axis mag values; it showed nothing about the chip's config
state. This adds the three register bytes that define gain/sample rate/continuous
mode to the existing calibration telemetry packet.

## Change

### FC

- `hmc5xxx_mag.c`: new globals `uint8 MagCfgA, MagCfgB, MagMode;` plus
  `SnapshotMagRegisters()` which reads back `CNTL/CFGA/CFGB` after the init
  registers are written. Called at the end of `InitMagnetometer()` (post-config
  write) for the HMC5XXX probe path; zeroed in the no-sensor `else` so a dead
  sensor shows 0x00 rather than stale data.
- `hmc5xxx_mag.h`: prototype + `extern` declarations for `telem.c`.
- `telem.c` `SendCalibrationPacket`: appends the 3 register bytes
  (`MagCfgA, MagCfgB, MagMode`) — packet body grows 117 → 119 bytes.

Reading the registers back is a 3-transaction bus exercise per boot — done once
at init, never in the control loop. No behavioural/control impact.

### GCS

- `packet_parser.py` `parse_calibration` (tag 62): parses the optional trailing
  bytes into `mag_cfg_a/b`, `mag_mode`; stays `None` when talking to an older
  FC build (guard `len(data) >= 119`; existing `len < 116` reject unchanged).
- `calibration_window.py`: mag chip label gains
  `[A=0x.. B=0x.. MODE=0x..]` when present.

## Reading the bytes

Typical healthy HMC5883L config = **A=0x70** (8-sample avg, 15 Hz, normal),
**B=0xA0** (gain ±4.7 Ga / 1370 LSB/G), **MODE=0x00** (continuous).
QMC5883L-compatible units differ (different register map/defaults) — the debug
value tells us *which* map the chip answers to, which is the whole point when
"mag present but heading frozen" shows up on the bench.

## Alternatives rejected

- **Periodic re-read of registers** (mag frame cycle): unnecessary bus traffic in
  the hot path for info that only changes at init; a one-shot snapshot at init is
  sufficient and cheaper.
- **D8 telemetry carrier**: the D8 downlink is already near its headroom and the
  bench session uses the USB/wired link; calibration packets ride the existing
  telemetry stream, zero marginal cost there.
- **New dedicated tag**: the calibration packet is exactly the payload the mag
  tab already subscribes to; a new tag duplicates the transport for three bytes.

## Follow-ups (initial)

- None at time of writing; superseded by the bench findings below, which
  escalated into the WDT root-cause fix and the reflash follow-up.

## Bench findings (same day, F4V3 test board)

The instrumentation immediately earned its keep:

- tag-62 arrives at **len=121** on the wire = SOH(1)+tag(1)+len(2)+**116-byte
  body**+chk(1) — i.e. the bench FC was still running the **pre-instrumentation
  build**, so `[CALIB]` showed `mag=[A=-- B=-- MODE=--]` (the GCS's None-safe
  format confirmed the shorter packet rather than a protocol regression). See
  "Follow-ups (updated)" — reflash is required before the registers show.
  (The observed 119-byte body would present as len=124.)
- `mag_id=0x48` = the HMC identity byte ('H') read in `ProbeMagat` — the chip is
  a real HMC5883/(5983) clone, `F.MagnetometerActive` set, `UseMag` on. The
  calibration loop **ran**: `mm` climbed 0→226, octant bins filling one at a
  time (bin cap 55) as the board was rotated. Healthy behaviour.
- `mm` freezing at 226 with bins `{7,5,1,0,4}` full and `{2,3,6}` empty is just
  the samples not visiting those orientations; target is
  `MAG_CAL_SAMPLES = 8×50 = 400`. Keep rotating through all axes.
- **False alarm (fixed):** the window printed "⚠️ Mag sensor not active -
  calibration cannot proceed" at a rate of one per telemetry tick for the first
  seconds, then stopped as soon as a tag-62 carrying `Mag=Y_` landed. Root
  cause = startup race: `_mag_active` defaults False until the first calibration
  packet arrives, and the alarm fired while `mag_calibration_requested` was
  already set (stale from a prior session). Fixed by tracking `_mag_status_seen`
  (set when a tag-62's flags decode) and only alarming/refusing on confirmed
  inactive; the "no magnetometer" refusal in `start_mag_calibration` is gated
  the same way, and both messages are deduped to once per request.
- **Reboot mid-calibration: ROOT CAUSE FOUND + FIXED (later same day).** The
  reboot was confirmed a real self-reset: next session's boot line read
  `[RST] last reset cause: 3 (independent-watchdog)` plus
  `[WDT] watchdog trip MARK=32 (i2c-ev-start-wait)`. Chain of cause:
  - `WdtFeed()` is called **only** at the top of the main task loop
    (`uavxarm-v3-gke.c:210`); no ISR feeds the IWDG.
  - Ground reload = 4095 steps × 2 ms = **8.19 s** leash (`wdt.c:51`); fed each
    loop pass, so normal running never comes near it.
  - `CalibrateMagnetometer()` runs inside `ProcessRxPacket` (telem.c — main-loop
    context) and is a **blocking** `do-while` with up to a **60 s** timeout
    (`hmc5xxx_mag.c:300`). While it spins, the main loop never iterates and the
    IWDG is never fed → any cal session taking >8.19 s (ours froze at mm=226
    waiting for octants {2,3,6}, then overshot the leash) trips the watchdog.
    `MARK=32` is just the last I2C EV-START marker written before the un-fed
    stretch — a marker of convenience, not where the "hang" was.
  - `CalibrateAccAndRate()` (`imu.c:230`) has the same flaw differently shaped:
    after pass 0 it waits for a **10 °C temp rise** (`RangeT=10`, `imu.c:262`)
    in a `Delay1mS(100)` spin — minutes on a cold bench → same IWDG trip.
    `CalibrateAccZeros()` (~600 ms, 300×2 ms) is inside the leash, but fed too
    for uniformity.
  - **Fix (this session):** `WdtFeed()` added inside all three cal loops — top
    of the `do-while` in `CalibrateMagnetometer` and `CalibrateAccAndRate`, top
    of the sample `for` in `CalibrateAccZeros` — plus feed points around the
    post-loop `SphereFit` and `RefreshConfig()` (long pure-CPU + flash
    erase/program, still inside the blocking handler). Loop-top feeding preserves
    hang detection: a *stalled* read/interrupt still outruns the 8.19 s leash
    and trips; only a *live, progressing* calibration survives past 8 s, which
    matches the wdt.c design intent ("calibration ... legitimately fence long;
    recover slowly" — it just was never taught to feed).
  - Build/verify: SPEEDYBEEF405WING ✓ BLUEBERRYF405 ✓; GCS py_compile ✓.
- GCS crash on the old-FC `[CALIB]` print fixed: `main_window.py` now formats
  `mag_cfg_*` through a None-safe helper (`--` when the older 116-byte packet
  lacks the registers) instead of `TypeError` — this also covers replay of
  pre-reflash `.rawlog` files.

## Comment policy (clarified, 2026-08-31)

AGENTS.md Coding Standard #4 refined: during refactoring preserve ALL code and
error handling, KEEP comments that still reflect what the code does, DELETE
comments that no longer reflect the code's function (a stale comment misleads
the next reader more than its absence), and NEVER remove the main file-header
comment block (copyright/licence/attribution header) from any file. Applied in
this session: the accidentally dropped "ellipsoid" comment was restored (it is
still a true statement of the code's intent); the new `WdtFeed()` comments say
*why* (feed through long cal, hang still trips) rather than the mechanics the
call itself shows.

## Post-reflash bench run (same day)

Reflash confirmed: tag-62 now at **len=124** (119-byte body). `mag=[A=0x78 B=0x78 MODE=0x00]`
matches exactly the `HMC5883LConfig[]` the init writes — **the register-snapshot
instrumentation works and the chip is a genuine HMC-class part** (readback echoes
programmed config, `mag_id='H'` at reg 0x0A). Decode: A=8-sample avg/75 Hz/normal,
B=±2.5 Ga/660 LSB/Ga, MODE=continuous.

Calibration then **completed**: `Octants: [55,49,55,55,22,55,55,55] total(mm)=401`,
ACK success, `IMU=YC Mag=YC`, and the fit survived the post-cal FC restart (the
`State=Init→Ready` relaunch is the intentional apply-config `systemReset` on
`TelemetrySerial`, telem.c). Octant 4 under-filled (22) but the 6 capped bins give
a solid sphere.

## Raw mag magnitude "16000 — too large for 12 bits" (resolved, non-issue)

The HMC5883-class ADC is 12-bit, but the sample is **left-justified** in the 16-bit
data register (low nibble = sign/status pattern), so raw int16 ∈ ±32767 is normal.
`ReadMagnetometer()` takes the full int16 (`hmc5xxx_mag.c:131`) and the `-4096`
"data-not-ready/overrange" sentinel the code already checks is defined in that same
left-justified layout — internally consistent. Raw ≈16000 ≈ 1000 LSB × 16 ≈ 1.5 Ga
at the ±2.5 Ga setting — plausible with bench iron. Heading `-atan2(yh,xh)` uses the
**ratio** of the horizontal components, so a uniform gain factor cancels; the
sphere-fit bias is fit in the same units. **No scaling bug; nothing to change.**

## Mag-cal integrity — "completes" on a failed partial run (FIXED)

### Symptom (user report)
- Second/third cal runs greyed out the Start button, the octant bars **stopped
  filling mid-way**, yet the FC eventually "completed" — and the compass then read
  **180° off** pointing north (reported N→360, E→270, i.e. **mirrored**: E reads
  270 instead of 90). The stored fit was garbage.

### Root cause
- If the user pauses/sets the board down before the sweep is done, the current
  octant bin (cap 55) fills, `mm` freezes, and the collection **only terminates via
  the 60 s timeout**. The **old FC then unconditionally** ran `SphereFit`, set
  `Calibrated=1`, wrote flash and **acked true** ("success") on the sparse,
  biased samples — a bad hard-iron fit that displaced the previous good one.
- The GCS ignored the tag-52 misc ACK entirely ("No pending request for tag 50/52")
  — it had no way to report the failure, and its finish detection (Calibrated bit in
  a later tag-62) could be lost across the post-commit FC restart.

### Fix
FC (`hmc5xxx_mag.c`):
- **Success is now earned, not assumed**: `CalibrationOK = !Timeout && (mm > MAG_CAL_SAMPLES)`.
  A timed-out/partial sweep **fails**: previous fit is restored
  (`SaveMagCalibration`/`RestoreMagCalibration`, incl. `F.MagnetometerCalibrated`),
  **nothing written to flash**, ACK **false**, red LED + 8 beeps.
- **`-4096` samples skipped**: the "data-not-ready/overrange" marker must never
  enter the fit (`!F.MagnetometerFailure` guard around sampling) — a saturated read
  is not a field sample.
- Completion block extracted to `CommitMagCalibration(s)` (fit → store → persist →
  ACK true) to stay within the 100-line cap; all WdtFeed points preserved.
- Refusal path (`!MagnetometerActive || !UsingMag`) still ACKs false, unchanged.

GCS:
- `main_window.py` now remembers the last MISC command sent
  (`_pending_misc_command`) and routes the tag-52 ACK to `calib_window.on_misc_ack`.
- `calibration_window.py::on_misc_ack`: CAL_MAG ACK **false** → abort state, show
  "⚠️ FAILED — incomplete sweep or timeout", re-enable Start, previous fit retained;
  ACK **true** → `_finish_mag_calibration()` immediately (no dependency on a later
  tag-62). The bit-driven finish remains as a fallback if an ACK is lost.

### Build/verify
`fc_build.py` SPEEDYBEEF405WING ✓ + **UAVXF4V3** ✓ (bench F4V3 test board target) +
BLUEBERRYF405 ✓; GCS `py_compile` ✓.

## Heading mirror (N→360, E→270) — pending verification, do NOT patch the formula yet

The reported mapping is `heading ≈ -true_heading` (E reads 270 not 90, N reads
360≡0), i.e. a **horizontal-axis sign flip** on this board's mag mount — but it was
*measured on the garbage fit*, so a sign conclusion is not yet justified. Sequence:

1. Reflash the fixed FC; clear/re-run a **full sweep** (keep rotating all 8 octants
   until `mm=401` is reached; don't pause).
2. Re-measure N/E/S/W with the clean fit.
3. Only if the clean-fit compass **still** reads 270 at East does this become a
   mount-sign issue — and then it is fixed **per-board** (mag axis sign/quadrant in
   the sensor mount config), **never** by changing the global `-atan2f` heading
   formula (that would break every other aircraft).

## Follow-ups (updated)

- **Reflash the F4V3 bench FC (target `UAVXF4V3`) with the fixed build** and
  complete a full mag sweep
  (`mm` must reach 401; the run now FAILS cleanly instead of persisting garbage).
- After a clean fit: re-measure the compass N/E/S/W. If East still reads 270 → per-
  board mount-sign correction (see "Heading mirror" above).
- Octant 4 was under-filled in the good run (22) — rotate especially through the
  "nose down/up" octants on the retry.