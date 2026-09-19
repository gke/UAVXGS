# Session Report — Ecks 220 mm Tumble (low flashed gains) + ESC-Prog boot-trap watchdog

Date: 2026-09-04 (Prof Greg & assistant)

## Summary

Two distinct items were resolved this session:

1. **Root-caused the Ecks 220 mm in-flight tumble** ("still tumbling, partially
   recovers when disarmed") to a set of **catastrophically low gains that had been
   flashed into the FC** — NOT a deficiency in the `.af` tuning. A duplicate
   "bad" `.af` file (saved a minute before the good one) carried those low gains and
   was evidently the source flashed at some point.
2. **Fixed a genuine boot-time "no way out" trap in the ESC-Prog 4way/MSP path**
   (`escprog/serial_4way.c`): a stalled ESC programmer could block `readByte()` forever,
   pinning boot before IMU/altitude/control init. Added a per-byte watchdog so the
   session always aborts and boot proceeds.

## 1. Ecks tumble — low-flashed-gains root cause

### Symptom
Rawlog `20260904_155922_Ecks_220mm.rawlog` (87 s, 768 Flight frames). At ~44.7 s the
aircraft entered a sustained tumble: `desired_pitch` held at a constant 30° while the
measured pitch blew through to ±90° and rolled ±180° repeatedly
(limit-cycle through the vertical). `acc_confidence` dropped to 0.00 for the whole
tumble (accelerometer attitude-anchor disabled once inverted), and `rate_pitch`
saturated at ~300°/s while `desired_rate_pitch` capped at ±120°/s — i.e. the aircraft
rotated ~2.5× faster than the controller even demanded.

### Comparison: `.af` vs what the FC actually ran
| Gain | `.af` (sim-PASS) | FC-flashed (from tag-71 log) |
|---|---|---|
| ROLL/PITCH ANGLE_Q_KP | 7.0 | **1.75** |
| ROLL/PITCH ANGLE_Q_KI | 0.25 | **0.0125** |
| ROLL/PITCH ANGLE_Q_INT_LIM | 0.01 | 0.01 |
| ROLL RATE_KP | 0.34 | **0.0015** |
| PITCH RATE_KP | 0.34 | **0.00225** |
| YAW RATE_KP | 0.11 | **0.00375** |
| ROLL/PITCH RATE_KD | 0.008 | **~2e-06** |
| YAW ANGLE_Q_KP | 3.0 | **0.75** |

Confirmed by sim (`test_pid_sim.py`, `simulate_axis`, MR plant):
- `.af` gains → rise 0.30 s, overshoot 0.3 %, settle 0.68 s, no overshoot — **passes**.
- Flashed gains → rise **4.22 s**, settle >8 s — ~14× slower, under-damped; exactly the
  regime that turns a 30° command into a blow-through-to-vertical tumble.

**Conclusion:** the gains in the `.af` are healthy (angle Q/Kp=7 is mid/upper fleet),
and our framework (now triangulated by iNav and ArduPilot) reproduces a clean hold.
The tumble was caused by the **flashed** values being ~5–200× too low
(angle-Kp 1.75, **rate-Kp 0.0015** = effectively no rate authority).

### The "bad" `.af` file trap
Two Ecks user files were saved a minute apart on 2026-09-04:
- `user/Ecks_220mm_20260904_162249.af` — **750 g**, carried exactly the bad gains
  (rateKp 0.0015/0.0022/0.0037, angleKp 1.75, angleKi 0.0125, rateKd 0).
- `user/Ecks_220mm_20260904_163040.af` — **910 g** (the good one, matches the loaded
  `[R/O] Ecks_220.af [910g, 9.0in, 4M]` label), angleKp 7 / rateKp 0.34 etc.

The bad 162249 file was **deleted** this session. Lesson for future sessions: the
`910g, 4M` descriptor (mass) is the reliable discriminator; verify the FC's *stored*
gains against the intended `.af` after any load (read via tag-71) rather than trusting
the pulled-down file name.

### Not the cause
- `ESCProg` (Config2 bit 2) was **CLEAR** in the flown FC (Config2Bits = 0x4B =
  BattComp|FastStart|GPS|NavBeep, the `DEFAULT_CONFIG2` value) — the tumble was not
  an ESC-Prog problem.
- CONFIG1_BITS read 0 in the log — a cleared/minimal config, consistent with the
  bad-gains save state (separate anomaly, noted).

## 2. ESC-Prog boot-trap watchdog (code change)

### The trap
`CheckESCProg()` runs **once at boot** (`uavxarm-v3-gke.c:165`) before
`InitIMU()/InitAltitude()/InitControl()`. If `UseESCProg` (Config2 bit 2) is set:
- `DoESCProg()` waits up to 5 s for a programmer (`mSTimer(ESCProgTimeoutmS, 5000)`),
  then enters the MSP (`DoMSPCmds`) and 4way (`esc4wayProcess`) layers.
- `esc4wayProcess`'s `readByte()` was a bare `while (!SerialAvailable(port));` with
  **no timeout** (the code even had a `// need timeout?` comment). A stalled/partial
  session hangs there forever.
- Because boot is blocked **before** the GCS/tuning link is usable, a stuck session
  cannot be recovered by clearing the bit — the classic "no way out" dead-end. Recovery
  was only a full flash erase (zeroing the config byte) or a hardware bootloader jump.

### The fix (`escprog/serial_4way.c` + `.h`)
- New compile-time constant `ESC_PROG_BYTE_TIMEOUT_MS 2000`.
- New flag `esc4wayAborted` (module + extern in `serial_4way.h`).
- `readByte()`: arms `mSTimer(ESCProgTimeoutmS, ESC_PROG_BYTE_TIMEOUT_MS)` per read and
  waits with the timeout; on trip sets `esc4wayAborted=true` and returns a harmless
  0 byte. A live tool re-arms on every byte, so only a genuinely silent link aborts
  (a normal BLHeliSuite session never trips it).
- `esc4wayProcess()`: resets `esc4wayAborted=false` and exits its loop on
  `!esc4wayExitRequested && !esc4wayAborted`, calling `esc4wayRelease()` as before.
- `DoMSPCmds()`: `GetMSPPacket` is non-blocking but its `do…while(mspContinue)` would
  busy-spin forever if the tool never sent `MSP_SET_4WAY_IF`; added a fresh arm before
  the loop, re-arm on each received frame, and `mspContinue=false` when the timeout
  trips.
- Result: `DoESCProg()` always returns within ~2–3 s → boot proceeds → normal
  GCS/tuning path is never locked out.

### Design notes / alternatives considered
- **Rejected**: relying on `esc4wayExitRequested` alone (only set by an explicit tool
  command) — does not cover a stalled/unresponsive tool.
- **Rejected**: a CPU-bound global timeout around the whole session — unnecessary and
  would mis-handle slow-but-alive tools; per-byte re-arm is both correct and minimal.
- **Per-byte timeout (2 s)** chosen over a longer global one: bounds a stall to ~2 s,
  never trips a session where bytes keep flowing (flash ops re-arm on their own).
  A user who genuinely pauses >2 s mid-programming merely exits the session (harmless).

## Build / verification
- All 6 FC targets rebuilt clean: `UAVXF4V3`, `UAVXF4V4`, `DEVEBOXF4`,
  `SPEEDYBEEF405WING`, `FLYINGRCF4WINGMINI`, `BLUEBERRYF405`. Only pre-existing RWX
  LOAD-segment linker warning.
- No GCS source change; GCS compile-checked (py_compile) for completeness.

## Follow-up for Greg
- **Reload the good `.af`** (`user/Ecks_220mm_20260904_163040.af`, 910 g) and write/
  commit so the FC runs angleKp 7 / rateKp 0.34, then verify the tag-71 readback
  matches. The Ecks should then hold a 30° pitch command per sim (0.30 s rise, no
  overshoot).
- Re-decode the next Ecks rawlog to confirm the storage gains are sane before trusting
  the next flight.
