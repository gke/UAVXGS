# Session Report — RC 18-Channel Telemetry + f16 Flight Packet (Sep 11)

**Date:** 2026-09-11
**Scope:** GCS RC Configuration panel overhaul (FC + GCS); f16 (IEEE half) flight-packet
telemetry (tag 13) on FC + GCS. **User approval: "RC configuration layout good — mark as done".
"No fallback for old log files" (format change is hard).**

---

## 1. RC telemetry: all discovered channels + flags

### Change
- **FC (telem.c `SendRCChannelsPacket`, tag 22):** payload body extended from 12 logical
  f32 values to `interval(2) + discovered(1) + flags(1) + logical RC[] f32×12
  + physical RCInp[] u16 µs × discovered`. For SBus/CRSF `discovered=18` → **88-byte body**
  (16 payload channels + 2 flag/link slots). Legacy GCS parse kept temporarily for whatever
  was already on the wire — now since f16 (see §3) is a hard format break anyway, old bodies
  are dropped.
- **FC (rc.c):** new global `uint8 RCFlags`, set only in `sbusDecode()` from `RCFrame.u.b[22]`
  (bit0 = ch17 flag, bit1 = ch18 flag, bit2 = signal lost, bit3 = failsafe); 0 for non-SBus.
  `RCInp[16]`/`RCInp[17]` already carried ch17/ch18 = 2000/1000 by flag. CRSF `RCInp[17]` =
  uplink LQ (0-100) from LINK_STATISTICS.
- **GCS (packet_parser.py `parse_rc_packet`):** new 88-byte format + `rc_physical`/`rc_flags`/
  `discovered_channels` on `FlightData`.
- **GCS (main_window.py):** propagates `rc_physical`/`rc_flags` into shared flight data.
- **GCS (parameter_window.py):** RC Configuration box final layout approved by user:
  - **Assignments:** original 2-column layout restored — 12 function rows ×
    `[µs][bar][Ch spin][Func]` (Throttle…CamPitch), µs/bar toggle button preserved.
  - **Third column:** spare channels **12-15** (unmapped SBus slots 13-16), same cell layout
    with **disabled fixed Ch spins** and CH13…CH16 labels.
  - **Flags line** at the bottom right, level with the Cam Pitch row: decodes
    ch17/ch18 values (or H/L), SIG-LOST, FAILSAFE in real time.
  - **Height = 160 px fixed, identical to the Motors/Servos box** (user: "make the RC box the
    same height as the Motors/Servos box. We do not want the Motors/Servos box to change").
    Motors/Servos restored to its original stretch — no max width.
  - Functional fix retained: function rows look up the **physical** channel value
    (`rc_physical`), so any non-default assignment (e.g. RateGain→ch10) displays correctly.
- GCS calibration_window: 12-row RC loops already handled both live and capture paths.

### Rationale / discourse
- The readout-that-duplicates-data approach was rejected after user review ("you have displayed
  the same data twice"). The winnowed design: assignments show config only; each Rx channel
  appears **once** (µs/bar cell) in its column; spare channels 12-15 get their own column so
  the user sees the *whole* 16-channel frame, not just what the FC maps.
- `discovered` populates 18 slots, but the FC maps only 12 logical functions; the flag bits in
  byte22 are exposed as a single `RCFlags` byte (no per-bit globals) to keep the RC decode
  minimal (ISR-safe).
- Height parity: the earlier iteration needed 320 px; moving the flags line up into the last
  function row's grid row let the box shrink to the original 160 px with no visual crowding —
  verified acceptable by user.

## 2. f16 flight-packet telemetry (tag 13)

### Change
- **FC (telem.c):** new soft `F32ToF16()` — IEEE-754 **binary32→binary16**, round-to-nearest-even,
  bit-exact (validated 500k-case sweep vs exact-f32 oracle: 0 mismatches; gimbal: round bit is
  bit 12 = `0x1000`, NOT bit 11 — first cut used the wrong mask and mis-rounded 0.1f to `0x2e67`
  instead of `0x2e66`). Subnormal f32 inputs flush to zero (below half's 5.96e-8 floor),
  NaN→quiet NaN, |f|≥65520→Inf. New `TxESCr16()` (LE 2-byte). **telem.c keeps single-exit /
  no-division style.**
- `ShowAttitude16()`/`ShowDrives16()` added; **`SendFlightPacket` now ships f16** for all
  display-grade values. **`SendControlPacket` (tag 14) is untouched — still f32
  `ShowAttitude`/`ShowDrives`.**
- **f16 fields:** volts, amps (µs-budget: battery block 12 B → **8 B**), DesiredThrottle,
  quaternion, 15 attitude values, ROC, Altitude, CruiseThrottleFF, RF.Altitude, DesiredAlt,
  headings, 3 comps, AccConfidence, BaroTemperature, MagHeading, IMUTemperature,
  GlideOffset, RawPW[], drive balance[]. (RateEnergy slot retained at 0.0 since
  2026-09-17 — removed the 0/0 NaN emitter, no consumer.)
- **stay f32:** Baro.Pressure (**Pa up to 101325 > f16 max 65504**), KFState.Altitude + 3 KF
  variances (filter estimates kept at full precision — they feed tuning/decision logic, not
  just the display; the flight packet is the only path for them).
- Length: **194+8N → 118+4N** (10 drives: 274 → 158 B body). Emu/replay consumers decode via
  the same GCS `parse_flight_packet`.
- **GCS (packet_parser.py):** `extract_half()` (`struct '<e'`); `parse_flight_packet`
  rewritten to the new offsets. **No legacy-length fallback** (user decision — old rawlogs
  will not replay against the new build; intent is a clean break).

### Rationale / discourse
- **Why f16 and not smaller integers:** display-grade telemetry is sub-realtime; half's ~0.05%
  relative precision is far below GCS display resolution; F4 has no FP16 hw so a soft RNE
  converter is ~30 instructions — negligible vs the 4-byte-per-field wire saving.
- **Why not trace capture:** user explicitly ruled it out after the assistant flagged that
  trace records are *measurement* data (rise/overshoot/settle analysis feeds the critic), the
  16-byte basis is protocol-confirmed, and the ring-duration gain (2.7→4.4 s) is marginal.
  User: "I think we do do it for trace capture" → assistant counter-reasoned (measurement
  fidelity + only ~10% ring win + format churn) → user agreed: **"flight packets only."**
- **Why not CRSF/S.Port:** the flight packet feeds the GCS (USB/soft-serial); the FrSky D8
  path already ships its own compact derived values and is out of scope.
- **Where the byte budget went (10-drive airframe):** 106 B of display payload → f16 ≈ 54 B;
  the 74 B saving is ~75 % of one D8 soft-serial frame/second — headroom for higher flight
  packet rate or future fields.

## 3. Build / verification
- **FC:** clean on all 5 targets (DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI,
  BLUEBERRYF405, MATEKF411WING). `MATEKF411WINGQ_r0.bin` = 234 508 B.
- **GCS:** `py_compile` clean on `parameter_window.py`, `packet_parser.py`, `main_window.py`.
- `F32ToF16` validated 500k-random sweep (f32-exact oracle) — 0 mismatches, incl. subnormal
  edge region and overflow-to-Inf.
- GCS round-trip: synthetic 10-drive body (158 B) parses to exact expected values (volts, q0,
  desPitch, acc_du, baro hPa, kfAlt, drives, mSClock).

## 4. Notes for next session
- RC box height = Motors/Servos height is now a **fixed contract** of the layout row — if the
  RC grid grows again, either shrink rows or widen the box, not the motor box.
- `RCFlags` is defined in rc.c (with extern in rc.h); only SBus populates it — CRSF has no
  equivalent flag byte (its LQ is in RCInp[17]).
- Old rawlogs (pre-2026-09-11) will mis-decode the flight packet — no fallback was added.
- Flight-packet f16 may later justify bumping the telemetry scheduler rate; the freed bytes are
  currently left as headroom.
---

# Session 2 (2026-09-11, afternoon) — Trim + dedup pass

## 1. GCS UI
- **RC Configuration:** flags line REMOVED (user: "if CRSF does not produce the flags we
  should not display them" — display only genuine per-frame protocol truth; the consolidated
  `Signal` flag (bit 35) + link-stats LQ already cover signal state). `RCFlags` stays on the
  wire (diagnostic/rawlog), just not displayed. Layout contract unchanged (160 px box).
- **Motors / Servos (main page):** value labels *under* the bars REMOVED — the QProgressBar
  `%v` already renders the value inside, and the grid overlap (`row+2` of block 1 colliding
  with `row` of block 2 at grid row 2) caused "labels of second row overlap values of first
  row". Grid now 4 rows (labels row0/2, bars row1/3); bars `setFixedHeight(14)` matching the
  Serial Ports `bar_style` height: 14px.
- **Configuration Summary (main page):** unused flags suppressed — "Unused" (Config1 b6) and
  "Unused 2-2" (Config2 b2) labels dropped; the label list is now data-driven
  (`self.config_flags` of `which/bit/name`, 12 real flags).

## 2. RCChannels packet (tag 22) — second dedup
Bench: the body sent every channel twice — logical 12×f32 (48 B) and physical u16×18 (36 B).
GCS displayed from physical; logical's only real consumers were the 4 main-page THR/ROL/PIT/YAW
control bars (mapped slots) and a now-dead `get_rc_data()`.
- **FC:** logical f32×12 block REPLACED by controls u16×4 (`RC[eThrottleRC..eYawRC]` folded to
  µs) + physical u16×discovered. Body 88 → **48 B** (SBus/CRSF).
- **GCS parser:** new format primary; both older body layouts kept for replay (length-dispatched,
  no live-link fallback). `rc_channels[0..3]` = mapped control µs; `rc_raw` = the physical
  µs (calibration window's 12 raw labels now show true received values).
- **Dead code removed:** tag-18 (MIN) case handler + `min_data` (FC no longer sends
  `UnusedUAVXMinPacketTag`), `get_rc_data()`.
- Rationale: same "send once, display many" principle as the f16 flight packet; the 40 B/pkt ≈
  ~0.4–0.5 kB/s is not bandwidth-critical (2.4–4.0 kB/s total on USB) but removes the only real
  cross-encoding duplicate in the streamed set.

## 3. Telemetry inventory (done together)
All streamed packets mapped (tag/body/rates B-G-H-A-C/consumer). Verdict: packet set is lean;
Nav (14) shares only navState with Flight (13); Control (16) is request-only f32 and stays
(request-driven, not streamed). Candidates for removal (Control-16/parse cascade) parked for a
future GCS cleanup — no further wire changes planned.

## 4. Build / verification
- **FC:** all 7 targets clean (UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING,
  FLYINGRCF4WINGMINI, BLUEBERRYF405, MATEKF411WING). `MATEKF411WINGQ_r0.bin` = 234 524 B.
- **GCS:** `py_compile` clean (`main_window.py`, `parameter_window.py`, `packet_parser.py`).
- Parser round-trip verified headless: new 48-B body (controls + phys 18), old 88-B body,
  and pre-2026 f32-only body all parse to expected values.
- **USER: rebuild GCS + reflash MATEKF411WING; verify RC box no flags line, control bars
  THR/ROL/PIT/YAW track, motors box single-value bars at Serial-Ports height.**

---

# Session 3 (2026-09-11, evening) — MATEKF411WING GPS: SERIAL ROLE SWAP FIXED

## Summary
Bench GPS silent (`hwVer=0`, sats=0) under our firmware; module works under iNav 7.1.2
with the same cabling. Root cause = **serial role map inverted**: our firmware had RC on
`eUsart1` and GPS on `eUsart2`; the board's physical layout is the opposite. Fix = swap
the role assignments. Rebuilt clean same-day.

## Diagnosis trail

### Stage 1 — 8N2 stop-bits (red herring, later superseded)
`matekf411wing.inc:54` had `USART_StopBits_2` (8N2) on the GPS (USART2) line — SBus
convention copy-pasted. Fixed to `USART_StopBits_1`. Post-fix logs showed dense RX
(peak 750 B/poll) but still `hwVer=0` — the module never answered UBX at any of 6 swept
bauds (230400 added to `gps.c` fast path + sweep, checksum `*1C`). The dense RX was not
the GPS.

### Stage 2 — Role swap found (root cause)
User loaded iNav 7.1.2, reported: "GPS is on UART1." iNav's `MATEKF411` target:
UART1 = PA9/PA10 (TX1/RX1 pads), UART2 = PA2/PA3 (SBUS pad feeds USART2 RX). Our
original map had **RC on eUsart1 (PA9/PA10) and GPS on eUsart2 (PA2/PA3)** — inverted.

| physical component | iNav config (working) | our firmware (broken) |
|---|---|---|
| GPS module (TX1/RX1 pads) | UART1 = PA9/PA10 | **eUsart2** = PA2/PA3 |
| SBus pad | UART2 = PA2/PA3 | **eUsart1** = PA9/PA10 |

The "dense non-UBX GPS RX" measured on eUsart2 was the **SBus receiver's stream**; the
GPS module's output drove our RC port (PA10), where no GPS decoder listens.

### Tag-66 evidence (rawlog GPS RxQ)

| rawlog | GPS RX cum | peak/poll | was actually |
|---|---|---|---|
| `161139` / `161900` | 0 B | 0 | receiver not yet active |
| `163036` | 955 B | 257 | SBus frames on USART2 |
| `163126` | 6 263 B | 188 | SBus at full rate |
| `172634` (post-8N1 fix) | 2 659 B | **750** | SBus at peak rate |

No `RxOverflow` — hundreds of correctly received bytes, never UBX.

## Fix applied (rebuilt 2026-09-11)
`matekf411wing.inc` serial roles swapped to match board + iNav:

```
RCSerial = eUsart2;  CRSFRxSerial = eUsart2;  SBusRxSerial = eUsart2;  // SBUS pad
GPSSerial = eUsart1;                                                 // TX1/RX1 pads
SerialPorts[eUsart1] = GPS 8N1 115200;
SerialPorts[eUsart2] = CRSF/SBUS 8N2;
```

`MATEKF411WINGQ_r0.bin` = 234 628 B (data-only change). Other boards untouched.
230400 fast-path probe retained in `gps.c` (correct for genuine u-blox modules, harmless).

## Next
User to reflash + bench: expect `hwVer`/sats on the nav panel AND healthy SBus RC.
