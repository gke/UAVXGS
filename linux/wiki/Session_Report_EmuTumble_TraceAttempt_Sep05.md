# Emu Tumble — Trace Attempt & Excitation-Dependency — Sep 05 (afternoon)

Companion to `Session_Report_EmuTumble_GoodGains_Sep04.md` and
`Session_Report_EmuAccelTumble_Sep03.md`. These three documents are the full
backtracking trail for the **emulator-only 30°-hold limit cycle / tumble**
problem. READ ALL THREE before resuming.

PDF conversion is the user's job (`scripts/md2pdf.sh` on host Merlin) — never
attempt PDF generation.

---

## 1. Objective (unchanged)

Fix the **emulator-only** 30°-hold tumble/limit cycle of the Ecks 220 mm inside
the FC `emu.c` emulator. The real-sensor bench behaviour (board in hand, healthy
motor response, no coupling) and the Python twin of the closed loop are both
**stable** — the failure is specific to the emulator execution path inside the
FC firmware.

## 2. New evidence this session — the initiator is EXCITATION-DEPENDENT

A fresh set of emulated flights was recorded (`20260905_13xxxx_Ecks_220mm_REFERENCE.rawlog`)
following the trace-capture recipe (ch8 armed before the pitch command — see
§5). Two of the flights now show behaviour that contradicts "the 30° hold
*always* tumbles":

| log | pitch-in profile | outcome |
|---|---|---|
| `131422` | **gentle ~4 s ramp** (dp 7→15→…→30 over ≈4 s, ms 68929–72830) | **HELD**. ap 33–39°, slow decaying ~0.5 Hz / ±3° wobble, no tumble |
| `132237` | faster pull (dp 0→30 in ≈300 ms), then **stick waggle** | held ~13 s at 30°, then a waggle event diverged: rr +231°/s spindle → **tumbled through vertical** (ap ±86°, rr ±231°/s) |
| `122607`, `122753` (morning) | abrupt pull-up (dp 9→28 in 100 ms) | tumbled **during** the pull-up |

So the emulator is **NOT unconditionally unstable at 30°**. A gentle ramp holds;
an aggressive pull-up or a stick waggle seeds the limit cycle. This matches the
mechanical picture of a lightly-damped (marginally unstable) resonance around
~10 Hz in the emulated closed loop that a broadband excitation (step, waggle)
drives into a rate-saturated **relay**, and a narrow-band low-amplitude
excitation (slow ramp) does not.

### 2a. The 131422 hold waveform (calm flight, telemetry 10 Hz)

Pitch commanded to 30° (ms 58340–61433, dp 29–30): ap ramps 16.8→31.2°, rp peaks
+206°/s, decays; ap settles 28–30°; then released (dp→0), returns to level with a
decaying rp swing ±170°/s (≈ 6 s). Second hold (ms 68929–89610): dp ramped
7→15→…→30 over ~4 s; ap rises to ~39° at ms 83607, then a **damped** oscillation
ap 32–39°, period ≈ 16 s, rp ±250°/s decaying by end of flight. This is a
**functional** (if sloppy) pitch hold. No evidence of the rail-slam relay of the
122607 freeze.

### 2b. The 132237 divergence (post-waggle)

After holding ~30° for ~13 s (ms ~155400–159800), the dp waveform shows stick
movements (dp 29→25→26→22 at ms 159769–160474) with rp −193…−220°/s; a later,
larger waggle at ms 168971 (dp 30→12, rr +231°/s with ap −0.6°) rolls the craft
through a full spin. ap excursions reach ±86°, rr ±231°/s. The coupling is
roll/pitch, exactly the signature of the earlier runs.

## 3. Previous session's key findings (still standing)

Full detail in `Session_Report_EmuTumble_GoodGains_Sep04.md`. Summary:

1. **Emu rate-clamp artifact fixed (Sep 04):** `emu.c` MR plant clamped achieved
   `Rate[]` to hardcoded 300/180°/s while the controller commanded
   `A[a].R.Max`=210/120°/s. Now clamps use `A[a].R.Max`. Correct physics, all
   targets build clean. Did NOT stop the tumble.
2. **The −390°/s "sustained rate" is ALIASED.** In the 122607 well-observed
   segment (t≈38194–38486 ms) rp sat at −361…−392°/s for ~0.6 s at 10 Hz
   telemetry while `accdu` (TrueQ-synthesised body-Z specific force) stayed
   −1.09…−1.25 → **true attitude parked at ≈36°, estimator correct** → the
   sustained-rate look is a **high-frequency rate limit cycle aliased at 10 Hz
   telemetry** (net-zero rotation). The estimator never diverged from truth.
3. **Python twins are ALL stable.** `twin.py` (original), `twin2.py` (fixed
   LPF2 second stage), `twin3.py` (plus full emulated-accel estimator):
   pitch holds 29.8–30.0°, rp ±40°/s, for damping-sign ±, rate-clamp on/off,
   torque ×2, cruise 0.73. The 10 Hz failure does NOT reproduce in the
   closed-loop twin of the emu MR plant + `control.c` quaternion loop.
4. **Why the accel/estimator branch is NOT the differentiator:** in an
   accelerating climb the specific force ≈1.49 g ⇒ `CalculateAccConfidence`
   magnitude score `exp(−0.5·((1−1.486)·25)²) ≈ e⁻⁷³` ⇒ **accel correction is
   effectively OFF in climb** (matches accconf≈0 in the logs) ⇒ the estimator is
   gyro-only, exactly what the stable twin simulates. The user's 2 g acc_d/U
   observation is consistent: `fz = FakeAccU + g ≈ 2 g` transiently during the
   dynamic phase (emulated vertical acceleration ≈ +1 g).
5. **Estimator runs every cycle:** the whole Madgwick/Euler block is inside
   `if (F.IMUCal)` (inertial.c:484); `F.IMUCal` is set only at boot/calibration
   (imu.c:131,219,293) and emu init (emu.c:748) — never cleared mid-flight.
   Only writers of `Rate[]` are emu.c and imu.c; Madgwick consumes exactly
   `Rate[eRoll],Rate[ePitch],Rate[eYaw]` (inertial.c:499) with the same dT.

## 4. What is NOT yet explained (the open core)

- The **physical mechanism** that a step/waggle feeds ~10 Hz energy into and that
  sustains the relay, while the twin (which shares the exact math) does not.
- The remaining structural differences between the emulator and the twin:
  - scale: 500 Hz control loop + telemetry sampling (10 Hz flight packet);
  - the full FC `DoControl` context (alt-hold loop, `CalcTiltThrFF`, dive/gate
    logic, TiltThrFFComp slew, `ConditionQuatIntE`, GPS/nav interactions);
  - float32 vs Python float; C call order vs Python loop order;
  - pilot input (the FC side is flight-performed by Greg on the bench Tx, the
    twin is scripted).

## 5. The trace-capture attempt — FAILED to record (the instrument to fix)

Recipe issued and followed (best-effort "sort of"): take off → settle level →
flick ch8/Aux2 OFF→ON (settle gate opens capture after 100 ms of calm rates) →
wait for the injected roll probe to decay → command 30° pitch → hold ch8 → dump.

**Result: `trace_samples/bb_dump_20260905_132537.bin` = 32 bytes of 0x00.**
`parse_trace` sees `magic=0, count=0` → **TraceCount==0 at dump time → the
capture never opened**, and no disarm-edge `TraceCommit()` wrote the CAPTURE
sector (flash copy also empty ⇒ nothing to dump).

Probable arming failures (from `TraceCapture()` in `UAVXArmQ/src/trace.c`):
- the OFF→ON edge on `RC[eTraceRC]` (≥0.5) never happened (ch8 already high, or
  the Tx Aux2 channel not actually changing on the cabled bench link);
- or the settle gate never closed: all 3 axes |Rate| < 0.10 rad/s (5.7°/s) for
  50 ticks (100 ms) while ch8 held — **in a climb the rates need not be that
  quiet**, and if the flick happened after the pitch command, the relay never
  settles ⇒ arming never succeeds;
- or `State != eInFlight` at the moment of the edge.

To resume: verify ch8 reaches the FC (RC chan displays on the GCS), do the
OFF→ON flick while truly level with small rates, keep ch8 held through the 30°
hold, land/disarm (ARC: disarm-edge commit), then Dump. The 500 Hz
Rate/Desired/Out × 3 axes stream is the decisive missing waveform — it will show
the relay directly (Desired vs Rate square wave, Out rail-slam, per-axis phase).
Consider also bumping `TRACE_SETTLE_RATE_RAD_S` (currently 0.10) for bench
emulation runs, since an emulated climb is noisier than a real hover.

## 6. Standout hypotheses to test on resume (banked, not exhaustively tested)

These were candidates at various points; explicit mechanical checks remain:

1. **High-frequency relay with net-zero rotation at ~10 Hz** (rate-clamp-bounded
   bang-bang relay whose *time-average* rotation is zero → aliases to a frozen
   attitude on 10 Hz telemetry). To find HOW it self-sustains, need the trace.
2. **MotorLag→torque-gain feedback loop:** torque ∝ MotorLag (`MotorLagState`
   converges on `motorInput` ≈0.73 in climb); if alt-hold/TiltThrFFComp raises
   motorInput with tilt, torque authority grows with error — a positive gain
   path the twin's constant-cruise setup cancels. Test: replay 132237 with
   `TiltThrFFComp` TC (or motorInput variation) pinned.
3. **Dive/`IsDiveActive` and the `GainSchedule` remnants** in the MR path — not
   yet audited for whether they inject anything at ~10 Hz.
4. **float32 accumulation in `MadgwickUpdate` gyro step at large accumulated
   rotation** (>1000° total) — the twin uses float64. Cheap to test: re-run the
   identical trace-free step on a twin built with float32 downcasting.
5. **Euler/`ConvertQuaternionToEuler` + `AttitudeCosine` sign handling at high
   tilt** — known-good inverted gate (Sep 05 fix), but the *transition* 36°→90°
   during a forced divergence has not been hunted on 500 Hz data.

## 7. Resume checklist (a "dead camel" pack)

- [ ] Get a REAL 500 Hz `eTraceRate` capture (§5) through the tumbling event.
      This is the single highest-value next action.
- [ ] From the waveform: motor Out vs Desired vs Rate per axis on 500 Hz —
      distinguishes relay, gain-scheduled pumping, and feedforward bleed.
- [ ] If still deferred, ALSO apply the cheap float32 twin (§6.4) and the
      motorInput-tracking torque-gain test (§6.2).
- [ ] Then choose: fix in `emu.c` (model fidelity) vs `control.c` (loop
      structure). The GCS/telemetry path already carries everything needed.
- [ ] Update this report + AGENTS.md when resolved. Do NOT delete the
      before/after waveforms or the rawlogs.

## 8. Files & artefacts to keep

- Logs (all rawlogs+csvs under `/home/gke/Documents/UAVXGS/` and decoded in
  `/tmp/opencode/`): `20260905_122607`, `20260905_122753` (tumble during
  pull-up), `20260905_131422` (held, calm), `20260905_132237` (held then
  waggle-diverged), plus short `131804/131941/132023` session logs.
- Decoder: `/tmp/opencode/decode_rawlog.py`
- Twins: `/tmp/opencode/twin.py` (original), `twin2.py` (corrected LPF2),
  `twin3.py` (+ full emulated-accel estimator) — all stable.
- Empty trace dump: `uavx-python/src/tests/trace_samples/bb_dump_20260905_132537.bin`
- FC sources: `UAVXArmQ/src/emu.c` (MR plant 447–523, MotorLag, EmuTrueQ),
  `inertial.c` (DoSensorUpdate/Madgwick/CalculateAccConfidence), `control.c`
  (ControlRate signs 472–496, quaternion angle loop 619–651), `trace.c`
  (`TraceCapture()`), `filters.c` (LPF1/LPF2/FIR).