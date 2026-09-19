# Session Report — Emulated Ecks Tumble persists with correct gains (rate-cap mismatch + coupled limit cycle)

Date: 2026-09-04 (Prof Greg & assistant)

## Context / correction
Greg clarified the flight log being analysed is **EMULATED**, not real flight data
(`CONFIG1_BITS` has `eEmulationEnable`, Config1 bit 3 → `F.Emulation`). This is the
same domain as the OPEN "Emu high-angle flip/tumble" TODO.

## 1. Confirmed: the correct `.af` is loaded
The most recent log `20260904_172834_Ecks_220mm_20260904_163040.rawlog` names the
loaded airframe `user/Ecks_220mm_20260904_163040.af` (the "good" 910 g, 4M file).
FC tag-71 readback in that log matches the `.af` exactly:
- ROLL/PITCH ANGLE_Q_KP = 7 (af 7)
- ANGLE_Q_KI = 0.25 (af 0.25)
- RATE_KP roll/pitch = 0.34 (af 0.34)
- RATE_KD = 0.008, YAW_RATE_KP = 0.11, YAW ANGLE_Q_KP = 3, etc.

Limits also verified: `.af` sets `MAX_ROLL_ANGLE=MAX_PITCH_ANGLE=0.5236` (30°),
`MAX_YAW_RATE=2.094` (120°/s), `MAX_ROLL_RATE=MAX_PITCH_RATE=3.665` (210°/s),
`MAX_HEADING_RATE=1.047` (60°/s). These map onto `A[a].P.Max`/`A[a].R.Max` (param
tags: `A[Pitch].P.Max`=74, `A[Roll].P.Max`=76, `A[Yaw].R.Max`=63, `A[Roll].R.Max`=82,
`A[Pitch].R.Max`=83) — so the controller clamps desired rate to **210°/s** and desired
angle to **30°**.

**Conclusion: the tuning is validated/correct and it is what the emulator ran. The
oscillation is NOT a bad-gains / wrong-file problem.**

## 2. The emulator still goes unstable with correct gains
Quantified from the log's held-30° segment (~55–70 s, state eFlying):
- Rise to 27° ≈ **0.8 s** for a 30° command, then overshoots to **55.9°** (Greg's
  "slow response to stick" + never-cleanly-arrives).
- Steady-state **pitch oscillates 39.8°→55.9°** (peak-to-peak ~16°) around the 30°
  setpoint, period ≈ **1.64 s**.
- **Uncommanded roll**: `desired_roll` stayed **0.0** throughout, yet roll oscillated
  **−12°→+10.5°** with roll rate hitting 300°/s ~17 % of the time.
- Both **pitch and roll rate saturate at the emulator's 300°/s cap** simultaneously →
  a genuine coupled 3D limit cycle, not a single-axis error.

## 3. Found: emulator-vs-controller rate-limit mismatch
The controller (via `MAX_ROLL_RATE`/`MAX_PITCH_RATE` = 3.665 rad/s) commands rate
clamped to **210°/s**. But the emulator's MR plant clamps achieved rate at a **hard-
coded `md->MaxRollRate`/`md->MaxPitchRate` = 300°/s** (`emu.c:90-91`, not sourced from
the `.af`). So the emulated plant can spin **~45 % faster than the controller's design
limit**, forcing the rate loop to brake from 300°/s where its D-term authority was
never sized. This is a concrete, fixable emulator discrepancy worth resolving before
trusting emulated tumble predictions.

## 4. Hypotheses for the residual oscillation (correct gains)
1. **Emu rate-cap mismatch (above)** — plant exceeds controller's 210°/s limit → the
   brake-out-of-300°/s regime destabilises the loop into the limit cycle.
2. **Quaternion attitude-loop cross-coupling at sustained high pitch** — with
   `desired_roll = 0` yet roll oscillating, `QeToQa` decomposition of the pitch error
   is injecting a roll rate command at ~50° tilt (the AGENTS TODO's "residual cause is
   in the ATTITUDE LOOP"). The pitch overshooting to 56° past the 30° setpoint suggests
   the angle-loop correction (2·Qa·Kp) is not driving back to 30 — possibly wrong-sign
   or insufficient at that tilt.
3. **Thrust vs torque at high tilt / throttle** at ~95 % throttle and ~50° nose-up.

## Applied fix (Greg direction): emu MR rate cap now follows the controller
Changed the **MR** branch of the emulator plant (`emu.c`, lines ~486-488) to clamp
the achieved `Rate[]` to `A[a].R.Max` (the loaded `.af` `MAX_ROLL_RATE`/
`MAX_PITCH_RATE`/`MAX_YAW_RATE`) instead of the hardcoded `md->MaxRollRate`/
`MaxPitchRate`/`MaxYawRate` = 300/180°/s. Rationale: the plant could previously
out-rotate the controller's commanded-rate design limit (210/120°/s), so the rate
loop had to brake from a regime it was never sized for — the suspected source of
the coupled limit cycle. The FW (`433-435`) and Land (`501`) branches use
`md->*` unchanged.

All 6 FC targets rebuilt clean.

## 5. Re-run with the rate-cap fix (log 205928): still tumbles — ROOT CAUSE found
Greg re-ran the emulator with the fix (MR rate cap now `A[a].R.Max` = 210/210/120°/s).
Log `20260904_205928_Ecks_220mm_20260904_163040_2026.rawlog` (21:00 hours — the log to
analyse; the other 20:55–20:59 logs `205536/205636/205706/205837` are bench/no-flight
logs, states 7/12, no in-flight samples).

Result: **rates now saturate at 210°/s (¬300) — the fix works — but the coupled limit
cycle is NOT an emulator rate-cap artifact.** The post-fix run diverges into full
flips (pitch ±80°, roll ±176°) whenever the 30° pitch command is applied, then
returns toward level when the sticks are released (`dp→0` → `ap`,`ar`→~0). Greg's
flight observation "wobble in roll, comes up to pitch, very unstable, returns to
level when sticks released" matches this pattern exactly.

### Mechanism (Greg's gimbal-lock / yaw-sense refinement — confirmed against data)
Two linked defects:

1. **Initiator — angle-loop limit cycle at moderate pitch** (pre-fix run `172834`
   never crossed 90°: max|ap|=55.9, `dy`=0.0 throughout). A 30° pitch command
   overshoots into a 40–56° limit cycle with uncommanded roll oscillation (dr=0).
2. **Accelerant — wrong-sense yaw hold when inverted (Greg 2026-09-04)**:
   - `F.YawActive = |yaw stick| > deadzone` (`control.c:521`) → in an autonomous hold
     (sticks centred) it is FALSE → the Euler path runs:
     `A[eYaw].P.Desired = Angle[eYaw] + MinimumTurn(DesiredHeading)`.
   - Crossing ±90° pitch, Euler `Angle[eYaw]` flips **180°** (gimbal lock).
   - The yaw-suppression guard `fabsf(Angle[ePitch]) > cGimbalLockPitchRad`
     (`control.c:598`, `cGimbalLockPitchRad = 1.309 rad ≈ 75°`) fires only inside
     |pitch| ∈ (75°, 90°) — a thin ±15° band around vertical. **Fully inverted,
     `Angle[ePitch]` reads back through −90→−60→… so the guard is OFF and heading
     hold is fully active.**
   - The controller then commands body yaw toward the 180°-flipped heading target.
     **Inverted, body +Z ≈ −world +Z, so the commanded body yaw rotates the actual
     heading AWAY from true `DesiredHeading` — wrong sense → positive feedback →
     yaw spins up.** Data: `dy` locked at **−102.8**° while `ay` sweeps −146→−153→
     −108→+80° (≈226° of continuous heading change despite the "hold"), and the
     tumble persists until stick release.

### Implication
The 30°→56° limit cycle is the trigger; the inverted yaw hold is the force that
converts an overshoot into an unrecoverable spin. Any heading-hold-derived-from-Euler
is ill-conditioned once the aircraft inverts — the fix must either gate/suppress yaw
in the inverted hemisphere (not just |pitch|>75°) or make the heading reference
singularity-free (F.YawActive-style quaternion path, or a mag-anchored world heading)
so it never flips 180°.

## Next steps (awaiting Greg)
- **Priority 1 — fix the yaw-sense/inversion gating**: suppress (or correctly sense)
  the heading-hold yaw command whenever the aircraft is inverted (body-Z vs world-Z
  sign, or a wider gimbal gate), so the phantom 180° heading error cannot command a
  sense-inverted spin. Re-run the emulator and confirm the tumble becomes recoverable.
- **Priority 2 — fix the initiator**: the angle-loop 30°→56° limit cycle /
  uncommanded roll at ~50° pitch (`QeToQa` cross-coupling suspect) so the aircraft
  does not reach the inverted region on a 30° command in the first place.

---

## 2026-09-05 supplemental — ROOT CAUSE of the emulated limit cycle found and FIXED

### Investigation (numerics on the correct-gains rawlog `20260905_093451`)
- **QeToQa outer-loop cross-coupling CLEARED as the initiator.** Replicating the
  logged `q0..q3` + desired-Euler offline (compare_outer.py, quat_err.py; with the
  real `AttitudeCosine` = q0²−q1²−q2²+q3², not the fast-asin form):
  - Link-average demanded `|2·Qa·Kp|` over the hold: roll 22.0, pitch 110.7,
    yaw 12.4 deg/s — yet the logged roll rate pegs ±210 deg/s with a requested
    roll rate ≈ 0 (correlation |Qe_roll| vs tilt = −0.234). The uncommanded roll
    is NOT commanded by the axis split.
  - An ArduPilot thrust-vector-first (thrust-first yaw-suppression) error
    decomposition would demand MORE under the same quaternion states when
    unclamped (roll 132.6, pitch 788.2, yaw 343.8 deg/s). Adopting it would not
    reduce the pegging — it is not the fix.
- **Limit cycle characterisation (re-run 093451, st5):** sustained 0.6 s-period
  pitch oscillation 43.2–48.8° (mean ≈ 46°) on a 30° command; pitch AND roll rates
  peg ±210 deg/s ~22–26 % of samples; yaw quiet (±15 deg/s); Euler roll ≈ 0.
  `AttitudeCosine < 0` (inversion, per inertial.c) begins only at t≈110.5 s AFTER
  throttle cut/disarm — so the Sep-05 inverted-yaw gate (`control.c:598`) is NOT
  exercised by this run. This is a self-sustaining planar rate limit cycle with ~0
  heading change — pure roll/pitch dynamics.
- **Root cause — emu plant roll/pitch damping sign inverted.** ODE algebra on
  emu.c:
  - MR roll/pitch (was `emu.c:474`): `Rate[a] -= (torque − 2·sign(r)·r²)·dT`
    expands to `Rate += (−torque + 2·sign·r²)` → **anti-damping** (adds energy at
    any nonzero rate, dual-relay style, exactly a self-sustaining oscillation
    pinned at the rate clamp).
  - All yaw branches (`+= (torque − 2·sign·r²)`) and the Python plant
    (`test_pid_sim.py:1001` `dRate=(torque−damping)/I`) subtract the damping
    correctly.
  - `Sign(v)` = +1 if v ≥ 0 else −1 (misctypes.h:194). Torque sign is consistent
    with control.c (rolled `−conditionOut`, yaw `+conditionOut`); only the damping
    arithmetic was wrong.
  - **This OVERTURNS the 2026-09-04 "not an emulator artifact" verdict.** The
    real-aircraft tumble was separately the bad-gains load; the emulated tumble on
    CORRECT gains was this emulator plant bug, which reproduces at any high rate
    regardless of the controller.
- A one-axis noiseless Python closed-loop replication (emu_pitch_loop.py)
  converges for both signs — it cannot reproduce the noise-coupled 3-axis cycle,
  but the ODE algebra plus the yaw/Python-sim/WR-control contrasts pin the sign
  error unambiguously.

### Fix (both roll/pitch loops, MR + FW)
- FW branch (`emu.c:422-424`) and MR branch (`emu.c:474-476`): damping term now
  `+ 2.0f * Sign(Rate[a]) * Sqr(Rate[a])` inside the subtracted term, so the net
  effect is `−2·sign(r)·r²` (damping opposes rotation). Yaw branches untouched
  (already correct). Explanatory comment added (GKE).
- All 6 FC targets rebuild clean.

### Next
- **Greg: re-flash the rebuilt SPEEDYBEEF405WING image and re-run the emulator's
  30° pitch hold** with the current parameters (`Ecks_220mm_REFERENCE.af` —
  user-scrubbed reference set, Sep-05 09:41, contains the good gains
  rateKp 0.34/0.11, angleKp 7/3, Kd 0.008, Ki 0.25). Expect: pitch converges to
  30° with no limit cycle, rates stay well under the 210 deg/s cap. Capture a
  fresh rawlog.
- If clean, close the emulator-limit-cycle investigation and return to the
  gains-lost-in-write/read bug (tag-17 → tag-71 round-trip, `_display_mult`
  legacy-scaling suspect).
- The inverted-yaw control fix (`control.c:598`) still needs a controlled-inverted
  validation (slow roll through vertical on the emulator, yaw held, verify no
  spin-up — unexercised by the 30° hold).

---

## 2026-09-05 supplemental (2) — post-fix log OVERTURNS the damping-sign verdict

**`20260905_103300_Ecks_220mm_REFERENCE.rawlog` (113 s, user re-flash of the
damping-sign-fixed build, same `Ecks_220mm_REFERENCE.af` gains) — STILL FAILS.**
The behaviour is indistinguishable from the pre-fix run (093451): the 30° hold
relays pitch 32–47.6° at ~10 Hz with `rp`/`rr` slamming the ±210 deg/s plant
clamp, then t≥63.4 s breaks into repeated full somersaults (`ap` ±87°, Euler
roll artifact `ar` ±180°, `AttitudeCosine`<0), then a slow spiral. The damping
sign was **correct physics but never had the authority to break this cycle**:

- Torque authority per unit Out at cruise MotorLag ~0.55: ~173 rad/s² (up to
  315 at full motor). Damping term at |r| = 210°/s (3.66 rad/s): only
  ~27 rad/s² — **~10% of torque**. Sign-correct or not, it is a 7% perturbation.
- A faithful Python closed-loop twin of the emu MR plant + `control.c`
  (exact `EulerToQuat`/`QError`/`QeToQa`/`ConditionQuatIntE`/Pavel-FIR+LPF2
  D-term/Madgwick kinematics, constant throttle) holds **STABLE at 30°** for both
  damping signs and every `MotorTau` 0.02–0.10 — the failure needs a coupling the
  idealized closure lacks.
- Log `thr` is flat (0.734) — the altitude/throttle → `MotorLag` → torque-gain
  coupling is not pumping the loop either.
- Telemetry `acc_confidence` = 0.00 throughout the hold is **correct** emulator
  behaviour: at ≥40° tilt, body-down specific force is 0.7g and |a|≈1.4g
  (tilted-hover geometry), so `dir_conf`/`mag_conf` → 0 → Madgwick gyro-only.
  (In the FLIP segment it briefly reaches 0.9 at level crossings — accel
  reactivates mid-tumble — but that is downstream.)
- The `drp=+208` (nose-up demand while 57° nose-down from target) only appears
  at `ap=±87°` where the Euler `ar` is ±180° (gimbal-ambiguous estimate); it is
  downstream of the hold relay, not the initiator.

**Mechanism now most consistent with all logs:** a rate-saturated angle-loop
limit cycle sustained by two emu-plant artifacts that a real aircraft does not
have — (a) the **hard ±A.R.Max rate clamp** (an energy-conserving wall: the emu
pins achieved rate at the commanded-rate limit, whereas a real aircraft rotates
through 210°/s freely and the angle error closes); (b) angular damping ~10% of
torque authority at the relay frequency.

### Next experiment (proposed, pending Greg)
1. Remove the emu MR/FW plant rate clamp (unclamped, or a generous physical
   wrapper ~720°/s) and re-run the 30° hold — directly tests "the hard wall
   sustains the relay".
2. Secondary knob: raise the `2·|r|·r` damping coefficient ~4×.
Do NOT re-write the root-cause claim until experiment (1) resolves it — the docs
have flip-flopped enough (QeToQa coupling → emu damping sign → back to open).

### Twin reference
`/tmp/opencode/twin.py` — constant-throttle closed-loop twin (stable at 30°);
`/tmp/opencode/20260905_103300_Ecks_220mm_REFERENCE.rawlog.csv` — decoded
post-fix log.

---

## 2026-09-05 supplemental (3) — clamp experiment: MR plant rate clamp REMOVED

**Decision (Greg): unclamp the MR plant rate.** `emu.c` MR branch: the three
`Rate[a] = Limit1(Rate[a], A[a].R.Max)` lines (added 2026-09-04) are removed;
the FW branch keeps its `md->Max*.Rate` clamp because those ARE physical model
maxima, not command limits. The MR plant now rotates as far as the torque and
the `2·r²` damping take it — terminal rate at full torque `sqrt(315/2) ≈ 720°/s`,
so one big (physical) equilibrium bounds it; there is no hard wall at the
controller's commanded-rate limit.

Rationale: the clamp was exactly the energy-conserving wall the relay rebounds
off. A real aircraft rotates through the 210°/s command limit freely under full
torque and the angle error closes; pinning achieved rate == commanded rate cap
guarantees the saturated angle-loop relay cannot close (see supplemental 2).

Build: all 6 targets rebuild clean (UAVXF4V3, UAVXF4V4, DEVEBOXF4,
SPEEDYBEEF405WING, FLYINGRCF4WINGMINI, BLUEBERRYF405). The damping-sign fix
(supplemental 1) remains in place; this build = correct damping + no MR clamp.

Expected on the 30° hold: the relay breaks — pitch should track to ~30° with
rates that briefly exceed 210°/s transients but decay; if the cycle persists
unchanged, the wall hypothesis is wrong and the residual is a real controller
coupling (`QeToQa` split at high tilt) or the emu's angular damping is simply
too weak at the relay frequency (next knob: raise `2·|r|·r` ~4×).

---

## 2026-09-05 supplemental (4) — clamp experiment FALSIFIED; emulated accel was estimate-locked; TRUE-attitude fix implemented

### Unclamp result (log `20260905_112839_Ecks_220mm_REFERENCE`; ~63 s hold, lift-off → still tumbles)
- The unclamp is **operative**: `rp` reached **±441–459°/s**, far past the old
  210°/s wall — the flash works, there is no hard clamp.
- The aircraft **STILL tumbles through vertical** (pitch ±83.6°, roll ±176.6°)
  from a 30° hold. **The "hard ±A.R.Max wall sustains the relay" hypothesis
  (supplemental 2/3) is FALSIFIED.**
- New onset evidence: near lift-off a mere 4→9° pitch demand (`drp≈33°/s`) pushes
  `rp` 27→170→320→338°/s within ~200 ms while `ap` advances only ~8° — the
  reported rate and the integrated angle disagree ~5× (rate oscillating above
  the 10 Hz telemetry sampling, mean small). Consistent with an estimator/plant
  mismatch rather than a plain controller relay.
- **Desk test (real sensors, NOT emulation):** board hand-tilt → motors respond
  sanely (increase/decrease on tilt, stick override, commanded limit respected at
  ~30°). The real FC loop + estimator are healthy; the failure is emulator-
  sensor-specific.

### Root lead: the emulated accelerometer is ESTIMATE-LOCKED (self-referential)
Code verification
- `DoEmulation` (emu.c:394) is called from `DoSensorUpdate` (inertial.c:474)
  with the same `dT` as `DoControl` — one consistent 500 Hz loop, no cadence
  mismatch. `Rate[]` is TRUE plant state, written by the emu, read by
  `ControlRate` (control.c:476), Madgwick (inertial.c:499), and telemetry
  (`rp = Rate[a]`, telem.c:336).
- BUT `emu.c` **never writes `Angle[]`** — that is written only by
  `ConvertQuaternionToEuler` (inertial.c:170–174) from the **Madgwick
  estimate**. The emu accel block therefore synthesised `Acc[]` from the
  *estimated* attitude: Madgwick's accel-correction leg fed on its own output.
  No independent truth — a small tilt error self-reinforces at high acc-conf,
  exactly a tumble generator the real accelerometer (an independent physical
  measurement) cannot have.
- The Sep-03 comment claiming "TRUE simulated Euler angles" misstated the code —
  it reads the estimate (the intended decoupling never existed). Only mag yaw
  has an independent anchor (`TrueYaw`, emu.c:656).
- Additional inconsistency verified numerically: the old accel mixes an
  Euler-frame thrust direction (`fx=sp·thrust`, `fy=−sr·cp·thrust`) with
  quaternion-frame DCM rows — the two disagree at tilt (e.g. 45° bank:
  `(−11.8, 0, −2.0)` vs the quaternion-consistent `(−2.0, 0, −11.8)`).

### Fix (implemented 2026-09-05)
- `emu.c` now keeps `EmuTrueQ[4]` — a TRUE-attitude quaternion gyro-integrated
  each `DoEmulation` tick from the true `Rate[]` using the exact Madgwick
  gyro-step algebra (`q += ṡ·dT`), unit-normalised, reset to identity in
  `InitEmulation`.
- The accel block synthesises `Acc[]` self-consistently from `EmuTrueQ`:
  thrust along the quaternion body-up `row_ud` (third DCM row), world
  `fx/fy/fz` as before, rotated into body by the TrueQ DCM rows. At level this
  is byte-identical to the old output; at tilt it now reflects the physical
  attitude (a tumble sweeps the accel through ±1 g; `AccConfidence`/gyro-
  fallback see real tilt).
- Options considered/rejected: keep the old Euler-fed construction but feed
  TRUE Euler from a true-attitude tracker — rejected because the Euler-frame
  vs quaternion-frame inconsistency above is exactly the tilt-growing error
  we are removing; the quaternion-consistent vector also guarantees
  `v`-convention agreement with Madgwick's correction. Full-SIM-body
  translational suspension was out of scope (drag stays in `FakeAccU`/
  `Aircraft[].Vel`).
- Build: all 6 targets rebuild clean (emu.c is compiled into every target).

### Next (Greg)
- Reflash `SPEEDYBEEF405WINGQ_r0.bin`, re-run the 30° hold with
  `Ecks_220mm_REFERENCE.af`. Expect the emulated accel to sweep ±1 g through
  any tumble so the estimator tracks truth; if the hold is now stable, close
  the emulator-tumble investigation.
- The inverted-yaw gate (`control.c:598`) still needs a controlled-inverted
  validation (slow roll through vertical, yaw held) — unexercised by the 30° hold.

Raw logs: `/tmp/opencode/20260905_112839_Ecks_220mm_REFERENCE_20260905_1.rawlog(.csv)`.
