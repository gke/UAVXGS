# Emulated Accelerometer — True-Attitude Projection (Tumble Fix)

Date: 2026-09-03
Module: `UAVXArmQ/src/emu.c` (FC emulation), affects GCS `acc_du/lr/fb` display

## Symptom
In emulation, an aggressive tumble showed the vertical/axial accel readout
(`acc_du` = `Acc[eYaw] = Acc[Z]`, body-down, normalized to g) **stuck near
+0.89 g** instead of sweeping through ±1 g as the aircraft rotated through
inverted. This masked the true dynamics.

## Root Cause
The emulated accelerometer was synthesised from the FC's **estimated attitude
quaternion** (`q0..q3`, i.e. the Madgwick filter state), not the simulator's
TRUE attitude. With no magnetometer, the estimate is gravity-anchored and
resists reporting a true upward flip, so the projected body accel hovered near
`cos(bank) ≈ 0.89` instead of swinging through zero and negative.

Consequence: the sensor feed to the estimator was already "blinded" by the
estimate — a circular dependency that hid real tumble physics and prevented
the acc-confidence/gyro-fallback path from exercising.

## Approach / Rationale

### Rejected (first attempt): hand-written ZYX Euler rotation
Replaced the world→body rotation with a literal Euler-angle matrix
(roll/pitch/yaw). **This was bogus** — the divergence/flip was real: the
aircraft started level, then any small pitch excitation grew into a persistent
flip with no recovery (user-verified). Rejected because the hand-built Euler
DCM did not match the codebase's quaternion convention (`ConvertEulerToQuaternion`
uses yaw/roll/pitch quarter-angles; the accel body-frame mapping differs), so
the axes were scrambled.

### Adopted: local TRUE quaternion + the exact dive-sim DCM
- Build a **local quaternion** from the true simulated Euler angles
  (`Angle[]`) using the **identical formula** as `ConvertEulerToQuaternion`
  (inertial.c:184-206) — yaw/roll/pitch quarter-angle products, then normalize.
- Use that local quaternion in the **same world→body DCM entries** the original
  dive-sim block used (the rows that map to `Acc[X/Y/Z]`).
- The world specific force construction (`fx = sp·thrust_acc`,
  `fy = -sr·cp·thrust_acc`, `fz = FakeAccU + g`) was **already** true-world
  (derived from the same true `Angle[]`); only the rotation was estimated. Now
  both are true-world.

This is a strict generalization: when the estimator tracks the true attitude,
`local_quat == estimated_quat` and the output is **byte-identical** to the
original. It cannot introduce a convention-induced flip because at level flight
the two quaternions coincide.

### Why this is correct for the tumble/confidence path
The estimator and the emulated sensor are now independent observers. During a
bogus/tumble the emulated accel truthfully reports the real body force, so the
estimation's acc-confidence logic (inertial.c gravity-vs-magnitude gating)
falls as designed and the **emulated gyro takes over** — exactly the behaviour
the user expects.

## Diff Summary (emu.c)
- Comment: now states the rotation uses a local quaternion from true Euler
  angles (identical convention to `ConvertEulerToQuaternion`).
- Added local true-quaternion build (`tq0..tq3`, normalized `nq0..nq3`).
- Reconstructed the same DCM rows (`Acc[X/Y]`, `f_body_ud`, `Acc[Z]`) from
  these local components instead of the global estimated `q0..q3`.
- Retained `sp/cp/sr/cr` (already from true `Angle[]`) for `fx/fy`.

## Build / Verification
- `emu.c` recompiles with no warnings.
- All 6 targets build clean (UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING
  [233,772 B], FLYINGRCF4WINGMINI, BLUEBERRYF405).
- GCS: no Python change needed (packet_parser/labels already read `Acc[Z]`;
  they now simply receive a truthful value).

## Notes
- `ConvertEulerToQuaternion` and `Body2World` (inertial.c:208+) are left
  untouched — this is a local, isolated computation in emu.c, not a refactor of
  the estimator.
- Regenerates the same CCM-ring/telemetry path; no protocol change.

## Follow-up: high-angle flip — AttitudeCosine() mono-thrust DISABLED (termination of this session)
Open question: after the truthful-accel fix, a high-angle hold broke into a
flip near/through vertical. User repro: angle mode, small held angle, then
increase → at ~450° it flips. The flip predates the estimator question.

Root cause (user's call, confirmed in code): **the Mono-thrust term in emu.c
multiplied `motorInput` by `AttitudeCosine()`. That was intended as a
low-angle feedforward, but it collapses Thrust to ~0 past ~90° and would
reverse sign past vertical, while roll/pitch differential torque (emu.c:460)
stayed at full authority — a thrust-vs-torque decoupling that flips the
emulated MR near/through vertical.** It is unrelated to the accel/estimator
recovery analysis; at the small-angle test attitude the accel change is
bit-identical to the old code (numerically verified).

ACTION TAKEN (2026-09-03): Disabled the `* AttitudeCosine()` mono-thrust
compensation in emu.c (MR branch) to confirm/refute it is the flip cause.
The mono-thrust now follows
`DesiredThrottle*TiltThrFFComp*BattThrFFComp + AltHoldThrComp` (the real
firmware's bounded `TiltThrFFComp` feedforward, control.c/mixer.c), never gated
by `AttitudeCosine()`. A NOTE comment marks it for easy re-enable.

Status: builds clean on all 6 targets. **USER: re-run the emulation** — if the
high-angle hold no longer flips, AttitudeCosine() was the cause, and a proper
fix (bounded feedforward / coherent thrust-vs-torque model) replaces the hack.
If it still flips, the diagnosis moves back to the estimator/imu coupling and
we instrument the signals.

### Update (same session): DISABLING did NOT stop the tumble
User re-ran the emulation after the disable — **still tumbling**. Two
independent, numerically-verified exclusions:
1. The accel change is bit-identical to the original whenever the estimator
   tracks true attitude (and ≤ ~2.9° rebuild error at exactly 90° pitch, 0
   elsewhere) — so it is not the flip generator.
2. Removing `* AttitudeCosine()` did not stop the tumble — so the mono-thrust
   decoupling was a *real* problem but NOT the (sole) tumble cause.

Given the user's direction that "the attitude comp does need to be bounded
however", the fix was re-designed: attitude compensation is now applied as a
**BOUNDED vertical-lift projection on `FakeAccU`** (the physically correct
place: only `Thrust·cos(tilt)` acts vertically), floored at
`cos(cNavMaxAngleRad)` ≈ 0.819 so it can NEVER collapse to ~0 or reverse past
vertical. `motorInput` keeps the real bounded `TiltThrFFComp` feedforward.

Status of THAT change: builds clean on all 6 targets. **USER: re-run the
emulation** — the bounded lift projection should allow level-altitude hold (via
TiltThrFFComp feedforward) while never losing authority past the max commanded
angle.

### Update (same session, continued): residual thrust-vs-torque decoupling
Beyond the vertical-lift fix, the roll/pitch **differential torque** was driven
by full `MaxThrust` regardless of actual motor thrust
(`Rate[a] -= (MaxThrust*0.25*Out*ArmLen*InertiaR[a] − damp)·dT`), while the
horizontal control force used actual `Thrust` and the yaw torque already scaled
by `MotorLagState`. A real rotor develops differential torque ∝ ΔRPM ∝ thrust,
so commanding full roll/pitch torque with no motor thrust is physically
impossible — this residual decoupling is the flip/tumble enabler distinct from
the `AttitudeCosine()` term. **FIXED: roll/pitch differential torque now scales
by `MotorLagState`** (mirroring the yaw torque), so all three axes tie their
authority to actual thrust. Builds clean on all 6 targets. **USER: re-run the
emulation.**

### Update (same session, final): AttitudeCosine() belongs in FLIGHT CODE — revert
User direction: **"AttitudeCosine is actually used by multicopters to control
thrust around the aircraft in fast flight. This provides feed forward
importantly for altitude hold. It does belong in the flight code not hidden
inside the emulation."** Agreed and applied:
- The previous bounded-lift projection on `FakeAccU` (the `Thrust·max(cosθ,
  cos(cNavMaxAngleRad))` addition) is **REVERTED**. The emu is a faithful
  plant and no longer re-implements attitude compensation; it models raw
  thrust and relies on the flight code's `CalcTiltThrFF` →
  `TiltThrFFComp` (bounded) feedforward + `AltHoldThrComp`, which
  `motorInput` already consumes. `FakeAccU = (Thrust − mg − drag)` again.
- The **thrust-scaled roll/pitch differential torque** (`·MotorLagState`)
  REMAINS — that is genuine rotor physics (torque ∝ ΔRPM ∝ thrust) and is
  the emu's job, not the flight code's.
- data channel confirmed: the existing ShowAttitude telemetry carries
  **q0–q3, Pitch/Roll/Yaw Euler, AccConfidence**, and the rawlog has rates +
  throttle, so the tumble is fully diagnosable from the terminal trace / raw
  log with NO new FC instrumentation.
- Builds clean on all 6 targets. **USER: reflash and fly — capture the raw
  log; the tumble signals (Rate[], Angle[], q, AccConfidence) are all in it.**

### Tumble persistence — still OPEN
The persistent tumble survives both the disable and the bounded-lift change, so
the residual cause is in the **attitude loop** (emu `Rate[]` torque →
Madgwick integration → `ConvertQuaternionToEuler` → control). Loose hypotheses
to investigate NEXT, in order:
1. Euler gimbal-lock / `Limit1(bi20,0.999999)` in `ConvertQuaternionToEuler`
   at pitch ~90° corrupting the recovered `Angle[]` that feeds both the emu
   accel and the controller's Desired build.
2. `A[eYaw].P.Desired = Angle[eYaw] + MinimumTurn(...)` (control.c:566)
   absorbing degenerate yaw near vertical.
3. Rate-loop / quaternion-integration divergence at large accumulated rotation
   (~450° = 90° past a full spin).
Diagnosis next step (needs instrumentation we cannot run headless here): dump
`Rate[]`, estimated `Angle[]`, `AccConfidence`, `TwoKpAcc`, `Qa`, `Out` during
the emulated tumble and watch which refuses to converge.

### Update (2026-09-03): Gimbal-lock root cause CONFIRMED and FIX APPLIED
**Root cause confirmed** (definitive rawlog analysis): `Angle[eYaw]` (Euler yaw
from `ConvertQuaternionToEuler`, inertial.c:174, `atan2f(bi10, bi00)`)
**wraps discontinuously at ±90° pitch** — the yaw gimbal lock. The chain:

1. `Angle[eYaw]` wraps from −1.4° → −163.3° at pitch = −89.8° (rawlog
   `094607` t=54212ms, row 446).
2. `A[eYaw].P.Desired = Angle[eYaw] + MinimumTurn(...)` (control.c:566)
   absorbs the wrap into the desired yaw angle.
3. `EulerToQuat(QDesired, ...)` builds a degenerate quaternion.
4. `QError` (inertial.c:637) computes the angular difference — commanding
   **maximum yaw rate**.
5. `ControlRateYaw` drives `Out[eYaw] = ±1.0` — sustained tumble, never
   recovers.

The **quaternion** attitude error (`QError`) is gimbal-lock free (it operates
on SO(3) directly), but the **construction of QDesired** from Euler angles
is not. The old `F.YawActive` codepath was unchanged — it happened to
survive because `F.YawActive` was false during the tumble.

**Fix applied** in `control.c` `DoQuaternionAttitudeControl`:

- **`F.YawActive` (user commanding yaw):** replaced Euler-based QDesired
  with body-frame quaternion composition: `QMul(QDesired, qCur, qStick)`
  where `qStick = EulerToQuat(roll, pitch, 0)`. No Euler yaw extraction
  needed. The quaternion representation is gimbal-lock free at all angles.
- **`!F.YawActive` (heading hold/nav):** retained the existing Euler path
  `Angle[eYaw] + MinimumTurn(DesiredHeading)` — this self-corrects at
  gimbal lock because `Angle[eYaw]` and `MinimumTurn`'s `Heading` wrap in
  sync (both derived from the same quaternion), so the two errors cancel in
  the sum.
- **Yaw suppression near gimbal lock** (for `!F.YawActive` only): when
  `Angle[ePitch]` exceeds `cGimbalLockPitchRad` (~75°, control.c:27),
  yaw rate and integral are zeroed to prevent the residual spurious qe[3]
  from coupling into pitch through the Euler decomposition.

The Euler `Angle[eYaw]` remains written (for display/telemetry) — only the
**attitude controller's QDesired** was changed. `UpdateHeading` still reads
`Angle[eYaw]` to derive `Heading`, and `MinimumTurn` still wraps the heading
error consistently — both wrapping paths remain correct.

User confirmed real FC with props: "took it out of emulation and the angles
and motor seems sensible. No tumbling."

**Also observed (separate issue):** at ~45° pitch, correction reverses —
forcing pitch causes the aircraft to resist then suddenly reverse,
assisting the forced angle change. This occurs on the real FC and is NOT
the gimbal-lock tumble. Needs further investigation (mode? stick inputs?
yaw active?).