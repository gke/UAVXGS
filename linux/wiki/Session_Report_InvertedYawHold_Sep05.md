# Session Report — Wrong-sense inverted yaw-hold: root cause, fix, and what iNav/ArduPilot/Betaflight do about it

Date: 2026-09-04→05 (Prof Greg & assistant)

Supersedes as the current tumble status: `Session_Report_EmuTumble_GoodGains_Sep04.md`
(rate-cap mismatch + coupled limit cycle, analysed 2026-09-04).

---

## Executive summary

The Ecks 220 mm emulated flight still tumbles even with a verified-correct `.af`
(gains confirmed via tag-71 readback) and even after the emulator-vs-controller
rate-cap mismatch was fixed. Re-analysis with Greg's gimbal-lock / yaw-sense
refinement found **two linked defects**, and this session implemented the fix for
the **accelerant**:

1. **Initiator** (still open): a 30° pitch command overshoots into a 40–56° coupled
   pitch/roll limit cycle (uncommanded roll ±12°, `desired_roll` = 0), period
   ~1.64 s — a real attitude-loop coupling, not an emulator artifact.
2. **Accelerant** (FIXED this session): while **inverted**, the autonomous
   heading-hold (Euler-derived yaw) commands body yaw in the **wrong sense**
   (body +Z ≈ −world +Z) → positive feedback → yaw spins up and the overshoot
   becomes an unrecoverable tumble. The `>75°` gimbal-lock band never fires once
   the aircraft is fully inverted (`Angle[ePitch]` reads back through −90°).

**Fix applied:** extend the heading-hold suppression to the whole inverted
hemisphere using the singularity-free quaternion test `AttitudeCosine() < 0.0`
(`control.c:598`). All 6 FC targets rebuild clean. The fix is MR-correct AND
FW-correct — arguably it is the same purpose as Greg's original FW-era `75°`
band, now applied without a gimbal-dependent threshold.

---

## 1. How we got here (recap)

- Ecks emulator (good 910 g `.af`: angleKp 7, angleKi 0.25, rateKp 0.34,
  rateKd 0.008, yaw angleKp 3; limits 210°/s R.Max, 30° P.Max) still produces a
  coupled pitch/roll limit cycle + full flips on a 30° hold. Log
  `20260904_172834_..._163040.rawlog` (pre-fix) and `20260904_205928_..._2026.rawlog`
  (post rate-cap fix, 21:00 hours — the log to analyse; the 20:55–20:59 logs were
  bench/no-flight, states 7/12, zero in-flight samples).
- **Rate-cap mismatch fixed (2026-09-04):** the emu MR plant clamped achieved
  `Rate[]` to hardcoded `md->MaxRollRate/MaxPitchRate/MaxYawRate` = 300/300/180°/s
  while the controller's design limit is `A[a].R.Max` = 210/210/120°/s. The MR
  branch now clamps to `A[a].R.Max` (`emu.c:486-488`; FW and Land branches
  unchanged). Post-fix the rates **do** cap at 210°/s, but the tumble persisted —
  so it is NOT an emulator rate-cap artifact.

## 2. Root cause — two linked defects (confirmed against data)

### 2.1 Accelerant — wrong-sense yaw hold when inverted (Greg, 2026-09-04)

- `F.YawActive = Abs(A[eYaw].Stick) > pStickDeadZone` (`control.c:521`): TRUE only
  while the pilot deflects the yaw stick past the deadzone → selects the body-frame
  quaternion stick-rotation path (gimbal-lock free at all angles). In an
  autonomous heading-hold (sticks centred) it is FALSE, so the controller runs the
  Euler path:
  `A[eYaw].P.Desired = Angle[eYaw] + MinimumTurn(DesiredHeading)` (control.c:573).
- Crossing ±90° pitch, Euler `Angle[eYaw]` flips **180°** (gimbal lock).
- The existing yaw-suppression guard
  `if (!F.YawActive && fabsf(Angle[ePitch]) > cGimbalLockPitchRad)` with
  `cGimbalLockPitchRad = 1.309 rad ≈ 75°` (`control.c:28,598`) fires only inside
  |pitch| ∈ (75°, 90°). **Fully inverted, `Angle[ePitch]` reads back through
  −90→−60→…, so the guard is OFF and the heading-hold is fully active.**
- The controller commands body yaw toward the 180°-flipped heading target.
  Inverted, body +Z ≈ −world +Z, so the commanded body rotation pushes the actual
  world heading AWAY from the true `DesiredHeading` — **wrong sense → positive
  feedback → yaw spin-up.** Data: `dy` locked at −102.8° while `ay` sweeps
  −146→−153→−108→+80° (~226° of heading "hold" drift), tumble persists until
  stick release — matching Greg's real-aircraft observation exactly.

### 2.2 Initiator — 30°→56° angle-loop limit cycle (open)

Pre-fix log `172834` never crossed 90° (max |ap| = 55.9°, `dy` = 0.0), yet hold
30° pitch still oscillates 39.8°→55.9° with uncommanded roll oscillation and both
axis rates pegging the cap simultaneously. This is the trigger that pushes the
aircraft through vertical into the inverted region. Working hypothesis:
`QeToQa` cross-coupling of a large pitch error into roll demand at ~50° tilt.
(Not addressed this session — see Next steps.)

## 3. The fix — suppress heading-hold yaw across the inverted hemisphere

Extend the suppression condition with the singularity-free quaternion cosine
`AttitudeCosine() = q0q0 − q1q1 − q2q2 + q3q3` (`inertial.c:229`): body-Z·world-Z
cosine < 0 ⟺ inverted. Same test already used for `TiltThrFFComp`
(`control.c:79`) and `F.NearLevel` (`control.c:701`).

```c
// Near gimbal lock (|pitch| > cGimbalLockPitchRad) the Euler-path yaw error is
// small but non-zero — suppress it and the integral to prevent any yaw rate
// coupling into pitch. Fully inverted (AttitudeCosine() < 0) the heading
// command is WRONG-SENSED (body +Z ≈ -world +Z), so holding it is positive
// feedback: yaw spins up into a tumble.
if (!F.YawActive
        && (AttitudeCosine() < 0.0f
                || fabsf(Angle[ePitch]) > cGimbalLockPitchRad)) {
    Qa[eYaw] = 0.0f;
    A[eYaw].P.IntE = 0.0f;
}
```

### Why this is safe and correct for BOTH MR and FW (Greg's historical context)

Greg's own history is exactly why the old scheme needs this upgrade:

- **"The old scheme was clumsy but it worked very well with FW."** `cGimbalLockPitchRad`
  (75°) is FW/Euler-era logic: at pitch >75° Euler roll is undefined → ailerons flick
  lock-to-lock (the classic vertical-dive / no-recovery scenario Greg posted about).
  It *worked* because an autopilot-commanded FW flight stays inside its attitude
  envelope; the 75° gate only ever needed to catch the pathological case.
- **"MultiCopters were not anticipated to be at such high angles of attack."** A
  MR doing a sustained 30–56° hold was never an operating point. The quaternion
  outer-loop rewrite solved the ROLL problem for all attitudes; the Euler-derived
  **yaw** reference is the surviving weak link — it is only well-defined in the
  upright hemisphere.
- **"The racers were and are flying on rate control only."** The entire
  Betaflight-lineage community that operates past ±90° does so in ACRO/rate mode —
  no angle loop in the path — so angle-mode correctness at high tilt never became a
  priority there (see §5). Our MR flies ANGLE at high tilt by design, so we cannot
  inherit that blind spot.

For FW specifically the new condition is strictly *more* protective than the 75°
band: it also catches **inverted at moderate pitch** (slow roll, inverted cruise —
body Z down, |pitch| < 75°), where the old band never fired; the heading-hold was
wrong-sensed there too. Knife-edge / 90° bank gives cosine ≈ 0, **not** < 0, so
high-bank turns are unaffected. Aerobatics are unaffected: the gate is predicated
on `!F.YawActive`, so any manoeuvre flown with stick input keeps its yaw authority.
A plane cannot lose anything by killing a wrong-sensed autonomous yaw hold.

### Alternative considered and rejected

Making the heading reference singularity-free (always use the quaternion
`F.YawActive` path, or a mag-anchored world heading) would also fix the inverted
sense, and is the "proper" long-term answer. Rejected for this iteration because
(1) the quaternion stick path needs a world-frame heading concept that doesn't
exist yet on the autopilot path, (2) a mag-dependent fix would not hold in the
emulator (no mag), and (3) the suppression gate is a one-line, zero-parameter,
zero-EM interference fix that covers the immediate unrecoverable-spin hazard.
Revisit the singularity-free heading reference if sustained inverted autonomous
flight is ever a requirement.

## 4. Verification

- `UAVXArmQ/src/control.c:598` — guard edited as above (comment updated to state
  WHY, matching the code rethink).
- `python3 UAVXArmQ/scripts/fc_build.py` — **all 6 targets build clean**:
  UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI, BLUEBERRYF405.
- No GCS changes (no `uavx-python/src` edits → `py_compile` not needed this session).
- Next validation is behavioural: **Greg re-runs the Ecks emulator** — expect the
  yaw spin to decay on inversion and the 30° overshoot to recover instead of
  tumbling; the pitch/roll limit cycle (initiator, §2.2) may still be present.

## 5. Do iNav / ArduPilot have these recovery processes?

Short answer: **ArduPilot effectively does (thrust-vector-first, quaternion aage
error, never commands beyond ~45°); iNav does not — it ships the same Euler-level
architecture we just patched and the quaternion work is still a draft; Betaflight
never needed to because its high-angle users fly rate control only.**

### Betaflight (Greg's posts — the same failure class)

Greg (handle **@gke**, github.com/gke, "Prof. Greg Egan", Melbourne) participated
directly in Betaflight's uncommanded-yaw-spin work — the racing-world analogue of
our inverted-yaw spin, though at the RATE level:

| thread | what it covers |
|---|---|
| [#3959 Gyro overflow causing high-speed yaw spin after crash](https://github.com/betaflight/betaflight/issues/3959) | **Greg's main thread**: gyro overflow sign-reversal + yaw ITerm windup → "yaw spin to the moon" (YSTTM). ctzsnooze: "two separate causes — yaw ITerm windup, yaw gyro overflow." Greg proposed the **slew filter** concept and described his ~20-year-old test rig ("reduced the roll/pitch authority, bottom strings from each arm") — the UAVX heritage. |
| [#3909 Fix "yaw spin to the moon" after crash](https://github.com/betaflight/betaflight/pull/3909) + [#3950](https://github.com/betaflight/betaflight/pull/3950) + [#3983](https://github.com/betaflight/betaflight/pull/3983) | The yaw-ITerm-windup mitigation chain; #3983 added **the slew filter** (Greg's proposal) to the gyro-overflow path (later restricted to Z in 32 kHz loops). |
| [#4058 Rate Limiter — prospective solution for YTTM (author: gke)](https://github.com/betaflight/betaflight/pull/4058) | Greg's own PR (closed, superseded) — prospective rate-limiter based fix. |
| [#4010 Small possible performance improvement](https://github.com/betaflight/betaflight/issues/4010) | Reviewed by Greg. |
| [#4407 / #4736 Slew filter not working in 3.2.1 / only in 32 kHz](https://github.com/betaflight/betaflight/issues/4407) | Post-merge regressions Greg helped diagnose ("slew filter is gone in 3.2", only Z). |

Relevance to us: Betaflight's problem is the *rate-loop* equivalent (gyro data gone
bad + yaw-I windup torque). Ours is the *angle-loop* equivalent (Euler heading
reference goes 180°-bad while quiet). Same family: an otherwise-good controller
receives a sense-corrupted yaw reference and spins itself up. Betaflight fixed it
with filtering + I-term measures at ~1 kHz rate level; we fix it by gating the
input that produces the bad reference at the angle level.

### iNav — the gimbal-lock / 180° ambiguity is a KNOWN, still-open gap

- [#272 Gimbal lock at 90° tilt (digitalentity)](https://github.com/iNavFlight/inav/issues/272):
  Euler gimbal-lock acknowledged; quaternions preferred for AHRS; **but the iNav MR
  level controller (`pidLevel`) remains per-axis Euler with a `max_angle_inclination`
  clamp** — the campaign below never shipped.
- [#734 Quaternion based level PIDs (HaukeRa, 2017)](https://github.com/iNavFlight/inav/issues/734):
  the exact antipodal problem we solved: *"When error goes to 180° it's impossible
  to determine which way it will choose to go into the target orientation. This can
  be seen in the video when the vehicle makes a flip instead of rotating back."* A
  quaternion attitude-error controller bench-tested to 90°+ but **never merged**;
  the discussion even flags "heading hold in earth's frame vs airframe frame" — the
  same design fork we chose between in §3. iNav ships the old scheme.
- [#1077 exposes orientation quaternion](https://github.com/iNavFlight/inav/pull/1077)
  (the enabling PR for #734) — also not the controller itself.
- [#11470 MC Poshold attitude failure on uncommanded climb](https://github.com/iNavFlight/inav/issues/11470)
  (2026): iNav MR flips "1 time in 3" when the altitude controller drops throttle to
  0 and the level loop loses motor authority at ~50° pitch — the *same* regime
  (high pitch + level control) that starts our initiator. Not fixed; workarounds
  suggested (min throttle, delay althold engage).
- [#11695 Fixed wing: quaternion orientation hold — INVERT / KNIFE / P-HANG, 3DLOCK (draft, 2026, swissembedded)](https://github.com/iNavFlight/inav/pull/11695):
  the first iNav FW change to treat 3D/inverted attitudes as a real operating point
  ("no gimbal lock, no special-casing at pitch 90; near-antipodal engage resolves
  deterministically by rolling about body X"; `small_angle = 180` instructions).
  **Draft, not flight-tested.** iNav proper still has no inverted-hemisphere
  handling on the shipped controller.
- AHRS side, iNav does validate/reset its quaternion (`imuValidateQuaternion`,
  reset to last-known-good or accel; later #10980 horizon-drift auto-reset draft) —
  that is estimation robustness, not controller recovery. With a valid attitude,
  iNav still cannot recover a *sensed wrong* heading command because the controller
  stays Euler-level.

### ArduPilot — genuinely does, structurally

ArduPilot Copter's `AC_AttitudeControl` went full quaternion attitude-error and
serialises the correction in a way that inherently de-flips:

- `attitude_controller_run_quat()` + `thrust_heading_rotation_angles()`: the
  attitude error is a **single quaternion product** decomposed by
  `to_axis_angle()` (no Euler extract in the error path) — **thrust-vector first
  (roll+pitch), heading second**. While the thrust error exceeds
  `2·AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD`, **yaw rate is slaved to `gyro.z`** (no
  active heading correction at all) — i.e. ArduPilot *deliberately suppresses yaw
  correction until the vehicle is back near upright*. This is functionally the same
  decision as our `Qa[eYaw]=0` gate, generalised to a smooth thrust-angle criterion
  instead of a signed inversion test. (Leonard Hall, the architect, covers the
  why — roll/pitch must not be dilated by a slow/dead yaw axis — in the
  [ArduCopter quaternion-attitude-control discourse thread](https://discuss.ardupilot.org/t/arducopter-quaternion-based-attitude-control/23554/44);
  see also [#17198](https://github.com/ArduPilot/ardupilot/pull/17198) and
  [#23386](https://github.com/ArduPilot/ardupilot/issues/23386).)
- **The angle loop never flies into trouble by design**: Copter's default
  `ANGLE_MAX` is **45°** — autonomous flight never commands higher tilt, so it
  rarely reaches the region where sense becomes ambiguous. The recovery path exists
  to rescue from a *disturbance* (knock inverted, lost motor), not to fly there.
- Plane: no inverted handling on the shipped controller (Euler roll/pitch with an
  angle clamp); yaw is coordinated via aileron→rudder, no autonomous yaw-hold —
  so the wrong-sense-inverted-yaw failure cannot arise there (there is no yaw
  command to invert). This matches "no default FW rate-yaw loop" noted in the
  ArduPilot tuning report.

**Net:** ArduPilot avoids the failure by (a) never commanding high tilt, and
(b) suppressing heading correction with a thrust-error criterion during recovery.
iNav inherits the same blind spot we just patched and its quaternion options are
unmerged. Neither has a signed-inversion (`AttitudeCosine() < 0`) heading gate
like ours — ArduPilot's gating is softer (angle-based, smooth), ours is binary but
explicit and covers exactly the wrong-sense region. A future upgrade could adopt
ArduPilot's smooth thrust-error-based yaw suppression; the binary gate is the
minimum that closes the flip hazard.

## 6. Next steps

- **Greg: re-run the Ecks emulator with the rebuilt firmware** — confirm the yaw
  spin decays once inverted (angle hold released) and the overshoot no longer
  diverges into a full flip. Capture a rawlog for the record.
- **Priority 2 (still open): the initiator** — the 30°→56° coupled pitch/roll
  limit cycle / uncommanded roll at ~50° tilt. Suspects: `QeToQa` cross-coupling
  of large pitch error into roll demand; rate-loop D-term size in the saturation
  brake-out; throttle/alt-hold interaction at ~50°. Investigate with the emulator
  once the recovered-inverted behaviour is confirmed, then look at hardware.
- Optional later: replace the binary inverted gate with ArduPilot-style smooth
  thrust-error yaw suppression; or move the heading reference fully to the
  quaternion path and delete the Euler `!F.YawActive` branch entirely.

## Files

- `UAVXArmQ/src/control.c:598` — the implemented guard (`AttitudeCosine() < 0.0f ||
  fabsf(Angle[ePitch]) > cGimbalLockPitchRad`); `F.YawActive` at :521; Euler yaw
  path :573; `cGimbalLockPitchRad` at :28.
- `UAVXArmQ/src/inertial.c:229` — `AttitudeCosine()`.
- `UAVXArmQ/src/emu.c` — MR rate-cap fix (previous session, this report's context).
- Logs `20260904_172834_...rawlog` (limit cycle), `20260904_205928_...rawlog`
  (post-fix flips), decoded CSVs under `/tmp/opencode/`.
- Reports: `Session_Report_EmuTumble_GoodGains_Sep04.md` (prior), this report.
- External: threads in §5 (Betaflight #3959/#3983/#4058 + Greg's author record;
  iNav #272/#734/#1077/#11470/#11695; ArduPilot AC_AttitudeControl + #17198 +
  discourse #23554).

---

*PDF conversion is the user's job (host "Merlin", `scripts/md2pdf.sh`) —
the assistant does not run pandoc/xelatex.*