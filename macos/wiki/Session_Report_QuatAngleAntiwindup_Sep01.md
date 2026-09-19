# Session Report — Quaternion Angle-Loop Integral Anti-Windup

Date: 2026-09-01 (Prof Greg)

## Summary

Restored the **error-sign-change integral reset** (the original `ConditionIntE`
anti-windup) into the **quaternion angle loop**, where it had been dropped during
the quaternion rewrite. This is the fix for the **angle overshoot** observed when
flying `XXX.af` (the originalparams factory-default airframe): `XXX.af` re-enables
the angle I-term (`P.Ki = 0.25`, `P.IntLim = 0.15 rad/s`) that the tuned set
effectively disabled (`IntLim ≈ 0.01`), and the post-rewrite integrator was a plain
continuous accumulator with only a disarm reset — stale trim pushed through every
setpoint reversal, causing overshoot.

## The verification that changed our understanding

The user challenged whether the quaternion angle gain was correctly scaled from
`originalparams.c`. We proved it IS (below) — so the angle **P** gain was exonerated
and suspicion fell on the **integrator**, which had been rebuilt without the
original's anti-windup.

**Quaternion P gain == original Angle Kp (proven, both source trees):**
- Original (`UAVXArm32F4Preserve/src/control.c:421` `DoAngleControl`):
  `P->PTerm = P->Error * P->Kp` with `P->Kp = RollAngleQKp(int) × 0.25`
  (`originalparams.c:186`). Factory int `28 × 0.25 = 7.0`.
- Current (`UAVXArmQ/src/control.c:583`): `rate = 2.0f * Qa[a] * P.Kp + P.IntE`
  with `Qa = sin(θ/2)` (error-quaternion body vector, control.h `QeToQa`).
- `2·sin(θ/2) ≡ θ` small-angle → the `×2` exactly cancels the `sin(θ/2)` half-angle
  and **restores** `θ × Kp`. The gain IS the original scaled Angle Kp (a factor-of-2
  was suspected but ruled out). `A[eRoll].P.Kp` is bound directly to `ROLL_ANGLE_Q_KP`
  (params.c:149, no extra scale in the apply/control path). Only divergence: large
  angles where `2·sin(θ/2) < θ` (quaternion is *less* aggressive — never more).

## Where the original anti-windup went

The original `ConditionIntE` (`UAVXArm32F4Preserve/src/control.c:355`) exists in the
preserved tree but:
1. was invoked **only for yaw** (`DoTurnControl`, control.c:472);
2. was **wholly removed** in the current quaternion rewrite — `ConditionIntE` no longer
   exists anywhere in `UAVXArmQ/src`.

Original roll/pitch used a *plain* continuous integrator (no conditioning);
current quaternion reintegrated roll/pitch/yaw as plain continuous with only a
disarm reset (`ZeroIntegrators`, control.c:430). So the roll/pitch angle I-term was
never sign-conditioned in either tree — and with `XXX.af` turning the I-term back on,
the overshoot appeared.

Note (user recollection): he recalled zeroing the integral when "the sign of the
error differed from the sign of the integral." Actual original semantics compare the
error sign to its **own previous value** (sign-change reset), not to the integral.
Documented decision: adopt the original sign-**change** reset (option A), which
directly targets overshoot on every reversal, versus a sign-vs-integral reset
(option B, more targeted for steady-trim preservation). Option A chosen: it is the
proven original behaviour, it cures the observed overshoot, and B's benefit (holding
a legitimate trim bias) is rarely needed for attitude in this codebase (wind/gust trim
is handled by the altitude integral and nav corrections). B remains a documented
future backstop.

## Implementation (`UAVXArmQ/src/control.c`)

- `static int8 quatAngleIntESign[eYaw + 1];` (line 432) — per-axis sign memory (3 axes),
  survives across `DoQuaternionAttitudeControl` calls.
- `static void ConditionQuatIntE(PIStruct *P, real32 errProxy, idx a, real32 dT)`
  (line 434) — integrates `errProxy·Ki·dT` only while `Sign(errProxy)` matches the
  previous tick's, else dumps `P->IntE=0`. Single logical exit (AGENTS.md rule 1).
  The error proxy is `2·Qa[a]` (≡ `θ`), preserving the original `Sign(P->Error)` keying.
- `ZeroIntegrators()` (line 458) also clears the sign memory so disarm fully resets
  the conditioning.
- Wired into the angle-PI branch (line 604): `ConditionQuatIntE(&A[a].P, 2.0f*Qa[a], a, dT)`.

Mirrors the original semantic exactly; the only change is that sign-conditioning now
applies to roll/pitch **and** yaw (original applied it to yaw only).

## Build / verification

All **six commissioned boards** built cleanly with the change (AGENTS.md: build all
as cheap early-warning):

| Board | Status |
|---|---|
| **UAVXF4V3** (current flight target) | ✅ 231308 B |
| UAVXF4V4 | ✅ 230852 B |
| DEVEBOXF4 | ✅ 231092 B |
| SPEEDYBEEF405WING | ✅ 232596 B |
| FLYINGRCF4WINGMINI | ✅ 232828 B |
| BLUEBERRYF405 | ✅ 232548 B |

## Reflash note

A reflash is required anyway for the **prop-sense changes** (`eUsePropSense` →
`ePropsInwards` rename + `MultiPropSense` polarity flip). One flash of
`obj/UAVXF4V3/UAVXF4V3Q_r0.bin` covers **both** the integral anti-windup and the
prop-sense corrections.

## Related context (same session)

- Created `airframes/XXX.af` (later `airframes/user/XXX.af`): originalparams factory
  defaults scaled to unified raw (`raw = integer × PARAM_SCALES`), e.g. RateKp 0.1
  (int 20), RateKd 0.0045 (int 45), AltPosKp 0.5124 (28), NavPosKp 0.2475 (15),
  NavPosKi 0.012 (3); non-legacy params from `original/Ecks_220mm.af`; corrected
  650 g/5"/2200 mAh physics; CONFIG1/CONFIG2 explicit, prop-sense bit clear.
- Confirmed `Ecks_Old.af` location: `airframes/user/Ecks_Old.af`.
- Legacy comparison (tuned vs XXX factory): tuned set runs ~3-6× hotter P (rate P
  68 vs 20 legacy, alt pos 114.8 vs 28, nav pos 90.9 vs 15) and **disabled the angle
  integrator** (IntLim 0.7 ≈ 0 vs factory 10) — consistent with the overshoot being
  the re-enabled integrator, and the 4 Hz OSC being the hotter rate P.
