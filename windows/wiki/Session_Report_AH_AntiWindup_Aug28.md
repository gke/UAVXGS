# Session Report — AH Overshoot on Descent: ROC-Integral Anti-Windup — Aug 28

Build status: `UAVXArmQ/scripts/fc_build.py` OK (FLYINGRCF4WINGMINI). GCS untouched (no
py_compile needed).

## Symptom
On MR (multirotor) altitude hold, a descent to a target dips (overshoots) past the
setpoint before recovering, i.e. the hold "hangs low" at the bottom of every descent.

## Mechanism (root cause)
The MR altitude loop is a cascade in `DoAltitudeControl()` (`control.c`):

1. `Alt.P` (position PI): `P.PTerm = Err*Kp`; `P.IntE` integrates `Err*Ki*dT`,
   clamped to tiny `P.IntLim` (0.00375 ≈ 0.004 m/s ROC equivalent).
2. `Alt.R` (velocity / ROC PI): command `Alt.R.Desired = clamp(P.PTerm + P.ITerm,
   R.Max)`, further clamped to `|-VRSROC|` (1.5 m/s) inside the spiral band.
   `R.IntE` integrates `(R.Desired - KFROC)*Ki*dT`, clamped to large `R.IntLim = 0.5`
   (ROC units, directly feeds throttle comp).
3. `AltHoldThrComp = clamp(R.PTerm + R.ITerm, ±pMaxAltHoldThrComp)` — this drives
   throttle.

During a long descent: `R.Error` is steady-negative; `R.IntE` winds toward +0.5 of
negative bias (in real terms ~ full negative comp). At the setpoint `R.Desired`
flips positive, but the stale negative `R.IntE` keeps `AltHoldThrComp` pinned at
`-pMaxAltHoldThrComp` — full thrust cut — until the integral unwinds (rate
`R.Error*Ki`, very slow). Result: the craft keeps sinking past the target and only
recovers after the integral bleeds off. This is classic cascade integrelor windup.

The existing `JustEngaged` path in `ControllingAltitude()` (control.c:327-331) zeroes
both `Alt.P.IntE`/`Alt.R.IntE` when hold (re)engages — but a *continuing* descent
inside a hold never re-engages, so the bias lives through the reversal.

## Options considered
1. **Gain-only tuning** (lower `AltVelKi`, raise `AltVelKp`, lower `R.IntLim`): never
   cures the reversal transient; only shrinks it while costing steady-state descend
   tracking and adding P-gain noise. Rejected — the mechanism is structural.
2. **Continuous unwinding** (decay PID term whenever `sign(P.x)<0` for integrator):
   softer but requires a new smoothing knob and is only asymptotically correct —
   still lets some bias act across the reversal. Rejected as more complex for weaker
   benefit.
3. **Zero `Alt.R.IntE` on demand-sign reversal** (adopted): the instant the *demanded*
   ROC changes sign, kill the ROC integral — exactly the `JustEngaged` philosophy, no
   new parameters, symmetric (also cures climb ballooning on back-switch). Benign
   under chatter at the reversal point: after a reset the integral simply re-builds
   from zero, which is correct at a zero-demand operating point.

## Change
`control.c` `DoAltitudeControl()` MR branch, after the spiral-band ROC clamp:

```
if ((PrevRDesiredMPS * Alt.R.Desired) < 0.0f)
    Alt.R.IntE = 0.0f;
PrevRDesiredMPS = Alt.R.Desired;
```

with `static real32 PrevRDesiredMPS = 0.0f;` at function head. Product < 0 detectssign change; the "previous value" is updated every `NewAltitudeValue` tick so both
edges (descent→climb and climb→descent) reset. `Alt.P.IntE` deliberately untouched:
its IntLim contribution (~0.004) is negligible compared to `R.IntLim` (0.5).

FW path (`eCatFw`) untouched — simple PI + `Alt.Kd` rate damping, no ROC cascade, no
windup of the same kind.

## Verification
- `fc_build.py` clean compile/link for the default target. On-air test still pending
  (user) — expect the bottom-of-descent dip to shrink to roughly the P-term
  proportional response (~ the value of the deadband, not plus 0.5 s of full cut).