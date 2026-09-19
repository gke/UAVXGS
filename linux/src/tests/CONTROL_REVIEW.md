# Commentary — Attitude/Nav Retune & Rate-Loop D-Term Review

## 1. Sampling interval `dT`: measured, not notional

The rate-loop derivative is computed as `RateD = (r − RateP) · (1/dT)` where `dT` is the
**actual elapsed time** between control cycles, taken from the microcontroller µs clock
(`dTUpdate` → `uSClock()`), not a fixed assumption like `1/CurrPIDCycleS`.

This matters specifically for the D-term:
- The derivative gain `Kd` multiplies a quantity already scaled by `1/dT`, so `Kd` is a true
  *derivative time constant* (seconds) that is **correct regardless of jitter** in the control
  period. If `dT` were a notional constant while the real period wandered (interrupt latency,
  branch variance), the D-term would scale wrongly and could either lose damping or amplify
  noise depending on the sign of the error.
- Using the live `dT` means the D-term stays physically consistent even when the loop is
  occasionally stretched. `CurrPIDCycleS` is used only to *pre-compute filter coefficients* at
  init; the runtime derivative honours the real clock. This is the right separation.

Confirmed in code:
- `uavxarm-v3-gke.c:198` → `dT = dTUpdate(&lastMainUpdateuS)`
- `clocks.c:131` → `dT = (uSClock() − Last) × 1e-6` (with rollover handling)
- `inertial.c:23` → `UsingPavelFilter = false`: **Branch B** (default) is active —
  `LPF2(error) → backward-difference → MAF`.

## 2. PI-PD vs P-PID: PI-PD is correct for this plant

The cascade is angle (outer, PI) → rate (inner, PD). The formal claim that "the integrator can
be anywhere in the loop" is algebraically true only for a linear, non-saturating loop with full
re-tuning — none of which holds here.

For a multicopter:
- **Angle disturbances are inertia-damped.** The airframe's rotational inertia is itself a heavy
  low-pass on attitude, so slow angle error is exactly what the outer **integrator** should trim.
- **Rate disturbances predominate and are fast**, often self-induced (prop wash, frame-arm flex,
  motor torque ripple). These require *damping* (a derivative action), not integration. An
  integrator in the rate path would (a) be redundant with the angle integrator, (b) add a second
  pole that pushes the cascade toward oscillation, and (c) wind up against the outer I during
  transients — for zero benefit, since a sustained rate error means continuous rotation, not a
  hold point.
- The rate setpoint is **constantly moving** (quaternion outer loop commands `2·Qe·QGain`), so
  the rate loop is a tracker, not a regulator around a fixed point. A PD with filtered D is the
  correct tracker; a PI there lags a moving setpoint and contributes phase lag precisely where it
  is unwanted.

**Conclusion:** keep PI-PD. The P-PID reformulation moves the one term (I) that is useless in the
rate path *into* it, and leaves the rate loop without a clean expression of what it actually
needs (filtered derivative damping). PI-PD is not a preference here — it is the structure the
disturbance spectrum dictates.

## 3. Rate-loop D-term implementation — sound, and the reason it has worked for years

`ControlRate` is a clean PD: `Out = ±(Kp·Error + Kd·RateD)`, no I-term. `RateD` comes from
`ComputeAttitudeRateDerivative`:

- **Default branch (active):** `r = LPF2(RateF, Error)` (low-pass the error first), then
  `RateD = (r − RateP)/dT` (backward difference), then `MAF(RateDF, RateD)` (moving-average the
  derivative).
- Both derivative branches **band-limit the signal before/after differentiating**. The
  differentiator never sees raw high-frequency rate ripple. This is precisely the firewall
  against self-induced disturbance amplification.

Why it is robust:
- The derivative is taken on the **error** (`Desired − Rate`), not the raw rate. Differentiating
  error ≈ −d(rate)/dt (setpoint is smooth from the outer loop), giving true rate damping without
  amplifying setpoint steps.
- The D-term's low-pass (`RateF`) is tied to the gyro LPF (`CurrRateLPFHz·0.6`, yaw
  `pYawLPFHz·0.6`) — it sits **below** the gyro bandwidth, damping structural/rate content while
  rolling off before the sensor noise band. The `RateDF` MAF adds a second stage. Two stages of
  D-term filtering is more than strictly necessary, but in the safe direction.
- `conditionOut` clamps the **sum** to ±1 (motor command), so the D-term cannot saturate the
  output on its own — bounded within P-term authority, no windup (there is no I).
- Filters are initialised for **all three axes** (loop `X..Z` in `InitInertialFilters`),
  including the quaternion-controlled roll/pitch/yaw, so the protection applies uniformly.

**Conclusion:** the D-term design is physically correct for the plant — PD with no rate I,
derivative on error, double-filtered. This is why it has flown stably for years. The only
caution (already observed): `Kd` must stay modest and the `RateF` corner must sit below the
arm/flex resonant frequency, because the D-term's job is to damp self-induced rate disturbance,
not to differentiate its own noise floor.

## 4. Attitude retune (this session) — summary

Driven by `test_quat_sim.py` after correcting the sim to match the augmented emulation (`emu.c`
inertia model, yaw torque fraction, realistic quadratic drag):

| Param | Was | Now | FC max |
|-------|-----|-----|--------|
| Roll/Pitch Q | 4 / 5 | **7 / 7** | 8.75 |
| Yaw Q | 8 | **3** | 10.0 |
| Roll/Pitch/Yaw RateKp | 0.10/0.15/0.25 | **0.30/0.45/0.75** | 0.50/0.50/1.0 |
| Roll/Pitch/Yaw RateKd | 0.004/0.006/0.010 | **0.015/0.0225/0.0375** | 0.02/0.025/0.05 |

- Yaw Q = 3 (not higher): yaw is under-actuated via the motor-drag differential; the sim shows
  Q > 3 oscillates (6 cycles at Q=8) regardless of flight regime. This is the axis-decoupling
  limitation, not a gain bug.
- Rate gains scale with per-axis inertia (1 : 1.5 : 2.5), consistent with the battery-fore-aft
  emulation model.
- Carried in new `EcksQuatQ.af` (all other tuning preserved from `EcksTuned.af`); GCS tables
  (`parameters.py`, `parameter_window.py`) converged; `params.c` defaults + maxes raised to
  accommodate.

## 5. GPS velocity LPF (Nav) — structural fix, not just tuning

`test_nav_sim.py` confirmed the Nav position/velocity cascade is **stable at 5 Hz and 10 Hz
with 150–300 ms GPS lag** using the existing conservative gains (PosKp=0.15, PosKi=0.012,
VelKp=0.2, MaxVel=6), with ~2× headroom before oscillation.

The change (option A): a first-order LPF (`LPF1`) at ~1 Hz on the GPS Doppler velocity, fed into
`Nav.C[a].Vel`. This also **fixes an omission** — the real-flight path never copied GPS velocity
into `Nav.C[a].Vel`, so the Nav velocity loop ran on stale/zero data. The LPF adds negligible
phase lag (0.16 s time constant), consistent at both 5/10 Hz (coefficient recomputed from live
`GPS.dT`). Wind estimator and heading still use the raw velocity. No Nav gain change needed.

## 6. Net assessment

All changes are conservative and physically motivated:
- PI-PD preserved (validated against the "integrator anywhere" claim).
- Rate D-term confirmed correctly band-limited against self-induced disturbance.
- Attitude gains raised only to match the now-accurate (agile) emulation model, with yaw
  deliberately capped.
- Nav stability verified by simulation; the velocity LPF is a structural robustness fix.

No destabilising action was taken. The FC-side files (`params.c`, `gps.c`, `emu.c`, `control.c`,
`dive.c`, `control.h`) remain for recompile; the GCS-side work is committed.
