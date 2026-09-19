# FW Self-Level: Horizon Mode — Implemented

**Status:** Implemented (option A) — both MR and FW.
**Date:** 2026-08-05
**Scope:** `UAVXArmQ` attitude control (`control.c`, `control.h`, `rc.c`, `rc.h`).

---

## 1. Decision

Reinstate **Horizon Mode** as a 3rd `AttitudeMode` (between Angle and Rate),
applied to **both** multicopter and fixed-wing. Rejected the alternative of
augmenting RateMode with a new self-level parameter — functionally identical
feel, but required a new gain param and duplicated legacy logic.

`A[Roll].R.Kp` (tag 0) / `A[Pitch].R.Kp` (tag 5) are unchanged rate-P gains.
No parameter was renamed or added. `pHorizonTransScale` (tag 30) is reused
as-is and is already visible on the GCS.

---

## 2. Changes

### `control.h:51`
```c
enum AttitudeModes { AngleMode, HorizonMode, RateMode, UnknownAttitudeMode };
```

### `rc.c:1203` — mode selection
Widened the Ch5/pot mapping `* 1.5f` → `* 3.0f` so the Aux1 pot sweeps
Angle → Horizon → Rate (Horizon = middle). `CurrMaxRollPitchStick` made
non-static (`:276`), `extern` added in `rc.h:82`.

### `control.c` — MR (`DoQuaternionAttitudeControl`)
`AttitudeMode == RateMode` unchanged. Otherwise compute the quaternion angle
error `Qa[]` once, then branch:
- **AngleMode**: existing angle PI (`2·Qa·P.Kp + IntE`).
- **HorizonMode**: `P.IntE = 0`, then
  ```c
  AngleRateMix = Limit(1.f - CurrMaxRollPitchStick * pHorizonTransScale, 0.f, 1.f);
  A[a].R.Desired = Limit1(2.f*Qa[a]*A[a].P.Kp, A[a].R.Max) * AngleRateMix
                 + Threshold(A[a].Stick, pStickDeadZone) * A[a].R.Max * (1.f - AngleRateMix);
  ```
  Yaw keeps its existing stick/hold handling.

### `control.c` — FW (`DoFWAttitudeControl`, refactor of the old inline switch)
Same structure as MR for Pitch/Roll: Angle / Horizon / Rate cases. The old
inline `switch (AttitudeMode)` in `DoControl()` was replaced by a single
`DoFWAttitudeControl(dT)` call (`:665`).

---

## 3. Dead code removed (quaternion/ROC-era)

- `ControllingAltitudeROC()` + `#if PIC_ROC_CTRL` gate + the `#define` — the
  abandoned **MR ROC-based altitude hold** (was laggy; replaced by the
  position-PI cascade).
- `ConditionIntE()` — defined but never called.

## 4. Retained

- `DoROCControl()` — still live for the **FW BoostClimb** path (`control.c:256`):
  rapid full-RC climb then motor-off glide between WPs (soaring cross-country).

---

## 5. Build note

`params.c` `FLOAT`/`U8` macros used a parameter named `cls`, which collided with
the `ParamMetaEntry` member `.cls` — the preprocessor substituted the argument
into the member name, producing `.classExplicit = ...`. Fixed by renaming the
macro parameters to `pcls` (`params.c:124-127`).
