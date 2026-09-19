# Emulated FW Plant had Essentially Zero Control Authority — Root Cause of the Opening WP Distance

**Date:** 2026-09-16
**FC:** `UAVXArmQ/src/emu.{c,h}`
**Status:** Fixed, all 7 targets build clean. Verify with a re-run of the WP bench mission.

---

## 1. The problem

Replaying `20260916_185504.rawlog` (emulated Shadow, 4 uploaded WPs) showed a
diagnostic failure: during `eTransiting` the FC commanded `droll = 0.35 rad`
(20° bank) for 120+ seconds while the aircraft rolled at ~0.01 rad/s and the
WP distance opened to ~1022 m. The nav steering chain was verified correct
(`Navigate → DesiredNavCorr → SlewLimit → NavCorr → P.Desired`), the attitude
mode was confirmed forced to `eAngleMode`, and the telemetry pipeline was
mapped — so the failure had to be in the emulated **plant**.

## 2. Root cause

The emulated FW torque was structurally broken:

```
Rate[a] -= ( A[a].Out * A[a].R.Max * md->InertiaRollPitch + 2·Sign(Rate)·Rate² ) · dT
```

For the Shadow emu model (`MODEL_ELEVON`): `R.Max = 1.571 rad/s`,
`InertiaRollPitch = 0.0035`, so full control output produced

**0.0055 rad/s²** of angular acceleration.

Against the quadratic damping `2·Rate²`, the terminal roll rate was:

```
Rate_max = sqrt(0.0055/2) = 0.052 rad/s  ≈ 3°/s
```

The emulated Shadow simply **could not bank** — every control authority even at
full aileron capped at 3°/s, and the observed 0.6°/s was a fraction of that.
That is the opening-WP-distance root cause, end to end.

### Quantified vs the validated sim plant

The sim (`test_pid_sim.py`) was field-validated (2026-09-13) with
`FW_AUTHORITY_SCALE=4` (`UAVX_FW_AUTHORITY`). Its Shadow descriptor
(`generic/Shadow.af`: `CL_D_AIL=0.18×4`, `qbar=43.2 Pa`, `S=0.45 m²`, `b=1.8 m`,
`I_roll=0.135 kg·m²`, `ail_max=40°`) gives:

| quantity | sim (validated) | emu (was) | ratio |
|---|---|---|---|
| roll α, full Out | 130.3 rad/s² | 0.0055 | ×23 700 |
| pitch α, full Out | 276.8 rad/s² | 0.0055 | ×50 000 |
| terminal roll (damping-limited) | 8.1 rad/s | 0.052 rad/s (3°/s) | ×156 |

## 3. The fix

The emu FW branch now uses the **same aerodynamic formula** the sim validates:

```
qbar_scale = Airspeed / md->CruiseSpeed               // precomputed once
Rate[a] -= ( A[a].Out * md->FwCtrlEff[a] * Sqr(qbar_scale)
           + 2·Sign(Rate[a])·Sqr(Rate[a]) ) · dT
```

- New `EmuModel.FwCtrlEff[3]` = angular accel per unit Out **at cruise speed**
  (rad/s²), per model, derived from the sim descriptors with
  `FW_AUTHORITY_SCALE=4` baked in.

| model | roll | pitch | yaw (rudder / elevon) |
|---|---|---|---|
| MODEL_ELEVON | 130.3 | 276.8 | 6.0 (drag diff) |
| MODEL_DELTA | 16.4 | 282.8 | 60.8 |
| MODEL_AILERON | 19.5 | 43.2 | 44.3 |
| MODEL_SPOILER | 32.9 | 175.0 | 68.2 |
| MODEL_VTAIL | 15.0 (est.) | 45.0 (est.) | 25.0 (est.) |
| MODEL_RUDDER_ELEV | 0.0 (no aileron) | 52.1 | 28.6 |

- yaw term now anchors at `CoordTurnRate` with the same `FwCtrlEff` qbar formula.
- `(Airspeed/CruiseSpeed)²` gives live qbar fidelity: authority collapses at low
  speed (stall → no control) and grows at high speed.
- `InertiaRollPitch`/`InertiaYaw` remain (MR physics); FW no longer reads them.
- Turbulence amplitudes `cEmuTurbAmpFw` raised `{0.003,0.003,0.002} → {0.5,0.5,0.2}`
  (now a few % of the new authority, matching the MR ratio). Estimate only; tune
  against the IdentifyDock.

### Options considered

1. **Flat multiplier on the old form** (`×23 700`) — rejected: hides the physics,
   per-model values would be meaningless numbers, and qbar fidelity would be lost.
2. **Full physical `qbar·S·b·CL·ail_max/I` every tick** — equivalent to the chosen
   form, but needs per-model `S, b, CL, ail_max` fields. The collapsed
   `FwCtrlEff` carries the same result at cruise with one 3-field addition and a
   single `(V/Vc)²` scale. Chosen.
3. **Wait for Identify** — rejected: Identify needs a plant to excite; the
   authority-limited plant was itself the reason FW identify excitation stalled
   (the header comment said so). Now Identify has a real plant to measure.

## 4. Validation status / next steps

- All 7 FC targets build clean (`fc_build.py`).
- **Next:** re-run the 4-WP bench mission in the emulator against Shadow. Expect
  the bank demand at `eTransiting` to close quickly (full-out roll α ~130 rad/s²
  against a 3°/s→ 200°/s-rate model with the `MaxRollRate` physical clamp) and WP
  distance to converge instead of opening.
- The remaining x4 uncertainty is per-airframe; `FwCtrlEff` for the non-Shadow
  models inherits the same provisional `FW_AUTHORITY_SCALE=4` (Greg:
  "of that order"). Shadow-only authority truth holds; the Identify dock is the
  mechanism to refine each airframe (note it drives `Out`/`Rate` directly against
  this plant).

## 5. References

- `emu.h:32-83` (EmuModel + FwCtrlEff), `emu.c:106-310` (EmuModels),
  `emu.c:488-519` (FW rate dynamics), `emu.c:31-41` (turbulence).
- Sim descriptors: `tests/test_pid_sim.py` `_compute_fw_inertia` (720),
  FW descriptors (442-740), `FW_AUTHORITY_SCALE` (98).
- Prior: `Session_Report_PlantIdentHandover_Sep14.md` §2.2 (updated); `FW_AUTHORITY_SCALE`
  field-derived fudge rationale in AGENTS.md (FW rate gains /4 fudge paragraph).