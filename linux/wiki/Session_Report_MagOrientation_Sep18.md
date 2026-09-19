# Session Report — Mag Orientation (fake mag + real flip gating)

Date: 2026-09-18
FC: `UAVXArmQ/src` (C, ARM Cortex-M4). GCS: unchanged.

## 1. What / where

Two changes, plus the analysis that justifies them.

### 1.1 Real path — flip gated on ExternalMag only

`UAVXArmQ/src/sensors/hmc5xxx_mag.c` `ReadMagnetometer()`:

```c
RotateSensor(&RawMag[X], &RawMag[Y], F.ExternalMag ? 0 : SensorQuadrant);
if (F.ExternalMag) {          // was: if (F.ExternalMag || SensorFlip)
    RawMag[X] = -RawMag[X];
    RawMag[Z] = -RawMag[Z];
}
```

`SensorFlip` is the **IMU mount flip** (`icm426xx.c:131`, `bmi270.c:139`). It was
conflated into the mag mount correction, so any board that flip-configured its
IMU would also silently invert the onboard mag X/Z. No current board has an
onboard mag with `SensorFlip=true`, so the old `|| SensorFlip` was a latent
trap, not a live fault — but it is removed per Greg's directive: **"we only do
the flip on the mag when the ext mag config bit is set."**

### 1.2 Emu fake mag — physical field rotated by the true attitude

`UAVXArmQ/src/emu.c` `DoEmulation()` mag synthesis was rewritten twice in the
same session. First pass fixed the *level* convention (below); then the deeper
defect surfaced: the block integrated its **own `static real32 TrueYaw`**
from `Rate[eYaw]` and held `Mag[Z]=0`, i.e. it ignored true roll/pitch entirely.
In a banked turn the real body-frame field tilts, so Madgwick's mag correction
was being fed a *level* field and fought the true attitude — a prime suspect for
the FW banked-turn yaw freeze. Final form:

```c
const real32 m0 = EmuTrueQ[0], m1 = EmuTrueQ[1];
const real32 m2 = EmuTrueQ[2], m3 = EmuTrueQ[3];
Mag[X] = m0*m0 + m1*m1 - m2*m2 - m3*m3;
Mag[Y] = 2.0f * (m1*m2 - m0*m3);
Mag[Z] = 2.0f * (m1*m3 + m0*m2);
```

This is the first row of the body-from-world DCM built from `EmuTrueQ` — the
same true-attitude quaternion the accel block already uses (`emu.c:690`). It
takes the earth's horizontal field as due north `(1,0,0)`, so it is the exact
`m` for which Madgwick's `h(q,m) = (1,0,0)` at **every** attitude. At heading ψ
and level it reduces byte-for-byte to the previously validated
`(cos ψ, −sin ψ, 0)`. The `TrueYaw` accumulator is deleted.

The earlier level-only form (kept here for the record) was:

```c
Mag[X] =  cosf(TrueYaw);   // was -sinf(TrueYaw)
Mag[Y] = -sinf(TrueYaw);   // was  cosf(TrueYaw)
Mag[Z] = 0.0f;
```

At heading ψ, level, the earth's horizontal field expressed in body axes
(x forward, y right, z down) is `(cos ψ, −sin ψ, 0)`. That is exactly what
Madgwick's magnetometer term expects; the old `(−sin ψ, +cos ψ, 0)` was a
reflection (X/Y swap) of it.

### 1.3 Init seed aligned

`UAVXArmQ/src/sensors/hmc5xxx_mag.c` `CalculateMagneticHeading()` — the legacy
tilt formulas were derived for the old `(−sin, +cos)` convention. Since B is
exactly the X/Y swap of A, the seed formula is fed the swapped vector:

```c
xh = Mag[X] * cP + sP * (Mag[Z] * cR - Mag[Y] * sR);
yh = Mag[Y] * cR + Mag[Z] * sR;
return -atan2f(yh, xh);
```

At level this returns ψ for B (was `−ψ−90°`). Only the **init seed** changes —
`CalculateMagneticHeading` is used solely by `InitMadgwick` (`inertial.c:246`);
continuous heading still comes from the quaternion (`GetYawFromQuaternion` /
`UpdateHeading`), so the global `-atan2f` heading path is untouched (the Aug-31
report's "do not patch the global heading formula" caution is respected).

## 2. Rationale / logical discourse

### 2.1 The convention conflict (the "Mag orientation problem")

A Python twin of `MadgwickUpdate` (`inertial.c:322-441`, incl. the legacy
`bx = InvSqrt(hx²+hy²)` at line 399 — inherited from the AQ/preserve code, NOT
patched) plus the emu `EmuTrueQ` accel synthesis was built and driven at a
constant yaw rate. Four candidate emu mag conventions were tested:

| conv | Mag[] at ψ | Madgwick final err @kp=0.005 | @kp=0.5 | `CalculateMagneticHeading` |
|---|---|---|---|---|
| A (old emu) | `(−sin ψ,  cos ψ, 0)` | −0.12° | **−27.6°** | **ψ** ✓ |
| B (physical) | `( cos ψ, −sin ψ, 0)` | 0.00° | +0.10° | ψ−? (needs swap) |
| C | `( cos ψ,  sin ψ, 0)` | −0.21° | −28.2° | wrong |
| D | `( sin ψ,  cos ψ, 0)` | −1.15° | −74.5° | wrong |

Only **B** makes Madgwick track at *every* gain. A happens to track at the
default `pKpMag=0.005` because the mag correction is then too weak to matter
over normal flight times (verified bounded to ±0.15° over 120 s) — but a user
raising `pKpMag` toward its 0.55 max gets wrong-sense yaw.

The `CalculateMagneticHeading` seed was the catch: at `pKpMag=0.005` the mag
term **cannot correct a bad seed** (seeded at −90°, B only moved 1.7° in 6 s;
A held the offset exactly). So switching the emu to B alone would freeze yaw at
the wrong seed. Both ends had to move together.

### 2.2 Options considered

1. **Keep A, leave as-is** — emu tracks only at low gain; mag stays
   mis-reflected vs physics. Rejected once the gain dependence was measured.
2. **B emu only** — would freeze the init seed. Rejected.
3. **Recreate the synthetic raw external mag and run it through
   `RotateSensor` + the ExternalMag flip** — most literal, but the emu has no
   raw sensor chain and the mount constants would be invented. Justification
   for the synthesis is physics, not mount emulation.
4. **B emu + X/Y-swapped seed (ADOPTED, Greg: "B + fix seed")** — Madgwick
   correct at all gains, seed correct at all gains, minimal surface.

### 2.3 Bypass note

The emu writes `Mag[]` directly, bypassing `ReadMagnetometer` + `GetMagnetometer`
(axis permutation X/Z/Y, bias, scale). That is by design — the emu synthesises
the **calibrated body-frame field** the filter consumes, not register bytes. The
1.1 fix keeps the real chain's mount handling honest, and 1.2 makes the fake
field the same physical quantity the real calibrated chain yields.

## 3. Verification

- Python twin (`/tmp/opencode/mag_twin.py`, scratch): convention table above.
- **Mag projection identity** (`/tmp/opencode/mag_proj_check.py`): for 200 000
  random unit quaternions, feeding the new emu `Mag[]` through Madgwick's own
  `h(q,m)` gives `(1,0,0)` to within `1.6e-15`. At level ψ it reproduces
  `(cos ψ, −sin ψ, 0)` exactly for ψ = 0/45/90/180/270.
- FC build: `python3 scripts/fc_build.py` (all `BOARDS_ALL`):
  `UAVXF4V3 / UAVXF4V4 / DEVEBOXF4 / SPEEDYBEEF405WING / FLYINGRCF4WINGMINI /
  BLUEBERRYF405 / MATEKF411WING` — **all 7 OK** (exit 0) after the mag-rotation
  rewrite. Log `/tmp/opencode/fc_build_magtilt.log`. (A prior 7-target clean
  build with the level-only form is at `/tmp/opencode/fc_build_mag.log`.)
- GCS: no Python touched.

## 4. Open / follow-ups

- **Emulation re-run (host)** — fly the 4-WP emu bench and confirm heading/yaw
  now tracks through the banked turn (the previous frozen-`Angle[eYaw]` symptom).
  This is the decisive test of the 1.2 true-attitude rotation fix. Not runnable
  in the sandbox.
- **Real board mag N/E/S/W** — the Aug-31 "heading ≈ −true_heading on East"
  reading was on a garbage calibration and is pending a clean re-measure
  (`Session_Report_MagCfgRegs_Aug31.md`). The 1.3 seed fix may also affect the
  reported display; re-measure before any per-board mount-sign change.
- MR-does-not-climb under emulation remains open (separate TODO).
