# Session Report — Shadow Tuning + Elevon Mixing Re-check (Sep 07)

## Objective
User deleted most `Shadow.af` airframe files, keeping `user/Shadow.af`.
Requested the **complete tuning process** run again on it, with a report of the
gains used, and the following settings:
- RX = SBus (`eFutabaSBusRx`)
- `VOLT_SCALE` = 37.5
- Airframe = elevon (`eElevonAF`)
- User's observation: "Angle Kp/Qp of 1.75 and rate KP of 0.002 are way too
  low" — these are the **FC ParamTable defaults** (`params.c` `A[].P.Kp`
  default 1.75, `A[].R.Kp` default 0.0015/0.00225), confirming the flashed
  stock config flies with near-zero authority. The tuned `.af` must override
  them.
- Also asked to **look at the elevon mixing again**.

## Final Gains Written to `user/Shadow.af`

| param | old (was generic × copy) | new | FC class bound | rationale |
|---|---|---|---|---|
| `ROLL_ANGLE_Q_KP` | 7.8 | **7.8** | 0..20 | unchanged |
| `ROLL_RATE_KP` | 0.53476 | **0.75** | 0..3.0 | cut overshoot 24.1% → 21.6% |
| `ROLL_RATE_KD` | 0.0168 | 0.0168 | 0..0.2 | unchanged |
| `PITCH_ANGLE_Q_KP` | 1 | **8** | 0..20 | was the FAIL (G=0.334); now G=16 |
| `PITCH_RATE_KP` | 0.33422 | **2.0** | 0..3.0 | part of G=16 |
| `PITCH_RATE_KD` | 0 | 0 | 0..0.2 | KD=0 passed; left as file |
| `PITCH_ANGLE_Q_KI` | 2 | 2 | 0..25 | unchanged |
| `YAW_*` | (gen) | unchanged | | yaw not scored (rudderless elevon) |
| `RX_TYPE` | `eCRSFRx` | **`eFutabaSBusRx`** | U8 enum | user |
| `SERVO_SENSE` | 9 | **0** | 0..127 bitmask | elevon mirrored-mount = all +1 |
| `VOLT_SCALE` | 18.5 | **37.5** | 0..255 | user |
| `AF_TYPE` | `eElevonAF` | `eElevonAF` | enum | already correct |

Slider-range (`[LIMITS]`) updates to hold the new values: `ROLL_ANGLE_Q_KP =
1.95, 20`; `PITCH_ANGLE_Q_KP = 0.25, 20` (was 0.25, 1 — too small for 8);
`VOLT_SCALE = 9.25, 50` (was 9.25, 37 — too small for 37.5).

## Method — the complete tuning process

1. **Critique baseline** (`tests/test_pid_sim.py user/Shadow.af`): Roll PASS
   (24.1% overshoot, marginal), **Pitch FAIL** (rise 8.0 s, final error
   23.21° — never crossed 27°).
2. **Static-hold equilibrium analysis.** The generic Shadow's elevator treats
   ~61% authority margin at 30° pitch against static pitch stability. With the
   Q-fork P-only (I-path negligible: `IntLim=0.01` caps it at 0.01 rad/s), the
   control law averages to some 30°·G/(G+1.17) where `G =
   PITCH_ANGLE_Q_KP · PITCH_RATE_KP`. Crossing the 90%-of-step threshold
   (27°) requires **G ≥ ~12**. Phoenix passes the same 30° step with G=6 only
   because its elevator authority is ~4× Shadow's (0.008 m² @ 0.60 m arm, S =
   0.45 vs 0.0044 m² @ 0.25 m, S = 0.1517) and its transient peaks 27.9°.
   Copying Phoenix gains onto Shadow fails (Roll overshoot 43.9%, Pitch rise
   8 s) — verified by sweep.
3. **Gain sweep** (`/tmp/opencode/sweep*.py` → `simulate_axis_coupled` with the
   real `GUST_MAG` and `FW_TEST_STEPS`, `critique` with `FW_CRITERIA`).
   Pitch candidates with G ≥ 12, all PASS with 0% overshoot, 0 oscillations:
   ang6/pr2 (final 27.4°), ang12/pr1 (27.4°), **ang8/pr2 (final 28.0°, rise
   0.38 s)** ← chosen, ang6/pr3 (28.3°), ang12/pr2 (28.7°), ang15/pr2
   (28.9°). Roll rate raised 0.535 → **0.75** to open the overshoot margin
   (24.1%→21.6%); KD=0.0168 kept (0.0035 causes 26.3% FAIL).
   Cascade-integrity caveats are informational in this sim (P-path saturates
   the rate clamp; normal for rate-limited loops).
4. **Full re-critique** on the edited file: **all sections PASS** (Roll/Pitch
   steps, disturbance rejection, Alt Hold 5 m, Nav 10 m, free-flight
   turbulence, lateral modes ζ=+0.49 / stable spiral).
5. **Fleet-wide regression** (`tests/fleet_tuning_check.py --csv`):
   `user/Shadow.af` PASS, `generic/Shadow.af` PASS slider-extremes. The
   `backup_angleunits/Shadow.af` FAIL is a stale backup dir, not our target.
6. **FC bounds check** against `params.c` ParamTable: all chosen gains within
   class bounds (`eClassGainRateP` 0..3.0, `eClassGainAngleQ` 0..20,
   `eClassGainRateD` 0..0.2, `eClassVolts` 0..255).
7. **GCS parse check** (`parse_af_file`): SERVO_SENSE=0.0, VOLT_SCALE=37.5,
   RX_TYPE=1.0 (`eFutabaSBusRx` = 1 in `params.h:226`), plus
   `py_compile` clean on `airframes.py`/`parameters.py`/`protocol_enums.py`.

## Elevon Mixing Re-check (the user's question)

Current Q-fork `eElevonAF` in `UAVXArmQ/src/mixer.c:247-255`:

```c
TempElevator = PWSense[ElevatorC] * (F.PassThru ? Pl : (Pl + pFWRollPitchFFFrac * Abs(Rl)));
PW[RightElevonC] = PWSense[RightElevonC] * (TempElevator + Rl + Yl) + cOutNeutral;
PW[LeftElevonC]  = PWSense[LeftElevonC]  * (-TempElevator + Rl + Yl) + cOutNeutral;
```

- **Pitch** goes into the **common mode** (`+TempElevator` right,
  `-TempElevator` left) — with opposite-hand (mirrored) servos this is
  symmetric surface deflection. Correct.
- **Roll and Yaw both ride the differential** (`+Rl + Yl` on both channels).
  Same-sign on both = mirrored servos deflect differentially = yaw via
  **drag-differential** (aileron-like), not a rudder that does not exist.
- This matches the sim exactly (`test_pid_sim.py:1758-1764`):
  `ail_rad = Rl·ail_max`, `ele_rad = TempElevator·ele_max`,
  `rud_rad = Yl·ail_max`, yaw torque via `C_n_elevon = adverse_yaw·C_l_ail`.
- **Heritage 32F4** mixed yaw onto `PW[RudderC]` only — a no-op on a
  rudderless plank (its elevons only carried `TempElevator + Rl`). The Q-fork
  moved `Yl` into both elevons — that is the intended fix, and **removing it
  or routing it to RudderC again would re-zero yaw authority on the plank**.
- **SERVO_SENSE must be 0** for this convention: PWSense all +1, opposite-hand
  servos located at the surface. The old airborne `9` (binary 1001, bits 0 and
  3 → inverts RightAileron(S3) and Rudder(S6)) would invert the **right
  elevon only** — wrong-sensed roll/pitch on a mirrored mount and the reason
  the airframe was previously flagged dangerous. `InitServoSense()`
  (`mixer.c:439-451`) maps bits onto `PWSense[SM[]]`.

## Verification Status
- `user/Shadow.af` critique: **47 PASS, 0 FAIL** across all sections.
- Fleet check: `user/Shadow.af` and `generic/Shadow.af` PASS.
- GCS parse + py_compile: clean. No FC source changed in this session.
- **USER: reflash the FC with the tuned Shadow gains (write + tag-72 commit),
  verify tag-71 readback shows PITCH_ANGLE_Q_KP=8, PITCH_RATE_KP=2.0,
  ROLL_RATE_KP=0.75 before flight.** Confirm SBus RX binds and the elevon
  servo direction is correct on the bench (mirrored mount, SERVO_SENSE=0)
  before the first flight.

## Follow-up (still open)
- **Flash-write alarm still RED** in GCS — the tag-72 flash-commit NACK /
  verify-mismatch investigation shown earlier is still not resolved. Commanded
  to the next session.