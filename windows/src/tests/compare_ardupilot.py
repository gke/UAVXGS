#!/usr/bin/env python3
"""
Cross-comparison: ArduPilot default rate tunings vs our UAVX airframe tunings,
run through OUR simulation framework (same plant, same criteria).

Sanity-check intent (Greg 2026-09-04): the same exercise as compare_inav.py but
against ArduPilot.  Plug ArduPilot's independently-developed, field-proven
default rate gains into our plant and see whether our sim's criteria treat them
as plausible.  A framework that blesses real-world-good ArduPilot defaults is
sane; one that rejects them or returns absurd numbers flags a problem.

Authority-normalisation differs per ArduPilot family, so it must be stated
explicitly (this is the crux of portability):

  * Copter (MR, the Quad): the rate PID (AC_PID) operates on RAD/S error and
    its output feeds directly to the motor mixer as the +/-1.0 roll/pitch/yaw
    differential-demand ("fraction of authority"), the SAME convention as our
    sim.  So Kp/Kd transfer 1:1.  Defaults (AC_AttitudeControl_Multi.h):
        Roll/Pitch  rate P=0.135  D=0.0036
        Yaw         rate P=0.180  D=0.0
    (Copter angle P = 4.5 rad->rad/s; NOT swapped - we keep our quaternion
    outer angle loop, isolating the RATE loop, exactly as in compare_inav.)

  * Plane (FW, SkySurfer + Shadow): APM_Control servos output in CENTI-degrees
    of servo travel, +-4500 = full deflection (fraction = out/4500), and the
    rate loop acts on DEG/S error.  Effective rate P (kp_ff) and D:
        Roll  (RLL2SRV):  kp_ff = ((P - I*tau)*tau - D)/EAS2TAS = 0.345, D=0.08
        Pitch(PTCH2SRV):  kp_ff = 0.385, D=0.04
        tau=0.5, I=0.3, FF=0 (default), EAS2TAS~1.
        Kp_ours = kp_ff/4500 * RAD2DEG ;  Kd_ours = D/4500 * RAD2DEG
    ArduPlane has NO default fixed-wing RATE-YAW loop (yaw is coordinated via
    the aileron->rudder mix / passive stability), so the FW yaw axis is not
    comparable and is omitted.

Only rate P/D are compared (the defensible core): our sim has no rate-I, and
ArduPilot's I/FF are wound against its own normalisation (like iNav).  MR runs
use single-axis `simulate_axis`; FW runs use the coupled 3-axis model
`simulate_axis_coupled` with the mapped rate P/D injected via params_override.

Airframes: Quad (MR), SkySurfer_Bixler (FW aileron), Shadow (elevon/delta).
"""

import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_pid_sim import (  # noqa: E402
    PIStruct, PIDStruct, simulate_axis, simulate_axis_coupled,
    get_axis_params, get_params_for_af, load_af_params,
    TEST_STEPS, FW_TEST_STEPS, DEG_TO_RAD, RAD_TO_DEG,
    AirframeCat,
)
from critic.criteria import CRITERIA, FW_CRITERIA, step_verdicts  # noqa: E402

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# ArduPilot Copter rate P/D (rad/s error -> +/-1 fraction authority)
ARP_COPTER = {"Roll": (0.135, 0.0036), "Pitch": (0.135, 0.0036), "Yaw": (0.180, 0.0000)}

# ArduPilot Plane rate kp_ff & D (per deg/s -> centi-deg servo travel)
ARP_PLANE_RAW = {"Roll": (0.345, 0.08), "Pitch": (0.385, 0.04)}
SERVO_FULL = 4500.0  # centi-degrees = full surface deflection
EAS2TAS = 1.0        # sea-level assumption; ArduPilot divides kp_ff by this


def plane_kp_kd(kp_ff, D):
    kp_ff_s = kp_ff / EAS2TAS
    return kp_ff_s / SERVO_FULL * RAD2DEG, D / SERVO_FULL * RAD2DEG


def _run_mr(af, name):
    params, af_type, fw_mid, is_fw = get_params_for_af(af)
    akp, aki, ail, ma, rkp, rkd, mr = get_axis_params(params, name)
    step = TEST_STEPS[name]

    def run(kp, kd):
        return simulate_axis(
            PIStruct(Kp=akp, Ki=aki, IntLim=ail, Max=ma),
            PIDStruct(Kp=kp, Kd=kd, Max=mr),
            step * DEG_TO_RAD, ma, cat=AirframeCat.MR, axis_name=name,
            af_filename=af, params=params)

    ours = run(rkp, rkd)
    arp = run(*ARP_COPTER[name])
    return ours, arp, step


def _run_fw(af, name):
    params, af_type, fw_mid, is_fw = get_params_for_af(af)
    step = FW_TEST_STEPS[name]
    step_rads = {a: (FW_TEST_STEPS[a] * DEG_TO_RAD) for a in ["Roll", "Pitch", "Yaw"]}
    ours = simulate_axis_coupled(af, step_axis=name, params_override=params,
                                 step_rads=step_rads)[name]
    p2 = dict(load_af_params(af))
    kp, kd = plane_kp_kd(*ARP_PLANE_RAW[name])
    up = name.upper()
    p2[f"{up}_RATE_KP"] = kp
    p2[f"{up}_RATE_KD"] = kd
    arp = simulate_axis_coupled(af, step_axis=name, params_override=p2,
                                step_rads=step_rads)[name]
    return ours, arp, step


def _l(m):
    return (f"rise={m.rise_time_s:.2f}s  ov={m.overshoot_pct:.1f}%  "
            f"settle={m.settling_time_s:.2f}s  final={m.final_angle*RAD2DEG:.1f}deg  "
            f"nosc={m.n_oscillations}")


def _v(m, crit):
    return "PASS" if all(r.ok for r in step_verdicts(m, crit)) else "FAIL"


def main():
    out = []
    out.append("=" * 76)
    out.append("  ArduPilot defaults vs ours, same plant + criteria")
    out.append("=" * 76)

    params, af_type, fw_mid, is_fw = get_params_for_af("generic/Quad.af")
    out.append("\n--- MR: generic/Quad.af (ArduPilot Copter) ---")
    for name in ["Roll", "Pitch", "Yaw"]:
        ours, arp, step = _run_mr("generic/Quad.af", name)
        out.append(f"\n  {name}  step {step:.0f} deg")
        out.append(f"    [OURS        ] {_l(ours)}  -> {_v(ours, CRITERIA[name])}")
        out.append(f"    [Ardu Copter ] {_l(arp)} -> {_v(arp, CRITERIA[name])}   "
                   f"(rate P={ARP_COPTER[name][0]}, D={ARP_COPTER[name][1]})")

    for af, label in [("generic/SkySurfer_Bixler.af", "FW aileron"),
                      ("generic/Shadow.af", "FW elevon/delta")]:
        out.append(f"\n\n--- FW: {af} (ArduPilot Plane) ---")
        for name in ["Roll", "Pitch"]:
            ours, arp, step = _run_fw(af, name)
            kp, kd = plane_kp_kd(*ARP_PLANE_RAW[name])
            out.append(f"\n  {name}  step {step:.0f} deg")
            out.append(f"    [OURS      ] {_l(ours)}  -> {_v(ours, FW_CRITERIA[name])}")
            out.append(f"    [Ardu Plane] {_l(arp)} -> {_v(arp, FW_CRITERIA[name])}   "
                       f"(rate P={kp:.5f}, D={kd:.5f} mapped from kp_ff/D)")
        out.append(f"    (FW yaw: ArduPlane has no default rate-yaw loop - not comparable)")

    print("\n".join(out))


if __name__ == "__main__":
    main()
