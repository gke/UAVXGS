#!/usr/bin/env python3
"""
Cross-comparison: iNav default rate tunings vs our UAVX airframe tunings,
run through OUR simulation framework (same plant, same criteria).

Sanity-check intent (Greg 2026-09-04): plug iNav's independently-developed,
field-proven default PID gains into our plant and see whether our sim's
criteria treat them as plausible.  A framework that blesses real-world-good
iNav defaults and yields realistic rise/settle/overshoot is sane; one that
rejects them or returns absurd numbers flags a problem.

Unit mapping (documented in Session_Report_InavTuning_Sep04.md):
  iNav pidSum output is clamped to pidSumLimit (500 MR roll/pitch + FW,
  400 MR yaw); our sim output is +/-1.0 fraction of authority.
  iNav gains act on deg/s; ours on rad/s.  Conversions:
     Kp_ours = (P/31)  * (180/pi) / limit
     Kd_ours = (D/1905) * (180/pi) / limit
  Only the rate P/D are compared (the defensible core): our sim has no rate-I
  (steady-state lives in the outer angle-loop I) and iNav's I/FF are wound
  against iNav's own output-normalisation/clamp, so a linear unit-map of I/FF
  is not portable (shown to overfeed/overshoot).  Each axis therefore runs
  iNav's P/D mapped into our units, keeping our outer angle loop.

MR runs use the single-axis `simulate_axis`; FW runs use the authoritative
coupled 3-axis model `simulate_axis_coupled` (the same plant the main sim and
the FC elevon/Deltal mixer use), with iNav rate P/D injected via params_override.

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

# iNav publish PID defaults (src/main/fc/settings.yaml): (P,I,D,FF)
INAV_MC = {"Roll": (40, 30, 23, 60), "Pitch": (40, 30, 23, 60), "Yaw": (85, 45, 0, 60)}
INAV_FW = {"Roll": (5, 7, 0, 50), "Pitch": (5, 7, 0, 50), "Yaw": (6, 10, 0, 60)}

RP_LIMIT = 500   # iNav pidSumLimit for roll/pitch (MC + FW)
YAW_LIMIT = 400  # iNav pidSumLimit for MC yaw


def inav_kp_kd(P, D, axis, is_fw):
    """iNav rate P,D -> our per-rad Kp,Kd (fraction-of-authority output)."""
    limit = YAW_LIMIT if (axis == "Yaw" and not is_fw) else RP_LIMIT
    return (P / 31.0) * RAD2DEG / limit, (D / 1905.0) * RAD2DEG / limit


def params_with_inav_rate(af, is_fw):
    """Clone our airframe params, overwriting rate P/D with iNav's mapped
    values so the coupled FW sim (or MR sim) runs iNav's rate loop on our
    plant, keeping our outer angle loop."""
    params = load_af_params(af)
    bank = INAV_FW if is_fw else INAV_MC
    p2 = dict(params)
    for name in ["Roll", "Pitch", "Yaw"]:
        P, I, D, FF = bank[name]
        kp, kd = inav_kp_kd(P, D, name, is_fw)
        up = name.upper()
        p2[f"{up}_RATE_KP"] = kp
        p2[f"{up}_RATE_KD"] = kd
    return p2


def _run_mr(af, name):
    """Return (ours_metrics, inav_metrics) using single-axis simulate_axis."""
    params, af_type, fw_mid, is_fw = get_params_for_af(af)
    akp, aki, ail, ma, rkp, rkd, mr = get_axis_params(params, name)
    step = TEST_STEPS[name]

    def run(pids_kp, pids_kd):
        return simulate_axis(
            PIStruct(Kp=akp, Ki=aki, IntLim=ail, Max=ma),
            PIDStruct(Kp=pids_kp, Kd=pids_kd, Max=mr),
            step * DEG_TO_RAD, ma, cat=AirframeCat.MR, axis_name=name,
            af_filename=af, params=params)

    ours = run(rkp, rkd)
    P, I, D, FF = INAV_MC[name]
    ikp, ikd = inav_kp_kd(P, D, name, False)
    inav = run(ikp, ikd)
    return ours, inav, step


def _run_fw(af, name):
    """Return (ours_metrics, inav_metrics) using the coupled 3-axis model."""
    params, af_type, fw_mid, is_fw = get_params_for_af(af)
    step = FW_TEST_STEPS[name]
    step_rads = {a: (FW_TEST_STEPS[a] * DEG_TO_RAD) for a in ["Roll", "Pitch", "Yaw"]}
    ours = simulate_axis_coupled(af, step_axis=name, params_override=params,
                                 step_rads=step_rads)[name]
    inav = simulate_axis_coupled(af, step_axis=name,
                                 params_override=params_with_inav_rate(af, True),
                                 step_rads=step_rads)[name]
    return ours, inav, step


def run_airframe(af, is_fw, axes, label):
    params, af_type, fw_mid, is_fw2 = get_params_for_af(af)
    bank = INAV_FW if is_fw else INAV_MC
    out = []
    out.append("=" * 76)
    out.append(f"  {af}   ({label}) — ours vs iNav defaults, same plant")
    out.append("=" * 76)
    for name in axes:
        if is_fw:
            ours, inav, step = _run_fw(af, name)
        else:
            ours, inav, step = _run_mr(af, name)
        crit = CRITERIA[name] if not is_fw else FW_CRITERIA[name]
        P, I, D, FF = bank[name]
        out.append(f"\n  -- {name}  step {step:.0f} deg --")
        out.append(f"     iNav {('FW' if is_fw else 'MC')} P={P} I={I} D={D} FF={FF}   "
                   f"(rate D=I/FF kept as ours)")
        out.append(f"     [OURS  ] {_l(ours)}  -> {_v(ours, crit)}")
        out.append(f"     [iNAV P] {_l(inav)} -> {_v(inav, crit)}   "
                   f"(iNav rate P/D mapped into ours)")
    return "\n".join(out)


def _l(m):
    return (f"rise={m.rise_time_s:.2f}s  ov={m.overshoot_pct:.1f}%  "
            f"settle={m.settling_time_s:.2f}s  final={m.final_angle*RAD2DEG:.1f}deg  "
            f"nosc={m.n_oscillations}")


def _v(m, crit):
    return "PASS" if all(r.ok for r in step_verdicts(m, crit)) else "FAIL"


def main():
    print(run_airframe("generic/Quad.af", False, ["Roll", "Pitch", "Yaw"], "MR"))
    print()
    print(run_airframe("generic/SkySurfer_Bixler.af", True, ["Roll", "Pitch", "Yaw"], "FW aileron"))
    print()
    print(run_airframe("generic/Shadow.af", True, ["Roll", "Pitch", "Yaw"], "FW elevon/delta"))


if __name__ == "__main__":
    main()
