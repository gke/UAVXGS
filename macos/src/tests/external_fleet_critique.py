#!/usr/bin/env python3
"""Step 3 of the generic-fleet study: what does the critique say when we run
field-proven EXTERNAL rate-loop params on our plant, matched per generic frame
class?

For each generic .af we create a scratch variant in which ONLY the rate P/D
params are replaced by the mapped external values (angle loop, limits line,
physics and everything else untouched):

    iNav (multicopter MC + fixed-wing FW) defaults from compare_inav.py
    ArduPilot (Copter for MR + Plane for FW) from compare_ardupilot.py

Mapping is identical to the two cross-reference studies so results are directly
comparable:
    iNav:    Kp = (P/31)*RAD2DEG/lim, Kd = (D/1905)*RAD2DEG/lim
             MC roll/pitch P=40 D=23, yaw P=85 D=0; lim=500/500/400
             FW  roll/pitch P=5  D=0,  yaw P=6  D=0; lim=500
    ArduPilot Plane: Kp = kp_ff/4500*RAD2DEG, Kd = D/4500*RAD2DEG
             roll kp_ff=0.345 D=0.08 ; pitch kp_ff=0.385 D=0.04 ; yaw NOT set
    ArduPilot Copter: rate P/D copied 1:1 (rad/s, our convention), yaw P=0.18
             roll/pitch P=0.135  D=0.0036 ;  yaw P=0.18 D=0.0

Then run the full as-saved critique (run_tests_for_af, no slider) and compare
external vs committed PASS/FAIL.

External banks are accepted as command-line args:
    --bank inav      use iNav defaults (MC + FW)
    --bank ardupilot use ArduPilot (Copter MR / Plane FW)
    --bank both      both (default)

Usage:
    python3 tests/external_fleet_critique.py --bank both
       --critique-out/--csv-out override output paths
"""

import argparse
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))               # tests/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # src/

import math

from airframes.airframes import parse_af_file, AIRFRAMES_DIR
from airframes.export_all_params import classify
from protocol_enums import ParamIndex

RAD2DEG = 180.0 / math.pi

IDX = {name: int(getattr(ParamIndex, name)) for name in (
    "ROLL_RATE_KP", "ROLL_RATE_KD", "PITCH_RATE_KP", "PITCH_RATE_KD",
    "YAW_RATE_KP", "YAW_RATE_KD", "AF_TYPE",
)}
RATE_TAGS = {
    "Roll":  ("ROLL_RATE_KP", "ROLL_RATE_KD"),
    "Pitch": ("PITCH_RATE_KP", "PITCH_RATE_KD"),
    "Yaw":   ("YAW_RATE_KP", "YAW_RATE_KD"),
}

# --- external banks ---------------------------------------------------------
# iNav publish PID defaults (src/main/fc/settings.yaml): (P,I,D,FF)
INAV_MC = {"Roll": (40, 30, 23, 60), "Pitch": (40, 30, 23, 60), "Yaw": (85, 45, 0, 60)}
INAV_FW = {"Roll": (5, 7, 0, 50), "Pitch": (5, 7, 0, 50), "Yaw": (6, 10, 0, 60)}
RP_LIMIT = 500   # iNav pidSumLimit roll/pitch (MC + FW)
YAW_LIMIT = 400  # iNav pidSumLimit MC yaw

# ArduPilot Copter rate P/D, rad/s (our exact convention)
COPTER = {"Roll": (0.135, 0.0036), "Pitch": (0.135, 0.0036), "Yaw": (0.18, 0.0)}
# ArduPilot Plane rate kp_ff & D (per deg/s -> centi-deg servo travel)
PLANE = {"Roll": (0.345, 0.08), "Pitch": (0.385, 0.04)}   # no FW rate-yaw
SERVO_FULL = 4500.0


def inav_rates(axis, is_fw):
    bank = INAV_FW if is_fw else INAV_MC
    P, I, D, FF = bank[axis]
    lim = YAW_LIMIT if (axis == "Yaw" and not is_fw) else RP_LIMIT
    return (P / 31.0) * RAD2DEG / lim, (D / 1905.0) * RAD2DEG / lim


def copter_rates(axis):
    return COPTER[axis]


def plane_rates(axis):
    if axis == "Yaw":
        return None  # ArduPlane has NO default rate-yaw loop
    kp_ff, D = PLANE[axis]
    return kp_ff / SERVO_FULL * RAD2DEG, D / SERVO_FULL * RAD2DEG


def external_rates(bank, axis, is_fw):
    if bank == "inav":
        return inav_rates(axis, is_fw)
    if bank == "ardupilot":
        return copter_rates(axis) if not is_fw else plane_rates(axis)
    raise ValueError(bank)


SCRATCH = "/tmp/gen_external"
VALUE_ONLY = re.compile(r"^\s*([A-Z0-9_]+)\s*=\s*(-?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)\s*$")


def derive_file(src, dst, bank):
    _n, vals, _m = parse_af_file(src)
    cat = classify(vals.get(IDX["AF_TYPE"], 0))
    is_fw = cat == "FW"
    wanted = {}
    for axis, (kp_t, kd_t) in RATE_TAGS.items():
        kr = external_rates(bank, axis, is_fw)
        if kr is None:
            continue  # e.g. ArduPilot FW yaw — leave our rate-yaw untouched
        wanted[kp_t], wanted[kd_t] = kr

    new_lines = []
    changed = 0
    for ln in open(src).read().splitlines(keepends=True):
        m = VALUE_ONLY.match(ln)
        if m and m.group(1) in wanted:
            v = wanted[m.group(1)]
            if isinstance(v, float):
                new_lines.append(f"{m.group(1)} = {v:.8g}\n")
                changed += 1
                continue
        new_lines.append(ln)
    with open(dst, "w") as f:
        f.writelines(new_lines)
    return changed


def per_file(path):
    """Return (passes, fails_repr) lists from a full as-saved critique output."""
    import tests.test_pid_sim as tps
    ok, out = tps.run_tests_for_af(path, slider_pct=None)
    clean = [re.sub(r"\x1b\[[0-9;]*m", "", l) for l in out]
    fails = [l.strip() for l in clean if "[FAIL]" in l]
    return ok, fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bank", default="both", choices=["inav", "ardupilot", "both"])
    ap.add_argument("--critique-out", default="/tmp/critique_step3_external.txt")
    args = ap.parse_args()

    banks = ["inav", "ardupilot"] if args.bank == "both" else [args.bank]

    if os.path.isdir(SCRATCH):
        shutil.rmtree(SCRATCH)
    os.makedirs(SCRATCH, exist_ok=True)

    results = {}
    for bank in banks:
        results[bank] = {}
        for src in sorted(os.path.join(AIRFRAMES_DIR, "generic", fn)
                          for fn in os.listdir(os.path.join(AIRFRAMES_DIR, "generic"))
                          if fn.endswith(".af")):
            name = os.path.basename(src)
            dst = os.path.join(SCRATCH, f"{bank}_{name}")
            n = derive_file(src, dst, bank)
            if n == 0:
                results[bank][name] = ("SKIP(none)", [])
                continue
            ok, fails = per_file(dst)
            from airframes.airframes import parse_af_file as _paf
            _nn, vals, _mm = _paf(src)
            cat = classify(vals.get(IDX["AF_TYPE"], 0))
            results[bank][name] = (f"{cat} [{n} rate tags]", fails)

    # Also re-run our committed baseline for side-by-side (from canonical)
    import tests.test_pid_sim as tps
    baseline = {}
    for src in sorted(os.path.join(AIRFRAMES_DIR, "generic", fn)
                      for fn in os.listdir(os.path.join(AIRFRAMES_DIR, "generic"))
                      if fn.endswith(".af")):
        name = os.path.basename(src)
        ok, fails = per_file(src)
        baseline[name] = fails

    lines = []
    lines.append(f"{'frame':24s} {'cat':6s} {'committed':10s} {'iNav':10s} {'ArduPilot':10s}")
    for name in baseline:
        def tag(fails):
            return "FAIL" if fails else "PASS"
        row = f"{name:24s} "
        # category from committed parse
        _nn, vals, _mm = parse_af_file(os.path.join(AIRFRAMES_DIR, "generic", name))
        row += f"{classify(vals.get(IDX['AF_TYPE'],0)):6s} "
        row += f"{tag(baseline[name]):10s} "
        for bank in ("inav", "ardupilot"):
            if bank in results and name in results[bank]:
                _st, fails = results[bank][name]
                row += f"{tag(fails):10s} "
            else:
                row += f"{'n/a':10s} "
        lines.append(row.rstrip())
    lines.append("")
    lines.append("Detail (first 3 FAIL lines per bank/file against external focus):")
    for bank in ("inav", "ardupilot"):
        if bank not in results:
            continue
        for name in baseline:
            st, fails = results[bank][name]
            if fails:
                lines.append(f"  [{bank}] {name} ({st})")
                for f in fails[:3]:
                    lines.append(f"      {f}")
                if len(fails) > 3:
                    lines.append(f"      ... (+{len(fails)-3} more)")

    out = "\n".join(lines)
    with open(args.critique_out, "w") as f:
        f.write(out + "\n")
    print(out)


if __name__ == "__main__":
    main()