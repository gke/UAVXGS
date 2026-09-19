#!/usr/bin/env python3
"""Step 2 of the generic-fleet study: derive the Angle parameters for the
generic airframes from their Max Angle / Max Rate limits, using exactly the
two-view derivation formula the GCS Rate-view labels show:

    QAngleKp = RateMax / (2·sin(AngleMax/2))     (rad/s over half-angle)
    KiAngle  = k · QAngleKp                       k = 0.026 MR / 0.05 FW
    I-Limit  = 0.01 · AngleMax
    yaw Ki   = 0.083 · Kp_yaw                     (fleet constant)
    yaw I-Limit = 0.03                            (fleet mode)

Yaw QAngleKp is NOT derived (no yaw angle-max; heading is unbounded) — it stays
as the stored value. Roll/pitch AngleQKp ARE derived from the limits.

Produces a scratch copy of the generic .af files with ONLY the angle-param value
lines rewritten (limits lines, physics metadata and everything else byte-left
alone), then runs the standard generic slider-extremes critique over the copies
via run_tests_for_af (absolute paths — reads AF_TYPE from the file itself).

Usage:
    python3 tests/derive_angle_params.py --critique-out /tmp/critique_step2_derived.txt
    python3 tests/derive_angle_params.py --dry-run
"""

import argparse
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))               # tests/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # src/

from airframes.airframes import parse_af_file, AIRFRAMES_DIR
from airframes.export_all_params import classify
from protocol_enums import ParamIndex

DEG2RAD = 0.017453292519943295
K_RATIO = 0.026          # MR/VTOL roll/pitch
K_FW = 0.05              # FW roll/pitch
K_YAW = 0.083            # fleet-tight yaw ratio
YAW_ILIM = 0.03          # fleet yaw I-limit (independent of Kp)
SCRATCH = "/tmp/gen_derived"

IDX = {name: int(getattr(ParamIndex, name)) for name in (
    "ROLL_ANGLE_Q_KP", "ROLL_ANGLE_Q_KI", "ROLL_ANGLE_Q_INT_LIMIT",
    "PITCH_ANGLE_Q_KP", "PITCH_ANGLE_Q_KI", "PITCH_ANGLE_Q_INT_LIMIT",
    "YAW_ANGLE_Q_KP", "YAW_ANGLE_Q_KI", "YAW_ANGLE_Q_INT_LIMIT",
    "MAX_ROLL_ANGLE", "MAX_PITCH_ANGLE",
    "MAX_ROLL_RATE", "MAX_PITCH_RATE", "MAX_HEADING_RATE",
    "ROLL_RATE_KP", "PITCH_RATE_KP", "YAW_RATE_KP",
    "AF_TYPE",
)}

ANGLE_TAGS = {
    "ROLL":  ("ROLL_ANGLE_Q_KP",  "ROLL_ANGLE_Q_KI",  "ROLL_ANGLE_Q_INT_LIMIT"),
    "PITCH": ("PITCH_ANGLE_Q_KP", "PITCH_ANGLE_Q_KI", "PITCH_ANGLE_Q_INT_LIMIT"),
    "YAW":   ("YAW_ANGLE_Q_KP",   "YAW_ANGLE_Q_KI",   "YAW_ANGLE_Q_INT_LIMIT"),
}

VALUE_ONLY = re.compile(r"^\s*([A-Z0-9_]+)\s*=\s*(-?\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)\s*$")


def derive_axes(vals):
    """Return {axis: {tag_str: derived_value}} from the limit/rate raw values."""
    import math
    cat = classify(vals.get(IDX["AF_TYPE"], 0))
    k = K_FW if cat == "FW" else K_RATIO
    out = {"ROLL": {}, "PITCH": {}, "YAW": {}}

    def _lim(name):
        return vals.get(IDX[name], 0.0)

    for axis, (kp_t, ki_t, il_t) in ANGLE_TAGS.items():
        if axis == "YAW":
            ykp = _lim("YAW_ANGLE_Q_KP")
            out["YAW"][kp_t] = ykp
            out["YAW"][ki_t] = K_YAW * ykp
            out["YAW"][il_t] = YAW_ILIM
            continue
        amax = _lim(f"MAX_{axis}_ANGLE")
        rmax = _lim(f"MAX_{axis}_RATE")
        denom = 2.0 * math.sin(amax * 0.5)
        qkp = (rmax / denom) if denom > 1e-12 else 0.0
        out[axis][kp_t] = qkp
        out[axis][ki_t] = k * qkp
        out[axis][il_t] = 0.01 * amax
    return out


def derive_file(src, dst):
    """Copy src -> dst rewriting only the single-value angle-param lines."""
    _n, vals, _m = parse_af_file(src)
    derived = derive_axes(vals)
    wanted = {t: v for axis in derived.values() for t, v in axis.items()}

    lines = open(src).read().splitlines(keepends=True)
    changed = 0
    new_lines = []
    for ln in lines:
        m = VALUE_ONLY.match(ln)
        if m:
            tag = m.group(1)
            if tag in wanted:
                new_lines.append(f"{tag} = {wanted[tag]:.8g}\n")
                changed += 1
                continue
        new_lines.append(ln)
    with open(dst, "w") as f:
        f.writelines(new_lines)
    return changed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--critique-out", default="/tmp/critique_step2_derived.txt")
    args = ap.parse_args()

    generic = sorted(os.path.join(AIRFRAMES_DIR, "generic", fn)
                     for fn in os.listdir(os.path.join(AIRFRAMES_DIR, "generic"))
                     if fn.endswith(".af"))

    if args.dry_run:
        for src in generic:
            _n, vals, _m = parse_af_file(src)
            d = derive_axes(vals)
            rel = os.path.basename(src)
            print(f"{rel}")
            for axis, tvals in d.items():
                print(f"   {axis}: " + "  ".join(f"{t}={v:.4g}" for t, v in tvals.items()))
        return

    if os.path.isdir(SCRATCH):
        shutil.rmtree(SCRATCH)
    os.makedirs(SCRATCH)

    import tests.test_pid_sim as tps
    all_pass = True
    all_output = []
    for src in generic:
        dst = os.path.join(SCRATCH, os.path.basename(src))
        n = derive_file(src, dst)
        rel = os.path.join("generic", os.path.basename(src))
        sliders = [(0.0, "Conservative (0%)"), (1.0, "Aggressive (100%)")]
        for pct, lbl in sliders:
            try:
                ok, output = tps.run_tests_for_af(dst, slider_pct=pct)
                all_output.extend(output)
                if not ok:
                    all_pass = False
            except Exception as e:
                all_pass = False
                all_output.append(f"Error testing {rel} at {lbl}: {e}")

    with open(args.critique_out, "w") as f:
        for l in all_output:
            f.write(re.sub(r"\x1b\[[0-9;]*m", "", l) + "\n")

    print(f"All generated copies in {SCRATCH}")
    print(f"Critique written to {args.critique_out}  ({'ALL PASS' if all_pass else 'HAS FAILURES'})")


if __name__ == "__main__":
    main()