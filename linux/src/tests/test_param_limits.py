#!/usr/bin/env python3
"""
ParamClass bounds cross-check — GCS vs FC source of truth.

Parses UAVXArmQ/src/params.c ParamTable and asserts that the GCS
PARAM_LIMITS (derived from PARAM_CLASS_OF + PARAM_EXPLICIT_LIMITS +
PARAM_CLASS_BOUNDS in parameters.py) exactly matches the raw (lo, hi)
the FC will hard-clamp to.

This is the single cross-repo guard for the class-based bounds scheme:
if the FC ParamTable changes bounds and the GCS tables are not updated in
lockstep, this test fails.

Usage:
  python3 src/tests/test_param_limits.py [path/to/UAVXArmQ]
"""

import os
import re
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(SRC_DIR))

import parameters as P  # noqa: E402

# Enum constants referenced by ParamTable classExplicit entries (params.h,
# telem.h, auto.h, main.h, sensors/mpu6xxx.h, outputs.h). Frozen FC values.
ENUM_VALUES = {
    # ArmingModes
    "eTxArming": 0, "eSwitchArming": 1,
    # RFs
    "eMaxSonarcm": 0, "eNoRF": 5,
    # IMUFilterTypes
    "eLP2Filt": 0, "IMU_FILT_TYPE_MAX": 5,
    # inflightLogs (telem.h)
    "eLogUAVX": 0, "eLogYaw": 4,
    # RxTypes
    "eCPPMRx": 0, "eUnknownRx": 5,
    # ESC
    "eESCPWM": 0, "eMotorsOff": 2,
    # AFs
    "eAFUnknown": 26,
    # TelemetryTypes
    "eUAVXDJTTelemetry": 0, "eU8Telemetry": 7,
    # sensors/mpu6xxx.h
    "GYRO_LPF_SEL_MAX": 7, "ACC_LPF_SEL_MAX": 5,
    # ASSensorTypes
    "eMS4525D0I2C": 0, "eNoAS": 4,
    # MotorStopActions (auto.h)
    "eLandNoStop": 0, "eLandDescentRateAndAccU": 4,
    # reset_causes (main.h)
    "eUnknownReset": 0, "eBrownoutReset": 7,
    # Config bits
    "CONFIG_BITS_MAX": 255,
    # Battery (params.h)
    "BATTERY_CAPACITY_MAH_MIN": 1500, "BATTERY_CAPACITY_MAH_MAX": 10000,
}

# Comparison tolerance: the FC table stores bounds as float32 literals (e.g.
# 1.047198f), the GCS uses double precision. Allow float32 rounding slack.
BOUND_TOL = 1e-3


def resolve_number(token):
    """Parse an FC table bound token (float literal or enum name) to float."""
    t = token.strip()
    try:
        return float(t)
    except ValueError:
        pass
    if t.endswith("f"):
        try:
            return float(t[:-1])
        except ValueError:
            pass
    if t in ENUM_VALUES:
        return float(ENUM_VALUES[t])
    raise ValueError("unresolved bound token: %r" % token)


def parse_param_table(params_c_path):
    """Return {tag: (cls, lo, hi)} parsed from the FC ParamTable block."""
    src = params_c_path.read_text()
    m = re.search(r"const ParamMetaEntry ParamTable\[MAX_PARAMETERS\]\s*=\s*\{(.*?)\};",
                  src, re.S)
    assert m, "ParamTable block not found in %s" % params_c_path
    body = m.group(1)

    entries = {}
    for line in body.splitlines():
        line = line.strip()
        if not line.startswith(("FLOAT(", "U8(")):
            continue
        line = line.split("//")[0].strip()
        head, rest = line.split("(", 1)
        macro = head.strip()
        args = [a.strip() for a in rest.split(")", 1)[0].split(",")]
        if macro == "FLOAT":
            assert len(args) == 7, "FLOAT arg count: %s" % args
            cls, lo, hi = args[1], resolve_number(args[3]), resolve_number(args[4])
        else:
            assert len(args) == 6, "U8 arg count: %s" % args
            cls, lo, hi = args[1], resolve_number(args[2]), resolve_number(args[3])
        # tag = table index (order of appearance)
        tag = len(entries)
        entries[tag] = (cls, lo, hi)
    assert len(entries) == 128, "parsed %d entries, expected 128" % len(entries)
    return entries


def main():
    fc_src = os.environ.get("UAVXARMQ_SRC")
    if fc_src:
        params_c = Path(fc_src) / "params.c"
    else:
        code_root = Path(__file__).resolve().parents[4]
        params_c = code_root / "UAVXArmQ" / "src" / "params.c"
    assert params_c.exists(), "FC params.c not found at %s" % params_c

    fc = parse_param_table(params_c)

    # 1) Every class-based tag must equal its class ceiling on both sides.
    for tag, (cls, lo, hi) in fc.items():
        assert cls in P.PARAM_CLASS_BOUNDS, "FC cls %r (tag %d) missing from GCS" % (cls, tag)
        assert P.PARAM_CLASS_OF.get(tag) == cls, \
            "PARAM_CLASS_OF[%d] = %r but FC table has %r" % (tag, P.PARAM_CLASS_OF.get(tag), cls)
        g_lo, g_hi = P.PARAM_LIMITS[tag]
        assert abs(g_lo - lo) < BOUND_TOL and abs(g_hi - hi) < BOUND_TOL, \
            "tag %d: GCS %s vs FC (%s, %s)" % (tag, (g_lo, g_hi), lo, hi)

    # 2) Explicit tags must carry per-entry bounds identical to the FC table.
    for tag, (cls, lo, hi) in fc.items():
        if cls == "eClassExplicit":
            assert tag in P.PARAM_EXPLICIT_LIMITS, \
                "eClassExplicit tag %d missing from PARAM_EXPLICIT_LIMITS" % tag

    # 3) Class-bound tables must be in lockstep with FC's ParamClass[] table.
    pc = re.search(r"static const ParamClassBounds_t ParamClass\[NUM_PARAM_CLASSES\]\s*=\s*\{(.*?)\};",
                   params_c.read_text(), re.S)
    assert pc, "ParamClass[] block not found"
    for line in pc.group(1).splitlines():
        m2 = re.search(r"\[(eClass\w+)\]\s*=\s*\{\s*([^,]+),\s*([^}]+)\}", line)
        if not m2:
            continue
        cls_name, lo_tok, hi_tok = m2.group(1), m2.group(2), m2.group(3)
        lo, hi = resolve_number(lo_tok), resolve_number(hi_tok)
        if cls_name == "eClassExplicit":
            continue
        g_lo, g_hi = P.PARAM_CLASS_BOUNDS[cls_name]
        assert abs(g_lo - lo) < BOUND_TOL and abs(g_hi - hi) < BOUND_TOL, \
            "class %s: GCS %s vs FC (%s, %s)" % (cls_name, (g_lo, g_hi), lo, hi)

    print("OK: all 128 tags + %d classes match FC params.c" % len(P.PARAM_CLASS_BOUNDS))


# .af files store values in FC-native units with %.6g formatting (6 sig figs),
# so a stored value may sit a few float32/format eps away from a class ceiling.
AF_TOL = 1e-4


def check_airframe_file(path):
    """Validate one .af: every [LIMITS] entry within its class bounds, and
    every stored value within its class bounds (float32/format tolerant)."""
    from airframes.airframes import parse_af_file
    _name, vals, meta = parse_af_file(str(path))
    lim = meta.get("LIMITS", {})
    problems = []
    for tag, raw in sorted(vals.items()):
        lo, hi = P.PARAM_LIMITS.get(tag, (0.0, 255.0))
        if raw < lo - AF_TOL or raw > hi + AF_TOL:
            problems.append("value out of range: tag %d raw=%s limits=(%s,%s)" % (tag, raw, lo, hi))
        if P.PARAM_CLASS_OF.get(tag) == "eClassPct" and raw > 1.0 + AF_TOL:
            problems.append("PCT param not normalized 0-1: tag %d raw=%s" % (tag, raw))
        if tag in lim:
            l, h = lim[tag]
            if not (l >= lo - AF_TOL and h <= hi + AF_TOL and l < h):
                problems.append("bad LIMITS for tag %d: (%s,%s) class=(%s,%s)" % (tag, l, h, lo, hi))
    assert not problems, "%s:\n  %s" % (path.name, "\n  ".join(problems))


def main_af():
    """Cross-check every shipped .af file's [LIMITS] blocks + values."""
    from airframes.airframes import AIRFRAMES_DIR
    from pathlib import Path
    import glob as _glob
    afs = sorted(_glob.glob(str(Path(AIRFRAMES_DIR) / "generic" / "*.af"))
                 + _glob.glob(str(Path(AIRFRAMES_DIR) / "user" / "*.af")))
    assert afs, "no .af files found"
    for path in afs:
        check_airframe_file(Path(path))
    print("OK: %d .af files validated ([LIMITS] within class bounds, values in range)" % len(afs))


if __name__ == "__main__":
    main()
    main_af()
