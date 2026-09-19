#!/usr/bin/env python3
"""
AF-type -> category cross-check — GCS authority vs FC source of truth.

Parses UAVXArmQ/src/params.c and asserts that the GCS single authority
(AIRFRAME_CATEGORY in protocol_enums.py, mirror-consumed by parameter_window,
test_pid_sim, export_all_params, migrate_limits, apply_external_rate_tuning)
exactly matches the FC ClassifyAFType() branch lists and the FC
AirframeCategory / AFs enum values.

Single cross-repo guard for the classification mirrors: if the FC adds an
airframe or changes a category assignment and protocol_enums.py is not
updated in lockstep, this test fails. Prior drift: test_pid_sim's local
AF_CATEGORY omitted eAileronVTailAF and silently simmed it as a multirotor.

Usage:
  python3 src/tests/test_af_category.py [path/to/UAVXArmQ]
"""

import os
import re
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(SRC_DIR))

from protocol_enums import (  # noqa: E402
    AirframeType,
    AirframeCategory,
    AIRFRAME_CATEGORY,
)


def parse_fc_classify(params_c: Path):
    """Return (fw, vtol, land) AF lists parsed from FC ClassifyAFType()."""
    src = params_c.read_text()
    start = src.find("void ClassifyAFType")
    assert start != -1, "ClassifyAFType() not found in %s" % params_c
    brace = src.find("{", start)
    assert brace != -1, "no body brace in ClassifyAFType()" % params_c
    # Brace-depth scan to the function's closing brace (the body is nested
    # inside `{ ... }`; the close is the '}' that returns depth to the level
    # we entered with the opening brace).
    depth = 0
    i = brace
    while i < len(src):
        if src[i] == "{":
            depth += 1
        elif src[i] == "}":
            depth -= 1
            if depth == 0:
                break
        i += 1
    body = src[brace + 1:i]

    # Each category arm is `if (<cond over AF names>) pAFTypeCategory = eCatX;`.
    # Anchor on the assignment line so the lazy condition capture stops at the
    # arm's own closing paren (a naive `if(...)` at each member finds the first
    # if for every arm and under-captures — that is how the mirror drifted).
    arms = {}
    for m in re.finditer(
            r"if\s*\((.*?)\)\s*\n\s*pAFTypeCategory\s*=\s*(eCat\w+)\s*;",
            body, re.S):
        names = set(re.findall(r"(e[A-Za-z0-9]+AF)", m.group(1)))
        arms.setdefault(m.group(2), set()).update(names)

    for arm in ("eCatFw", "eCatVtol", "eCatLand"):
        assert arm in arms, "no classifiable %s branch found (would silently fall to MR)" % arm
    return arms["eCatFw"], arms["eCatVtol"], arms["eCatLand"]


def parse_fc_enums(params_h: Path):
    """Return (category_enum, af_enum) value maps from FC params.h."""
    src = params_h.read_text()
    cat = {}
    m = re.search(
        r"typedef enum\s*\{(.*?)\}\s*AirframeCategory;", src, re.S)
    assert m, "AirframeCategory enum not found in %s" % params_h
    for line in m.group(1).splitlines():
        line = line.split("//")[0].strip()
        if not line:
            continue
        mm = re.match(r"(\w+)\s*=\s*(\d+)", line)
        if mm:
            cat[mm.group(1)] = int(mm.group(2))

    af = {}
    m = re.search(r"enum AFs\s*\{(.*?)\};", src, re.S)
    assert m, "AFs enum not found in %s" % params_h
    for line in m.group(1).splitlines():
        line = line.split("//")[0].strip().rstrip(",")
        if not line:
            continue
        mm = re.match(r"(\w+)\s*=\s*(\d+)", line)
        if mm:
            af[mm.group(1)] = int(mm.group(2))
        elif line:
            af[line.rstrip(",")] = len(af)
    return cat, af


def main():
    fc_src = os.environ.get("UAVXARMQ_SRC")
    if fc_src:
        root = Path(fc_src)
    else:
        root = Path(__file__).resolve().parents[4] / "UAVXArmQ"
    params_c = root / "src" / "params.c"
    params_h = root / "src" / "params.h"
    assert params_c.exists(), "FC params.c not found at %s" % params_c
    assert params_h.exists(), "FC params.h not found at %s" % params_h

    fw, vtol, land = parse_fc_classify(params_c)
    cat_enum, af_enum = parse_fc_enums(params_h)

    # 1) Enum values must be identical (protocol_enums AirframeType /
    #    AirframeCategory mirror params.h).
    for py_af, val in {
        getattr(AirframeType, key): v for key, v in af_enum.items()
    }.items():
        assert py_af == val, "AirframeType.%s = %d, FC AFs = %d" % (py_af.name, py_af, val)
    for py_cat, val in {
        getattr(AirframeCategory, key): v for key, v in cat_enum.items()
    }.items():
        assert py_cat == val, "AirframeCategory.%s = %d, FC = %d" % (py_cat.name, py_cat, val)

    # 2) Category membership must match ClassifyAFType() exactly.
    def names(members):
        return {getattr(AirframeType, n) for n in members}
    g_fw = {t for t, c in AIRFRAME_CATEGORY.items() if c == AirframeCategory.eCatFw}
    g_vtol = {t for t, c in AIRFRAME_CATEGORY.items() if c == AirframeCategory.eCatVtol}
    g_land = {t for t, c in AIRFRAME_CATEGORY.items() if c == AirframeCategory.eCatLand}
    g_mr = {t for t in AirframeType if t not in AIRFRAME_CATEGORY}

    assert g_fw == names(fw), "FW mismatch: GCS %s vs FC %s" % (
        sorted(t.name for t in g_fw), sorted(fw))
    assert g_vtol == names(vtol), "VTOL mismatch: GCS %s vs FC %s" % (
        sorted(t.name for t in g_vtol), sorted(vtol))
    assert g_land == names(land), "LAND mismatch: GCS %s vs FC %s" % (
        sorted(t.name for t in g_land), sorted(land))

    # 3) eAFUnknown + eInstrumentation must fall to MR on the FC (else
    #    branch) and on the GCS authority (default eCatMr).
    for amb in (AirframeType.eAFUnknown, AirframeType.eInstrumentation):
        assert amb not in AIRFRAME_CATEGORY, \
            "%s must be implicit MR (FC else branch), not enumerated" % amb.name

    # 4) Every FC AF (bar eAFUnknown) must appear in the enum mirror.
    assert len(af_enum) == len(list(AirframeType)) == 27, \
        "AF enum size drift: FC %d, GCS %d" % (len(af_enum), len(list(AirframeType)))

    print("OK: FC ClassifyAFType() == GCS AIRFRAME_CATEGORY (FW %d, VTOL %d, LAND %d, MR %d)"
          % (len(g_fw), len(g_vtol), len(g_land), len(g_mr)))


if __name__ == "__main__":
    main()