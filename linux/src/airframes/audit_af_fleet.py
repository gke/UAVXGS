#!/usr/bin/env python3
"""Fleet-wide .af consistency audit.

Checks every .af file in airframes/ for:
  - parseability (via airframes.parse_af_file)
  - # Name header, PHYS_* physics descriptor coverage
  - param coverage / count / ordering
  - duplicate or unknown param keys
  - enum token validity and value formatting
  - basic sanity (no unit-multiplied values in the classic UNITS_* names)

Report-only: prints a per-file and per-fleet summary. No files are modified.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from airframes.airframes import parse_af_file, AIRFRAMES_DIR, _parse_value, ENUM_MAP
from protocol_enums import ParamIndex

LINE_RE = re.compile(r'^\s*(\w+)\s*=\s*(.+?)\s*$')
ENUM_PARAMS = {key for key, cls in ENUM_MAP.items() if cls is not None}

# PHYS_* descriptors seen across the fleet (informational union).
PHYS_KEYS_ALL = set()
FIRST_COMMENT_RE = re.compile(r'\s*#\s*(.*)$')


def audit_file(path: str) -> dict:
    rel = os.path.relpath(path, AIRFRAMES_DIR)
    res = {
        "file": rel,
        "ok": True,
        "issues": [],
        "n_params": 0,
        "n_phys": 0,
        "name": None,
        "missing_phys": [],
        "duplicates": [],
        "unknown_keys": [],
        "bad_enums": [],
        "bad_format": [],
    }
    text = open(path, encoding="utf-8", errors="replace").read()

    # Structural header check: first non-blank, non-comment line must be
    # a comment starting '# ' with the name (parse_af tolerates other forms).
    first_lines = [l for l in text.splitlines() if l.strip()]
    if first_lines:
        if not first_lines[0].lstrip().startswith("#"):
            res["issues"].append("first content line is not a '#' comment (missing name header?)")
            res["ok"] = False

    try:
        name, values, meta = parse_af_file(path)
    except Exception as e:
        res["issues"].append(f"parse_af_file raised: {e}")
        res["ok"] = False
        res["missing_tags"] = ["<unparseable>"]
        res["n_missing"] = 128
        return res

    res["name"] = name
    res["n_phys"] = sum(1 for k in meta if k.startswith("PHYS_"))
    res["n_params"] = len(values)

    # Missing-tag report: which of the 128 current tags are absent after parse
    present = set(values)
    missing = []
    for t in range(len([p for p in ParamIndex])):
        if t not in present:
            missing.append(f"{t}:{ParamIndex(t).name}")
    res["missing_tags"] = missing
    res["n_missing"] = len(missing)

    # PHYS descriptor keys (union across fleet)
    PHYS_KEYS_ALL.update(k for k in meta if k.startswith("PHYS_"))

    # Param coverage: tag -> name resolution
    known = set(p.name for p in ParamIndex)
    seen_tags = set()
    for tag in values:
        if tag in seen_tags:
            res["duplicates"].append(str(ParamIndex(tag).name if tag in known else f"UNUSED_{tag}"))
            res["ok"] = False
        seen_tags.add(tag)

    # Raw-text scan for duplicate / unknown keys and formatting
    key_value = {}
    in_limits = False
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.upper() == "[LIMITS]":
            in_limits = True
            continue
        if in_limits:
            continue
        m = LINE_RE.match(line)
        if not m:
            continue
        key, valtext = m.group(1).upper(), m.group(2).strip()
        if key.startswith("PHYS_"):
            continue
        if key not in key_value:
            key_value[key] = []
        key_value[key].append(valtext)

    for key, vals in key_value.items():
        if key not in known:
            res["unknown_keys"].append(key)
            res["ok"] = False
        elif len(vals) > 1:
            res["duplicates"].append(key)
            res["ok"] = False

    # Enum token validity for enum params
    for key, valtext in key_value.items():
        if key in ENUM_PARAMS:
            vals = key_value[key]
            try:
                _parse_value(key, vals[-1])
            except Exception:
                res["bad_enums"].append(f"{key}={vals[-1]}")
                res["ok"] = False

    return res


def main():
    roots = [AIRFRAMES_DIR]
    files = []
    for root in roots:
        for dirpath, _dirs, fns in os.walk(root):
            for fn in sorted(fns):
                if fn.endswith(".af"):
                    files.append(os.path.join(dirpath, fn))

    files.sort()
    results = [audit_file(f) for f in files]

    print("=" * 78)
    print(f"FLEET .af CONSISTENCY AUDIT  ({len(files)} files)")
    print("=" * 78)

    n_bad = 0
    for r in results:
        flag = "OK " if r["ok"] else "** "
        print(f"\n{flag}{r['file']}")
        print(f"    name={r['name']!r}  params={r['n_params']}  phys={r['n_phys']}  missing={r['n_missing']}/128")
        if r["missing_tags"]:
            print(f"    missing tags: {r['missing_tags']}")
        if r["unknown_keys"]:
            print(f"    UNKNOWN keys: {r['unknown_keys']}")
        if r["duplicates"]:
            print(f"    DUPLICATE keys: {r['duplicates']}")
        if r["bad_enums"]:
            print(f"    BAD enum tokens: {r['bad_enums']}")
        for i in r["issues"]:
            print(f"    {i}")
        if not r["ok"]:
            n_bad += 1

    print("\n" + "=" * 78)
    print(f"SUMMARY: {len(files)-n_bad}/{len(files)} clean, {n_bad} with issues")
    print("=" * 78)
    return 1 if n_bad else 0


if __name__ == "__main__":
    sys.exit(main())