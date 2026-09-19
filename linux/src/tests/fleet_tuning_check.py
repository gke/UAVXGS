#!/usr/bin/env python3
"""Fleet-wide tuning check (critique) across all .af files EXCEPT original/.

Only the `original/` directory is format-check-only (never tuned), per the
project convention. Every other parseable .af (user/, generic/, proposed/,
backup/, backup_angleunits/) is critiqued via run_tests_for_af. Format/content
audit flags are reported as annotations but do NOT exclude a parseable file
from tuning. Truly unparseable files are reported as ERROR.

Usage:
    python3 tests/fleet_tuning_check.py [--csv out.csv]
"""

import csv
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))       # tests/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # src/

from airframes.audit_af_fleet import audit_file, AIRFRAMES_DIR
import tests.test_pid_sim as tps

ANSI = re.compile(r"\x1b\[[0-9;]*m")
LEVELS = {"PASS": 0, "FAIL": 1, "ERROR": 2, "SKIP": 3, "FORMAT-ISSUE": 4}


def collect_af_files():
    files = []
    for dirpath, _dirs, fns in os.walk(AIRFRAMES_DIR):
        for fn in sorted(fns):
            if fn.endswith(".af"):
                files.append(os.path.join(dirpath, fn))
    return sorted(files)


def tune(path, rel):
    """Run the critique for a parseable file; return (status, note, detail).

    Generic base templates run at BOTH slider extremes (0% conservative and
    100% aggressive) mirroring test_pid_sim.main(); every other file runs at
    its as-saved values.
    """
    audit = audit_file(path)
    flags = []
    if not audit["ok"]:
        flags.append(" ".join(audit["unknown_keys"] or []))
        flags.append(f"missing {audit['n_missing']} tags")

    def _crit(slider=None):
        ok, output = tps.run_tests_for_af(rel, slider_pct=slider)
        ansi_free = [ANSI.sub("", l) for l in output]
        fail_lines = [l.strip() for l in ansi_free if "FAIL" in l or "!" in l]
        return ok, ansi_free, fail_lines

    is_generic = rel.startswith("generic/")
    try:
        if is_generic:
            ok0, _, f0 = _crit(0.0)
            ok1, out1, f1 = _crit(1.0)
            ok = ok0 and ok1
            note = f"slider extremes 0%/100% {'ok' if ok else 'FAILED'}"
            fails = [f for pair in ((ok0, f0), (ok1, f1)) for f in pair[1]]
            detail = out1 if ok else fails
        else:
            ok, ansi_free, fail_lines = _crit()
            note = f"critique {'ok' if ok else 'FAILED'}"
            detail = ansi_free if ok else fail_lines
        if flags:
            note += f"  [audit: {', '.join(f for f in flags if f)}]"
        return ("PASS" if ok else "FAIL"), note, detail
    except Exception as e:
        return "ERROR", f"run_tests_for_af raised: {e}", []


def main():
    csv_out = None
    if len(sys.argv) > 1 and sys.argv[1] == "--csv":
        csv_out = sys.argv[2]
    files = collect_af_files()
    rows = []
    for path in files:
        rel = os.path.relpath(path, AIRFRAMES_DIR)
        if rel.startswith("original/"):
            audit = audit_file(path)
            if audit["ok"]:
                status, note, detail = "SKIP", "original — format-check only (not tuned)", []
            else:
                status, note, detail = "FORMAT-ISSUE", "original — format issues (not tuned)", audit["issues"]
            rows.append((rel, status, note, detail))
            continue
        # Non-original: attempt tuning; exclude only truly unparseable files
        try:
            _name, _vals, _meta = __import__("airframes.airframes", fromlist=["parse_af_file"]).parse_af_file(path)
        except Exception as e:
            rows.append((rel, "ERROR", f"unparseable: {e}", []))
            continue
        rows.append((rel,) + tune(path, rel)[:])

    rows.sort(key=lambda r: (LEVELS.get(r[1], 9), r[0]))

    print("=" * 78)
    print("FLEET TUNING CHECK (original/ format-only; all other parseable files critiqued)")
    print("=" * 78)
    for rel, status, note, detail in rows:
        print(f"  {status:13s} {rel}   {note}")
        if status == "FORMAT-ISSUE" or status == "ERROR":
            for line in detail:
                print(f"        {line}")
        else:
            for line in detail[:8]:
                print(f"        {line}")
        print()

    if csv_out:
        with open(csv_out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["file", "status", "note", "detail"])
            for rel, status, note, detail in rows:
                w.writerow([rel, status, note, "\n".join(detail)])
        print(f"Wrote {csv_out}")

    n_pass = sum(1 for r in rows if r[1] == "PASS")
    n_fail = sum(1 for r in rows if r[1] == "FAIL")
    n_err = sum(1 for r in rows if r[1] == "ERROR")
    n_skip = sum(1 for r in rows if r[1] in ("SKIP", "FORMAT-ISSUE"))
    print(f"SUMMARY: PASS={n_pass} FAIL={n_fail} ERROR={n_err} SKIP/format-only={n_skip}  of {len(rows)}")
    return 1 if (n_fail + n_err) else 0


if __name__ == "__main__":
    sys.exit(main())