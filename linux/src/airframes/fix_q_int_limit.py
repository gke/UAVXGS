"""One-time migration: purge legacy-scaled ANGLE_Q_INT_LIMIT values from .af files.

The unified-float migration stored the legacy uint8 value (10/15/20/28) as the raw
float, i.e. 10 rad/s of integral authority — 3800x over budget and a cascade windup
bomb (see wiki/docs/Cascade_Integrity_Checks.md §4). This script rewrites any
standalone value >= 1.0 (and its [LIMITS] range) for ROLL/PITCH/YAW_ANGLE_Q_INT_LIMIT
to the current FC/GCS defaults (0.01 roll/pitch, 0.03 yaw) and a sane tuning band.

Surgical: only the offending lines are rewritten; everything else is byte-preserved.
Idempotent: runs over all copies (canonical, kit installers, local mirror).

Usage:
  python3 airframes/fix_q_int_limit.py [--roots DIR,...] [--dry-run]
"""
import argparse
import glob
import os
import re
import sys

DEFAULTS = {
    'ROLL_ANGLE_Q_INT_LIMIT': 0.01,
    'PITCH_ANGLE_Q_INT_LIMIT': 0.01,
    'YAW_ANGLE_Q_INT_LIMIT': 0.03,
}
BANDS = {
    'ROLL_ANGLE_Q_INT_LIMIT': (0.005, 0.02),
    'PITCH_ANGLE_Q_INT_LIMIT': (0.005, 0.02),
    'YAW_ANGLE_Q_INT_LIMIT': (0.015, 0.06),
}

VALUE_RE = re.compile(
    r'^(?P<key>(?:ROLL|PITCH|YAW)_ANGLE_Q_INT_LIMIT)\s*=\s*(?P<val>\d+(?:\.\d*)?)\s*$')
RANGE_RE = re.compile(
    r'^(?P<key>(?:ROLL|PITCH|YAW)_ANGLE_Q_INT_LIMIT)\s*=\s*'
    r'(?P<lo>\d+(?:\.\d*)?)\s*,\s*(?P<hi>\d+(?:\.\d*)?)\s*$')


def fix_line(line):
    """Return (new_line or None) if the line holds a legacy-wrong value/range."""
    m = VALUE_RE.match(line)
    if m:
        val = float(m.group('val'))
        if val >= 1.0:
            return f"{m.group('key')} = {DEFAULTS[m.group('key')]:.8g}"
        return None
    m = RANGE_RE.match(line)
    if m:
        lo = float(m.group('lo'))
        hi = float(m.group('hi'))
        if lo >= 1.0 or hi >= 1.0:
            lo, hi = BANDS[m.group('key')]
            return f"{m.group('key')} = {lo:.8g}, {hi:.8g}"
    return None


def default_roots():
    here = os.path.dirname(os.path.abspath(__file__))
    airframes = os.path.dirname(here)
    src = os.path.dirname(airframes)
    uavxgs = os.path.dirname(src)
    extra = []
    for d in ('linux', 'macos', 'windows'):       # kit installers
        extra.append(os.path.join(uavxgs, d, 'src', 'airframes'))
    extra.append(os.path.abspath(os.path.join(    # local mirror repo
        uavxgs, '..', 'gitUAVXGS', 'uavx-python', 'src', 'airframes')))
    return [airframes] + [x for x in extra if os.path.isdir(x)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--roots', nargs='*', default=None,
                    help='airframe dirs to scan (default: all copies)')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    roots = args.roots or default_roots()

    total_lines = 0
    files_changed = 0
    for root in roots:
        files = glob.glob(os.path.join(root, '**', '*.af'), recursive=True)
        for path in sorted(files):
            with open(path) as f:
                text = f.read()
            lines = text.splitlines(keepends=True)
            new_lines = list(lines)
            changed = 0
            for i, raw in enumerate(lines):
                new = fix_line(raw.rstrip('\n'))
                if new is not None:
                    new_lines[i] = new + '\n'
                    changed += 1
                    total_lines += 1
            if changed:
                files_changed += 1
                if not args.dry_run:
                    with open(path, 'w') as f:
                        f.writelines(new_lines)
                print(f"{'DRY-RUN ' if args.dry_run else 'fixed  '}{path} "
                      f"({changed} line{'s' if changed != 1 else ''})")

    print(f"\n{total_lines} line(s) fixed across {files_changed} file(s), "
          f"{len(roots)} root(s)")
    if args.dry_run:
        print("(dry run — no files written)")


if __name__ == '__main__':
    main()