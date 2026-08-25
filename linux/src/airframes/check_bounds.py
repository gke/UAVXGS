"""Airframe bounds auditor: ensure every .af value and [LIMITS] entry stays
inside the FC ParamTable-derived PARAM_LIMITS ceilings.

REALIS: the GCS derailed its class ceilings on exact-math constants (pi/3,
600 deg/s) while the FC table stores decimal literals (1.047198, 10.471976).
That drift made a legitimately-maxed 60 deg limit (1.047198) format to
1.0472 with %.6g and trip the range alarm. The ceilings now mirror the FC
table exactly, and this pass clamps any straggler values/limits inside them.

Usage:
  python3 airframes/check_bounds.py [--dry-run]
"""
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from airframes import airframes
from parameters import PARAM_LIMITS

AIRFRAMES_DIR = os.path.dirname(__file__)
EPS = 1e-9


def clamp_value(tag, v, lo, hi):
    if v < lo:
        return lo, lo - v
    if v > hi:
        return hi, hi - v
    return v, 0.0


def main():
    dry = '--dry-run' in sys.argv
    paths = sorted(glob.glob(os.path.join(AIRFRAMES_DIR, 'generic', '*.af'))
                   + glob.glob(os.path.join(AIRFRAMES_DIR, 'user', '*.af')))
    total = 0
    for path in paths:
        name, values, meta = airframes.parse_af_file(path)
        changed = []

        for tag in sorted(values):
            lo, hi = PARAM_LIMITS.get(tag, (0.0, 255.0))
            v = values[tag]
            new, d = clamp_value(tag, v, lo, hi)
            if d != 0.0:
                changed.append(f"  {airframes.ParamIndex(tag).name}: value {v:.7g} -> {new:.7g}")
                values[tag] = new

        limits = meta.get('LIMITS')
        if limits:
            for tag in sorted(limits):
                lo, hi = PARAM_LIMITS.get(tag, (0.0, 255.0))
                l, h = limits[tag]
                nl, dl = clamp_value(tag, l, lo, hi)
                nh, dh = clamp_value(tag, h, lo, hi)
                if dl != 0.0 or dh != 0.0:
                    changed.append(f"  {airframes.ParamIndex(tag).name}: limits [{l:.7g}, {h:.7g}] -> [{nl:.7g}, {nh:.7g}]")
                    limits[tag] = (nl, nh)

        if changed:
            total += len(changed)
            if not dry:
                with open(path, 'w') as f:
                    f.write(airframes.format_af(name, values, metadata=meta))
            print(f"{'DRY-RUN ' if dry else ''}{os.path.basename(path)}:")
            for c in changed:
                print(c)

    print(f"\nTotal clamped entries: {total}")
    if dry:
        print("(dry run — no files written)")


if __name__ == '__main__':
    main()