"""One-time .af repair: re-anchor [LIMITS] bounds so every file's stored
values lie inside their own [LIMITS] block (see wiki/Session_Report_*
for the load-clamp corruption it fixes).

Background (2026-09-06): migrate_limits.py::gen_limits() generates each
[LIMITS] entry as [v*0.5, v*2.0] clamped to the class ceiling. That makes
"value in [lo, hi]" the design invariant. A batch of .af files (Shadow,
Shadow2, Delta, Dragon, Horten FW rate gains; Quad_Medium/ SmallSpoileron/
Radian_Tuned over-tight hi) were left with STALE LIMITS after later
tuning, so their stored values sit outside their own tuning window.
Loading any such file runs _apply_airframe_limits() -> QDoubleSpinBox
setRange(), which CLAMPS THE CURRENT VALUE UP/INTO the new lo -- e.g.
Shadow ROLL_RATE_KP 0.53476 inflated to 2.577 (== LIMITS lo) on
load->save, then written to the FC at 4.8x the intended gain.

Repair policy (values are the authoritative, critique-validated tuning --
never touch a value):
  - value < lo  -> lower lo  to max(class_lo, siground(value*0.5))
  - value > hi  -> raise hi  to min(class_hi, siground(value*2.0))
  Only the violated bound is moved; the healthy bound and class ceilings
  are preserved.

Corrupted-save exception: user/Shadow_20260906_095138.af had values
ALREADY clamped to the generic LIMITS lo (2.577/1.61/1.61/0.0809/0.0054)
during a load->save in the GCS. For that file the 5 rate-gain VALUES are
restored first to the generic Shadow raw values, then bounds re-anchored.

Usage:
  python3 airframes/repair_af_limits.py [--dry-run]
"""
import glob
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from airframes import airframes
from parameters import PARAM_LIMITS

AIRFRAMES_DIR = os.path.dirname(__file__)

# (file-under user/, tag) -> raw value from generic/Shadow.af to restore
SHADOW_USER_RESTORE = {os.path.join('airframes', 'user', 'Shadow_20260906_095138.af'): {0: 0.53476, 5: 0.33422, 10: 0.33422, 11: 0.0168, 90: 0.00112}}


def siground(x, sig=4):
    if x == 0:
        return 0.0
    return round(x, sig - int(math.floor(math.log10(abs(x)))) - 1)


def reanchor(tag, lo, hi, v):
    """Move only the violated bound so v lands inside [lo, hi]."""
    cls_lo, cls_hi = PARAM_LIMITS.get(tag, (0.0, 255.0))
    nl, nh = lo, hi
    if v < lo:
        nl = max(cls_lo, siground(v * 0.5))
    if v > hi:
        nh = min(cls_hi, siground(v * 2.0))
    if not (nl < nh):
        nl, nh = lo, hi
    return nl, nh


def main():
    dry = '--dry-run' in sys.argv
    paths = sorted(glob.glob(os.path.join(AIRFRAMES_DIR, 'generic', '*.af'))
                   + glob.glob(os.path.join(AIRFRAMES_DIR, 'user', '*.af')))
    n_files = 0
    n_bounds = 0
    n_vals = 0

    # Corrupted-save exception: restore clamped VALUES first so the bounds
    # pass below re-anchors around the CORRECTED values, not the inflated ones.
    for path, restores in SHADOW_USER_RESTORE.items():
        p = os.path.join(AIRFRAMES_DIR, '..', path)
        if not os.path.exists(p):
            continue
        name, values, meta = airframes.parse_af_file(p)
        changed = []
        for tag, raw in restores.items():
            if abs(values.get(tag, 0.0) - raw) > 1e-12:
                changed.append(f"  {airframes.ParamIndex(tag).name}: value {values[tag]:.7g} -> {raw:.7g}")
                values[tag] = raw
        if changed:
            n_vals += len(changed)
            if not dry:
                with open(p, 'w') as f:
                    f.write(airframes.format_af(name, values, metadata=meta))
            print(f"{'DRY-RUN ' if dry else ''}{os.path.basename(p)} (value restore):")
            for c in changed:
                print(c)

    for path in paths:
        name, values, meta = airframes.parse_af_file(path)
        limits = meta.get('LIMITS')
        if not limits:
            continue
        changed = []
        for tag in sorted(limits):
            lo, hi = limits[tag]
            v = values.get(tag)
            if v is None:
                continue
            nl, nh = reanchor(tag, lo, hi, v)
            if (nl, nh) != (lo, hi):
                changed.append(f"  {airframes.ParamIndex(tag).name}: limits [{lo:.7g}, {hi:.7g}] -> [{nl:.7g}, {nh:.7g}] (value {v:.7g})")
                limits[tag] = (nl, nh)
        if not changed:
            continue
        n_files += 1
        n_bounds += len(changed)
        if not dry:
            with open(path, 'w') as f:
                f.write(airframes.format_af(name, values, metadata=meta))
        print(f"{'DRY-RUN ' if dry else ''}{os.path.basename(path)}:")
        for c in changed:
            print(c)

    print(f"\nRepaired {n_files} files: {n_bounds} LIMITS bounds re-anchored, {n_vals} values restored.")
    if dry:
        print("(dry run — no files written)")


if __name__ == '__main__':
    main()