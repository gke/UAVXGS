"""Fleet sweep: adopt EXTERNAL proven rate-loop tunings into every non-legacy
.af file (generic/, user/, original/, proposed/ — NOT backup/, _retired_tuned/).

Sources are field-proven autopilots with documented, unit-mapped constants,
NOT our own fleet values (Greg 2026-09-12: "the files have to be external to
us").  Our fleet-derived medians remain the critique/sanity REFERENCE only.

  MR (iNav 7.1.2 MC defaults, P/D mapped via compare_inav.py):
      Roll/Pitch  P=40 D=23 pidSum 500 -> Kp = (40/31)*RAD2DEG/500, Kd = (23/1905)*RAD2DEG/500
      Yaw         P=85 D=0  pidSum 400 -> Kp = (85/31)*RAD2DEG/400,       Kd = 0
  FW (ArduPilot Plane RLL2SRV/PTCH2SRV, kp_ff/D mapped via compare_ardupilot.py):
      Roll  kp_ff=0.345 D=0.08 -> Kp = 0.345/4500*RAD2DEG, Kd = 0.08/4500*RAD2DEG
      Pitch kp_ff=0.385 D=0.04 -> Kp = 0.385/4500*RAD2DEG, Kd = 0.04/4500*RAD2DEG
      Yaw: ArduPlane has NO default FW rate-yaw loop -> FW yaw rate gains left
      UNTOUCHED (not comparable).

Surgical: only the rewritten value lines (and their [LIMITS] entries, re-anchored
like migrate_limits.py) change; everything else is byte-preserved.  Idempotent.
Runs over all copies (canonical, kit installers, local mirror).  The [LIMITS]
design invariant "value inside its own [lo, hi]" is preserved (repair_af_limits).

Usage:
  python3 airframes/apply_external_rate_tuning.py [--roots DIR,...] [--dry-run]
"""
import argparse
import glob
import math
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from airframes import airframes  # noqa: E402
from protocol_enums import ParamIndex  # noqa: E402
from parameters import PARAM_LIMITS  # noqa: E402
from export_all_params import classify  # noqa: E402

RAD2DEG = 180.0 / math.pi
INAV_MC = {'Roll': (40, 23, 500), 'Pitch': (40, 23, 500), 'Yaw': (85, 0, 400)}
ARP_PLANE = {'Roll': (0.345, 0.08), 'Pitch': (0.385, 0.04)}
SERVO_FULL = 4500.0

TAG = {
    'Roll_KP': ParamIndex.ROLL_RATE_KP.value,     # 0
    'Pitch_KP': ParamIndex.PITCH_RATE_KP.value,   # 5
    'Yaw_KP': ParamIndex.YAW_RATE_KP.value,       # 10
    'Roll_KD': ParamIndex.ROLL_RATE_KD.value,     # 11
    'Pitch_KD': ParamIndex.PITCH_RATE_KD.value,   # 27
    'Yaw_KD': ParamIndex.YAW_RATE_KD.value,       # 90
}
RATE_TAGS = list(TAG.values())


def adopted_for(cat):
    """External adopted {tag: raw} for a category.  FW yaw absent == untouched."""
    if cat == 'MR':
        d = {}
        for axis, (P, D, lim) in INAV_MC.items():
            d[TAG[f'{axis}_KP']] = (P / 31.0) * RAD2DEG / lim
            d[TAG[f'{axis}_KD']] = (D / 1905.0) * RAD2DEG / lim
        return d
    if cat == 'FW':
        d = {}
        for axis, (kff, D) in ARP_PLANE.items():
            d[TAG[f'{axis}_KP']] = (kff / SERVO_FULL) * RAD2DEG
            d[TAG[f'{axis}_KD']] = (D / SERVO_FULL) * RAD2DEG
        return d
    return {}


def siground(x, sig=4):
    if x == 0:
        return 0.0
    return round(x, sig - int(math.floor(math.log10(abs(x)))) - 1)


def new_limits(tag, v):
    """gen_limits-style [v*0.5, v*2.0] band clamped to the class ceiling."""
    lo, hi = PARAM_LIMITS.get(tag, (0.0, 255.0))
    if v == 0.0:
        return lo, hi
    l = max(lo, siground(v * 0.5))
    h = min(hi, siground(v * 2.0))
    return (l, h) if l < h else (lo, hi)


VALUE_RE = re.compile(r'^(?P<key>[A-Z][A-Z0-9_]*)\s*=\s*(?P<val>[^,\s]+)\s*$')
RANGE_RE = re.compile(
    r'^(?P<key>[A-Z][A-Z0-9_]*)\s*=\s*(?P<lo>[0-9.eE+-]+)\s*,\s*(?P<hi>[0-9.eE+-]+)\s*$')


def fmt(v):
    """Match the .af writer's numeric style (see airframes._format_value):
    small-magnitude floats use %.8g; integer-valued 1..255 render as ints."""
    if abs(v) < 0.001:
        return f'{v:.8g}'
    if v == int(v) and abs(v) < 1e9 and 0 <= v <= 255 and v != 0:
        return str(int(v))
    return f'{v:.8g}'


def category_of(path, lines):
    """Resolve the airframe category from just the AF_TYPE line (enum-name or
    raw int), so stale legacy Config1Bits pipe-flag lines elsewhere in the file
    cannot abort classification.  Falls back to full-parse, then 'MR'."""
    for raw in lines:
        m = VALUE_RE.match(raw.rstrip('\n'))
        if not m or m.group('key').upper() != 'AF_TYPE':
            continue
        try:
            af = airframes._parse_value('AF_TYPE', m.group('val'))
            return classify(af)
        except (TypeError, ValueError):
            return 'MR'
    try:
        _name, values, _meta = airframes.parse_af_file(path)
        if values:
            return classify(values.get(ParamIndex.AF_TYPE.value, 0))
    except Exception:
        pass
    return 'MR'


def rewrite_file(path, dry):
    with open(path) as f:
        text = f.read()
    lines = text.splitlines(keepends=True)

    cat = category_of(path, lines)
    adopted = adopted_for(cat)
    if not adopted:
        return 0

    new_lines = list(lines)
    in_limits = False
    changed = 0
    for i, raw in enumerate(lines):
        line = raw.rstrip('\n')
        if line.upper() == '[LIMITS]':
            in_limits = True
            continue
        m = VALUE_RE.match(line)
        if m and not in_limits:
            key = m.group('key')
            try:
                tag = ParamIndex[key].value
            except KeyError:
                continue
            if tag in adopted:
                try:
                    old = float(m.group('val'))
                except ValueError:
                    continue
                new = adopted[tag]
                if abs(new - old) > 1e-12:
                    new_lines[i] = f'{key} = {fmt(new)}\n'
                    changed += 1
            continue
        m = RANGE_RE.match(line)
        if m and in_limits:
            key = m.group('key')
            try:
                tag = ParamIndex[key].value
            except KeyError:
                continue
            if tag in adopted:
                l, h = new_limits(tag, adopted[tag])
                new_lines[i] = f'{key} = {l:.8g}, {h:.8g}\n'
                changed += 1
    if not changed:
        return 0
    if not dry:
        with open(path, 'w') as f:
            f.writelines(new_lines)
    return changed


def default_roots():
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.dirname(here)                # .../uavx-python/src
    uavxgs = os.path.dirname(src)              # .../UAVXGS
    extra = []
    for d in ('linux', 'macos', 'windows'):
        extra.append(os.path.join(uavxgs, d, 'src', 'airframes'))
    extra.append(os.path.abspath(os.path.join(
        uavxgs, '..', 'gitUAVXGS', 'uavx-python', 'src', 'airframes')))
    return [here] + [x for x in extra if os.path.isdir(x)]


SWEEP_DIRS = ('generic', 'user', 'original', 'proposed')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--roots', nargs='*', default=None)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    roots = args.roots or default_roots()

    total = 0
    files = 0
    for root in roots:
        for sub in SWEEP_DIRS:
            subdir = os.path.join(root, sub)
            for path in sorted(glob.glob(os.path.join(subdir, '*.af'))):
                rel = os.path.relpath(path, root)
                n = rewrite_file(path, args.dry_run)
                if n:
                    files += 1
                    total += n
                    print(f"{'DRY-RUN ' if args.dry_run else 'rewrote '}{rel} "
                          f"({n} line{'s' if n != 1 else ''})")
    print(f"\n{total} rate-gain line(s) rewritten across {files} file(s), "
          f"{len(roots)} root(s)")
    if args.dry_run:
        print("(dry run — no files written)")


if __name__ == '__main__':
    main()