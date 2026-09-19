"""Fix the legacy x100 parameter mis-scaling in .af files (2026-09-17).

During the uint8 -> raw-float parameter migration, two legacy-tagged params
whose GCS display multiplier is 100 (PARAM_SCALES = 0.01) were saved into the
.af raw float slot as their DISPLAY value instead of their raw value:

    MADGWICK_KP_MAG  (tag 31)   stored 0.5  -> should be raw 0.005
    YAW_RATE_INT_LIM (tag 109)  stored 0.03 -> should be raw 0.0003

The FC applies .af values verbatim (no runtime rescaling — ParamTable `.scale`
is display metadata only), so a stored 0.5 is a 100x Mag fusion gain at
runtime: the Madgwick mag cross-term then overpowers the gyro and drives the
heading wrong-sense (frozen / inverted yaw hold). See
wiki/Session_Report_*.md (2026-09-17) for the diagnosis.

Surgical + idempotent:
  * only a base value EXACTLY equal to the known-wrong legacy value is
    rewritten (0.5 / 0.03); correct (0.005 / 0.0003) and deliberate 0 stay;
  * a [LIMITS] band that no longer brackets the corrected base is widened
    (far edge kept, inner edge pulled in), same rule as adopt_angle_params.py.

Usage:
  python3 airframes/fix_legacy_scaling.py [--roots DIR,...] [--dirs D1,D2]
                                          [--dry-run] [--file PATH]
"""
import argparse
import glob
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(_HERE)))

# key -> (legacy_wrong_value, corrected_raw_value)
TARGETS = {
    'MADGWICK_KP_MAG': (0.5, 0.005),
    'YAW_RATE_INT_LIM': (0.03, 0.0003),
}

LINE_RE = re.compile(r'^(?P<key>[A-Z_]+)\s*=\s*(?P<val>.+?)\s*$')


def log(msg):
    print(msg, flush=True)


def rewrite(text):
    """Rewrite wrong base values and widen non-bracketing [LIMITS] bands."""
    from protocol_enums import ParamIndex
    from parameters import PARAM_LIMITS

    out = []
    changed_base = 0
    changed_band = 0
    for line in text.splitlines():
        m = LINE_RE.match(line.strip())
        if not m or m.group('key') not in TARGETS:
            out.append(line)
            continue
        key = m.group('key')
        wrong, correct = TARGETS[key]
        payload = m.group('val').strip()
        if ',' in payload:
            parts = [p.strip() for p in payload.split(',') if p.strip()]
            if len(parts) != 2:
                out.append(line)
                continue
            try:
                lo, hi = float(parts[0]), float(parts[1])
            except ValueError:
                out.append(line)
                continue
            if lo <= correct <= hi:
                out.append(line)
                continue
            nlo = lo if correct >= lo else correct
            nhi = hi if correct <= hi else correct
            nlo = max(0.0, nlo)
            try:
                ceiling = PARAM_LIMITS[ParamIndex[key].value][1]
                nhi = min(ceiling, nhi)
            except (KeyError, ValueError):
                pass
            out.append(f'{key} = {nlo:.8g}, {nhi:.8g}')
            changed_band += 1
        else:
            try:
                val = float(payload)
            except ValueError:
                out.append(line)
                continue
            if abs(val - wrong) > 1e-9:
                out.append(line)
                continue
            out.append(f'{key} = {correct:.8g}')
            changed_base += 1

    if out and not out[-1].endswith('\n'):
        out[-1] = out[-1] if out[-1] == '' else out[-1] + '\n'
    return '\n'.join(out), changed_base, changed_band


def collect_files(roots, dirs):
    files = []
    for r in roots:
        for d in dirs:
            files.extend(glob.glob(os.path.join(r, d, '*.af')))
    return sorted(set(files))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--roots', nargs='*', default=None)
    ap.add_argument('--dirs', default='generic,user,proposed,original')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--file', default=None)
    args = ap.parse_args()

    dirs = [d.strip() for d in args.dirs.split(',') if d.strip()]

    if args.file:
        files = [args.file]
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        uavxgs = os.path.dirname(os.path.dirname(os.path.dirname(here)))
        extra = []
        for d in ('linux', 'macos', 'windows'):
            extra.append(os.path.join(uavxgs, d, 'src', 'airframes'))
        extra.append(os.path.abspath(os.path.join(
            uavxgs, '..', 'gitUAVXGS', 'uavx-python', 'src', 'airframes')))
        extra.append(os.path.abspath(os.path.join(
            uavxgs, '..', 'gitUAVXGS', 'windows', 'src', 'airframes')))
        roots = args.roots or [here] + [x for x in extra if os.path.isdir(x)]
        files = collect_files(roots, dirs)

    total_base = total_band = affected = 0
    for fp in files:
        try:
            with open(fp) as f:
                text = f.read()
        except OSError as e:
            log(f'{fp}: read FAIL {e}')
            continue
        new_text, cb, cband = rewrite(text)
        if not cb and not cband:
            continue
        affected += 1
        total_base += cb
        total_band += cband
        log(f'{fp}: {cb} base + {cband} band')
        if not args.dry_run:
            assert new_text != text
            with open(fp, 'w') as f:
                f.write(new_text)

    log('=' * 60)
    tag = 'DRY-RUN: ' if args.dry_run else ''
    log(f'{tag}{affected} file(s) changed, '
        f'{total_base} base + {total_band} band rewrite(s)')


if __name__ == '__main__':
    main()
