"""Batch-apply derived angle gains to all generic + user .af files.

Mirrors the GCS Rate-view Apply-Derived path (ui/parameter_window.py
_refresh_pid_derived + _on_apply_pid_derived):

  QAngleKp   = RateMax / (2 * sin(AngleMax/2))          [roll/pitch, ALL frames]
  KiAngle    = k * QAngleKp,  k = 0.026 (MR/VTOL), 0.05 (FW)   [MC only — FW dormant]
  I-Limit    = 0.01 * AngleMax                                [MC only — FW dormant]
  yaw Ki     = 0.083 * yaw_QAngleKp                           [MC only]
  yaw I-Limit = 0.03                                          [MC only]

Units: .af stores FC-native raw values — angles rad, rates rad/s, gains as-is
(mult 1.0). The GCS display path multiplies by RAD_TO_DEG for angles/rates and
divides back, so deriving on raw rad values is identical to deriving on the
deg spins.

Bands: any [LIMITS] `lo, hi` pair that no longer brackets the new base value
is widened (keeping the polar edge —— the inner edge is moved back) so the
spinbox clamp on load never clips the authored base.

Surgical + idempotent, same pattern as fix_q_int_limit.py. Only the target
lines are rewritten; everything else is byte-preserved.

Usage:
  python3 airframes/apply_derived_angle_gains.py [--roots DIR,...] [--dry-run]
"""
import argparse
import glob
import math
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))          # src/ on path
sys.path.insert(0, os.path.dirname(os.path.dirname(_HERE)))  # uavx-python on path


def log(msg):
    print(msg, flush=True)


LINE_RE = re.compile(
    r'^(?P<key>[A-Z_]+)\s*=\s*(?P<val>.+?)\s*$')


def is_fw_airframe(vals):
    """Highest-fidelity FW test fn available outside the sim: AF_TYPE 13..18."""
    from protocol_enums import ParamIndex
    aft = vals.get(ParamIndex.AF_TYPE.value)
    return aft is not None and 13 <= int(aft) <= 18


def compute(filepath):
    """Return {key: raw_value} of the angle-gain writes for one .af file."""
    from airframes.airframes import parse_af_file
    from protocol_enums import ParamIndex
    name, vals, _meta = parse_af_file(filepath)

    def raw(key):
        return vals.get(ParamIndex[key].value)

    writes = {}
    fw = is_fw_airframe(vals)

    for U, RMAXK, AMAXK, QKPK, KIK, ILIMK in (
            ('ROL', 'MAX_ROLL_RATE', 'MAX_ROLL_ANGLE',
             'ROLL_ANGLE_Q_KP', 'ROLL_ANGLE_Q_KI', 'ROLL_ANGLE_Q_INT_LIMIT'),
            ('PIT', 'MAX_PITCH_RATE', 'MAX_PITCH_ANGLE',
             'PITCH_ANGLE_Q_KP', 'PITCH_ANGLE_Q_KI', 'PITCH_ANGLE_Q_INT_LIMIT')):
        rmax = raw(RMAXK)
        amax = raw(AMAXK)
        if rmax is None or amax is None or amax <= 1e-9 or rmax <= 0:
            log(f"  {os.path.basename(filepath)}: skip {U} — no sensible "
                f"{RMAXK}/{AMAXK}")
            continue
        denom = 2.0 * math.sin(amax * 0.5)
        if denom <= 1e-12:
            continue
        qkp = rmax / denom
        writes[QKPK] = qkp
        if not fw:
            k = 0.026
            writes[KIK] = k * qkp
            writes[ILIMK] = 0.01 * amax

    if not fw:
        ykp = raw('YAW_ANGLE_Q_KP')
        if ykp is not None:
            writes['YAW_ANGLE_Q_KI'] = 0.083 * ykp
            writes['YAW_ANGLE_Q_INT_LIMIT'] = 0.03

    return writes


def widen_band(lo, hi, val, ceiling):
    """Return (lo, hi) bracketing val; keep the far edge, pull inner edge in."""
    lo = max(0.0, min(lo, val))
    hi = min(ceiling, max(hi, val))
    return lo, hi


def rewrite(text, writes, fw=None):
    """Rewrite base-value lines and [LIMITS] pairs for the target keys.

    Returns (new_text, changed_base, changed_bands).
    """
    from protocol_enums import ParamIndex
    from parameters import PARAM_LIMITS
    from airframes.airframes import _parse_value

    new_lines = []
    changed_base = 0
    changed_bands = 0
    in_limits = False
    base_seen = set()

    for line in text.splitlines():
        orig = line
        stripped = line.strip()
        if stripped.upper() == '[LIMITS]':
            in_limits = True
            new_lines.append(line)
            continue
        m = LINE_RE.match(stripped)
        if not m:
            new_lines.append(line)
            continue
        key = m.group('key')
        if key not in writes:
            new_lines.append(line)
            continue
        try:
            idx = ParamIndex[key].value
        except (KeyError, ValueError):
            new_lines.append(line)
            continue
        ceiling = PARAM_LIMITS.get(idx, (0.0, 255.0))[1]
        new_val = writes[key]
        payload = m.group('val').strip()
        if ',' in payload:
            parts = [p.strip() for p in payload.split(',') if p.strip()]
            if len(parts) != 2:
                new_lines.append(line)
                continue
            try:
                lo, hi = float(parts[0]), float(parts[1])
            except ValueError:
                new_lines.append(line)
                continue
            nlo, nhi = widen_band(lo, hi, new_val, ceiling)
            new_band = f'{nlo:.8g}, {nhi:.8g}'
            if new_band != f'{lo:.8g}, {hi:.8g}':
                new_lines.append(f'{key} = {new_band}')
                changed_bands += 1
            else:
                new_lines.append(line)
        else:
            # Guard: identical base value already present earlier -> leave as-is
            if key in base_seen:
                new_lines.append(line)
                continue
            base_seen.add(key)
            new_repr = f'{new_val:.8g}'
            if new_repr == payload:
                new_lines.append(orig)
            else:
                new_lines.append(f'{key} = {new_repr}')
                changed_base += 1

    if new_lines and not new_lines[-1].endswith('\n'):
        last = new_lines[-1]
        new_lines[-1] = last if last == '' else last.rstrip('\n') + '\n'
    return '\n'.join(new_lines), changed_base, changed_bands


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--roots', nargs='*', default=None,
                    help='airframe dirs to scan (default: canonical + kit mirrors)')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--file', default=None,
                    help='single .af file to edit (overrides roots scan)')
    args = ap.parse_args()

    if args.file:
        files = [args.file]
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        airframes = here
        uavxgs = os.path.dirname(os.path.dirname(here))
        extra = []
        for d in ('linux', 'macos', 'windows'):
            extra.append(os.path.join(uavxgs, d, 'src', 'airframes'))
        extra.append(os.path.abspath(os.path.join(
            uavxgs, '..', 'gitUAVXGS', 'uavx-python', 'src', 'airframes')))
        roots = args.roots or [airframes] + [x for x in extra if os.path.isdir(x)]
        files = sorted(
            [f for r in roots for d in ('generic', 'user', 'proposed')
             for f in glob.glob(os.path.join(r, d, '*.af'))])

    total_base = total_bands = affected = 0
    for fp in files:
        try:
            writes = compute(fp)
        except Exception as e:
            log(f'{fp}: compute FAIL {e}')
            continue
        if not writes:
            log(f'{fp}: no writes (no roll/pitch rate+angle present)')
            continue
        with open(fp) as f:
            text = f.read()
        new_text, cb, cbad = rewrite(text, writes)
        if not cb and not cbad:
            continue
        affected += 1
        total_base += cb
        total_bands += cbad
        log(f'{fp}: {cb} base + {cbad} band line(s)')
        if not args.dry_run:
            assert new_text != text
            with open(fp, 'w') as f:
                f.write(new_text)

    log('=' * 60)
    if args.dry_run:
        log(f'DRY-RUN: {affected} file(s) would change, '
            f'{total_base} base + {total_bands} band line(s)')
    else:
        log(f'{affected} file(s) updated, '
            f'{total_base} base + {total_bands} band line(s)')


if __name__ == '__main__':
    main()