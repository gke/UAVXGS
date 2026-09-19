"""Batch-adopt the study Angle parameters into generic .af files (2026-09-13).

Writes the adopted rate/angle MAXIMUMS per airframe class AND the angle-loop
gains DERIVED FROM THOSE ADOPTED LIMITS (not from each file's stored values):

  Adopted limits:
    MR  (AF_TYPE not FW): RollAngle 45 deg, PitchAngle 45 deg,
                         Roll/PitchRate 200 deg/s, YawRate 200 deg/s
    FW  (AF_TYPE 13..18): RollAngle 45 deg, PitchAngle 15 deg,
                         Roll/PitchRate 90 deg/s   (yaw kept per-file)

  Derived angle PI (same formula as the GCS Rate view + derive_angle_params.py):
    QAngleKp        = RateMax / (2·sin(AngleMax/2))
    KiAngle         = k · QAngleKp,   k = 0.026 MR / 0.05 FW
    I-Limit         = 0.01 · AngleMax
    MR yaw Ki       = 0.083 · yaw_QAngleKp (per-file yaw Kp retained)
    MR yaw I-Limit  = 0.03

  MR files historically carry NO YAW_ANGLE_Q_INT_LIMIT value or [LIMITS] line —
  both are INSERTED here so the .af fleet matches the GCS fleet convention.
  FW files ship the derived roll/pitch Ki/ILim as DORMANT values (the FC angle
  loop is P-only for FW, control.c `pAFTypeCategory != eCatFw`) — they are
  written so .af round-trips are canonical, and the sim gates them (test_pid_sim
  `if is_fw: aki = 0.0`).

Bands: any [LIMITS] `lo, hi` pair that no longer brackets the new base value is
widened (keeping the polar edge — the inner edge is moved back / the far edge is
pulled past the value) so the spinbox clamp on load never clips the authored
base. Values missing a band get one inserted next to the PARAM_LIMITS ceiling.

Surgical + idempotent, same pattern as apply_derived_angle_gains.py. Only the
target lines are rewritten; anything else is byte-preserved.

Usage:
  python3 airframes/adopt_angle_params.py [--roots DIR,...] [--dry-run]
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

# Adopted maxima (radians / rad/s). MR yaw rate adopted too; FW yaw stays
# per-file (never touched here).
ADOPTED = {
    'MR': {
        'MAX_ROLL_ANGLE':  0.78539816,   # 45 deg
        'MAX_PITCH_ANGLE': 0.78539816,   # 45 deg
        'MAX_ROLL_RATE':   3.49065904,   # 200 deg/s
        'MAX_PITCH_RATE':  3.49065904,   # 200 deg/s
        'MAX_YAW_RATE':    3.49065904,   # 200 deg/s
    },
    'FW': {
        'MAX_ROLL_ANGLE':  0.78539816,   # 45 deg
        'MAX_PITCH_ANGLE': 0.26179939,   # 15 deg
        'MAX_ROLL_RATE':   1.57079633,   # 90 deg/s
        'MAX_PITCH_RATE':  1.57079633,   # 90 deg/s
    },
}

K_MR = 0.026
K_FW = 0.05
K_YAW = 0.083
YAW_ILIM = 0.03

# band insertion for keys with neither a stored value nor a stored band
# (default = centred on the authored value, clipped to PARAM_LIMITS)
BAND_SPAN = 0.5     # lo = val/2, hi = val*2


def is_fw_airframe(vals):
    """Highest-fidelity FW test fn available outside the sim: AF_TYPE 13..18."""
    from protocol_enums import ParamIndex
    aft = vals.get(ParamIndex.AF_TYPE.value)
    return aft is not None and 13 <= int(aft) <= 18


def derive(amax, rmax, k):
    denom = 2.0 * math.sin(amax * 0.5)
    if denom <= 1e-12 or rmax <= 0:
        return None
    qkp = rmax / denom
    return qkp, k * qkp, 0.01 * amax


def compute(filepath):
    """Return {key: raw_value} of the adopted-limit + derived-gain writes."""
    from airframes.airframes import parse_af_file
    from protocol_enums import ParamIndex
    name, vals, _meta = parse_af_file(filepath)
    fw = is_fw_airframe(vals)

    def raw(key):
        return vals.get(ParamIndex[key].value)

    writes = {}

    for key, val in ADOPTED['FW' if fw else 'MR'].items():
        writes[key] = val

    k = K_FW if fw else K_MR
    for U, AMAXK, RMAXK, QKPK, KIK, ILIMK in (
            ('ROL', 'MAX_ROLL_ANGLE', 'MAX_ROLL_RATE',
             'ROLL_ANGLE_Q_KP', 'ROLL_ANGLE_Q_KI', 'ROLL_ANGLE_Q_INT_LIMIT'),
            ('PIT', 'MAX_PITCH_ANGLE', 'MAX_PITCH_RATE',
             'PITCH_ANGLE_Q_KP', 'PITCH_ANGLE_Q_KI', 'PITCH_ANGLE_Q_INT_LIMIT')):
        amax = ADOPTED['FW' if fw else 'MR'][AMAXK]
        rmax = ADOPTED['FW' if fw else 'MR'][RMAXK]
        d = derive(amax, rmax, k)
        if d is None:
            log(f"  {os.path.basename(filepath)}: skip {U} — derive failed")
            continue
        qkp, ki, ilim = d
        writes[QKPK] = qkp
        writes[KIK] = ki
        writes[ILIMK] = ilim

    if not fw:
        ykp = raw('YAW_ANGLE_Q_KP')
        if ykp is not None:
            writes['YAW_ANGLE_Q_KI'] = K_YAW * ykp
            writes['YAW_ANGLE_Q_INT_LIMIT'] = YAW_ILIM

    return writes


def widen_band(lo, hi, val, ceiling):
    """Return (lo, hi) bracketing val; keep the far edge, pull inner edge in."""
    lo = max(0.0, min(lo, val))
    hi = min(ceiling, max(hi, val))
    return lo, hi


def rewrite(text, writes):
    """Rewrite base-value lines and [LIMITS] pairs; INSERT missing keys.

    'writes' holds raw values (numbers) keyed by tag. A pair key that exists in
    the file as a value line is rewritten; a pair key that exists as a [LIMITS]
    line is widened. Keys present in neither are inserted: the value line right
    before the [LIMITS] header (or appended), the band line just after it.

    Returns (new_text, changed_base, changed_bands, inserted).
    """
    from protocol_enums import ParamIndex
    from parameters import PARAM_LIMITS

    writes = dict(writes)
    new_lines = []
    changed_base = 0
    changed_bands = 0
    inserted = []
    in_limits = False
    inserted_limits = False
    has_limits = False
    base_seen = set()
    band_seen = set()

    for line in text.splitlines():
        orig = line
        stripped = line.strip()
        if stripped.upper() == '[LIMITS]':
            in_limits = True
            has_limits = True
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
        idx = ParamIndex[key].value
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
            band_seen.add(key)
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

    # keys missing as a VALUE line that still need a band — handled by the
    # main pass when the band line exists. Bands only ever originate from a
    # real '[LIMITS]' block (Shadow.af has none -> value lines only).
    need_base = [k for k in writes if k not in base_seen]
    need_band = ([] if not has_limits
                 else [k for k in writes if k not in band_seen])

    base_ins = {k: f'{k} = {writes[k]:.8g}'
                for k in need_base if ',' not in str(writes[k])}
    band_ins = {}
    for k in need_base + need_band:
        if k in band_ins:
            continue
        if ',' in str(writes[k]):       # pre-formatted band string
            band_ins[k] = writes[k]
            continue
        try:
            idx = ParamIndex[k].value
        except (KeyError, ValueError):
            continue
        ceiling = PARAM_LIMITS.get(idx, (0.0, 255.0))[1]
        val = writes[k]
        lo = max(0.0, min(val, BAND_SPAN * val))
        hi = min(ceiling, max((1.0 / BAND_SPAN) * val, val))
        band_ins[k] = f'{k} = {lo:.8g}, {hi:.8g}'

    if need_base or need_band:
        inserted_keys = sorted(set(need_base) | set(need_band))
        out = []
        for line in new_lines:
            if line.strip().upper() == '[LIMITS]' and not inserted_limits:
                out.extend(base_ins[k] for k in need_base if k in base_ins)
                out.append(line)
                out.extend(band_ins[k] for k in need_band if k in band_ins)
                inserted_limits = True
            else:
                out.append(line)
        if not inserted_limits:
            for k in need_base:
                if k in base_ins:
                    out.append(base_ins[k])
            for k in need_band:
                if k in band_ins:
                    out.append(band_ins[k])
        new_lines = out
        inserted.extend(inserted_keys)

    if new_lines and not new_lines[-1].endswith('\n'):
        last = new_lines[-1]
        new_lines[-1] = last if last == '' else last.rstrip('\n') + '\n'
    return '\n'.join(new_lines), changed_base, changed_bands, inserted


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
        uavxgs = os.path.dirname(os.path.dirname(os.path.dirname(here)))
        extra = []
        for d in ('linux', 'macos', 'windows'):
            extra.append(os.path.join(uavxgs, d, 'src', 'airframes'))
        extra.append(os.path.abspath(os.path.join(
            uavxgs, '..', 'gitUAVXGS', 'uavx-python', 'src', 'airframes')))
        roots = args.roots or [airframes] + [x for x in extra if os.path.isdir(x)]
        files = sorted(
            [f for r in roots for d in ('generic',)
             for f in glob.glob(os.path.join(r, d, '*.af'))])

    total_base = total_bands = total_ins = affected = 0
    for fp in files:
        try:
            writes = compute(fp)
        except Exception as e:
            log(f'{fp}: compute FAIL {e}')
            continue
        if not writes:
            log(f'{fp}: no writes (no yaw Kp / class limits)')
            continue
        with open(fp) as f:
            text = f.read()
        new_text, cb, cbad, ins = rewrite(text, writes)
        if not cb and not cbad and not ins:
            continue
        affected += 1
        total_base += cb
        total_bands += cbad
        total_ins += len(ins)
        log(f'{fp}: {cb} base + {cbad} band + {len(ins)} ins ({", ".join(ins)})')
        if not args.dry_run:
            assert new_text != text
            with open(fp, 'w') as f:
                f.write(new_text)

    log('=' * 60)
    if args.dry_run:
        log(f'DRY-RUN: {affected} file(s) would change, '
            f'{total_base} base + {total_bands} band + {total_ins} insert(s)')
    else:
        log(f'{affected} file(s) updated, '
            f'{total_base} base + {total_bands} band + {total_ins} insert(s)')


if __name__ == '__main__':
    main()