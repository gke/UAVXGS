#!/usr/bin/env python3
"""Normalize the generic/*.af study fleet to canonical 128-param files.

Every airframe gets the full 128-param set in canonical key names:
  - values already present in the file are PRESERVED unchanged,
  - tags the file omits are filled from the FC ParamTable default_val column
    (params.c UseDefaultParametersEx) — making the implicit FC fallback
    explicit and uniform across the fleet.
  - legacy alias keys (EST_CRUISE_THR, KF_*, FW_ROLL_CONTROL_PITCH_LIMIT) are
    resolved to their canonical names via the parse/format bridge.
  - '# <name>' header set to the airframe's proper name (cosmetic: parse_af's
    read_name is only ever logged).
  - PHYS_* / Character metadata preserved.
  - [LIMITS] bands preserved as-is; Shadow.af (which has no block) gets a
    synthesized block spanning the FW fleet's existing bands.

The FC ParamTable is the single source of truth for the fill-in defaults; the
GCS PARAM_DEFAULTS table is NOT used (it is stale/display-unit — it even
carries the pre-2026-09-17 ×100 mis-scale for MADGWICK_KP_MAG/…).

Usage:
    python3 airframes/normalize_generic.py            # dry-run report, no writes
    python3 airframes/normalize_generic.py --write    # apply (canonical + kits + gitUAVXGS)
"""

import os
import re
import sys
import shutil

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from airframes.airframes import parse_af_file, format_af, _parse_value
from protocol_enums import ParamIndex, Config1Bits, Config2Bits

SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # uavx-python/src
GENERIC = os.path.join(SRC, 'airframes', 'generic')
FC_PARAMS_C = '/home/gke/Documents/Flight/Code/UAVXArmQ/src/params.c'

# Display name for the '# <name>' header (cosmetic).
NAMES = {
    'Delta.af': 'Delta',
    'Dragon.af': 'Dragon',
    'Elevon.af': 'Elevon',
    'Hex.af': 'Hexacopter',
    'Oct.af': 'Octocopter',
    'Quad.af': 'Quadcopter',
    'Quad_Medium.af': 'Quadcopter Medium',
    'Quad_Racer.af': 'Quadcopter Racer',
    'Radian.af': 'Radian',
    'RudderElevator.af': 'RudderElevator',
    'Shadow.af': 'Shadow',
    'SkySurfer_Bixler.af': 'SkySurfer Bixler',
    'SmallSpoileron.af': 'Small Spoileron',
    'Spoileron.af': 'Spoileron',
}

UAVXGS_ROOT = os.path.dirname(os.path.dirname(SRC))           # UAVXGS/
GITUAVXGS = '/home/gke/Documents/Flight/Code/gitUAVXGS'

# Mirror roots (canonical + 3 kits + gitUAVXGS local source mirror).
MIRRORS = [
    SRC,
    os.path.join(UAVXGS_ROOT, 'linux', 'src'),
    os.path.join(UAVXGS_ROOT, 'macos', 'src'),
    os.path.join(UAVXGS_ROOT, 'windows', 'src'),
    os.path.join(GITUAVXGS, 'uavx-python', 'src'),
    os.path.join(GITUAVXGS, 'windows', 'src'),
]

FW_FILES = ['Delta', 'Dragon', 'Elevon', 'Radian', 'RudderElevator',
            'SkySurfer_Bixler', 'SmallSpoileron', 'Spoileron']

# ParamTable defaults that exist FC-side but have no GCS enum mapping.
# eLogUAVX=0 (protocol_enums LogType), eLandContactSw=1 (auto.h
# MotorStopActions), eTraceRate=1 (trace.h TraceTypes), eUnknownReset=0
# (main.h ResetCauses). Kept numeric here; format_af emits them numerically.
OVERRIDES = {13: 0.0, 100: 1.0, 125: 1.0, 127: 0.0}


def split_args(s):
    out = []
    depth = 0
    cur = ''
    for ch in s:
        if ch == '(':
            depth += 1
        elif ch == ')':
            depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += ch
    out.append(cur.strip())
    return out


def fc_defaults():
    """tag -> raw float default, read from the FC ParamTable default_val column."""
    text = open(FC_PARAMS_C).read()
    m = re.search(r'const ParamMetaEntry ParamTable\[MAX_PARAMETERS\]\s*=\s*\{(.*?)\n\};',
                  text, re.S)
    body = m.group(1)
    defaults = {}
    for line in body.split('\n'):
        mm = re.match(r'^\s*(FLOAT|U8)\((.*)\)\s*,', line)
        if not mm:
            continue
        mac, args_str = mm.group(1), mm.group(2)
        args = split_args(args_str)
        def_idx = 5 if mac == 'FLOAT' else 4
        token = args[def_idx] if def_idx < len(args) else '?'
        tag = len(defaults)
        token = token.strip()
        if re.match(r'^-?\d*\.?\d+(?:e[+-]?\d+)?[fF]?\Z', token):
            defaults[tag] = float(token.rstrip('fF'))
            continue
        if re.match(r'^[0-9.eE+*\-/( )]+\Z', token):
            defaults[tag] = float(eval(token))
            continue
        try:
            param_name = ParamIndex(tag).name
        except ValueError:
            param_name = f'UNUSED_{tag}'
        if token.startswith('DEFAULT_CONFIG') or token == 'UseBatteryCompMask | UseFastStartMask | UseGPSMask | UseNavBeepMask':
            defaults[tag] = float(int(Config1Bits.DEFAULT if token.startswith('DEFAULT_CONFIG1')
                                       else Config2Bits.DEFAULT))
            continue
        try:
            defaults[tag] = float(_parse_value(param_name, token))
        except (ValueError, TypeError):
            if tag in OVERRIDES:
                defaults[tag] = OVERRIDES[tag]
                continue
            raise SystemExit(f'cannot resolve default for tag {tag} ({param_name}): {token!r}')
    if len(defaults) != 128:
        raise SystemExit(f'ParamTable parse gave {len(defaults)} entries, expected 128')
    return defaults


def synthesize_shadow_bands():
    """FW-fleet-span [LIMITS] for Shadow (which has none): for every tag that at
    least one other FW file bands, band = min(lo)..max(hi) across the fleet."""
    spans = {}
    for fn in FW_FILES:
        path = os.path.join(GENERIC, fn + '.af')
        if not os.path.exists(path):
            continue
        _, _, meta = parse_af_file(path)
        for tag, (lo, hi) in meta.get('LIMITS', {}).items():
            if tag not in spans:
                spans[tag] = [lo, hi]
            else:
                spans[tag][0] = min(spans[tag][0], lo)
                spans[tag][1] = max(spans[tag][1], hi)
    return spans


def main():
    write = '--write' in sys.argv
    defaults = fc_defaults()
    shadow_bands = synthesize_shadow_bands()

    files = [fn for fn in sorted(os.listdir(GENERIC)) if fn.endswith('.af')]
    report = []
    any_change = False
    for fn in files:
        path = os.path.join(GENERIC, fn)
        name, vals, meta = parse_af_file(path)
        new_name = NAMES.get(fn, fn[:-3])
        added = sorted(set(range(128)) - set(vals))
        changed_meta = name != new_name
        merged = dict(vals)
        for tag in added:
            merged[tag] = defaults[tag]
        meta2 = dict(meta)
        if fn == 'Shadow.af':
            bands = dict(shadow_bands)
            for tag, v in vals.items():
                if tag in bands:
                    lo, hi = bands[tag]
                    bands[tag] = (min(lo, v), max(hi, v))
            meta2['LIMITS'] = bands
        if changed_meta or added or (fn == 'Shadow.af' and not meta.get('LIMITS')):
            any_change = True
        report.append((fn, name, new_name, added, changed_meta,
                       bool(meta.get('LIMITS')), len(meta2.get('LIMITS', {}))))
        if write:
            text = format_af(new_name, merged, meta2)
            for root in MIRRORS:
                dst = os.path.join(root, 'airframes', 'generic', fn)
                if os.path.isdir(os.path.dirname(dst)):
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    with open(dst, 'w') as f:
                        f.write(text)
            with open(path, 'w') as f:
                f.write(text)

    print(f'{"file":26s} {"old #":22s} {"new #":22s} {"added":>5s} {"lim":>4s}')
    tags_added = set()
    for fn, old, new, added, cmeta, had_lim, lim_count in report:
        added_names = [ParamIndex(t).name if t in [m.value for m in ParamIndex] else f'UNUSED_{t}'
                       for t in added]
        print(f'{fn:26s} {old:22s} {new:22s} {len(added):5d} {lim_count:4d}')
        tags_added.update(added)
        if not write:
            print(f'      added ({len(added)}): {", ".join(added_names)}' if added else '      (no additions)')
    print()
    print(f'{"APPLIED to " + str(len(MIRRORS)) + " roots" if write else "DRY-RUN, nothing written"} '
          f'| {len(files)} files | union of newly-added tags: {len(tags_added)}')
    if not write and not any_change:
        print('No changes needed anywhere.')
    elif not write and any_change:
        print('Use --write to apply.')


if __name__ == '__main__':
    main()