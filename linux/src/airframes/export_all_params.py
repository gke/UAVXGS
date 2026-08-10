#!/usr/bin/env python3
"""Export every airframe's full parameter set (all 128 params + PHYS_*) to CSV.

Row order:  original group first, then user, then generic.
Within each group: sorted by class (FW, MR, VTOL, Land, Sensor), then by filename.

Values are shown exactly as stored in the .af files (raw FC units, enum names).

Usage:
    python3 export_all_params.py [output.csv]
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from airframes.airframes import parse_af_file, _format_value
from protocol_enums import ParamIndex, AirframeType

GROUPS = ['original', 'user', 'generic']
CLASS_ORDER = {'FW': 0, 'MR': 1, 'VTOL': 2, 'Land': 3, 'Sensor': 4}

FW_TYPES = (AirframeType.ELEVON, AirframeType.DELTA, AirframeType.AILERON,
            AirframeType.AILERON_SPOILER_FLAPS, AirframeType.AILERON_VTAIL,
            AirframeType.RUDDER_ELEVATOR)
VTOL_TYPES = (AirframeType.VTOL, AirframeType.VTOL2)
LAND_TYPES = (AirframeType.TRACKED, AirframeType.TWO_WHEEL, AirframeType.FOUR_WHEEL)


def classify(af_type) -> str:
    """Return 'FW'/'MR'/'VTOL'/'Land'/'Sensor' matching FC ClassifyAFType()."""
    try:
        af = AirframeType(int(af_type))
    except (ValueError, TypeError):
        return 'MR'
    if af in FW_TYPES:
        return 'FW'
    if af in VTOL_TYPES:
        return 'VTOL'
    if af in LAND_TYPES:
        return 'Land'
    if af is AirframeType.INSTRUMENTATION:
        return 'Sensor'
    return 'MR'


def main():
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'airframes_all_params.csv')
    if len(sys.argv) > 1:
        out_path = sys.argv[1]

    rows = []
    phys_keys = set()
    for group in GROUPS:
        gdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), group)
        if not os.path.isdir(gdir):
            continue
        for fn in sorted(os.listdir(gdir)):
            if not fn.endswith('.af'):
                continue
            rel = f'{group}/{fn}'
            name, values, meta = parse_af_file(os.path.join(gdir, fn))
            cat = classify(values.get(ParamIndex.AF_TYPE.value, 0))
            phys = {k: v for k, v in meta.items() if k.startswith('PHYS_')}
            phys_keys.update(phys.keys())
            rows.append({'group': group, 'cat': cat, 'rel': rel, 'name': name,
                         'values': values, 'phys': phys})

    phys_cols = sorted(phys_keys)
    param_cols = []
    for i in range(128):
        try:
            param_cols.append(ParamIndex(i).name)
        except ValueError:
            param_cols.append(f'UNUSED_{i}')

    header = ['airframe', 'group', 'class', 'name'] + phys_cols + param_cols
    rows.sort(key=lambda r: (GROUPS.index(r['group']),
                             CLASS_ORDER.get(r['cat'], 9), r['rel']))

    with open(out_path, 'w', newline='') as f:
        import csv
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            line = [r['rel'], r['group'], r['cat'], r['name']]
            line += [r['phys'].get(k, '') for k in phys_cols]
            for i, pn in enumerate(param_cols):
                if i in r['values']:
                    line.append(_format_value(pn, r['values'][i]))
                else:
                    line.append('')
            w.writerow(line)

    print(f'wrote {out_path}')
    print(f'  {len(rows)} airframes, {len(header)} columns '
          f'({len(phys_cols)} PHYS + {len(param_cols)} params)')
    for g in GROUPS:
        n = sum(1 for r in rows if r['group'] == g)
        print(f'  {g}: {n} airframes')


if __name__ == '__main__':
    main()
