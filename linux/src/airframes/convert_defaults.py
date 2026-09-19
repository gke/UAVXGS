"""
Convert old defaults.h uint8 array format → human-readable .af files.

Usage:
    python convert_defaults.py /path/to/defaults.h [output_dir]

The uint8 values in defaults.h are in legacy (scale * display) format.
We convert to raw float values for the .af format.
"""

import re
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from protocol_enums import ParamIndex
from parameters import PARAM_SCALES, PARAM_TYPES

AIRFRAME_BLOCK_RE = re.compile(
    r'\{\s*"([^"]+)"\s*,\s*\{\s*([0-9,\s]+)\s*\}\s*\}'
)

def extract_uint8_arrays(path: str):
    """Parse defaults.h → [(name, [uint8_values])]"""
    with open(path) as f:
        text = f.read()
    # Strip C++ line comments
    text = re.sub(r'//.*', '', text)
    results = []
    for m in AIRFRAME_BLOCK_RE.finditer(text):
        name = m.group(1)
        nums = [int(x.strip()) for x in m.group(2).split(',') if x.strip()]
        results.append((name, nums))
    return results


def convert_to_raw(airframes):
    """Convert uint8 arrays to raw float .af text."""
    outputs = []
    for name, u8_vals in airframes:
        lines = [f'# {name}', '']
        for i in range(min(len(u8_vals), 128)):
            u8 = u8_vals[i]
            scale = PARAM_SCALES.get(i, 1.0)
            ptype = PARAM_TYPES.get(i, 'U8')
            try:
                pname = ParamIndex(i).name
            except ValueError:
                pname = f'UNUSED_{i}'
            if ptype == 'FLOAT':
                raw = u8 * scale
            else:
                raw = float(u8)
            lines.append(f'{pname} = {raw:.6g}')
        lines.append('')
        outputs.append('\n'.join(lines))
    return outputs


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    src = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(__file__)
    airframes = extract_uint8_arrays(src)
    print(f"Found {len(airframes)} airframes in {src}")
    for name, u8_vals in airframes:
        print(f"  {name}: {len(u8_vals)} params")
    texts = convert_to_raw(airframes)
    for (name, _), text in zip(airframes, texts):
        safe = re.sub(r'[^a-zA-Z0-9]+', '_', name).strip('_')
        out_path = os.path.join(out_dir, f'{safe}.af')
        with open(out_path, 'w') as f:
            f.write(text)
        print(f"  Wrote {out_path}")


if __name__ == '__main__':
    main()
