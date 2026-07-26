"""Reformat all .af files using airframes module for better enum/hex formatting."""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from airframes import airframes

DIR = os.path.dirname(__file__)
for fn in sorted(os.listdir(DIR)):
    if not fn.endswith('.af'):
        continue
    path = os.path.join(DIR, fn)
    with open(path) as f:
        text = f.read()
    name, values = airframes.parse_af(text)
    new_text = airframes.format_af(name, values)
    with open(path, 'w') as f:
        f.write(new_text)
    print(f"Reformatted {fn} ({name})")
