# Export just physical parameters from default airframes

import sys, os, csv

_TOP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TOP not in sys.path:
    sys.path.insert(0, _TOP)

import airframes.airframes as afmod
from protocol_enums import ParamIndex
parse_af = afmod.parse_af

ROOT = os.path.dirname(os.path.abspath(__file__))

def af_files(dirname):
    p = os.path.join(ROOT, dirname)
    if not os.path.isdir(p):
        return []
    return [(dirname, fn, os.path.join(p, fn))
            for fn in sorted(os.listdir(p)) if fn.endswith('.af')]

# Build index → name mapping for PHYS params
IDX_TO_NAME = {}
for name, idx in ParamIndex.__members__.items():
    if name.startswith('PHYS_'):
        IDX_TO_NAME[idx.value] = name

def export(dirnames=['original', 'generic']):
    files = []
    for d in dirnames:
        files.extend(af_files(d))
    
    # Collect all PHYS indices that appear
    phys_indices = set()
    for _, _, path in files:
        with open(path) as f:
            _, vals, _ = parse_af(f.read())
        for idx in vals:
            if idx in IDX_TO_NAME:
                phys_indices.add(idx)
    
    phys_indices = sorted(phys_indices)
    phys_names = [IDX_TO_NAME[i] for i in phys_indices]
    
    w = csv.writer(sys.stdout)
    w.writerow(['airframe'] + phys_names)
    for d, fn, path in files:
        with open(path) as f:
            _, vals, _ = parse_af(f.read())
        row = [f'{d}/{fn}']
        for idx in phys_indices:
            v = vals.get(idx)
            row.append(f'{v:.6f}' if v is not None else '')
        w.writerow(row)

if __name__ == '__main__':
    dirs = sys.argv[1:] if len(sys.argv) > 1 else ['original', 'generic']
    export(dirs)