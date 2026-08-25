# Export/import .af parameter values as editable CSV.
# Usage (from uavx-python/src/):
#   python3 -m airframes.af_table export [dirs...] > af_params.csv
#   python3 -m airframes.af_table import <csv>
# Default dirs: original

import sys, os, csv
from collections import OrderedDict

# resolve imports regardless of cwd
_TOP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TOP not in sys.path:
    sys.path.insert(0, _TOP)

import airframes.airframes as afmod
from protocol_enums import ParamIndex

parse_af = afmod.parse_af
format_af = afmod.format_af
ROOT = os.path.dirname(os.path.abspath(__file__))

# Build param-name lookup: index → GCS display name
_IDX_TO_NAME = {}
for name, idx in ParamIndex.__members__.items():
    _IDX_TO_NAME[idx.value] = name

def af_files(dirname):
    p = os.path.join(ROOT, dirname)
    if not os.path.isdir(p):
        return []
    return [(dirname, fn, os.path.join(p, fn))
            for fn in sorted(os.listdir(p)) if fn.endswith('.af')]

def export(dirs):
    files = []
    for d in dirs:
        files.extend(af_files(d))

    all_idxs = OrderedDict()
    for _, _, path in files:
        with open(path) as f:
            _, vals, _ = parse_af(f.read())
        for idx in sorted(vals.keys()):
            all_idxs[idx] = _IDX_TO_NAME.get(idx, f'P{idx}')

    col_names = ['airframe']
    col_idxs = []
    for idx, pname in all_idxs.items():
        col_names.append(pname)
        col_idxs.append(idx)

    w = csv.writer(sys.stdout)
    w.writerow(col_names)
    for d, fn, path in files:
        with open(path) as f:
            _, vals, _ = parse_af(f.read())
        row = [f'{d}/{fn}']
        for idx in col_idxs:
            v = vals.get(idx)
            row.append(f'{v:.6f}' if v is not None else '')
        w.writerow(row)

def import_csv(csv_path):
    with open(csv_path, newline='') as cf:
        reader = csv.DictReader(cf)
        # map column name → index
        col_idx = {}
        for col in reader.fieldnames:
            if col == 'airframe':
                continue
            if col.startswith('P') and col[1:].isdigit():
                col_idx[col] = int(col[1:])
            else:
                for name, idx in ParamIndex.__members__.items():
                    if name == col:
                        col_idx[col] = idx.value
                        break

        updates = []
        for row in reader:
            af_path = row['airframe']
            if '/' not in af_path:
                continue
            parts = af_path.split('/', 1)
            if len(parts) != 2:
                continue
            d, fn = parts
            full_path = os.path.join(ROOT, d, fn)
            if not os.path.exists(full_path):
                print(f"WARNING: {full_path} not found", file=sys.stderr)
                continue
            with open(full_path) as f:
                text = f.read()
            name, old_vals, meta = parse_af(text)
            new_vals = {}
            for col, idx in col_idx.items():
                v_str = row.get(col, '').strip()
                if not v_str:
                    continue
                v = float(v_str)
                if idx not in old_vals or abs(old_vals[idx] - v) > 1e-9:
                    new_vals[idx] = v
            if new_vals:
                merged = dict(old_vals)
                merged.update(new_vals)
                out = format_af(name, merged, meta)
                with open(full_path, 'w') as f:
                    f.write(out)
                for idx, v in sorted(new_vals.items()):
                    old = old_vals.get(idx, '—')
                    oname = _IDX_TO_NAME.get(idx, f'P{idx}')
                    print(f"  {af_path}: {oname} {old} → {v}")
                updates.append(af_path)
        if not updates:
            print("No changes applied.", file=sys.stderr)
        else:
            print(f"Updated {len(updates)} files.", file=sys.stderr)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 -m airframes.af_table export|import <file>", file=sys.stderr)
        print("  export: python3 -m airframes.af_table export [dir...] > af_params.csv", file=sys.stderr)
        print("  import: python3 -m airframes.af_table import af_params.csv", file=sys.stderr)
        sys.exit(1)
    cmd = sys.argv[1]
    if cmd == 'export':
        dirs = sys.argv[2:] if len(sys.argv) > 2 else ['original']
        export(dirs)
    elif cmd == 'import':
        import_csv(sys.argv[2])
    else:
        print(f"Unknown: {cmd}", file=sys.stderr)
        sys.exit(1)
