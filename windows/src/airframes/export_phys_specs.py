# Export physical specs (PHYS_*) from all .af files as editable CSV

import sys, os, csv, re

ROOT = os.path.dirname(os.path.abspath(__file__))

def af_files(dirname):
    p = os.path.join(ROOT, dirname)
    if not os.path.isdir(p):
        return []
    return [(dirname, fn, os.path.join(p, fn))
            for fn in sorted(os.listdir(p)) if fn.endswith('.af')]

def parse_phys(path):
    phys = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith('PHYS_'):
                m = re.match(r'PHYS_(\w+)\s*=\s*(.+)', line)
                if m:
                    phys[m.group(1)] = m.group(2)
    return phys

def export(dirnames=['original', 'generic', 'user']):
    files = []
    for d in dirnames:
        files.extend(af_files(d))
    
    # Collect all PHYS keys
    all_keys = set()
    file_data = []
    for d, fn, path in files:
        phys = parse_phys(path)
        all_keys.update(phys.keys())
        file_data.append((f'{d}/{fn}', phys))
    
    keys = sorted(all_keys)
    w = csv.writer(sys.stdout)
    w.writerow(['airframe'] + keys)
    for name, phys in file_data:
        row = [name] + [phys.get(k, '') for k in keys]
        w.writerow(row)

if __name__ == '__main__':
    dirs = sys.argv[1:] if len(sys.argv) > 1 else ['original', 'generic', 'user']
    export(dirs)