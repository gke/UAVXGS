"""Fleet critique runner: run_tests_for_af over every .af in a dir set.

Prints one compact verdict line per file plus the per-axis step/cascade lines.
Usage: python3 tests/critique_fleet.py [--out FILE] [generic user proposed]
DEFAULT: generic user proposed
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # tests/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.test_pid_sim import run_tests_for_af  # noqa: E402

KEY = ('PASS', 'FAIL', 'Rise', 'Overshoot', 'Settle', 'Cascade')


def compact(output):
    """Pull the important verdict lines out of a full critique run."""
    sel = []
    for l in output:
        s = l.strip()
        if any(c in l for c in ('==', 'Cascade', 'Roll @', 'Pitch @', 'Yaw @',
                                'PASS', 'FAIL', 'I-limit', 'QP demand',
                                'P demand', 'Structural limit')):
            # strip ANSI codes
            for esc in ('\033[32m', '\033[31m', '\033[33m', '\033[34m',
                        '\033[0m', '\033[1m', '\033[91m'):
                s = s.replace(esc, '')
            s = s.replace('\033[', '<ESC>')
            sel.append(s)
    return sel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='*', default=['generic', 'user', 'proposed'])
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    af_root = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'airframes')
    files = []
    for d in args.dirs:
        files += sorted(glob.glob(os.path.join(af_root, d, '*.af')))

    lines = []
    npass = nfail = 0
    fails = []
    for f in files:
        rel = os.path.relpath(f, af_root)
        try:
            ok, output = run_tests_for_af(rel)
        except Exception as e:
            ok, output = False, [f'Exception: {e}']
        lines.append(f"{'='*70}\n{rel}: {'PASS' if ok else 'FAIL'}")
        lines.extend(compact(output))
        if ok:
            npass += 1
        else:
            nfail += 1
            fails.append(rel)

    lines.append('=' * 70)
    lines.append(f"FLEET: {npass} PASS, {nfail} FAIL over {len(files)} files")
    if fails:
        lines.append('FAILING: ' + ', '.join(fails))
    out = '\n'.join(lines)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(out)
        print(f"wrote {args.out}")
    else:
        print(out)


if __name__ == '__main__':
    main()