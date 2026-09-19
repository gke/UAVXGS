#!/usr/bin/env python3
"""
Critique an airframe's tuned gains at reduced control authority.

The FW plant's deflection→rate gain is uncalibrated.  This driver re-runs the
full critique (step + gust + alt-hold + nav + free-flight) with the control
torque coefficients scaled down by 1×, 2× and 4×, so the field-tuned gains can
be judged against the authority uncertainty AND the sim can be calibrated: the
scale where a "feels right" gain set starts/ stops passing reveals the real
surface effectiveness.

Usage:
  python3 src/tests/critique_authority.py [generic/Shadow.af [1.0 0.5 0.25]]
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def summarize(pass_ok: bool, out: list) -> list:
    """Extract the verdict lines and headline failures from a critique run."""
    lines = []
    verdicts = [ln.strip() for ln in out if "[PASS]" in ln or "[FAIL]" in ln]
    for ln in out:
        if "[PASS]" in ln or "[FAIL]" in ln:
            lines.append(ln.replace("\u001b[", "ESC[").strip())
    fails = [ln for ln in verdicts if "[FAIL]" in ln]
    return lines, fails


def main():
    af_filename = sys.argv[1] if len(sys.argv) > 1 else "generic/Shadow.af"
    scales = [float(x) for x in sys.argv[2:]] if len(sys.argv) > 2 else [1.0, 0.5, 0.25]

    import test_pid_sim as sim

    print(f"{'='*70}")
    print(f"Authority critique: {af_filename}  (scales {scales})")
    print(f"{'='*70}")

    for s in scales:
        sim.FW_AUTHORITY_SCALE = s
        ok, output = sim.run_tests_for_af(af_filename)
        print(f"\n{'#'*70}")
        print(f"# AUTHORITY SCALE {s:g} (deflection→rate ÷{1.0/s:g})  "
              f"→ {'PASS' if ok else 'FAIL'}")
        print(f"{'#'*70}")
        verdict_lines = [ln for ln in output
                         if "[PASS]" in ln or "[FAIL]" in ln or "fail" in ln.lower()]
        # de-color for log safety
        shown = []
        for ln in verdict_lines:
            clean = (ln.replace("\x1b[32m", "").replace("\x1b[31m", "")
                      .replace("\x1b[33m", "").replace("\x1b[34m", "")
                      .replace("\x1b[0m", "").replace("\x1b[1m", ""))
            shown.append(clean.strip())
        for ln in shown:
            print(ln)

    sim.FW_AUTHORITY_SCALE = 1.0


if __name__ == "__main__":
    main()