#!/usr/bin/env python3
"""
BEFORE vs AFTER tabulation of the FW tuning regime on generic/Shadow.af.

The FW plant's deflection→rate authority was re-baselined 2026-09-10 against
external flying-wing literature (see wiki/Session_Report_ShadowAuthorityTuning_Sep10.md):

  BEFORE (uncalibrated assumed scheme):
      CL_D_AIL 0.025 (5-13x below the external 0.12-0.32/rad band)
      pitch_damp -8.0 (quadratic; over-damped ~40x at operating rates -> the
                       "needs 40x authority" red herring)
      pitch_damp_lin 0.02 (basically no linear term)
  AFTER (re-based):
      CL_D_AIL 0.18 (mid of external band; Grillo flight-ID 0.173)
      pitch_damp -0.04 (quadratic residual only)
      pitch_damp_lin 0.18 (PLANK Cmq ~ -2.5 wing-only, linear-in-rate)

Both panels run the SAME REF4 quick-tune gain set (angle P=4 / rate P=0.1)
that generic/Shadow.af carries, over an FW_AUTHORITY_SCALE sweep.  The scale
is a pure multiplier on CL_D_AIL/CM_D_ELE/CN_D_RUD; 1x = the descriptor as-is.
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_pid_sim as sim

AF = "generic/Shadow.af"
SCALES = [1.0, 1.5, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0, 20.0, 30.0, 40.0]

BEFORE = {"CL_D_AIL": 0.025, "pitch_damp": -8.0, "pitch_damp_lin": 0.02}
AFTER = {"CL_D_AIL": 0.18, "pitch_damp": -0.04, "pitch_damp_lin": 0.18}


def run_panel(label, coeffs):
    desc = sim.get_fw_descriptor(AF)
    orig = {}
    for k, v in coeffs.items():
        orig[k] = desc[k]
        desc[k] = v
    print(f"\n{'='*76}\n  {label}\n{'='*76}")
    print(f"  {'scale':>6} {'verdict':>5}   headline failures")
    print(f"  {'-'*70}")
    rows = []
    for s in SCALES:
        sim.FW_AUTHORITY_SCALE = s
        ok, out = sim.run_tests_for_af(AF)
        vex = [ln.strip() for ln in out if "FAIL" in ln]
        fails = []
        for ln in vex:
            for code in ("\x1b[91m", "\x1b[31m", "\x1b[33m", "\x1b[32m",
                         "\x1b[34m", "\x1b[0m", "\x1b[1m"):
                ln = ln.replace(code, "")
            fails.append(ln.strip())
        head = "; ".join(fails[:4])
        rows.append((s, "PASS" if ok else "FAIL", head))
        print(f"  {s:>6} {('PASS' if ok else 'FAIL'):>5}   {head[:60]}")
    for k, v in coeffs.items():
        desc[k] = orig[k]
    sim.FW_AUTHORITY_SCALE = 1.0
    return rows


run_panel("BEFORE  — uncalibrated scheme  (CL_D_AIL 0.025, pitch_damp -8.0)", BEFORE)
run_panel("AFTER   — re-based plant       (CL_D_AIL 0.18,  pitch_damp -0.04, lin 0.18)", AFTER)