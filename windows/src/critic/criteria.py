"""Shared PID-tuning critic: pass/fail criteria.

Stage 1a of the live auto-tuning roadmap: the criteria tables and verdict
evaluation used by the Python-side PID critique, lifted out of
``tests/test_pid_sim.py`` so the FC, sim, and GCS tuner all judge a measured
response against the SAME thresholds.

Pure data + verdict helpers. ``criteria`` never measures anything (see
:mod:`critic.metrics`); it only compares measured values against limits.

Verdict convention: every row is ``value <= limit`` for ``kind == "max"``,
``value >= limit`` for ``kind == "min"``. A step passes when every row
passes. The C port maps 1:1: the tables become ``const`` tables, the
evaluators become ``<=``/``>=`` comparisons per row.
"""

import math
from dataclasses import dataclass

RAD_TO_DEG = 180.0 / math.pi

# A single judged row: display value (already converted to the compared
# units), the limit, the comparison kind, and the unit string for display.
class Verdict:
    __slots__ = ("label", "value", "limit", "kind", "unit", "ok")

    def __init__(self, label, value, limit, kind, unit):
        self.label = label
        self.value = value
        self.limit = limit
        self.kind = kind
        self.unit = unit
        self.ok = (value <= limit) if kind == "max" else (value >= limit)


@dataclass
class Criteria:
    max_rise_time_s: float
    max_overshoot_pct: float
    max_settling_time_s: float
    max_ss_error_deg: float
    max_int_ratio: float
    min_rate_ratio: float
    max_oscillations: int


@dataclass
class DisturbanceCriteria:
    max_peak_deviation_deg: float
    max_settling_time_s: float
    max_integrated_error: float


# Attitude step criteria, per axis. MR and FW plants differ enough that a
# single table cannot fit both (MR: fast attitude authority, small overshoot;
# FW: control-surface authority, larger structural tolerances).
#
# Roll/Pitch rise re-baselined 1.0 -> 2.0s (2026-09-12): with the clean
# at-cap derived angle gains the MR angle loop is first-order with tau=1/QKp,
# so rise ~= 2.2/QKp. Low-authority reference frames (45° max at 80/60°/s
# caps -> derived QKp 1.825/1.369) have a ~1.20/1.61s angle-loop floor — the
# old 1.0s limit was calibrated against the pre-derived (over-demanding ~3x
# cap) stored gains and is physically unreachable under clean sizing. 2.0s
# keeps the test binding (agile frames still land ~0.3s) while admitting the
 # derived-cascade floor.
# 2026-09-12: MC Roll/Pitch settle 2.5 -> 4.0s (Greg). The generic frames reach
# rate-Kp band-top 0.5 before the 10% overshoot band binds; their settle
# (~3.0–4.2s, low-authority generic descriptors) is a response-speed/overshoot
# trade at the band ceiling, not a low-gain error. 4.0s admits the band-top
# generic settle wall while staying binding (agile frames still ~0.6–2.3s).
CRITERIA = {
    "Roll": Criteria(2.0, 10.0, 4.0, 1.0, 1.0, 0.3, 3),
    "Pitch": Criteria(2.0, 10.0, 4.0, 1.0, 1.0, 0.3, 3),
    "Yaw": Criteria(3.0, 15.0, 5.0, 2.0, 1.0, 0.2, 4),
}

FW_CRITERIA = {
    "Roll": Criteria(4.0, 25.0, 8.0, 5.0, 1.0, 0.05, 5),
    "Pitch": Criteria(5.0, 30.0, 12.0, 7.5, 1.5, 0.05, 6),
    "Yaw": Criteria(5.0, 25.0, 10.0, 5.0, 1.0, 0.03, 5),
}

DIST_CRITERIA = {
    "Roll": DisturbanceCriteria(15.0, 1.5, 10.0),
    "Pitch": DisturbanceCriteria(12.0, 1.5, 8.0),
    "Yaw": DisturbanceCriteria(12.0, 2.0, 10.0),
}

# AH/Nav criteria — more relaxed than attitude (these are outer loops)
AH_CRITERIA_MR = Criteria(2.5, 20.0, 5.0, 1.0, 1.0, 0.3, 3)   # 5m step; rise 2.5s = 3:1-T/W physics floor at FC comp ceiling (Aug 29)
AH_CRITERIA_FW = Criteria(6.0, 15.0, 8.0, 2.0, 1.0, 0.3, 3)   # rise 6s/settle 8s — FW climb-tau physics floor ~4.4s
NAV_CRITERIA_MR = Criteria(2.0, 20.0, 5.0, 1.0, 1.0, 0.3, 3)
NAV_CRITERIA_FW = Criteria(3.0, 15.0, 5.0, 2.0, 1.0, 0.3, 3)


def step_verdicts(m, c, linear=False, linear_units="m"):
    """Verdict rows for a StepMetrics against a Criteria.

    ``linear=True`` for the altitude/navigation meters-based critiques: the
    final error is compared in raw units (``linear_units``) instead of
    degrees, and the rate-headroom and oscillation rows are omitted
    (those sims do not command a rate).
    """
    int_ratio = m.max_integrator / m.intlim if m.intlim > 0 else 0.0
    rows = [
        Verdict("Rise time (10→90%)", m.rise_time_s,
                c.max_rise_time_s, "max", "s"),
        Verdict("Overshoot", m.overshoot_pct,
                c.max_overshoot_pct, "max", "%"),
        Verdict("Settling time (±2%)", m.settling_time_s,
                c.max_settling_time_s, "max", "s"),
        Verdict("Final error",
                m.steady_state_error if linear else m.steady_state_error * RAD_TO_DEG,
                c.max_ss_error_deg, "max", linear_units if linear else "°"),
        Verdict("Integrator utilization", int_ratio,
                c.max_int_ratio, "max", ""),
    ]
    if not linear:
        rate_ratio = (m.max_rate / (abs(m.setpoint) * 2)
                      if m.setpoint != 0 else 1.0)
        rows.append(Verdict("Rate headroom (peak/2×sp)", rate_ratio,
                            c.min_rate_ratio, "min", ""))
        rows.append(Verdict("Zero crossings (oscillations)",
                            float(m.n_oscillations),
                            float(c.max_oscillations), "max", ""))
    return rows


def disturbance_verdicts(m, c):
    """Verdict rows for a DisturbanceMetrics against a DisturbanceCriteria."""
    return [
        Verdict("Peak rate deviation", m.peak_deviation_rad * RAD_TO_DEG,
                c.max_peak_deviation_deg, "max", "°/s"),
        Verdict("Settling after gust", m.settling_time_s,
                c.max_settling_time_s, "max", "s"),
        Verdict("Integrated error", m.integrated_error * RAD_TO_DEG,
                c.max_integrated_error, "max", "°"),
    ]


# Terminal colour constants used by the console report renderers below.
# Exported so any consumer (sim, GCS tuner) colours its output the same way.
G = "\033[92m"
Y = "\033[93m"
R = "\033[91m"
B = "\033[1m"
N = "\033[0m"


def check(label, v, limit, kind="max", unit=""):
    """Single pass/fail comparison row; returns ``(ok, formatted_line)``."""
    ok = (v <= limit) if kind == "max" else (v >= limit)
    icon = f"{G}PASS{N}" if ok else f"{R}FAIL{N}"
    return ok, f"  [{icon}] {label}: {v:.4g}{unit}  (limit: {kind} {limit:.4g}{unit})"


def critique(m, c, name):
    """Console critique for an angular (attitude) step response."""
    lines = []
    lines.append(f"\n{B}── {name} @ {m.setpoint * RAD_TO_DEG:.0f}° step ──{N}")
    lines.append(f"     Final: {m.final_angle * RAD_TO_DEG:.1f}°   Peak: {m.peak_angle * RAD_TO_DEG:.1f}°   Max rate: {m.max_rate * RAD_TO_DEG:.0f}°/s")
    results = []
    for row in step_verdicts(m, c):
        results.append(check(row.label, row.value, row.limit, row.kind, row.unit))
    for ok, msg in results:
        lines.append(msg)
        if not ok:
            lines.append(f"    {R}╰─ recommend tuning{N}")
    all_pass = all(r[0] for r in results)
    lines.append(f"  → {name}: {G}PASS{N}" if all_pass else f"  → {name}: {R}ISSUES{N}")
    return lines, all_pass


def critique_linear(m, c, name, units="m"):
    """Console critique for a linear (meters) step — AH and Nav sims."""
    lines = []
    lines.append(f"\n{B}── {name} @ {m.setpoint:.1f}{units} step ──{N}")
    lines.append(f"     Final: {m.final_angle:.2f}{units}   Peak: {m.peak_angle:.2f}{units}")
    results = []
    for row in step_verdicts(m, c, linear=True, linear_units=units):
        results.append(check(row.label, row.value, row.limit, row.kind, row.unit))
    for ok, msg in results:
        lines.append(msg)
        if not ok:
            lines.append(f"    {R}╰─ recommend tuning{N}")
    all_pass = all(r[0] for r in results)
    lines.append(f"  → {name}: {G}PASS{N}" if all_pass else f"  → {name}: {R}ISSUES{N}")
    return lines, all_pass


def critique_disturbance(m, c, name):
    """Console critique for a rate disturbance (gust) rejection response."""
    lines = []
    lines.append(f"\n{B}── {name} disturbance rejection @ gust={c.max_peak_deviation_deg:.0f}°/s limit ──{N}")
    lines.append(f"     Peak deviation: {m.peak_deviation_rad * RAD_TO_DEG:.2f}°/s   Settle: {m.settling_time_s:.3f}s   IE: {m.integrated_error * RAD_TO_DEG:.3f}°")
    results = []
    for row in disturbance_verdicts(m, c):
        results.append(check(row.label, row.value, row.limit, row.kind, row.unit))
    for ok, msg in results:
        lines.append(msg)
        if not ok:
            lines.append(f"    {R}╰─ recommend tuning{N}")
    all_pass = all(r[0] for r in results)
    lines.append(f"  → {name} disturbance rejection: {G}PASS{N}" if all_pass else f"  → {name}: {R}ISSUES{N}")
    return lines, all_pass


def recommend(metrics, is_fw=False):
    """Broad-stroke tuning recommendations from a mapping of StepMetrics."""
    lines = [f"\n{B}Recommendations{N}"]
    any_rec = False
    for name, m in metrics.items():
        os = m.overshoot_pct
        st = m.settling_time_s
        if os > (15 if is_fw else 8):
            lines.append(f"  {Y}• {name}: reduce AngleKp or increase RateKp to cut overshoot ({os:.0f}%){N}")
            any_rec = True
        if st > (5 if is_fw else 2.5):
            lines.append(f"  {Y}• {name}: response slow ({st:.1f}s settle) — consider higher gains{N}")
            any_rec = True
    if not any_rec:
        lines.append(f"  {G}No issues detected — tuning is appropriate for this airframe.{N}")
    return lines