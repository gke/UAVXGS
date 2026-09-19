"""Shared PID-tuning critic package.

Stage 1a of the live auto-tuning roadmap: the metric definitions and the
pass/fail criteria used by the Python-side PID critique, extracted from
``tests/test_pid_sim.py`` so the FC (``trace.c`` captures) and the sim critic
measure a step response identically and judge it against identical limits.

Layout
  metrics.py   raw measurement over time series (times, values) -> metrics.
               No criteria knowledge. FC port lives here (in C).
  criteria.py  pass/fail thresholds (CRITERIA tables), verdict helpers, and
               the ANSI console report renderers (critique*/recommend/check).
               No measurement knowledge. Criteria maps 1:1 to C constants.

The test script retains ownership of the sim physics and the airframe data;
it imports the measurement, verdict, and report logic from here.
"""

from .criteria import (
    Criteria,
    DisturbanceCriteria,
    CRITERIA,
    FW_CRITERIA,
    DIST_CRITERIA,
    AH_CRITERIA_MR,
    AH_CRITERIA_FW,
    NAV_CRITERIA_MR,
    NAV_CRITERIA_FW,
    Verdict,
    step_verdicts,
    disturbance_verdicts,
    check,
    critique,
    critique_linear,
    critique_disturbance,
    recommend,
    G,
    Y,
    R,
    B,
    N,
)
from .metrics import (
    StepMetrics,
    DisturbanceMetrics,
    STEP_NOT_REACHED_S,
    count_zero_crossings,
    step_metrics_window,
    step_metrics_persist,
    disturbance_metrics,
)

__all__ = [
    "Criteria",
    "DisturbanceCriteria",
    "CRITERIA",
    "FW_CRITERIA",
    "DIST_CRITERIA",
    "AH_CRITERIA_MR",
    "AH_CRITERIA_FW",
    "NAV_CRITERIA_MR",
    "NAV_CRITERIA_FW",
    "Verdict",
    "step_verdicts",
    "disturbance_verdicts",
    "check",
    "critique",
    "critique_linear",
    "critique_disturbance",
    "recommend",
    "G",
    "Y",
    "R",
    "B",
    "N",
    "StepMetrics",
    "DisturbanceMetrics",
    "STEP_NOT_REACHED_S",
    "count_zero_crossings",
    "step_metrics_window",
    "step_metrics_persist",
    "disturbance_metrics",
]