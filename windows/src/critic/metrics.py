"""Shared PID-tuning critic: metric measurement.

Stage 1a of the live auto-tuning roadmap: the response-shape measurement
definitions used by the Python-side PID critique, lifted out of
``tests/test_pid_sim.py`` so the FC (``trace.c`` probe captures) and the sim
critic measure a step response IDENTICALLY.

Scope: pure measurement. Given ``(times, values)`` time series, produce the
quantities the criteria tables (@see :mod:`critic.criteria`) are judged
against. No criteria knowledge here, no rendering, no Qt, no sim imports.

FC-porting contract
  The FC port implements these three functions (plus ``count_zero_crossings``)
  on its ``eTraceRate`` capture rows: tick (ms) -> s is the time base, Rate
  (rad/s) is ``values`` for a rate-step, and the same thresholds below
  (10/90 % rise, ±2 % settle band, windowed settle) apply verbatim.

  ``not_reached_s`` is the sentinel for a phase that never completes within
  the measurement window. The sim's window is SIM_TIME = 8.0 s; an FC capture
  is ~2.7 s at 500 Hz, so the FC passes its own capture duration and a phase
  never reached reports exactly that. The sentinel is deliberately a
  parameter, because "did not reach" must mean "within MY window".
"""

from dataclasses import dataclass

# Sentinel for a rise/settle phase that never completes within the window.
# The sim's per-axis window is SIM_TIME = 8.0 s (see tests/test_pid_sim.py).
STEP_NOT_REACHED_S = 8.0


@dataclass
class StepMetrics:
    """Step-response metrics for one axis/loop.

    rise_time_s / overshoot_pct / settling_time_s come from the shared
    measurement functions; the remaining fields are plant diagnostics the
    sim keeps from its internal state (and the FC reads straight from the
    trace record): integrator utilization, peak/max rate, final angle.
    """
    axis_name: str = ""
    rise_time_s: float = 0.0
    overshoot_pct: float = 0.0
    settling_time_s: float = 0.0
    steady_state_error: float = 0.0
    max_integrator: float = 0.0
    intlim: float = 0.0
    max_rate: float = 0.0
    peak_angle: float = 0.0
    final_angle: float = 0.0
    setpoint: float = 0.0
    n_oscillations: int = 0


@dataclass
class DisturbanceMetrics:
    """Rate-disturbance-rejection metrics (torque gust on the rate loop)."""
    peak_deviation_rad: float = 0.0
    settling_time_s: float = 0.0
    integrated_error: float = 0.0
    steady_rate_error: float = 0.0


def count_zero_crossings(values, start_index):
    """Count sign zero-crossings of ``values`` from ``start_index`` onward.

    Mirrors the sim's inline oscillation counter: each contiguous same-sign
    run scores one crossing (consecutive same-sign samples do not re-score).
    The first pair inspected is ``(values[start_index-1], values[start_index])``.

    ``start_index`` is a SAMPLE index into the full-rate series - the caller
    passes the full-resolution angle history and the warmup cutoff expressed
    in full-rate samples (the sim: ``n_warmup * 3 + 1``).
    """
    if start_index < 1:
        start_index = 1
    if start_index >= len(values):
        return 0
    prev = values[start_index - 1]
    crossings = 0
    crossed = False
    for v in values[start_index:]:
        if prev * v < 0 and not crossed:
            crossings += 1
            crossed = True
        elif prev * v >= 0:
            crossed = False
        prev = v
    return crossings


def step_metrics_window(times, values, setpoint, not_reached_s=STEP_NOT_REACHED_S):
    """Rise / overshoot / settle over a step series (windowed settle).

    The definition used for angle-step tests (and the FC rate-step probe):

    - rise time: first 10 % cross to first 90 % cross of ``abs(setpoint)``.
    - settling:  first sample after the 90 % cross whose next ``window``
      samples (``window = min(200, len(times))``) all stay inside
      ``±2 %·abs(setpoint)``.
    - overshoot: max peak above ``setpoint`` (signed values) as % of it.

    Returns ``(rise_time_s, overshoot_pct, settling_time_s)`` RAW (no
    rounding) - callers round for display exactly as before. Phases that
    never complete report ``not_reached_s``.
    """
    rise_time = not_reached_s
    overshoot = 0.0
    settling = not_reached_s

    if setpoint > 0.01:
        lo_thresh = 0.1 * setpoint
        hi_thresh = 0.9 * setpoint
        settle_band = 0.02 * setpoint
        window = min(200, len(times))
        t10 = t90 = None
        settled = False

        for j in range(len(times)):
            a = abs(values[j])
            if t10 is None and a >= lo_thresh:
                t10 = times[j]
            if t90 is None and a >= hi_thresh:
                t90 = times[j]
            if t90 is not None and not settled:
                ok = True
                for k in range(j, min(j + window, len(times))):
                    if abs(values[k] - setpoint) > settle_band:
                        ok = False
                        break
                if ok:
                    settling = times[j]
                    settled = True
                    break

        if t10 is not None and t90 is not None:
            rise_time = t90 - t10

        peak_above = max((v - setpoint for v in values if v > setpoint),
                         default=0.0)
        if setpoint > 0:
            overshoot = (peak_above / setpoint) * 100.0

    return rise_time, overshoot, settling


def step_metrics_persist(times, values, setpoint, deadtime_s=1.0, dt=0.001,
                         not_reached_s=STEP_NOT_REACHED_S):
    """Rise / overshoot / settle over a step series (persist-until-end survey).

    The definition used by the altitude-hold and navigation (linear, meters)
    tests, and by the FC's future ``eTraceAltHold`` capture:

    - rise time: first sample with ``abs(value) >= 0.9·abs(setpoint)``.
    - settling:  the start of the FINAL contiguous run of samples that stay
      inside ``±2 %·abs(setpoint)``, considering only samples after
      ``deadtime_s``; if the trace ends outside the band, the last escape
      time plus one ``dt``. (Matches the incremental loop's settle_time
      bookkeeping exactly.)
    - overshoot: max peak above ``abs(setpoint)`` as % of it (clamped >= 0).

    ``deadtime_s=1.0`` mirrors the sims' ``if t > 1.0`` gate (ignores the
    initial transient before the new setpoint has had time to register).

    Returns RAW ``(rise_time_s, overshoot_pct, settling_time_s)``.
    """
    rise_time = not_reached_s
    for i in range(len(times)):
        if abs(values[i]) >= 0.9 * abs(setpoint):
            rise_time = times[i]
            break

    peak = max((abs(v) for v in values), default=0.0)
    if setpoint:
        overshoot = max(0.0, (peak - abs(setpoint)) / abs(setpoint) * 100.0)
    else:
        overshoot = 0.0

    settle_band = 0.02 * abs(setpoint)
    settle_time = not_reached_s
    settled = False
    for i in range(len(times)):
        if times[i] > deadtime_s:
            if abs(values[i] - setpoint) < settle_band:
                if not settled:
                    settle_time = times[i]
                    settled = True
            else:
                settled = False
                settle_time = times[i] + dt
    settling = settle_time if settle_time < not_reached_s else not_reached_s

    return rise_time, overshoot, settling


def disturbance_metrics(times, rates, gust_start_s, gust_duration_s, dt,
                        not_reached_s=STEP_NOT_REACHED_S):
    """Rate-loop disturbance-rejection metrics from full-rate ``rates``.

    Matches the sim's gust test verbatim:

    - peak deviation: max ``abs(rate)`` over the whole run.
    - integrated error: sum ``abs(rate)·dt`` from gust start through twice
      the gust duration (the sim's ``<= gust_end_idx * 2`` window).
    - settling: first sample AFTER the gust has ended whose ``abs(rate)`` is
      inside ``±(peak·5 % + 1e-6)``, measured relative to gust end.
    - steady-state: ``abs(rate)`` at the final sample.

    Returns RAW ``(peak_dev_rad, settling_rel_s, integrated_error, steady_rate)``.
    """
    start_idx = int(gust_start_s / dt)
    end_idx = int((gust_start_s + gust_duration_s) / dt)

    peak_dev = max((abs(r) for r in rates), default=0.0)

    ie_end = min(len(rates), end_idx * 2 + 1)
    total_ie = sum(abs(rates[i]) * dt for i in range(start_idx, ie_end))

    band = peak_dev * 0.05 + 1e-6
    settle_time = not_reached_s
    for i in range(end_idx, len(rates)):
        if abs(rates[i]) < band:
            settle_time = times[i]
            break
    settle_rel = (settle_time - gust_start_s - gust_duration_s
                  if settle_time < not_reached_s else not_reached_s)

    steady = abs(rates[-1]) if rates else 0.0

    return peak_dev, settle_rel, total_ie, steady