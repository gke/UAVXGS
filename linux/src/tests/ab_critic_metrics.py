#!/usr/bin/env python3
"""Regression guard: critic/metrirs.py reproduces the original inline metric
math from test_pid_sim.py EXACTLY (Aug 30 pre-refactor).

The ORIGINAL_* functions below are copied verbatim from the pre-refactor
test_pid_sim.py sources (the blocks that were replaced). We feed the same
synthetic series to both the original inline algorithm and the shared
critic.metrics extractor and require identical results. If any of these
drift, the FC trace port (which will share these definitions) inherits the
drift too — that is what this guard exists to catch.

Run: python3 src/tests/ab_critic_metrics.py (exit 0 = equivalent).
"""

import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

SIM_TIME = 8.0
CONTROL_DT = 0.001

from critic.metrics import (
    disturbance_metrics,
    count_zero_crossings,
    step_metrics_window,
    step_metrics_persist,
)

failures = 0


def check(name, got, want):
    global failures
    ok = got == want
    if not ok:
        failures += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: got={got!r} want={want!r}")


def round4(x):
    return round(x, 4)


def round6(x):
    return round(x, 6)


# ── ORIGINAL simulate_axis drop-in metric block (post-loop) ────────────────
def original_step_full(times, angles, setpoint, n_warmup=100, decimate=5,
                       SIM_TIME=SIM_TIME):
    final_angle = angles[-1] if angles else 0.0
    ss_error = abs(final_angle - setpoint)

    rise_time = SIM_TIME
    overshoot = 0.0
    settling = SIM_TIME

    if setpoint > 0.01:
        lo_thresh = 0.1 * setpoint
        hi_thresh = 0.9 * setpoint
        settle_band = 0.02 * setpoint
        t10 = t90 = None
        settled = False
        window = min(200, len(times))

        for j in range(len(times)):
            a = abs(angles[j])
            if t10 is None and a >= lo_thresh:
                t10 = times[j]
            if t90 is None and a >= hi_thresh:
                t90 = times[j]
            if t90 is not None and not settled:
                ok = True
                for k in range(j, min(j + window, len(times))):
                    if abs(angles[k] - setpoint) > settle_band:
                        ok = False
                        break
                if ok:
                    settling = times[j]
                    settled = True
                    break

        if t10 is not None and t90 is not None:
            rise_time = t90 - t10

        peak_above = max((a - setpoint for a in angles if a > setpoint),
                         default=0.0)
        if setpoint > 0:
            overshoot = (peak_above / setpoint) * 100.0

    return (round4(rise_time), round4(settling), round(overshoot, 1))


def original_alt_hold(times, alts, step_m, dT, SIM_TIME=SIM_TIME):
    rise_time_s = SIM_TIME
    settled = False
    settle_time = SIM_TIME
    for j, t in enumerate(times):
        alt = alts[j]
        if rise_time_s == SIM_TIME and alt >= 0.9 * step_m:
            rise_time_s = t
        if t > 1.0:
            if abs(alt - step_m) < 0.02 * abs(step_m):
                if not settled:
                    settle_time = t
                    settled = True
            else:
                settled = False
                settle_time = t + dT
    ss_err = abs(alts[-1] - step_m)
    peak = max(alts)
    overshoot = max(0, (peak - step_m) / step_m * 100.0) if step_m > 0 else 0.0
    return (round4(rise_time_s), round(overshoot, 1),
            round4(settle_time if settle_time < SIM_TIME else SIM_TIME),
            round4(ss_err))


# ── ORIGINAL disturbance settle/IE block ──────────────────────────────────
def original_disturbance(rates_full, times_full, gust_start, gust_dur, dT):
    peak_dev = max(abs(r) for r in rates_full)
    settle_band = peak_dev * 0.05 + 1e-6
    n = len(rates_full)
    settle_time = SIM_TIME
    after_gust = int((gust_start + gust_dur) / dT)
    settled = False
    for i in range(after_gust, n):
        t = i * dT
        if abs(rates_full[i]) < settle_band:
            if not settled:
                settle_time = t
                settled = True
            break
        settled = False
    settle_rel = (settle_time - gust_start - gust_dur
                  if settle_time < SIM_TIME else SIM_TIME)
    total_ie = 0.0
    start_i = int(gust_start / dT)
    end_i = int((gust_start + gust_dur) / dT)
    for i in range(start_i, end_i * 2 + 1):
        if i < n:
            total_ie += abs(rates_full[i]) * dT
    return (round6(peak_dev), round4(settle_rel), round6(total_ie))


# ── ORIGINAL zero-crossing counter (from simulate_axis) ───────────────────
def original_zero_crossings(samples, start_i):
    n = 0
    crossed = False
    prev = samples[0]
    for i in range(max(start_i, 1), len(samples)):
        a = samples[i]
        if prev * a < 0 and not crossed:
            n += 1
            crossed = True
        elif prev * a >= 0:
            crossed = False
        prev = a
    return n


# ── Representative generated series ──────────────────────────────────────
def gen_damped_step(step, wn=2.0, zeta=0.6, dt=0.005, n=1600, phase=0.0):
    ts = [round(i * dt, 6) for i in range(n)]
    ys = []
    wd = math.sqrt(abs(1 - zeta * zeta))
    for t in ts:
        osc = math.exp(-zeta * wn * t)
        y = step * (1.0 - osc * (math.cos(wn * t * wd)
                                 + zeta / wd * math.sin(wn * t * wd)))
        if phase:
            y *= math.sin(t / 5.0 + phase)  # noise-ish phase wobble
        ys.append(y)
    return ts, ys


def gen_not_settled(step, k=0.6, dt=0.005, n=1600):
    ts = [round(i * dt, 6) for i in range(n)]
    ys = [step * (1.0 - math.exp(-0.15 * t)) * (1.0 + 0.9 * math.sin(0.6 * t))
          for t in ts]
    return ts, ys


def gen_overdamped_alt(step, rise_tau=1.2, dt=0.001, n=8000):
    ts = [round(i * dt, 6) for i in range(n)]
    ys = [step * (1.0 - math.exp(-t / rise_tau)) for t in ts]
    return ts, ys


def gen_overshoot_alt(step, dt=0.001, n=8000):
    ts = [round(i * dt, 6) for i in range(n)]
    ys = [step * (1.0 + 0.25 * math.exp(-1.4 * t) * math.sin(3.0 * t))
          for t in ts]
    return ts, ys


def gen_gust(gust_start, gust_dur, dt=0.001, n=8000, model_max=30.0):
    ts = [round(i * dt, 6) for i in range(n)]
    ys = []
    pulse = 20.0
    for t in ts:
        v = 0.5 * pulse * (1.0 - math.cos(2 * math.pi * (t - gust_start) / gust_dur)) \
            if gust_start <= t <= gust_start + gust_dur else 0.0
        r = -4.0 * math.exp(-(t - gust_start - gust_dur) / 0.8)
        ys.append(v + r)
    return ts, ys


print("== step_metrics_window vs original step metric (decimated series) ==")
for name, ts, ys, sp in [
    ("nominal damped step", *gen_damped_step(0.5), 0.5),
    ("overdamped step", *gen_damped_step(1.0, wn=1.0, zeta=1.05), 1.0),
    ("heavy overshoot", *gen_damped_step(0.7, wn=3.0, zeta=0.18), 0.7),
    ("never settles", *gen_not_settled(1.0), 1.0),
]:
    got = (round4(step_metrics_window(ts, ys, sp)[0]),
           round4(step_metrics_window(ts, ys, sp)[2]),
           round(step_metrics_window(ts, ys, sp)[1], 1))
    want = original_step_full(ts, ys, sp)
    check(f"{name}", got, want)

print("== step_metrics_persist vs original alt-hold metric (full-res) ==")
for name, ts, ys, sp in [
    ("alt: overdamped climb", *gen_overdamped_alt(5.0), 5.0),
    ("alt: overshooting 5m", *gen_overshoot_alt(5.0), 5.0),
]:
    got = step_metrics_persist(ts, ys, sp, dt=0.001)
    got = (round4(got[0]), round(got[1], 1),
           round4(got[2]), round4(abs(ys[-1] - sp)))
    want = original_alt_hold(ts, ys, sp, 0.001)
    check(f"{name}", got, want)

print("== disturbance_metrics vs original disturb metric ==")
for gs, gd in [(3.0, 1.5), (1.0, 0.5), (4.0, 2.5)]:
    ts, ys = gen_gust(gs, gd)
    got = disturbance_metrics(ts, ys, gs, gd, 0.001)
    got = (round6(got[0]), round4(got[1]), round6(got[2]))
    want = original_disturbance(ys, ts, gs, gd, 0.001)
    check(f"gust {gs}s +{gd}s", got, want)

print("== count_zero_crossings vs original counter ==")
sin_ts, sin_ys = gen_damped_step(0.5, wn=2.0, zeta=0.18)
got = count_zero_crossings(sin_ys, 301)
want = original_zero_crossings(sin_ys, 301)
check("underdamped sine (full-res)", got, want)
dec_ys = sin_ys[::10]
got = count_zero_crossings(dec_ys, 61)
want = original_zero_crossings(dec_ys, 61)
check("underdamped sine (decimated)", got, want)

print()
if failures:
    print(f"FAILURES: {failures}")
    sys.exit(1)
print("ALL EQUIVALENT")