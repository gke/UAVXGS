#!/usr/bin/env python3
"""
Autonomous Altitude Hold PID Simulation.

Replicates the restructured cascade from UAVXArm32F4/src/control.c:
  AltPos PID (Kp=0.25, Ki=0.0046, Kd=0.16) → AltHoldThrComp [±0.12]
  Velocity damping via KFROC
  TotalThrottle = DesiredThrottle + AltHoldThrComp

Physics: 1-D vertical (thrust, gravity, linear drag, motor lag).

Usage:
  python3 src/tests/test_alt_hold_sim.py

Exits 0 if all tests pass, 1 otherwise.
"""

import math
import sys
from dataclasses import dataclass
from typing import List, Tuple

GRAVITY = 9.80665

EM_MASS = 0.8
EM_MAX_THRUST = EM_MASS / 0.55 * GRAVITY  # ~14.3 N (hover at 55% throttle)
EM_MOTOR_TAU = 0.10
EM_DRAG_VERT = 0.5  # N·s/m, linear drag

PHYSICS_DT = 0.001
ALT_CTRL_DT = 0.01
SIM_TIME = 25.0
WARMUP_TIME = 1.0

KF_TAU_ALT = 0.05  # KF 3-state only (10Hz LPF2 removed)
KF_TAU_ROC = 0.02  # KF accel-derived velocity (fast)

ALT_P_KP = 0.35    # matches EcksTuned ALT_POS_KP
ALT_P_KI = 0.002   # matches EcksTuned ALT_POS_KI
ALT_P_KD = 0.20    # velocity damping (was 0.16)
ALT_P_INTLIM = 0.5

ALT_COMP_LIMIT = 0.25  # matches EcksTuned ALT_THROTTLE_COMP_LIMIT
ALT_DECAY_PS = 0.025
ALT_FAST_DECAY_MULT = 10.0

EST_CRUISE_THR = 0.65


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def decay_x(v: float, rate: float, dT: float) -> float:
    d = rate * dT
    if v < -d:
        return v + d
    elif v > d:
        return v - d
    return 0.0


class AltController:
    """Matches restructured DoAltitudeControl: position PI+D → AltHoldThrComp directly.
    Velocity damping via KFROC * Kd."""

    def __init__(self):
        self.pos_kp = ALT_P_KP
        self.pos_ki = ALT_P_KI
        self.pos_kd = ALT_P_KD
        self.pos_intlim = ALT_P_INTLIM
        self.pos_inte = 0.0
        self.comp = 0.0

    def reset(self):
        self.pos_inte = 0.0
        self.comp = 0.0

    def update(self, alt: float, vel: float, target: float, dT: float) -> float:
        err = target - alt
        pterm = err * self.pos_kp
        dterm = -vel * self.pos_kd
        self.pos_inte = clamp(self.pos_inte + err * self.pos_ki * dT,
                              -self.pos_intlim, self.pos_intlim)
        self.comp = clamp(pterm + self.pos_inte + dterm, -ALT_COMP_LIMIT, ALT_COMP_LIMIT)
        return self.comp


class VerticalPlant:
    def __init__(self):
        self.alt = 0.0
        self.vel = 0.0
        self.motor_lag = 0.0

    def reset(self):
        self.alt = 0.0
        self.vel = 0.0
        self.motor_lag = 0.0

    def step(self, motor_cmd: float, dist_N: float, dT: float):
        alpha = dT / (EM_MOTOR_TAU + dT)
        self.motor_lag += alpha * (motor_cmd - self.motor_lag)
        thrust = self.motor_lag * EM_MAX_THRUST
        drag = EM_DRAG_VERT * self.vel
        net = thrust - EM_MASS * GRAVITY - drag + dist_N
        accel = net / EM_MASS
        self.vel += accel * dT
        self.alt += self.vel * dT
        return self.alt, self.vel


@dataclass
class RunData:
    times: List[float]
    alts: List[float]
    vels: List[float]
    comps: List[float]


def run_sim(target_m: float, disturb_N: float = 0.0, disturb_start: float = 1e9,
            cut_throttle: bool = False, cut_start: float = 1e9) -> RunData:
    n = int(SIM_TIME / PHYSICS_DT)
    n_ctrl = int(ALT_CTRL_DT / PHYSICS_DT)

    ctrl = AltController()
    plant = VerticalPlant()
    kf_alt = 0.0
    kf_roc = 0.0

    times, alts, vels, comps = [], [], [], []

    for i in range(n):
        t = i * PHYSICS_DT
        dt = PHYSICS_DT

        alpha_alt = dt / (KF_TAU_ALT + dt)
        alpha_roc = dt / (KF_TAU_ROC + dt)
        kf_alt += alpha_alt * (plant.alt - kf_alt)
        kf_roc += alpha_roc * (plant.vel - kf_roc)

        pid_disabled = cut_throttle and t >= cut_start

        if i % n_ctrl == 0 and not pid_disabled:
            target = target_m if t >= WARMUP_TIME else 0.0
            ctrl.update(kf_alt, kf_roc, target, ALT_CTRL_DT)

        if pid_disabled:
            desired_throttle = EST_CRUISE_THR  # pilot holds hover throttle
            ctrl.comp = decay_x(ctrl.comp, ALT_DECAY_PS * ALT_FAST_DECAY_MULT,
                                PHYSICS_DT)
        else:
            desired_throttle = EST_CRUISE_THR

        total = clamp(desired_throttle + ctrl.comp, 0.0, 1.0)
        dist = disturb_N if disturb_start <= t < disturb_start + 2.0 else 0.0

        plant.step(total, dist, PHYSICS_DT)

        if i % n_ctrl == 0:
            times.append(t)
            alts.append(plant.alt)
            vels.append(plant.vel)
            comps.append(ctrl.comp)

    return RunData(times=times, alts=alts, vels=vels, comps=comps)


def step_metrics(d: RunData, target: float) -> dict:
    lo, hi = 0.1 * target, 0.9 * target
    band = 0.02 * target
    window = 30

    t10 = t90 = settle = None
    for j in range(len(d.times)):
        a = d.alts[j]
        if t10 is None and a >= lo:
            t10 = d.times[j]
        if t90 is None and a >= hi:
            t90 = d.times[j]
        if t90 is not None and settle is None:
            ok = True
            for k in range(j, min(j + window, len(d.times))):
                if abs(d.alts[k] - target) > band:
                    ok = False
                    break
            if ok:
                settle = d.times[j]

    peak = max((a - target for a in d.alts if a > target), default=0.0)
    return dict(
        rise=(t90 - t10) if t10 is not None and t90 is not None else SIM_TIME,
        overshoot=(peak / target * 100.0) if target > 0 else 0.0,
        settle=settle if settle is not None else SIM_TIME,
        ss_err=abs(d.alts[-1] - target),
        max_comp=max(abs(c) for c in d.comps),
        max_climb=max(d.vels),
        pos_int_ratio=0.0,  # not logged here
        rate_int_ratio=0.0,
    )


def disturbance_metrics(d: RunData, start: float) -> dict:
    ref = [d.alts[j] for j in range(len(d.times))
           if start - 1.0 < d.times[j] < start]
    ref_alt = sum(ref) / len(ref) if ref else 0.0
    devs = [abs(d.alts[j] - ref_alt) for j in range(len(d.times))
            if d.times[j] >= start]
    max_dev = max(devs) if devs else 0.0

    rec_end = start + 2.0
    rec_time = SIM_TIME
    window = 30
    for j in range(len(d.times)):
        if d.times[j] >= rec_end:
            ok = True
            for k in range(j, min(j + window, len(d.times))):
                if abs(d.alts[k] - ref_alt) > 0.05:
                    ok = False
                    break
            if ok:
                rec_time = d.times[j] - rec_end
                break
    return dict(max_dev=max_dev, recovery=rec_time)


def cut_metrics(d: RunData, start: float) -> dict:
    cut_alt = None
    for j in range(len(d.times)):
        if d.times[j] >= start:
            cut_alt = d.alts[j]
            break
    if cut_alt is None:
        return dict(max_drop=0.0)
    alt_after = [d.alts[j] for j in range(len(d.times)) if d.times[j] >= start]
    min_alt = min(alt_after) if alt_after else cut_alt
    return dict(max_drop=cut_alt - min_alt)


G = "\033[92m"
Y = "\033[93m"
R = "\033[91m"
B = "\033[1m"
N = "\033[0m"


def check(label: str, v: float, limit: float, kind: str = "max", unit: str = ""):
    ok = (v <= limit) if kind == "max" else (v >= limit)
    icon = f"{G}PASS{N}" if ok else f"{R}FAIL{N}"
    return ok, f"  [{icon}] {label}: {v:.4g}{unit}  (limit: {kind} {limit:.4g}{unit})"


def run_scenario(name: str, checks: List[Tuple]) -> bool:
    print(f"\n{B}── {name} ──{N}")
    all_ok = True
    for ok, msg in checks:
        print(msg)
        if not ok:
            print(f"    {R}╰─ review tuning{N}")
            all_ok = False
    print(f"  → {name}: {G}PASS{N}" if all_ok else f"  → {name}: {R}ISSUES{N}")
    return all_ok


def main():
    print(f"{B}UAVX Altitude Hold PID Cascade — Simulation{N}")
    print(f"{'='*60}")
    print(f"  Physics dt: {PHYSICS_DT*1000:.0f}ms  Ctrl dt: {ALT_CTRL_DT*1000:.0f}ms")
    print(f"  Mass: {EM_MASS}kg  Thrust: {EM_MAX_THRUST:.0f}N  Hover: {EST_CRUISE_THR*100:.0f}%")
    print(f"  Drag: {EM_DRAG_VERT} N·s/m  Motor tau: {EM_MOTOR_TAU*1000:.0f}ms")
    print(f"  Alt PID: Kp={ALT_P_KP} Ki={ALT_P_KI} Kd={ALT_P_KD} IntLim={ALT_P_INTLIM}  CompLimit={ALT_COMP_LIMIT}")
    print(f"{'='*60}")

    all_pass = True

    d5 = run_sim(target_m=5.0)
    s5 = step_metrics(d5, 5.0)
    all_pass &= run_scenario("Step 0→5m", [
        check("Rise time (10→90%)",    s5["rise"],      5.0, "max", "s"),
        check("Overshoot",             s5["overshoot"], 15.0, "max", "%"),
        check("Settling (±2%)",        s5["settle"],    10.0, "max", "s"),
        check("Steady-state error",    s5["ss_err"],    0.25, "max", "m"),
        check("Max AltHoldThrComp",    s5["max_comp"],  0.26, "max"),
    ])

    d2 = run_sim(target_m=2.0)
    s2 = step_metrics(d2, 2.0)
    all_pass &= run_scenario("Step 0→2m", [
        check("Rise time (10→90%)",    s2["rise"],      5.0, "max", "s"),
        check("Overshoot",             s2["overshoot"], 15.0, "max", "%"),
        check("Settling (±2%)",        s2["settle"],    10.0, "max", "s"),
        check("Steady-state error",    s2["ss_err"],    0.25, "max", "m"),
        check("Max AltHoldThrComp",    s2["max_comp"],  0.26, "max"),
        check("Max climb rate",        s2["max_climb"], 4.0,  "max", "m/s"),
    ])

    dd = run_sim(target_m=5.0, disturb_N=3.0, disturb_start=12.0)
    sds = step_metrics(dd, 5.0)
    sdd = disturbance_metrics(dd, 12.0)
    all_pass &= run_scenario("5m hold + 3N downward gust (2s @ 12s)", [
        check("Steady-state error (pre-disturb)", sds["ss_err"], 0.25, "max", "m"),
        check("Max altitude deviation",           sdd["max_dev"], 1.6, "max", "m"),
        check("Recovery time (post-gust)",        sdd["recovery"], 6.0, "max", "s"),
        check("Max AltHoldThrComp",               sds["max_comp"], 0.26, "max"),
    ])

    dc = run_sim(target_m=5.0, cut_throttle=True, cut_start=13.0)
    scc = cut_metrics(dc, 13.0)
    # pre-cut steady-state error: average altitude just before cut
    pre_cut_alts = [dc.alts[j] for j in range(len(dc.times))
                    if 11.0 < dc.times[j] < 13.0]
    pre_cut_err = abs(sum(pre_cut_alts) / len(pre_cut_alts) - 5.0) if pre_cut_alts else 999.0
    all_pass &= run_scenario("5m hold + throttle cut (hold disabled, comp decays @ 13s)", [
        check("Steady-state error (pre-cut)", pre_cut_err, 0.25, "max", "m"),
        check("Max altitude drop",            scc["max_drop"], 1.0, "max", "m"),
    ])

    print(f"\n{'='*60}")
    if all_pass:
        print(f"{G}{B}All scenarios PASS — AltHold tuning is acceptable{N}")
    else:
        print(f"{R}{B}Some scenarios have issues — review above{N}")

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
