#!/usr/bin/env python3
"""
Quaternion Attitude Controller Simulation — UAVXArmQ.

Replicates the quaternion cascade from UAVXArmQ/src/control.c:
  Quaternion error: qe = q_desired * conj(q_current)
  Rate command:     R.Desired = 2 * qe_vec * QuatGain
  Rate loop (PD):   R.PTerm = Error * R.Kp; DTerm = LPF(d(error)/dt) * Kd; Out = PTerm + DTerm

Physics (same as test_pid_sim.py, from emu.c):
  Motor lag:    1st-order lag (tau=80 ms)
  Torque:       kT * Out * arm_length → angular acceleration
  Drag:         opposes rotation (proportional to rate)
  Integration:  quaternion integration from body rates

Tests step response at both small (15°) and large (90°) angles to
verify the quaternion controller handles extreme attitudes where
Euler would gimbal-lock.

Usage:
  python3 src/tests/test_quat_sim.py
"""

import math
import sys
from dataclasses import dataclass, field
from typing import List, Tuple

RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0
GRAVITY = 9.80665

def DegreesToRadians(d):
    return d * DEG_TO_RAD

# ── Emulation constants (emu.h) — Racerstar BR2205 2300kv + 5x3x3, 800g AUW ──
EM_MASS = 0.8
EM_ARM_LEN = 0.17
EM_THR_CRUISE_STICK = 0.50          # hover at 50%
EM_MAX_THRUST = (EM_MASS / EM_THR_CRUISE_STICK) * GRAVITY   # 15.7 N
EM_MAX_YAW_THRUST = EM_MAX_THRUST * 0.015                   # emu.h
EM_MOTOR_TAU = 0.08
CONTROL_DT = 0.001
SIM_TIME = 6.0
WARMUP_TIME = 0.1

# Effective inertias matching emu.c InertiaR[a] = 12/(m*r^2) * (1/ratio)
#   I_eff = EM_MASS * EM_ARM_LEN^2 / 12 * ratio
# Ratios: I_roll : I_pitch : I_yaw = 1.0 : 1.5 : 2.5 (battery fore-aft)
_I0 = EM_MASS * EM_ARM_LEN**2 / 12.0
INERTIA_ROLL = _I0
INERTIA_PITCH = _I0 * 1.5
INERTIA_YAW = _I0 * 2.5

# Yaw torque fraction of EM_MAX_THRUST (matches EM_MAX_YAW_THRUST)
TORQUE_FRAC_YAW = 0.015

DTERM_LPF_HZ = 50.0
EM_DRAG_QUAD = 0.02                 # realistic aerodynamic angular drag (emu.c uses 2.0 — too high)

# ── Quaternion math ──

def q_mul(r, p, q):
    r[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3]
    r[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2]
    r[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1]
    r[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0]

def q_conj(r, q):
    r[0] = q[0]; r[1] = -q[1]; r[2] = -q[2]; r[3] = -q[3]

def q_error(qe, q_tar, q_cur):
    qc = [0.0]*4
    q_conj(qc, q_cur)
    q_mul(qe, q_tar, qc)
    if qe[0] < 0.0:
        qe[0] = -qe[0]; qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3]

def q_normalize(q):
    n = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    if n > 0:
        q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n

def euler_to_quat(roll, pitch, yaw):
    t0 = math.cos(yaw * 0.5); t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5); t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5); t5 = math.sin(pitch * 0.5)
    q = [0.0]*4
    q[0] = t0*t2*t4 + t1*t3*t5
    q[1] = t0*t3*t4 - t1*t2*t5
    q[2] = t0*t2*t5 + t1*t3*t4
    q[3] = t1*t2*t4 - t0*t3*t5
    q_normalize(q)
    return q

def quat_to_euler(q):
    # ZYX Euler extraction
    roll = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
    sinp = 2*(q[0]*q[2] - q[3]*q[1])
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))
    return roll, pitch, yaw

def q_integrate(q, rates, dT):
    # q_dot = 0.5 * q ⊗ [0, p, q, r]
    p, qq, r = rates
    qd = [0.0]*4
    qd[0] = 0.5 * (-q[1]*p - q[2]*qq - q[3]*r)
    qd[1] = 0.5 * ( q[0]*p - q[3]*qq + q[2]*r)
    qd[2] = 0.5 * ( q[3]*p + q[0]*qq - q[1]*r)
    qd[3] = 0.5 * (-q[2]*p + q[1]*qq + q[0]*r)
    q[0] += qd[0] * dT; q[1] += qd[1] * dT
    q[2] += qd[2] * dT; q[3] += qd[3] * dT
    q_normalize(q)

# ── PID struct (rate loop only) ──

@dataclass
class PIDStruct:
    Desired: float = 0.0
    Error: float = 0.0
    Kp: float = 0.0
    Kd: float = 0.0
    Max: float = 0.0
    PTerm: float = 0.0
    DTerm: float = 0.0
    last_error: float = 0.0
    d_filtered: float = 0.0

@dataclass
class StepMetrics:
    axis_name: str = ""
    step_deg: float = 0.0
    rise_time_s: float = 0.0
    overshoot_pct: float = 0.0
    settling_time_s: float = 0.0
    steady_state_error_deg: float = 0.0
    max_rate_dps: float = 0.0
    peak_angle_deg: float = 0.0
    final_angle_deg: float = 0.0
    n_oscillations: int = 0
    quat_gain: float = 0.0

# ── Control functions ──

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def run_rate_pd(pid: PIDStruct, rate: float, dT: float) -> float:
    pid.Error = clamp(pid.Desired, -pid.Max, pid.Max) - rate
    pid.PTerm = pid.Error * pid.Kp
    error_deriv = (pid.Error - pid.last_error) / dT
    tau_d = 1.0 / (2.0 * math.pi * DTERM_LPF_HZ)
    alpha = dT / (tau_d + dT)
    pid.d_filtered += alpha * (error_deriv - pid.d_filtered)
    pid.DTerm = pid.d_filtered * pid.Kd
    pid.last_error = pid.Error
    return clamp(pid.PTerm + pid.DTerm, -1.0, 1.0)

# ── Physics ──

def run_physics(rate, out, motor_lag, dT, inertia=INERTIA_ROLL, torque_frac=0.25):
    alpha = dT / (EM_MOTOR_TAU + dT)
    motor_effort = motor_lag + alpha * (out - motor_lag)
    torque = EM_MAX_THRUST * torque_frac * motor_effort * EM_ARM_LEN
    drag_torque = EM_DRAG_QUAD * math.copysign(rate * rate, rate)
    net_torque = torque - drag_torque
    accel = net_torque / inertia
    rate += accel * dT
    return rate, motor_effort

# ── Parameters ──

def get_params():
    return {
        "RollRateKp":   0.30,
        "RollRateKd":   0.015,
        "MaxRollRate":  DegreesToRadians(360.0),
        "MaxRollAngle": 0.523599,
        "PitchRateKp":  0.45,
        "PitchRateKd":  0.0225,
        "MaxPitchRate": DegreesToRadians(360.0),
        "MaxPitchAngle": 0.523599,
        "YawRateKp":    0.75,
        "YawRateKd":    0.0375,
        "MaxYawRate":   DegreesToRadians(180.0),
    }

# ── Simulator: single-axis quaternion step ──

def simulate_quat_step(axis: str, step_deg: float, quat_gain: float,
                       rate_kp: float, rate_kd: float, max_rate: float,
                       dT: float = CONTROL_DT) -> StepMetrics:
    """
    Simulate a step input on one axis using the quaternion controller.
    axis: 'roll' (q1), 'pitch' (q2), or 'yaw' (q3)
    """
    axis_map = {"roll": 1, "pitch": 2, "yaw": 3}
    axis_idx = axis_map[axis]  # q1=roll, q2=pitch, q3=yaw
    rate_idx = axis_idx - 1    # 0=roll(p), 1=pitch(q), 2=yaw(r)

    inertia = {"roll": INERTIA_ROLL, "pitch": INERTIA_PITCH, "yaw": INERTIA_YAW}[axis]
    torque_frac = TORQUE_FRAC_YAW if axis == "yaw" else 0.25

    # Current quaternion (starts at identity = level)
    q_cur = [1.0, 0.0, 0.0, 0.0]
    # Body rates
    rates = [0.0, 0.0, 0.0]

    # Rate PID
    pid = PIDStruct(Kp=rate_kp, Kd=rate_kd, Max=max_rate)

    step_rad = step_deg * DEG_TO_RAD
    # Desired quaternion: pure rotation on the test axis
    q_desired = [math.cos(step_rad * 0.5), 0.0, 0.0, 0.0]
    q_desired[axis_idx] = math.sin(step_rad * 0.5)

    motor_lag = 0.0
    n = int(SIM_TIME / dT)
    n_warmup = int(WARMUP_TIME / dT)
    decimate = 5

    angles, times = [], []
    max_rate_achieved = 0.0
    peak_angle = 0.0
    zero_crossings = 0
    prev_err = 0.0
    crossed = False

    qe = [0.0]*4

    for i in range(n):
        t = i * dT
        active = i >= n_warmup

        if active:
            q_target = q_desired
        else:
            q_target = [1.0, 0.0, 0.0, 0.0]

        # Quaternion error
        q_error(qe, q_target, q_cur)

        # Desired body rate from quaternion error (includes I-term for yaw)
        rate_max = max_rate
        pid.Desired = clamp(2.0 * qe[axis_idx] * quat_gain, -rate_max, rate_max)

        # Rate PID
        out = run_rate_pd(pid, rates[rate_idx], dT)

        # Physics with per-axis inertia and torque
        rates[rate_idx], motor_lag = run_physics(
            rates[rate_idx], out, motor_lag, dT,
            inertia=inertia, torque_frac=torque_frac
        )

        # Quaternion integration
        q_integrate(q_cur, rates, dT)

        # Extract Euler angle for metrics
        roll, pitch, yaw_e = quat_to_euler(q_cur)
        angle = {"roll": roll, "pitch": pitch, "yaw": yaw_e}[axis]

        if i % decimate == 0:
            angles.append(angle)
            times.append(t)

        max_rate_achieved = max(max_rate_achieved, abs(rates[rate_idx]))
        peak_angle = max(peak_angle, abs(angle))

        # Count zero crossings of error
        err = step_rad - angle if active else 0.0
        if i > n_warmup * 3:
            if prev_err * err < 0 and not crossed:
                zero_crossings += 1
                crossed = True
            elif prev_err * err >= 0:
                crossed = False
        prev_err = err

    final_angle = angles[-1] if angles else 0.0
    setpoint = step_rad
    ss_error_deg = abs(final_angle - setpoint) * RAD_TO_DEG

    # Step response analysis
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

        peak_above = max((a - setpoint for a in angles if a > setpoint), default=0.0)
        if setpoint > 0:
            overshoot = (peak_above / setpoint) * 100.0

    if rise_time <= 0:
        rise_time = SIM_TIME

    return StepMetrics(
        axis_name=axis,
        step_deg=step_deg,
        rise_time_s=round(rise_time, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settling, 4),
        steady_state_error_deg=round(ss_error_deg, 4),
        max_rate_dps=round(max_rate_achieved * RAD_TO_DEG, 1),
        peak_angle_deg=round(peak_angle * RAD_TO_DEG, 1),
        final_angle_deg=round(final_angle * RAD_TO_DEG, 1),
        n_oscillations=zero_crossings,
        quat_gain=quat_gain,
    )

# ── Reporting ──

G = "\033[92m"
Y = "\033[93m"
R = "\033[91m"
B = "\033[1m"
N = "\033[0m"

def print_metrics(m: StepMetrics):
    print(f"\n{B}── {m.axis_name.title()} {m.step_deg:.0f}° step (QuatGain={m.quat_gain}) ──{N}")
    print(f"     Final: {m.final_angle_deg:.1f}°   Peak: {m.peak_angle_deg:.1f}°   Max rate: {m.max_rate_dps:.0f}°/s")
    print(f"  [{G}{'PASS' if m.rise_time_s <= 1.5 else R+'FAIL'+N}] Rise time: {m.rise_time_s:.3f}s  (max 1.5s)")
    print(f"  [{G}{'PASS' if m.overshoot_pct <= 15.0 else R+'FAIL'+N}] Overshoot: {m.overshoot_pct:.1f}%  (max 15%)")
    print(f"  [{G}{'PASS' if m.settling_time_s <= 3.0 else R+'FAIL'+N}] Settling: {m.settling_time_s:.3f}s  (max 3.0s)")
    print(f"  [{G}{'PASS' if m.steady_state_error_deg <= 1.0 else R+'FAIL'+N}] SS error: {m.steady_state_error_deg:.2f}°  (max 1.0°)")
    print(f"  [{'--'}] Oscillations: {m.n_oscillations}")

def sweep_gain(axis: str, step_deg: float, rate_kp: float, rate_kd: float, max_rate: float,
               gain_max=6.0, gain_step=0.5):
    print(f"\n{B}═══ {axis.title()} Q sweep ({axis} {step_deg:.0f}° step, range 0.5–{gain_max:.0f}) ═══{N}")
    print(f"{'Gain':>6} {'Rise':>8} {'Overshoot':>10} {'Settle':>8} {'SS err':>8} {'MaxRate':>9} {'Result':>8}")
    print("-" * 65)

    best_gain = 0.0
    best_score = 1e9
    steps = int(gain_max / gain_step)

    for i in range(1, steps + 1):
        gain = i * gain_step
        m = simulate_quat_step(axis, step_deg, gain, rate_kp, rate_kd, max_rate)
        # Score: lower is better (weighted sum of rise time, overshoot, settling, ss error)
        score = (m.rise_time_s * 1.0 + m.overshoot_pct * 0.1 +
                 m.settling_time_s * 0.5 + m.steady_state_error_deg * 2.0)
        ok = (m.rise_time_s <= 1.5 and m.overshoot_pct <= 15.0 and
              m.settling_time_s <= 3.0 and m.steady_state_error_deg <= 1.0)
        icon = f"{G}PASS{N}" if ok else f"{R}fail{N}"
        print(f"{gain:6.1f} {m.rise_time_s:8.3f} {m.overshoot_pct:10.1f} "
              f"{m.settling_time_s:8.3f} {m.steady_state_error_deg:8.2f} "
              f"{m.max_rate_dps:9.0f} {icon:>8}")
        if ok and score < best_score:
            best_score = score
            best_gain = gain

    return best_gain

def main():
    p = get_params()

    print(f"{B}UAVXArmQ Quaternion Controller — Gain Sweep Simulation{N}")
    print(f"{'='*65}")
    print(f"  Control:  {1/CONTROL_DT:.0f} Hz   Motor tau: {EM_MOTOR_TAU*1000:.0f} ms")
    print(f"  Mass:     {EM_MASS} kg   Arm: {EM_ARM_LEN} m")
    print(f"  Inertia:  roll={INERTIA_ROLL}  pitch={INERTIA_PITCH}  yaw={INERTIA_YAW} kg·m²")
    print(f"  Drag:     quad, coeff {EM_DRAG_QUAD}   Thrust: {EM_MAX_THRUST:.0f} N")
    print(f"  Roll Kp:  {p['RollRateKp']}   Kd: {p['RollRateKd']}   Max: {p['MaxRollRate']*RAD_TO_DEG:.0f} dps")
    print(f"  Pitch Kp: {p['PitchRateKp']}   Kd: {p['PitchRateKd']}   Max: {p['MaxPitchRate']*RAD_TO_DEG:.0f} dps")
    print(f"  Yaw Kp:   {p['YawRateKp']}   Kd: {p['YawRateKd']}   Max: {p['MaxYawRate']*RAD_TO_DEG:.0f} dps")
    print(f"{'='*65}")

    # Test cases: (axis, step_deg, rate_kp, rate_kd, max_rate)
    tests = [
        ("roll",  15.0, p["RollRateKp"], p["RollRateKd"], p["MaxRollRate"]),
        ("roll",  90.0, p["RollRateKp"], p["RollRateKd"], p["MaxRollRate"]),
        ("pitch", 15.0, p["PitchRateKp"], p["PitchRateKd"], p["MaxPitchRate"]),
        ("pitch", 90.0, p["PitchRateKp"], p["PitchRateKd"], p["MaxPitchRate"]),
        ("yaw",   15.0, p["YawRateKp"], p["YawRateKd"], p["MaxYawRate"]),
    ]

    best_gains = {}
    for axis, step, rkp, rkd, mr in tests:
        key = f"{axis}_{step:.0f}"
        gain_max = 8.0 if axis == "yaw" else 7.0  # FC param max for Q
        best = sweep_gain(axis, step, rkp, rkd, mr, gain_max=gain_max)
        best_gains[key] = best
        if best > 0:
            print(f"  → {Y}Best {axis.title()} Q: {best:.1f}{N}")
            m = simulate_quat_step(axis, step, best, rkp, rkd, mr)
            print_metrics(m)
        else:
            print(f"  → {R}No gain passed all criteria — relax thresholds or check rate PID{N}")

    print(f"\n{'='*65}")
    print(f"{B}Summary{N}")
    for key, gain in best_gains.items():
        status = f"{G}{gain:.1f} PASS{N}" if gain > 0 else f"{R}none — FAIL{N}"
        print(f"  {key:12s}: {status}")

    # Recommend per-axis Q gains
    for axis in ("roll", "pitch", "yaw"):
        g15 = best_gains.get(f"{axis}_15", 0)
        g90 = best_gains.get(f"{axis}_90", 0)
        name = {"roll": "Roll Q", "pitch": "Pitch Q", "yaw": "Yaw Q"}[axis]
        if g90 > 0:
            rec = min(g15, g90) if g15 > 0 else g90
            print(f"\n{B}Recommended {name}: {rec:.1f}{N}  (conservative — works for 90° steps)")
        elif g15 > 0:
            print(f"\n{B}Recommended {name}: {g15:.1f}{N}  (small-angle only)")
        else:
            print(f"\n{R}No suitable {name} — review rate PID tuning{N}")

    sys.exit(0)

if __name__ == "__main__":
    main()
