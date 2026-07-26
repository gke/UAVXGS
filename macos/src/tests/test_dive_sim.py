#!/usr/bin/env python3
"""
UAVXArmQ Dive Mode Simulation.

Simulates the full dive cycle:
  1. Aircraft starts level at altitude
  2. Dive switch triggered → target 90° nose-down
  3. Aircraft pitches to 90°, descends ballistically
  4. At DIVE_RECOVER_ALT_M (20m) → recovery triggers
  5. Target returns to entry attitude (level), full throttle
  6. Aircraft pitches back to level, exits dive

Uses the same physics model as test_quat_sim.py plus vertical dynamics
for altitude (gravity + thrust projection onto body z-axis).

Usage:
  python3 src/tests/test_dive_sim.py
"""

import math
import sys
from dataclasses import dataclass

RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0
GRAVITY = 9.80665

# ── Emulation constants — Racerstar BR2205 2300kv + 5x3x3, 800g AUW ──
EM_MASS = 0.8
EM_ARM_LEN = 0.17
EM_MAX_THRUST = EM_MASS / 0.50 * GRAVITY
EM_INERTIA = 0.008
EM_MOTOR_TAU = 0.08
EM_DRAG = 0.3
CONTROL_DT = 0.001
SIM_TIME = 15.0

DTERM_LPF_HZ = 50.0

# ── Dive constants (from dive.c) ──
DIVE_RECOVER_ALT_M = 20.0
DIVE_RECOVER_TIME_S = 1.5
DIVE_ATT_THRESH = 0.08
QUAT_GAIN = 4.0
MAX_RATE_DPS = 180.0
MAX_RATE_RAD = MAX_RATE_DPS * DEG_TO_RAD

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
        for i in range(4): qe[i] = -qe[i]

def q_normalize(q):
    n = math.sqrt(sum(x*x for x in q))
    if n > 0:
        for i in range(4): q[i] /= n

def q_integrate(q, rates, dT):
    p, qq, r = rates
    qd = [0.0]*4
    qd[0] = 0.5 * (-q[1]*p - q[2]*qq - q[3]*r)
    qd[1] = 0.5 * ( q[0]*p - q[3]*qq + q[2]*r)
    qd[2] = 0.5 * ( q[3]*p + q[0]*qq - q[1]*r)
    qd[3] = 0.5 * (-q[2]*p + q[1]*qq + q[0]*r)
    for i in range(4): q[i] += qd[i] * dT
    q_normalize(q)

def quat_to_euler(q):
    roll = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
    sinp = 2*(q[0]*q[2] - q[3]*q[1])
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))
    return roll, pitch, yaw

def body_thrust_to_world_z(q, throttle_frac):
    # Thrust in body frame is [0, 0, T]. World Z component = T * (1 - 2*(q1^2 + q2^2))
    # When level: all thrust is vertical. At 90° pitch: zero vertical thrust.
    return EM_MAX_THRUST * throttle_frac * (1.0 - 2.0*(q[1]**2 + q[2]**2))

# ── Rate PID ──

@dataclass
class PIDStruct:
    Desired: float = 0.0
    Error: float = 0.0
    Kp: float = 0.0
    Kd: float = 0.0
    Max: float = 0.0
    last_error: float = 0.0
    d_filtered: float = 0.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def run_rate_pd(pid: PIDStruct, rate: float, dT: float) -> float:
    pid.Error = clamp(pid.Desired, -pid.Max, pid.Max) - rate
    pterm = pid.Error * pid.Kp
    error_deriv = (pid.Error - pid.last_error) / dT
    tau_d = 1.0 / (2.0 * math.pi * DTERM_LPF_HZ)
    alpha = dT / (tau_d + dT)
    pid.d_filtered += alpha * (error_deriv - pid.d_filtered)
    dterm = pid.d_filtered * pid.Kd
    pid.last_error = pid.Error
    return clamp(pterm + dterm, -1.0, 1.0)

# ── Dive state machine (mirrors dive.c) ──

DiveIdle, DiveActive, DiveRecover = 0, 1, 2

@dataclass
class DiveState:
    phase: int = DiveIdle
    armed: bool = True
    q_entry: list = None
    q_pitch90: list = None
    vert_vel: float = 0.0

    def __post_init__(self):
        if self.q_entry is None:
            self.q_entry = [1.0, 0.0, 0.0, 0.0]
        if self.q_pitch90 is None:
            self.q_pitch90 = [0.7071068, 0.0, -0.7071068, 0.0]

def do_dive_control(ds: DiveState, dive_mode: bool, altitude: float,
                    vert_vel: float, q_cur: list, q_target: list,
                    stick_throttle: float):
    """Returns (active, desired_throttle)"""
    ds.vert_vel = vert_vel
    if ds.phase == DiveIdle:
        if not dive_mode:
            ds.armed = True
            return False, stick_throttle
        if not ds.armed:
            return False, stick_throttle
        ds.armed = False
        ds.q_entry = list(q_cur)
        ds.phase = DiveActive
        # fall through to DiveActive

        if ds.phase == DiveActive:
            q_mul(q_target, ds.q_entry, ds.q_pitch90)
            desired_thr = stick_throttle
            if (not dive_mode or
                altitude < DIVE_RECOVER_ALT_M or
                altitude < -ds.vert_vel * DIVE_RECOVER_TIME_S):
                ds.phase = DiveRecover
            return True, desired_thr

    if ds.phase == DiveRecover:
        q_target[:] = ds.q_entry[:]
        qe = [0.0]*4
        q_error(qe, q_target, q_cur)
        if qe[1]**2 + qe[2]**2 + qe[3]**2 < DIVE_ATT_THRESH**2:
            ds.phase = DiveIdle
            return False, stick_throttle
        return True, 1.0  # full throttle during recovery

    return False, stick_throttle

# ── Simulation ──

def simulate_dive(start_alt: float = 60.0, stick_throttle: float = 0.3,
                  rate_kp: float = 0.25, rate_kd: float = 0.008):
    """Run a full dive cycle simulation."""
    q_cur = [1.0, 0.0, 0.0, 0.0]
    rates = [0.0, 0.0, 0.0]  # body rates p, q, r
    altitude = start_alt
    vert_vel = 0.0

    # Two rate PIDs: pitch (axis 2) and roll (axis 1)
    pid_pitch = PIDStruct(Kp=rate_kp, Kd=rate_kd, Max=MAX_RATE_RAD)
    pid_roll = PIDStruct(Kp=rate_kp, Kd=rate_kd, Max=MAX_RATE_RAD)

    ds = DiveState()
    ds.q_entry = list(q_cur)

    n = int(SIM_TIME / CONTROL_DT)
    dive_switch_on = False
    dive_switch_time = 1.0  # turn on at t=1s

    log = []
    motor_lag_pitch = 0.0
    motor_lag_roll = 0.0

    qe = [0.0]*4
    q_target = [0.0]*4

    for i in range(n):
        t = i * CONTROL_DT

        # RC: turn dive switch on at dive_switch_time
        dive_mode = t >= dive_switch_time

        # Dive control: get target quaternion and throttle
        dive_active, desired_throttle = do_dive_control(
            ds, dive_mode, altitude, vert_vel, q_cur, q_target, stick_throttle)

        # Quaternion error
        if dive_active:
            q_error(qe, q_target, q_cur)
        else:
            # Level hover target
            q_level = [1.0, 0.0, 0.0, 0.0]
            q_error(qe, q_level, q_cur)

        # Body rate commands from quaternion error
        pid_pitch.Desired = clamp(2.0 * qe[2] * QUAT_GAIN, -MAX_RATE_RAD, MAX_RATE_RAD)
        pid_roll.Desired = clamp(2.0 * qe[1] * QUAT_GAIN, -MAX_RATE_RAD, MAX_RATE_RAD)

        # Rate PID
        out_pitch = run_rate_pd(pid_pitch, rates[1], CONTROL_DT)
        out_roll = run_rate_pd(pid_roll, rates[0], CONTROL_DT)

        # Physics: rotational
        alpha_m = CONTROL_DT / (EM_MOTOR_TAU + CONTROL_DT)

        motor_lag_pitch += alpha_m * (out_pitch - motor_lag_pitch)
        motor_lag_roll += alpha_m * (out_roll - motor_lag_roll)

        torque_pitch = EM_MAX_THRUST * 0.25 * motor_lag_pitch * EM_ARM_LEN
        torque_roll = EM_MAX_THRUST * 0.25 * motor_lag_roll * EM_ARM_LEN

        rates[1] += (torque_pitch - EM_DRAG * rates[1]) / EM_INERTIA * CONTROL_DT
        rates[0] += (torque_roll - EM_DRAG * rates[0]) / EM_INERTIA * CONTROL_DT

        # Physics: quaternion integration
        q_integrate(q_cur, rates, CONTROL_DT)

        # Physics: vertical dynamics
        # Vertical thrust = body thrust projected onto world Z
        # At 90° pitch, thrust is horizontal → zero vertical lift
        thrust_world_z = body_thrust_to_world_z(q_cur, desired_throttle)
        net_force = thrust_world_z - EM_MASS * GRAVITY
        vert_vel += (net_force / EM_MASS) * CONTROL_DT
        altitude += vert_vel * CONTROL_DT

        # Prevent going underground
        if altitude < 0:
            altitude = 0
            vert_vel = 0

        # Log every 50ms
        if i % 50 == 0:
            roll, pitch, yaw = quat_to_euler(q_cur)
            phase_names = {0: "Idle", 1: "Active", 2: "Recover"}
            log.append({
                't': t,
                'alt': altitude,
                'vert_vel': vert_vel,
                'pitch_deg': pitch * RAD_TO_DEG,
                'roll_deg': roll * RAD_TO_DEG,
                'pitch_rate_dps': rates[1] * RAD_TO_DEG,
                'phase': phase_names[ds.phase],
                'dive_mode': dive_mode,
                'throttle': desired_throttle,
                'thrust_z': thrust_world_z,
            })

    return log

# ── Reporting ──

G = "\033[92m"
Y = "\033[93m"
R = "\033[91m"
B = "\033[1m"
N = "\033[0m"

def print_timeline(log):
    print(f"\n{B}{'t':>6} {'Alt':>7} {'Vz':>7} {'Pitch':>8} {'Rate':>8} {'Phase':>8} {'Thr':>5} {'Fz':>6}{N}")
    print("-" * 62)
    for e in log:
        # Highlight phase transitions
        prev_phase = log[max(0, log.index(e)-1)]['phase'] if log.index(e) > 0 else ""
        marker = ""
        if e['phase'] != prev_phase and prev_phase != "":
            marker = f" {Y}←{N}"
        print(f"{e['t']:6.1f} {e['alt']:7.1f} {e['vert_vel']:7.1f} "
              f"{e['pitch_deg']:8.1f} {e['pitch_rate_dps']:8.0f} "
              f"{e['phase']:>8} {e['throttle']:5.2f} {e['thrust_z']:6.1f}{marker}")

def analyze_dive(log):
    print(f"\n{B}═══ Dive Analysis ═══{N}")

    # Find key events
    dive_start = None
    recover_start = None
    dive_complete = None
    min_alt = 1e9
    min_alt_t = 0
    max_rate = 0
    max_rate_t = 0

    for e in log:
        if e['phase'] == 'Active' and dive_start is None:
            dive_start = e['t']
        if e['phase'] == 'Recover' and recover_start is None:
            recover_start = e['t']
        if e['phase'] == 'Idle' and recover_start is not None and dive_complete is None:
            dive_complete = e['t']
        if e['alt'] < min_alt:
            min_alt = e['alt']
            min_alt_t = e['t']
        if abs(e['pitch_rate_dps']) > abs(max_rate):
            max_rate = e['pitch_rate_dps']
            max_rate_t = e['t']

    # Check if we hit the ground
    crashed = min_alt <= 0.5

    # Time to pitch to 90°
    t_to_90 = None
    for e in log:
        if abs(e['pitch_deg']) >= 85 and dive_start is not None:
            t_to_90 = e['t'] - dive_start
            break

    # Time to recover to level
    t_to_recover = None
    if recover_start and dive_complete:
        t_to_recover = dive_complete - recover_start

    print(f"  Dive start:         t={dive_start:.1f}s" if dive_start else "  Dive start:         never")
    print(f"  Time to 90° pitch:  {t_to_90:.1f}s" if t_to_90 else "  Time to 90° pitch:  never reached")
    print(f"  Recovery start:     t={recover_start:.1f}s" if recover_start else "  Recovery start:     never")
    print(f"  Time to level:      {t_to_recover:.1f}s" if t_to_recover else "  Time to level:      never recovered")
    print(f"  Dive complete:      t={dive_complete:.1f}s" if dive_complete else "  Dive complete:      never")
    print(f"  Min altitude:       {min_alt:.1f}m at t={min_alt_t:.1f}s")
    print(f"  Max pitch rate:     {max_rate:.0f}°/s at t={max_rate_t:.1f}s")

    # Verdict
    print(f"\n  {'['+G+'PASS'+N+']' if not crashed else '['+R+'FAIL'+N+']'} Aircraft survived (min alt {min_alt:.1f}m)")
    if t_to_90:
        print(f"  [{G}PASS{N}] Reached 90° pitch in {t_to_90:.1f}s")
    else:
        print(f"  [{R}FAIL{N}] Never reached 90° pitch")
    if t_to_recover:
        print(f"  [{G}PASS{N}] Recovered to level in {t_to_recover:.1f}s")
    else:
        print(f"  [{R}FAIL{N}] Never recovered to level")
    if min_alt < 5.0:
        print(f"  [{R}WARN{N}] Minimum altitude {min_alt:.1f}m — increase DIVE_RECOVER_ALT_M")
    elif min_alt < 10.0:
        print(f"  [{Y}WARN{N}] Minimum altitude {min_alt:.1f}m — marginal recovery margin")
    else:
        print(f"  [{G}PASS{N}] Recovery margin adequate ({min_alt:.1f}m min)")

def main():
    print(f"{B}UAVXArmQ Dive Mode Simulation{N}")
    print(f"{'='*62}")
    print(f"  QuatGain: {QUAT_GAIN}   Max rate: {MAX_RATE_DPS}°/s")
    print(f"  Recover alt: {DIVE_RECOVER_ALT_M}m   Att thresh: {DIVE_ATT_THRESH}")
    print(f"  Start alt: 60m   Stick throttle: 0.3")
    print(f"  Mass: {EM_MASS}kg   Inertia: {EM_INERTIA}   Thrust: {EM_MAX_THRUST:.0f}N")
    print(f"{'='*62}")

    log = simulate_dive(start_alt=60.0, stick_throttle=0.3)
    print_timeline(log)
    analyze_dive(log)

    # Also test with higher start altitude and lower throttle
    print(f"\n{'='*62}")
    print(f"{B}Test 2: Start alt=100m, stick throttle=0.5{N}")
    print(f"{'='*62}")
    log2 = simulate_dive(start_alt=100.0, stick_throttle=0.5)
    print_timeline(log2)
    analyze_dive(log2)

    # Test with switch release mid-dive
    print(f"\n{'='*62}")
    print(f"{B}Test 3: Start alt=100m, switch released at t=3s{N}")
    print(f"{'='*62}")
    log3 = simulate_dive_switch_release(start_alt=100.0, stick_throttle=0.5, release_t=3.0)
    print_timeline(log3)
    analyze_dive(log3)

    sys.exit(0)

def simulate_dive_switch_release(start_alt=100.0, stick_throttle=0.5, release_t=3.0):
    """Simulate dive with manual switch release."""
    q_cur = [1.0, 0.0, 0.0, 0.0]
    rates = [0.0, 0.0, 0.0]
    altitude = start_alt
    vert_vel = 0.0

    pid_pitch = PIDStruct(Kp=0.25, Kd=0.008, Max=MAX_RATE_RAD)
    pid_roll = PIDStruct(Kp=0.25, Kd=0.008, Max=MAX_RATE_RAD)

    ds = DiveState()
    ds.q_entry = list(q_cur)

    n = int(SIM_TIME / CONTROL_DT)
    log = []
    motor_lag_pitch = motor_lag_roll = 0.0
    qe = [0.0]*4
    q_target = [0.0]*4

    for i in range(n):
        t = i * CONTROL_DT

        # Switch on at t=1.0, off at release_t
        dive_mode = 1.0 <= t < release_t

        dive_active, desired_throttle = do_dive_control(
            ds, dive_mode, altitude, q_cur, q_target, stick_throttle)

        if dive_active:
            q_error(qe, q_target, q_cur)
        else:
            q_error(qe, [1.0, 0.0, 0.0, 0.0], q_cur)

        pid_pitch.Desired = clamp(2.0 * qe[2] * QUAT_GAIN, -MAX_RATE_RAD, MAX_RATE_RAD)
        pid_roll.Desired = clamp(2.0 * qe[1] * QUAT_GAIN, -MAX_RATE_RAD, MAX_RATE_RAD)

        out_pitch = run_rate_pd(pid_pitch, rates[1], CONTROL_DT)
        out_roll = run_rate_pd(pid_roll, rates[0], CONTROL_DT)

        alpha_m = CONTROL_DT / (EM_MOTOR_TAU + CONTROL_DT)
        motor_lag_pitch += alpha_m * (out_pitch - motor_lag_pitch)
        motor_lag_roll += alpha_m * (out_roll - motor_lag_roll)

        torque_pitch = EM_MAX_THRUST * 0.25 * motor_lag_pitch * EM_ARM_LEN
        torque_roll = EM_MAX_THRUST * 0.25 * motor_lag_roll * EM_ARM_LEN

        rates[1] += (torque_pitch - EM_DRAG * rates[1]) / EM_INERTIA * CONTROL_DT
        rates[0] += (torque_roll - EM_DRAG * rates[0]) / EM_INERTIA * CONTROL_DT

        q_integrate(q_cur, rates, CONTROL_DT)

        thrust_world_z = body_thrust_to_world_z(q_cur, desired_throttle)
        net_force = thrust_world_z - EM_MASS * GRAVITY
        vert_vel += (net_force / EM_MASS) * CONTROL_DT
        altitude += vert_vel * CONTROL_DT

        if altitude < 0:
            altitude = 0; vert_vel = 0

        if i % 50 == 0:
            roll, pitch, yaw = quat_to_euler(q_cur)
            phase_names = {0: "Idle", 1: "Active", 2: "Recover"}
            log.append({
                't': t, 'alt': altitude, 'vert_vel': vert_vel,
                'pitch_deg': pitch * RAD_TO_DEG, 'roll_deg': roll * RAD_TO_DEG,
                'pitch_rate_dps': rates[1] * RAD_TO_DEG,
                'phase': phase_names[ds.phase], 'dive_mode': dive_mode,
                'throttle': desired_throttle, 'thrust_z': thrust_world_z,
            })

    return log

if __name__ == "__main__":
    main()
