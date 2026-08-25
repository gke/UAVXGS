#!/usr/bin/env python3
"""
UAVXArmQ Crash-Descent (Dive) Emulator-Fold Simulation.

Mirrors the FC end-to-end so the off-board sim exercises the SAME paths a
harnessed FC would:

  - dive.c      : demand-gated timed pitch profile (punch 60 -> cruise 45 ->
                  flare 0), Doppler supervision, abort -> forced flare,
                  recovery -> handback to altitude-hold.
  - emu.c (MR)  : quad physical plant (model 0, eQuadXAF) — motor lag, thrust,
                  vertical drag, differential-torque rates, rate clamps, and
                  the accel synthesis feeding Madgwick.
  - inertial.c  : CalculateAccConfidence replica (no emulation special case)
                  and the Madgwick step with its confidence-scaled accel gain.

Emulator fold (the point of this file):
  * The emulated accel drives the REAL confidence model. During the steep
    powered dive the accel magnitude collapses -> confidence -> 0 -> Madgwick
    dead-reckons on the gyros (the 30 s DIVE_MAX_PROFILE_S budget).
  * A bounded pitch gyro bias (+0.05 deg/s) is injected while the dive is
    active, so the estimator must reject a constant rate error with no accel
    bail-out. A true/estimated attitude split measures the residual drift and
    shows the flare re-blend corrects it.

Usage:
  python3 src/tests/test_dive_sim.py
"""

import math
import sys
from dataclasses import dataclass

RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0
GRAVITY = 9.80665

# ── FC dive constants (from dive.c) ──
DIVE_PUNCH_TIME_S = 2.0        # pitch-over to entry attitude
DIVE_RAMP_TIME_S = 2.0         # entry -> cruise ease
DIVE_FLARE_TIME_S = 5.0        # cruise -> level ease
DIVE_ENTRY_PITCH = 60.0 * DEG_TO_RAD
DIVE_CRUISE_PITCH = 45.0 * DEG_TO_RAD
DIVE_PROFILE_THR = 0.55        # fixed descent throttle (physics, not stick)
DIVE_EXPECTED_SINK_MPS = 12.0  # cruise equilibrium sink assumption (scheduler)
DIVE_MAX_PROFILE_S = 30.0      # hard ceiling = gyro-only attitude budget
DIVE_SUPER_WINDOW_S = 5.0      # Doppler supervision averaging window
DIVE_FLARE_LEAD_S = 3.0        # begin flare this margin early
DIVE_ATT_THRESH = 0.08         # attitude error below which recovery hands back
DIVE_ENTRY_PLUS_RAMP_S = DIVE_PUNCH_TIME_S + DIVE_RAMP_TIME_S

PDIVE_RECOVER_ALT = 15.0       # tag 116 default (m) — floor / budget base

# ── FC imu/params constants ──
P_ACC_CONF_SDEV_R = 25.0       # AccConfSD tag 52 default
P_TWO_KP_ACC = 0.2             # MadgwickKpAcc tag 38 default
CONF_LPF_ALPHA = 0.1           # confp low-pass in CalculateAccConfidence

# ── Emulator constants — model 0 eQuadXAF (emu.c EmuModels[0]) ──
EM_MASS = 0.8
EM_ARM_LEN = 0.17
EM_MAX_THRUST = 14.3
EM_MOTOR_TAU = 0.10
EM_DRAG_SCALE = 20.0 / (14.0 * 14.0)   # AS_MAX_MPS = 14.0 (emu.h)
EM_MAX_ROLL_RATE = 300.0 * DEG_TO_RAD
EM_MAX_PITCH_RATE = 300.0 * DEG_TO_RAD
EM_MAX_YAW_RATE = 180.0 * DEG_TO_RAD
EMU_GYRO_BIAS_RAD_S = 0.000873  # ~0.05 deg/s bounded gyro corruption (emu.c probe)

CONTROL_DT = 0.001

QUAT_KP = 4.0                  # angle-P QGain (rate desired = 2*Qa*Kp)
RATE_KP = 0.25
RATE_KD = 0.008
RATE_DTERM_LPF_HZ = 50.0

STICK_THROTTLE = 0.5

eIdle, eActive, eRecover, eSpiral = 0, 1, 2, 3
PHASE_NAME = ["Idle", "Active", "Recover", "Spiral"]


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
        for i in range(4):
            qe[i] = -qe[i]

def q_normalize(q):
    n = math.sqrt(sum(x*x for x in q))
    if n > 0:
        for i in range(4):
            q[i] /= n

def q_integrate(q, rates, dT):
    p, qq, r = rates
    qd = [0.0]*4
    qd[0] = 0.5 * (-q[1]*p - q[2]*qq - q[3]*r)
    qd[1] = 0.5 * ( q[0]*p - q[3]*qq + q[2]*r)
    qd[2] = 0.5 * ( q[3]*p + q[0]*qq - q[1]*r)
    qd[3] = 0.5 * (-q[2]*p + q[1]*qq + q[0]*r)
    for i in range(4):
        q[i] += qd[i]*dT
    q_normalize(q)

def euler_to_quat(q, roll, pitch, yaw):
    cr, cp, cy = math.cos(roll*0.5), math.cos(pitch*0.5), math.cos(yaw*0.5)
    sr, sp, sy = math.sin(roll*0.5), math.sin(pitch*0.5), math.sin(yaw*0.5)
    q[0] = cy*cp*cr + sy*sp*sr
    q[1] = cy*cp*sr - sy*sp*cr
    q[2] = cy*sp*cr + sy*cp*sr
    q[3] = sy*cp*cr - cy*sp*sr

def quat_to_euler(q):
    roll = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
    sinp = 2*(q[0]*q[2] - q[3]*q[1])
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))
    return roll, pitch, yaw

def attitude_error_deg(q_true, q_est):
    qe = [0.0]*4
    q_error(qe, q_true, q_est)
    w = abs(qe[0])
    v = math.sqrt(qe[1]**2 + qe[2]**2 + qe[3]**2)
    if w > 1.0:
        w = 1.0
    return 2.0 * math.degrees(math.atan2(v, w))

def attitude_cosine(q):
    return q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]


# ── FC replicas ──

def lpf1(prev, target, alpha):
    return prev + alpha * (target - prev)

def signf(v):
    return -1.0 if v < 0.0 else (1.0 if v > 0.0 else 0.0)

def drag(v):            # Drag() in emu.c
    return signf(v) * v * v * EM_DRAG_SCALE

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

@dataclass
class PID:
    Desired: float = 0.0
    Error: float = 0.0
    Kp: float = 0.0
    Kd: float = 0.0
    Max: float = 0.0
    last_error: float = 0.0
    d_filtered: float = 0.0

def run_rate_pd(pid, rate, dT):
    # FC ControlRate: Error = clamp(Desired,+-Max) - Rate, then A.Out = -cond(P+D)
    pid.Error = clamp(pid.Desired, -pid.Max, pid.Max) - rate
    pterm = pid.Error * pid.Kp
    error_deriv = (pid.Error - pid.last_error) / dT
    tau_d = 1.0 / (2.0 * math.pi * RATE_DTERM_LPF_HZ)
    alpha = dT / (tau_d + dT)
    pid.d_filtered += alpha * (error_deriv - pid.d_filtered)
    dterm = pid.d_filtered * pid.Kd
    pid.last_error = pid.Error
    raw = clamp(pterm + dterm, -1.0, 1.0)
    return -raw, raw                # (FC output Out, informative raw)

def calculate_acc_confidence(confp, AccMag, AccUD):
    """Faithful replica of CalculateAccConfidence (inertial.c) with the
    emulation special case REMOVED — the emulated accel drives it directly."""
    temp = AccMag / GRAVITY
    mag_conf = math.exp(-0.5 * ((1.0 - temp) * P_ACC_CONF_SDEV_R) ** 2)
    norm_vert = AccUD / AccMag if AccMag > 0.0 else 0.0
    if norm_vert < -0.7:
        dir_conf = 1.0
    elif norm_vert < -0.3:
        dir_conf = 0.5
    else:
        dir_conf = 0.0
    confp = lpf1(confp, mag_conf * dir_conf, CONF_LPF_ALPHA)
    return confp

def madgwick_step(q_est, gyro, acc, two_kp_acc, dT):
    """Madgwick estimate: corrected gyro = gyro + 2*TwoKpAcc*(v x a).
    TwoKpAcc is the confidence-scaled accel gain (inertial.c) — it collapses
    with CalculateAccConfidence in the dive so the estimator goes gyro-only."""
    gx, gy, gz = gyro
    q0, q1, q2, q3 = q_est

    vx = 2.0*(q1*q3 - q0*q2)
    vy = 2.0*(q0*q1 + q2*q3)
    vz = 2.0*(q0*q0 + q3*q3) - 1.0

    amag = math.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)
    if amag > 0.0:
        ax, ay, az = acc[0]/amag, acc[1]/amag, acc[2]/amag
        gx += (vy*az - vz*ay) * (2.0 * two_kp_acc)
        gy += (vz*ax - vx*az) * (2.0 * two_kp_acc)
        gz += (vx*ay - vy*ax) * (2.0 * two_kp_acc)

    q_integrate(q_est, (gx, gy, gz), dT)


# ── Dive state machine (mirrors dive.c DoDiveControl) ──

@dataclass
class DiveState:
    phase: int = eIdle
    armed: bool = True
    flying: bool = False
    prof_t: float = 0.0
    flare_t: float = 1.0e30
    entry_alt: float = 0.0
    target_alt: float = 0.0
    budget_m: float = 0.0
    sink_est: float = 0.0
    descent_m: float = 0.0
    q_entry: list = None
    gps_lost: bool = False

    def __post_init__(self):
        if self.q_entry is None:
            self.q_entry = [1.0, 0.0, 0.0, 0.0]


def dive_pitch_at(prof_t, flare_t):
    """DivePitchAt() from dive.c."""
    if prof_t < DIVE_PUNCH_TIME_S:
        p = DIVE_ENTRY_PITCH * (prof_t / DIVE_PUNCH_TIME_S)
    elif prof_t < DIVE_PUNCH_TIME_S + DIVE_RAMP_TIME_S:
        p = DIVE_ENTRY_PITCH - (DIVE_ENTRY_PITCH - DIVE_CRUISE_PITCH) \
            * ((prof_t - DIVE_PUNCH_TIME_S) / DIVE_RAMP_TIME_S)
    elif prof_t < flare_t:
        p = DIVE_CRUISE_PITCH
    elif prof_t < flare_t + DIVE_FLARE_TIME_S:
        p = DIVE_CRUISE_PITCH * (1.0 - (prof_t - flare_t) / DIVE_FLARE_TIME_S)
    else:
        p = 0.0
    return p


def do_dive_control(ds, dive_mode, altitude, roc, q_est, q_target, orbit_alt,
                    stick_throttle, dT):
    """Returns (active, desired_throttle). Mirrors dive.c DoDiveControl."""
    target_floor = max(orbit_alt, PDIVE_RECOVER_ALT)
    active = True

    if ds.phase == eIdle:
        # Feasibility gate: not commissioned / below floor / no pull-out room.
        if (PDIVE_RECOVER_ALT <= 0.0 or
                altitude < target_floor + PDIVE_RECOVER_ALT):
            return False, stick_throttle
        if not dive_mode:
            ds.armed = True
            return False, stick_throttle
        if not ds.armed:
            return False, stick_throttle
        # StartDiveProfile()
        ds.armed = False
        ds.entry_alt = max(altitude, 0.0)
        ds.target_alt = target_floor
        ds.budget_m = max(ds.entry_alt - ds.target_alt, 0.0)
        nom_cruise = min(ds.budget_m / DIVE_EXPECTED_SINK_MPS,
                         DIVE_MAX_PROFILE_S - DIVE_ENTRY_PLUS_RAMP_S
                         - DIVE_FLARE_TIME_S)
        ds.flare_t = DIVE_ENTRY_PLUS_RAMP_S + max(nom_cruise, 0.0)
        ds.sink_est = 0.0
        ds.descent_m = 0.0
        ds.q_entry = list(q_est)
        ds.flying = True
        ds.phase = eActive

    if ds.phase == eActive:
        # Abort (switch release) forces the flare now — smooth, never a step.
        # Baro floor is a last-resort flare trigger (never the authority).
        if (not dive_mode or altitude < PDIVE_RECOVER_ALT):
            if ds.flying and ds.prof_t < ds.flare_t:
                ds.flare_t = ds.prof_t

        ds.prof_t += dT

        if altitude < 0.0:
            ds.gps_lost = True   # mirrors F.HaveGPS && !F.GPSValid (no GPS loss in sim)

        # SuperviseDiveProfile() — Doppler sink rescales the flare moment.
        if ds.prof_t < ds.flare_t:
            # GPS is valid: NED down positive when sinking.
            sink = max(-roc, 0.0)
            ds.sink_est = lpf1(ds.sink_est, sink, dT / DIVE_SUPER_WINDOW_S)
            ds.descent_m += sink * dT
            residual = ds.budget_m - ds.descent_m
            sinkmax = max(ds.sink_est, 1.0)
            lead = sinkmax * DIVE_FLARE_LEAD_S
            new_f = min(ds.prof_t + max(residual, 0.0) / sinkmax
                        + DIVE_FLARE_LEAD_S, DIVE_MAX_PROFILE_S)
            if new_f < ds.flare_t:
                ds.flare_t = max(new_f, ds.prof_t + DIVE_FLARE_TIME_S)
            if residual <= lead:
                ds.flare_t = ds.prof_t

        theta = dive_pitch_at(ds.prof_t, ds.flare_t)
        euler_to_quat(q_target, 0.0, -theta, 0.0)  # bank 0, yaw 0

        if ds.prof_t >= ds.flare_t + DIVE_FLARE_TIME_S and theta <= 1.0*DEG_TO_RAD:
            ds.flying = False
            ds.phase = eRecover
        else:
            return True, DIVE_PROFILE_THR

    if ds.phase == eRecover:
        # Handback to the entry attitude at full throttle. DIVE_ATT_THRESH
        # declares the flare complete — then hold the target (altitude-hold).
        q_target[:] = ds.q_entry[:]
        qe = [0.0]*4
        q_error(qe, q_target, q_est)
        if qe[1]**2 + qe[2]**2 + qe[3]**2 < DIVE_ATT_THRESH**2:
            ds.phase = eIdle
            ds.armed = False
            return False, stick_throttle
        return True, 1.0

    if ds.phase == eSpiral:
        return True, DIVE_PROFILE_THR

    return False, stick_throttle


# ── Simulation ──

def simulate_dive(start_alt=100.0, orbit_alt=0.0, engage_t=2.0,
                  release_t=None, gyro_bias=EMU_GYRO_BIAS_RAD_S,
                  sim_t=45.0):
    q_true = [1.0, 0.0, 0.0, 0.0]
    q_est = [1.0, 0.0, 0.0, 0.0]
    rates = [0.0, 0.0, 0.0]
    altitude = start_alt
    roc = 0.0
    altitude_out = 0.0  # fake_accu / specific force vertical term
    motor_lag = 0.0
    confp = 1.0
    alt_hold_comp = 0.0

    pid_roll = PID(Kp=RATE_KP, Kd=RATE_KD, Max=EM_MAX_ROLL_RATE)
    pid_pitch = PID(Kp=RATE_KP, Kd=RATE_KD, Max=EM_MAX_PITCH_RATE)
    pid_yaw = PID(Kp=RATE_KP, Kd=RATE_KD, Max=EM_MAX_YAW_RATE)

    iner_r = 12.0 / (EM_MASS * EM_ARM_LEN * EM_ARM_LEN)   # emu.c MR
    iner = [iner_r, iner_r/1.5, iner_r/2.5]

    ds = DiveState()
    q_target = [0.0]*4
    q_dive = [0.0]*4
    qe = [0.0]*4

    n = int(sim_t / CONTROL_DT)
    log = []
    max_rate = [EM_MAX_PITCH_RATE, EM_MAX_PITCH_RATE, EM_MAX_YAW_RATE]

    dive_was_active = False
    hold_target = None

    for i in range(n):
        t = i * CONTROL_DT
        dive_mode = t >= engage_t and (release_t is None or t < release_t)

        q_est[:] = q_est
        q_target[:] = [0]*4
        active, desired_throttle = do_dive_control(
            ds, dive_mode, altitude, roc, q_est, q_target, orbit_alt,
            STICK_THROTTLE, CONTROL_DT)
        if active:
            dive_was_active = True

        # Attitude error source: dive target when active, level otherwise.
        if active:
            q_error(qe, q_target, q_est)
        else:
            hold_level = [1.0, 0.0, 0.0, 0.0]
            q_error(qe, hold_level, q_est)

        qa = [qe[1], qe[2], qe[3]]
        pid_roll.Desired = clamp(2.0 * qa[0] * QUAT_KP, -max_rate[0], max_rate[0])
        pid_pitch.Desired = clamp(2.0 * qa[1] * QUAT_KP, -max_rate[1], max_rate[1])
        pid_yaw.Desired = clamp(2.0 * qa[2] * QUAT_KP, -max_rate[2], max_rate[2])

        out_roll, _ = run_rate_pd(pid_roll, rates[0], CONTROL_DT)
        out_pitch, _ = run_rate_pd(pid_pitch, rates[1], CONTROL_DT)
        out_yaw, _ = run_rate_pd(pid_yaw, rates[2], CONTROL_DT)

        # ── Rotational plant (emu.c MR): Rate -= (T*0.25*Out*L*InR - damp) ──
        for a, out in ((1, out_pitch), (0, out_roll)):
            damp = 2.0 * signf(rates[a]) * rates[a] * rates[a]
            rates[a] -= (EM_MAX_THRUST * 0.25 * out * EM_ARM_LEN * iner[a]
                         - damp) * CONTROL_DT
        damp_y = 2.0 * signf(rates[2]) * rates[2] * rates[2]
        rates[2] += (EM_MAX_THRUST * 0.015 * out_yaw * EM_ARM_LEN * iner[2]
                     * motor_lag - damp_y) * CONTROL_DT
        rates[2] = clamp(rates[2], -EM_MAX_YAW_RATE, EM_MAX_YAW_RATE)
        rates[0] = clamp(rates[0], -EM_MAX_ROLL_RATE, EM_MAX_ROLL_RATE)
        rates[1] = clamp(rates[1], -EM_MAX_PITCH_RATE, EM_MAX_PITCH_RATE)

        # ── Translational plant (emu.c MR): motor lag -> thrust -> ROC ──
        if active:
            alt_hold_comp = 0.0
        else:
            # Simple altitude hold after handback / before engage.
            target = ds.target_alt if hold_target is not None else start_alt
            alt_hold_comp = clamp((target - altitude) * 0.05 - roc * 0.10,
                                  -0.25, 0.25)
            desired_throttle = clamp(STICK_THROTTLE + alt_hold_comp, 0.2, 0.9)

        motor_input = clamp((desired_throttle + alt_hold_comp)
                            * attitude_cosine(q_est), 0.0, 1.0)
        alpha_m = CONTROL_DT / (EM_MOTOR_TAU + CONTROL_DT)
        motor_lag += alpha_m * (motor_input - motor_lag)
        thrust = motor_lag * EM_MAX_THRUST
        fake_accu = (thrust - EM_MASS * GRAVITY - drag(roc)) / EM_MASS
        roc += fake_accu * CONTROL_DT
        altitude += roc * CONTROL_DT
        if altitude <= 0.0:
            altitude = 0.0
            roc = max(roc, 0.0)
            fake_accu = max(fake_accu, 0.0)

        # ── True attitude (plant) from the true, unbissed rates ──
        q_integrate(q_true, tuple(rates), CONTROL_DT)

        # ── Accel synthesis: TRUE specific force from plant dynamics ──
        # Specific force in world: f_world = a_world - g_world (both up-positive).
        # World accel components:
        #   a_z = FakeAccU (vertical, already includes thrust cosθ - mg - drag)
        #   a_x = sin(pitch)*thrust_accel - drag_x  (thrust tilt horizontal component)
        #   a_y = -sin(roll)*thrust_accel - drag_y
        # g_world = (0, 0, -G).  f_world = (a_x, a_y, a_z + G).
        # Rotate f_world into body frame using TRUE quaternion.
        thrust_accel = motor_lag * EM_MAX_THRUST / EM_MASS
        # Drag is already applied in FakeAccU calculation, skip horizontal drag
        rr, pp, yy = quat_to_euler(q_true)
        sp, cp = math.sin(pp), math.cos(pp)
        sr, cr = math.sin(rr), math.cos(rr)
        # World specific force (up-positive world frame):
        fx = sp * thrust_accel
        fy = -sr * cp * thrust_accel
        fz = fake_accu + GRAVITY
        # World→body rotation (q_true): body = R_world2body · f_world
        # R from q: R[0][2] = 2(q1q3 + q0q2), etc.  Use euler for clarity.
        # f_body_x = cp*fy + sp*fz - sy*sx...  Just use the matrix from q:
        q0,q1,q2,q3 = q_true
        R = [
            [q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
            [2*(q1*q2+q0*q3), q0*q0-q1*q1+q2*q2-q3*q3, 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0*q0-q1*q1-q2*q2+q3*q3]
        ]
        f_body = [R[0][0]*fx + R[0][1]*fy + R[0][2]*fz,
                  R[1][0]*fx + R[1][1]*fy + R[1][2]*fz,
                  R[2][0]*fx + R[2][1]*fy + R[2][2]*fz]
        # FC Acc[] convention: UD positive DOWN.  f_body is up-positive.
        acc = [f_body[0], f_body[1], -f_body[2]]
        acc_mag = math.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)

        # ── Confidence fold: emulated accel drives the real model ──
        confp = calculate_acc_confidence(confp, acc_mag, acc[2])
        two_kp_acc = P_TWO_KP_ACC * confp

        # ── Estimator: measured gyro = true + bounded dive-phase bias ──
        gyro = list(rates)
        if ds.phase in (eActive, eRecover):
            gyro[1] += gyro_bias          # pitch-axis probe (emu.c fold)
        madgwick_step(q_est, gyro, acc, two_kp_acc, CONTROL_DT)

        if active and ds.flying:
            hold_target = ds.target_alt if ds.target_alt > 0 else None

        if i % 50 == 0:
            rr_e, pp_e, yy_e = quat_to_euler(q_est)
            rr_t, pp_t, yy_t = quat_to_euler(q_true)
            log.append({
                't': t, 'alt': altitude, 'roc': roc,
                'pitch_true': pp_t * RAD_TO_DEG, 'pitch_est': pp_e * RAD_TO_DEG,
                'roll_true': rr_t * RAD_TO_DEG,
                'pitch_rate': rates[1] * RAD_TO_DEG,
                'phase': ds.phase, 'dive_mode': dive_mode,
                'throttle': desired_throttle, 'conf': confp,
                'acc_mag_g': acc_mag / GRAVITY,
                'err_deg': attitude_error_deg(q_true, q_est),
                'flare_t': ds.flare_t,
            })

    return log, ds


# ── Reporting ──

G = "\033[92m"
Y = "\033[93m"
R = "\033[91m"
B = "\033[1m"
N = "\033[0m"

def analysis(log, title, bias):
    print(f"\n{B}── {title} ──{N}")
    engage = next((e for e in log if e['phase'] == eActive), None)
    active_end = next((e for e in reversed(log) if e['phase'] != eIdle), None)
    handback = next((e for e in log if e['phase'] == eIdle
                     and e['t'] > (engage['t'] if engage else 0.0)
                     and e['t'] > 2.0 and e['t'] > (engage['t'] if engage else 0.0) + 1.0), None)

    t_engage = engage['t'] if engage else None
    t_handback = handback['t'] if handback else None

    prof = [e for e in log if e['t'] >= (t_engage or 0.0) and e['t'] <= (t_handback or 1e9)]
    total_profile = (t_handback - t_engage) if (t_handback and t_engage) else None

    min_conf = min((e['conf'] for e in prof), default=1.0)
    end_conf = prof[-1]['conf'] if prof else 1.0
    min_alt = min(e['alt'] for e in log)
    max_err = max((e['err_deg'] for e in prof), default=0.0)
    max_err_t = max(prof, key=lambda e: e['err_deg'])['t'] if prof else 0.0
    max_pitch = max(abs(e['pitch_true']) for e in log)

    max_rate_all = max(abs(e['pitch_rate']) for e in log)

    env_pass = total_profile is not None and total_profile <= DIVE_MAX_PROFILE_S + 0.5
    crash_pass = min_alt > 0.5
    conf_pass = min_conf <= 0.10 and end_conf >= 0.90
    bias_pass = max_err < 5.0

    def mark(ok): return f"[{G}PASS{N}]" if ok else f"[{R}FAIL{N}]"

    print(f"  Engage:              t={t_engage:.1f}s" if t_engage else "  Engage:              never")
    print(f"  Handback to AH:      t={t_handback:.1f}s" if t_handback else "  Handback:            never")
    print(f"  Total dive cycle:    {total_profile:.1f}s  ({mark(env_pass)} envelope <= {DIVE_MAX_PROFILE_S:.0f}s)"
          if total_profile else "  Total dive cycle:    n/a")
    print(f"  Min altitude:        {min_alt:.1f}m  ({mark(crash_pass)} no crash)")
    print(f"  Max pitch reached:   {max_pitch:.1f} deg")
    print(f"  Max pitch rate:      {max_rate_all:.0f} deg/s")
    print(f"  Acc confidence:      min {min_conf:.3f} -> end {end_conf:.3f}"
          f"  ({mark(conf_pass)} collapse in dive, re-blend at flare)")
    if bias:
        print(f"  Max true/est error:  {max_err:.2f} deg at t={max_err_t:.1f}s ({mark(bias_pass)} bias drift < 5 deg)")
        print(f"  WARNING: gyro bias {"ON" if bias else "OFF"}")
    return (env_pass, crash_pass, conf_pass, bias_pass)


def pprint_timeline(log, step=25):
    last_phase = None
    for e in log[::step]:
        marker = "  <--" if e['phase'] != last_phase and last_phase is not None else ""
        print(f"{e['t']:5.1f} alt={e['alt']:7.1f} roc={e['roc']:6.1f} "
              f"pitch_t={e['pitch_true']:6.1f} pitch_e={e['pitch_est']:6.1f} "
              f"conf={e['conf']:5.2f} err={e['err_deg']:5.2f} "
              f"thr={e['throttle']:4.2f} {PHASE_NAME[e['phase']]:>8}{marker}")
        last_phase = e['phase']


def main():
    print(f"{B}UAVXArmQ Crash-Descent Emulator-Fold Simulation{N}")
    print("="*72)
    print(f"  Pitch profile: punch to 60 deg/2s, cruise 45 deg, flare 0 deg/5s, "
          f"thr {DIVE_PROFILE_THR}")
    print(f"  DIVE_MAX_PROFILE_S = {DIVE_MAX_PROFILE_S:.0f}s   "
          f"pDiveRecoverAlt = {PDIVE_RECOVER_ALT:.0f}m   "
          f"gyro bias = {EMU_GYRO_BIAS_RAD_S*RAD_TO_DEG:.2f} deg/s")
    print("="*72)

    results = []

    # Scenario A — nominal dive, 100 m -> 15 m floor, supervisory flare.
    log, _ = simulate_dive(start_alt=100.0, orbit_alt=0.0)
    print(f"\n{B}Scenario A: nominal 100 m dive (gyro bias ON){N}")
    pprint_timeline(log)
    results.append(analysis(log, "Scenario A nominal", True))

    # Scenario B — mid-dive abort (switch release) forces a smooth flare.
    logb, _ = simulate_dive(start_alt=100.0, orbit_alt=0.0, release_t=6.0)
    print(f"\n{B}Scenario B: abort at t=6.0 s (switch released){N}")
    pprint_timeline(logb)
    results.append(analysis(logb, "Scenario B abort", True))

    # Scenario C — full 30 s envelope: budget far exceeds the profile cap.
    logc, dsc = simulate_dive(start_alt=500.0, orbit_alt=0.0, sim_t=40.0)
    print(f"\n{B}Scenario C: 500 m start — profile hits the 30 s ceiling{N}")
    pprint_timeline(logc)
    results.append(analysis(logc, "Scenario C envelope", True))

    # Scenario D — bias OFF control: the same nominal dive without corruption.
    logd, _ = simulate_dive(start_alt=100.0, orbit_alt=0.0, gyro_bias=0.0)
    print(f"\n{B}Scenario D: nominal dive, gyro bias OFF (baseline){N}")
    pprint_timeline(logd)
    results.append(analysis(logd, "Scenario D baseline", False))

    all_pass = all(all(r) for r in results)
    print(f"\n{'='*72}")
    print(f"{B}Overall: {'['+G+'PASS'+N+'] all chains & envelope hold' if all_pass
          else '['+R+'FAIL'+N+'] investigate the FAIL above'}")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()