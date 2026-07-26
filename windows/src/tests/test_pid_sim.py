#!/usr/bin/env python3
"""
Autonomous PID Parameter Critique via Python-Side Simulation.

Replicates the PID cascade from UAVXArm32F4/src/control.c:
  Angle loop (PI): P.PTerm = Error * P.Kp; ITerm = IntE; R.Desired = PTerm + ITerm
  Rate loop  (PD): R.PTerm = Error * R.Kp; DTerm = LPF(d(error)/dt) * Kd; Out = PTerm + DTerm

Physics:
  MR:   torque = EM_MAX_THRUST × 0.25 × Out × EM_ARM_LEN × InertiaR  (matches emu.c)
  FW:   torque = qbar × S × C_ctrl × surface_deflection  (aerodynamic model)
        damping = C_d × rate² × sign(rate)  (quadratic aerodynamic damping)
        adverse yaw: aileron deflection couples into yaw axis (Bixler-class effect)

Airframe category selects between MR / FW / VTOL physics models.

Usage:
  python3 src/tests/test_pid_sim.py [airframe_name]

Exits 0 if all axes pass, 1 if any axis has tuning issues.
"""

import math
import sys
import os
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict

# Find project root (UAVXGS directory)
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AIRFRAMES_DIR = os.path.join(PROJECT_ROOT, "uavx-python", "src", "airframes")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0
GRAVITY = 9.80665

# ═══════════════════════════════════════════
#  Airframe Category (matches FC AirframeCategory enum)
# ═══════════════════════════════════════════
class AirframeCat:
    MR   = 0  # multirotor
    FW   = 1  # fixed wing
    VTOL = 2  # quadplane
    LAND = 3  # rover

# ═══════════════════════════════════════════
#  Active airframe configs — loaded from protocol_enums
# ═══════════════════════════════════════════
from protocol_enums import AirframeType, ACTIVE_AIRFRAMES, AIRFRAME_NAMES, REDACTED_AIRFRAMES

# Map AirframeType enum value → category
AF_CATEGORY = {
    AirframeType.QUAD_X: AirframeCat.MR,
    AirframeType.HEX_X: AirframeCat.MR,
    AirframeType.OCT_X: AirframeCat.MR,
    AirframeType.DELTA: AirframeCat.FW,
    AirframeType.AILERON_SPOILER_FLAPS: AirframeCat.FW,
    AirframeType.RUDDER_ELEVATOR: AirframeCat.FW,
    AirframeType.DIFFERENTIAL_TWIN: AirframeCat.FW,
    AirframeType.TRACKED: AirframeCat.LAND,
    AirframeType.GIMBAL: AirframeCat.LAND,
    AirframeType.INSTRUMENTATION: AirframeCat.LAND,
    # Redacted types with _Tuned files:
    AirframeType.AILERON: AirframeCat.FW,
}

# FW model index per active FW airframe (for physics params)
FW_MODEL_IDX = {
    AirframeType.DELTA: 2,
    AirframeType.AILERON_SPOILER_FLAPS: 3,
    AirframeType.RUDDER_ELEVATOR: 5,
    AirframeType.DIFFERENTIAL_TWIN: 1,  # approximates as AileronAF
    AirframeType.AILERON: 1,          # Sky Surfer uses AileronAF model
}

# .af file to AirframeType mapping (from actual .af files)
AF_FILES = {
    # MR - QuadXAF (tag 4)
    "EcksTuned.af": AirframeType.QUAD_X,
    "Ecks_220mm_Tuned.af": AirframeType.QUAD_X,
    "Ecks_800g_Moderate.af": AirframeType.QUAD_X,
    "Ecks_800g_Sport.af": AirframeType.QUAD_X,
    "Ecks_1kg_Moderate.af": AirframeType.QUAD_X,
    "Ecks_1kg_Sport.af": AirframeType.QUAD_X,
    "DevEBox_Tuned.af": AirframeType.QUAD_X,
    "S500_1137_Tuned.af": AirframeType.QUAD_X,
    "Ken_s_Alpha_Test_Tuned.af": AirframeType.QUAD_X,
    "Ken_s_450_1165_Tuned.af": AirframeType.QUAD_X,
    "Ken_s_LadyBug_Tuned.af": AirframeType.QUAD_X,
    "150mm_Brushed_Tuned.af": AirframeType.QUAD_X,
    # FW - AileronSpoilerFlapsAF (tag 16)
    "Spoileron_Tuned.af": AirframeType.AILERON_SPOILER_FLAPS,
    # FW - RudderElevatorAF (tag 18)
    "Wing_900mm_Tuned.af": AirframeType.RUDDER_ELEVATOR,
    "Phoenix_FW_Tuned.af": AirframeType.RUDDER_ELEVATOR,
    # FW - AileronAF (tag 15) - redacted but present in files
    "Sky_Surfer_Test_Tuned.af": AirframeType.AILERON,
    # FW - DeltaAF (tag 14) - none currently in _Tuned files
}

AIR_DENSITY = 1.225          # kg/m³ at sea level

# Per-airframe physical descriptors for aerodynamic simulation.
# Each entry defines the real-world geometry that drives torque and damping.
# Control derivatives (C_l_ail, C_m_elev, C_n_rud) computed from these.
FW_AIRFRAMES = {
    # Sky Surfer / Bixler 1800mm: 1000kv, 10x6, ~800g thrust
    # CL_α≈4.7, SM=3%→Cm_α=-0.14, Vh≈0.12→Cm_δe≈0.50
    "Sky_Surfer_Test_Tuned.af": {
        "mass": 1.0, "wingspan": 1.8, "wing_area": 0.28,
        "cruise_speed": 12.0,
        "aileron_area": 0.0126, "aileron_arm": 0.35,
        "elevator_area": 0.0081, "elevator_arm": 0.65,
        "rudder_area": 0.012, "rudder_arm": 0.75,
        "aileron_max_deg": 20.0, "elevator_max_deg": 15.0, "rudder_max_deg": 25.0,
        # Control derivatives (dimensionless, per radian of surface deflection)
        "CL_D_AIL": 0.040, "CM_D_ELE": 0.50, "CN_D_RUD": 0.12,
        # Aerodynamic damping coefficients
        "pitch_damp": -12.0, "yaw_damp": -0.15,
        # Cross-coupling
        "adverse_yaw": 0.15, "dihedral_coeff": 0.18,
        # Static stability (Cm_α, Cn_β — per radian)
        "pitch_stability": -0.14, "yaw_stability": 0.03,
        # Linear damping at low rates (Nm per rad/s)
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.04, "yaw_damp_lin": 0.015,
        "servo_tau": 0.08,
    },
    # Wing 900mm: small flying wing, elevon control (no tail → low pitch authority)
    # CL_α≈4.9, SM=3%→Cm_α=-0.15, elevon Cm_δe≈0.20
    "Wing_900mm_Tuned.af": {
        "mass": 0.55, "wingspan": 0.9, "wing_area": 0.11,
        "cruise_speed": 13.0,
        "aileron_area": 0.0044, "aileron_arm": 0.25,
        "elevator_area": 0.0044, "elevator_arm": 0.25,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 20.0, "elevator_max_deg": 15.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.20, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.01,
        "pitch_stability": -0.15, "yaw_stability": 0.01,
        "roll_damp_lin": 0.01, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # Phoenix: 1kg Radian-class, rudder+elevator (no ailerons)
    # CL_α≈5.4, SM=4%→Cm_α=-0.22, Vh≈0.19→Cm_δe≈0.60
    "Phoenix_FW_Tuned.af": {
        "mass": 1.0, "wingspan": 1.6, "wing_area": 0.22,
        "cruise_speed": 13.0,
        "aileron_area": 0.0, "aileron_arm": 0.0,
        "elevator_area": 0.008, "elevator_arm": 0.60,
        "rudder_area": 0.012, "rudder_arm": 0.70,
        "aileron_max_deg": 0.0, "elevator_max_deg": 15.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.0, "CM_D_ELE": 0.60, "CN_D_RUD": 0.10,
        "pitch_damp": -10.0, "yaw_damp": -0.12,
        "adverse_yaw": 0.0, "dihedral_coeff": 0.12,
        "pitch_stability": -0.22, "yaw_stability": 0.04,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.05, "yaw_damp_lin": 0.02,
        "servo_tau": 0.08,
    },
    # Spoileron: trainer with spoilerons + flaps + rudder
    # CL_α≈5.1, SM=3%→Cm_α=-0.15, Vh≈0.14→Cm_δe≈0.45
    "Spoileron_Tuned.af": {
        "mass": 0.65, "wingspan": 1.2, "wing_area": 0.16,
        "cruise_speed": 14.0,
        "aileron_area": 0.0096, "aileron_arm": 0.28,
        "elevator_area": 0.0084, "elevator_arm": 0.40,
        "rudder_area": 0.006, "rudder_arm": 0.50,
        "aileron_max_deg": 20.0, "elevator_max_deg": 15.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.035, "CM_D_ELE": 0.45, "CN_D_RUD": 0.08,
        "pitch_damp": -9.0, "yaw_damp": -0.10,
        "adverse_yaw": 0.10, "dihedral_coeff": 0.08,
        "pitch_stability": -0.15, "yaw_stability": 0.03,
        "roll_damp_lin": 0.015, "pitch_damp_lin": 0.03, "yaw_damp_lin": 0.012,
        "servo_tau": 0.08,
    },
}

# Derived inertia and max-rate tuples (roll_pitch, yaw) — computed from physical params
def _compute_fw_inertia(af: dict) -> Tuple[float, float]:
    m, b, l = af["mass"], af["wingspan"], af["wing_area"] / af["wingspan"]
    I_roll  = m * b * b / 12.0
    I_pitch = m * (b * b + l * l) / 12.0
    return (I_roll, I_pitch)

def _compute_fw_max_rates(af: dict) -> Tuple[float, float, float]:
    rho, V = AIR_DENSITY, af["cruise_speed"]
    qbar = 0.5 * rho * V * V
    b, S, m = af["wingspan"], af["wing_area"], af["mass"]
    I_roll, I_pitch = _compute_fw_inertia(af)
    chord = S / b

    C_l_ail = qbar * S * b * af.get("CL_D_AIL", 0.0)
    C_m_ele = qbar * S * chord * af.get("CM_D_ELE", 0.0)
    C_n_rud = qbar * S * b * af.get("CN_D_RUD", 0.0)

    ail_max = af.get("aileron_max_deg", 0.0) * DEG_TO_RAD
    ele_max = af.get("elevator_max_deg", 0.0) * DEG_TO_RAD
    rud_max = af.get("rudder_max_deg", 0.0) * DEG_TO_RAD

    d_roll  = C_l_ail * ail_max / I_roll if I_roll > 1e-6 and ail_max > 0 else 0.0
    d_pitch = C_m_ele * ele_max / I_pitch if I_pitch > 1e-6 and ele_max > 0 else 0.0
    d_yaw   = C_n_rud * rud_max / I_pitch if I_pitch > 1e-6 and rud_max > 0 else 0.0
    return (d_roll, d_pitch, d_yaw)

# Legacy fallback (for airframes not in FW_AIRFRAMES)
FW_INERTIAS = [
    (0.0035, 0.005),   # 0 ElevonAF
    (0.035,  0.05),    # 1 AileronAF
    (0.006,  0.008),   # 2 DeltaAF
    (0.035,  0.05),    # 3 AileronSpoilerFlapsAF
    (0.015,  0.02),    # 4 AileronVTailAF
    (0.02,   0.03),    # 5 RudderElevatorAF
    (0.02,   0.03),    # 6 VTOLAF
]

FW_MAX_RATES = [
    (3.5, 1.2, 1.5),   # 0 ElevonAF
    (1.5, 1.0, 1.0),   # 1 AileronAF
    (3.0, 1.2, 1.5),   # 2 DeltaAF
    (1.5, 1.0, 1.0),   # 3 AileronSpoilerFlapsAF
    (2.5, 1.2, 1.2),   # 4 AileronVTailAF
    (2.0, 1.0, 1.0),   # 5 RudderElevatorAF
    (2.5, 1.2, 1.5),   # 6 VTOLAF
]

AXIS_NAMES = {"Roll": 0, "Pitch": 1, "Yaw": 2}

# ── Common constants (matching emu.h / emu.c) ──
EM_SERVO_TAU = 0.08         # servo lag (FW), seconds
EM_MOTOR_TAU = 0.10         # motor lag (MR)
CONTROL_DT = 0.001
SIM_TIME = 8.0
WARMUP_TIME = 0.1

# MR-specific (emu.h defaults)
EM_MASS = 0.8
EM_ARM_LEN = 0.17
EM_THR_CRUISE = 0.55
EM_MAX_THRUST = EM_MASS / EM_THR_CRUISE * GRAVITY
_IR = 12.0 / (EM_MASS * EM_ARM_LEN * EM_ARM_LEN)  # RollPitchInertiaR
MR_INERTIA_R = [_IR, _IR / 1.5, _IR / 2.5]        # Roll, Pitch, Yaw

DTERM_LPF_HZ = 50.0

# Test step sizes (degrees)
TEST_STEPS = {"Roll": 15.0, "Pitch": 10.0, "Yaw": 45.0}
# Gust magnitudes for disturbance rejection
GUST_MAG = {"Roll": 0.02, "Pitch": 0.015, "Yaw": 0.01}

# ═══════════════════════════════════════════
#  PID structs
# ═══════════════════════════════════════════
@dataclass
class PIStruct:
    Desired: float = 0.0
    Error: float = 0.0
    Kp: float = 0.0
    Ki: float = 0.0
    IntE: float = 0.0
    IntLim: float = 0.0
    Max: float = 0.0
    PTerm: float = 0.0
    ITerm: float = 0.0

@dataclass
class PIDStruct:
    Desired: float = 0.0
    Error: float = 0.0
    Kp: float = 0.0
    Ki: float = 0.0
    IntE: float = 0.0
    IntLim: float = 0.0
    Kd: float = 0.0
    Max: float = 0.0
    PTerm: float = 0.0
    ITerm: float = 0.0
    DTerm: float = 0.0
    last_error: float = 0.0
    d_filtered: float = 0.0

# ═══════════════════════════════════════════
#  Disturbance model
# ═══════════════════════════════════════════
@dataclass
class Gust:
    """A torque disturbance representing a wind gust."""
    magnitude: float = 0.0       # N·m
    start_s: float = 2.0         # seconds
    duration_s: float = 0.5      # duration
    axis: str = "Roll"           # which axis

    def torque(self, t: float) -> float:
        if self.start_s <= t <= self.start_s + self.duration_s:
            return self.magnitude
        return 0.0

# ═══════════════════════════════════════════
#  Metrics
# ═══════════════════════════════════════════
@dataclass
class StepMetrics:
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
    peak_deviation_rad: float = 0.0
    settling_time_s: float = 0.0
    integrated_error: float = 0.0
    steady_rate_error: float = 0.0

# ═══════════════════════════════════════════
#  Control functions
# ═══════════════════════════════════════════
def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def sgn(v: float) -> float:
    return 1.0 if v >= 0 else -1.0

def run_angle_loop(pi: PIStruct, angle: float, stick: float, nav_corr: float, dT: float) -> float:
    pi.Desired = stick * pi.Max + nav_corr
    clamped_desired = clamp(pi.Desired, -pi.Max, pi.Max)
    pi.Error = clamped_desired - angle
    pi.PTerm = pi.Error * pi.Kp
    pi.IntE = clamp(pi.IntE + pi.Error * pi.Ki * dT, -pi.IntLim, pi.IntLim)
    pi.ITerm = pi.IntE
    return pi.PTerm + pi.ITerm

def run_rate_pd(pid: PIDStruct, angle_rate: float, dT: float) -> float:
    clamped_desired = clamp(pid.Desired, -pid.Max, pid.Max)
    pid.Error = clamped_desired - angle_rate
    pid.PTerm = pid.Error * pid.Kp
    error_deriv = (pid.Error - pid.last_error) / dT if dT > 0 else 0.0
    tau_d = 1.0 / (2.0 * math.pi * DTERM_LPF_HZ)
    alpha = dT / (tau_d + dT)
    pid.d_filtered += alpha * (error_deriv - pid.d_filtered)
    pid.DTerm = pid.d_filtered * pid.Kd
    pid.last_error = pid.Error
    return clamp(pid.PTerm + pid.DTerm, -1.0, 1.0)

# ═══════════════════════════════════════════
#  Physics (matching emu.c exactly)
# ═══════════════════════════════════════════
def run_physics_mr(angle: float, rate: float, out: float, lag: float, dT: float,
                   axis: str = "Roll") -> Tuple[float, float, float]:
    """Multirotor physics — torque from motor differential thrust."""
    alpha = dT / (EM_MOTOR_TAU + dT)
    effort = lag + alpha * (out - lag)

    kT = EM_MAX_THRUST * 0.25 * EM_ARM_LEN
    torque = kT * effort
    damping = 2.0 * sgn(rate) * rate * rate
    inertia_r = MR_INERTIA_R[AXIS_NAMES[axis]]
    dRate = (torque - damping) * inertia_r * dT
    rate += dRate
    angle += rate * dT
    return angle, rate, effort


def run_physics_fw(angle: float, rate: float, out: float, lag: float, dT: float,
                   r_max: float, inertia: float, max_rate_limit: float,
                   axis: str = "Roll",
                   af_params: Optional[dict] = None) -> Tuple[float, float, float]:
    """Fixed-wing physics with aerodynamic torque and damping.

    Control torque:  L = qbar × S × b × CL_D_AIL × δ_a   (roll)
                    M = qbar × S × c × CM_D_ELE × δ_e   (pitch)
                    N = qbar × S × b × CN_D_RUD × δ_r   (yaw)
    Damping:         quadratic, rate² × sign(rate), axis-dependent
    Adverse yaw:     aileron deflection couples into yaw axis

    af_params keys: mass, wingspan, wing_area, cruise_speed,
        aileron_area/arm/max_deg, elevator_area/arm/max_deg, rudder_area/arm/max_deg,
        CL_D_AIL, CM_D_ELE, CN_D_RUD,
        pitch_damp, yaw_damp, adverse_yaw, servo_tau
    """
    if af_params is None:
        # Fallback: old simple model (emu.c original)
        alpha = dT / (EM_SERVO_TAU + dT)
        effort = lag + alpha * (out - lag)
        torque = effort * r_max * inertia
        damping = 2.0 * sgn(rate) * rate * rate
        dRate = (torque - damping) * dT
        rate += dRate
        rate = clamp(rate, -max_rate_limit, max_rate_limit)
        angle += rate * dT
        return angle, rate, effort

    servo_tau = af_params.get("servo_tau", EM_SERVO_TAU)
    alpha = dT / (servo_tau + dT)
    effort = lag + alpha * (out - lag)

    rho = AIR_DENSITY
    V = af_params["cruise_speed"]
    qbar = 0.5 * rho * V * V
    S = af_params["wing_area"]
    b = af_params["wingspan"]
    chord = S / b

    # Moment of inertia (kg·m²) — thin rod approximation
    I_roll  = af_params["mass"] * b * b / 12.0
    I_pitch = af_params["mass"] * (b * b + chord * chord) / 12.0
    I_yaw   = I_pitch

    I = {"Roll": I_roll, "Pitch": I_pitch, "Yaw": I_yaw}[axis]
    I_eff = max(I, 1e-6)

    # Surface deflection in radians (PID output → surface angle)
    ail_rad = 0.0
    ele_rad = 0.0
    rud_rad = 0.0

    if axis == "Roll":
        surface_max = af_params.get("aileron_max_deg", 0.0) * DEG_TO_RAD
        ail_rad = clamp(effort, -1.0, 1.0) * surface_max if surface_max > 0 else 0.0
    elif axis == "Pitch":
        surface_max = af_params.get("elevator_max_deg", 0.0) * DEG_TO_RAD
        ele_rad = clamp(effort, -1.0, 1.0) * surface_max if surface_max > 0 else 0.0
    elif axis == "Yaw":
        surface_max = af_params.get("rudder_max_deg", 0.0) * DEG_TO_RAD
        rud_rad = clamp(effort, -1.0, 1.0) * surface_max if surface_max > 0 else 0.0

    # Aerodynamic control torque (N·m)
    # L = qbar × S × b × CL_D_AIL × δ_a
    # M = qbar × S × c × CM_D_ELE × δ_e
    # N = qbar × S × b × CN_D_RUD × δ_r
    C_l_ail = qbar * S * b * af_params.get("CL_D_AIL", 0.0)
    C_m_ele = qbar * S * chord * af_params.get("CM_D_ELE", 0.0)
    C_n_rud = qbar * S * b * af_params.get("CN_D_RUD", 0.0)

    torque = {"Roll": C_l_ail * ail_rad,
              "Pitch": C_m_ele * ele_rad,
              "Yaw": C_n_rud * rud_rad}[axis]

    # Quadratic aerodynamic damping (opposes rotation — all values positive magnitudes)
    C_d_roll  = abs(0.5 * rho * V * b * b * b * 0.04)
    C_d_pitch = abs(af_params.get("pitch_damp", 10.0))
    C_d_yaw   = abs(af_params.get("yaw_damp", 0.10)) if af_params.get("rudder_area", 0) > 0 else 0.2

    damp_coeff = {"Roll": C_d_roll, "Pitch": C_d_pitch, "Yaw": C_d_yaw}[axis]
    damping = damp_coeff * rate * abs(rate)

    # Adverse yaw: aileron deflection couples into yaw (Bixler-class effect)
    adverse_yaw_coeff = af_params.get("adverse_yaw", 0.0)
    adverse_yaw_torque = 0.0
    if axis == "Yaw" and adverse_yaw_coeff > 0 and af_params.get("aileron_area", 0) > 0:
        adverse_yaw_torque = adverse_yaw_coeff * C_l_ail * ail_rad

    # Dihedral: yaw rate creates sideslip → roll moment (positive yaw → positive roll)
    dihedral_coeff = af_params.get("dihedral_coeff", 0.0)
    dihedral_torque = 0.0
    if axis == "Roll" and dihedral_coeff > 0:
        # Cross-axis yaw rate passed via r_max (overloaded for coupled sim)
        pass  # Handled in coupled simulator below

    # Integrate: α = τ / I, rate += α × dt, angle += rate × dt
    dRate = (torque + adverse_yaw_torque - damping) / I_eff * dT
    rate += dRate
    rate = clamp(rate, -max_rate_limit, max_rate_limit)
    angle += rate * dT
    return angle, rate, effort

# ═══════════════════════════════════════════
#  Simulators
# ═══════════════════════════════════════════
def simulate_axis(pi: PIStruct, pid: PIDStruct, step_rad: float, max_angle_rad: float,
                  dT: float = CONTROL_DT,
                  cat: int = AirframeCat.MR, fw_model_idx: int = -1,
                  axis_name: str = "Roll",
                  gust: Optional[Gust] = None,
                  af_filename: str = "") -> StepMetrics:
    angle = 0.0
    rate = 0.0
    lag = 0.0
    ai = AXIS_NAMES[axis_name]

    is_fw = (cat == AirframeCat.FW)
    model_max_rate = 100.0
    inertia = 1.0 / MR_INERTIA_R[ai]
    r_max = pid.Max
    af_params = None

    if is_fw and af_filename in FW_AIRFRAMES:
        af_params = FW_AIRFRAMES[af_filename]
        I_roll, I_pitch = _compute_fw_inertia(af_params)
        inertia = I_pitch if axis_name == "Yaw" else I_roll
        model_max_rate = _compute_fw_max_rates(af_filename if False else af_params)[AXIS_NAMES[axis_name]]
        r_max = pid.Max
    elif is_fw and fw_model_idx >= 0:
        # Legacy fallback
        rp_i, y_i = FW_INERTIAS[fw_model_idx]
        inertia = y_i if axis_name == "Yaw" else rp_i
        model_max_rate = FW_MAX_RATES[fw_model_idx][ai]
        r_max = pid.Max

    n = int(SIM_TIME / dT)
    n_warmup = int(WARMUP_TIME / dT)
    decimate = 5

    angles, times, intes = [], [], []

    max_int = max_rate = peak_angle = 0.0
    zero_crossings = 0
    prev_angle = 0.0
    crossed_zero = False

    pi.Max = max_angle_rad
    stick = step_rad / max_angle_rad if max_angle_rad > 0 else 0.0

    for i in range(n):
        t = i * dT
        s = stick if i >= n_warmup else 0.0

        desired_rate = run_angle_loop(pi, angle, s, 0.0, dT)
        pid.Desired = clamp(desired_rate, -pid.Max, pid.Max)
        out = run_rate_pd(pid, rate, dT)

        d_torque = gust.torque(t) if gust else 0.0

        if is_fw and fw_model_idx >= 0:
            angle, rate, lag = run_physics_fw(angle, rate, out, lag, dT,
                                               r_max, inertia, model_max_rate, axis_name,
                                               af_params=af_params)
        else:
            angle, rate, lag = run_physics_mr(angle, rate, out, lag, dT, axis_name)

        if d_torque != 0.0:
            if is_fw:
                rate += d_torque / inertia * dT
                rate = clamp(rate, -model_max_rate, model_max_rate)
            else:
                rate += d_torque * MR_INERTIA_R[ai] * dT

        if i % decimate == 0:
            angles.append(angle)
            times.append(t)
            intes.append(pi.IntE)

        max_int = max(max_int, abs(pi.IntE))
        max_rate = max(max_rate, abs(rate))
        peak_angle = max(peak_angle, abs(angle))

        if i > n_warmup * 3:
            if prev_angle * angle < 0 and not crossed_zero:
                zero_crossings += 1
                crossed_zero = True
            elif prev_angle * angle >= 0:
                crossed_zero = False
        prev_angle = angle

    final_angle = angles[-1] if angles else 0.0
    setpoint = step_rad
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

        peak_above = max((a - setpoint for a in angles if a > setpoint), default=0.0)
        if setpoint > 0:
            overshoot = (peak_above / setpoint) * 100.0

    return StepMetrics(
        rise_time_s=round(rise_time, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settling, 4),
        steady_state_error=round(ss_error, 6),
        max_integrator=round(max_int, 6),
        intlim=pi.IntLim,
        max_rate=round(max_rate, 4),
        peak_angle=round(peak_angle, 4),
        final_angle=round(final_angle, 4),
        setpoint=round(setpoint, 4),
        n_oscillations=zero_crossings,
    )


def simulate_rate_disturbance(pid: PIDStruct, dT: float = CONTROL_DT,
                               cat: int = AirframeCat.FW, fw_model_idx: int = -1,
                               axis_name: str = "Roll",
                               gust: Gust = Gust(0.1, 2.0, 0.5),
                               own_inertia: Optional[float] = None,
                               af_filename: str = "") -> DisturbanceMetrics:
    """Pure rate-loop disturbance rejection test (no angle loop)."""
    rate = 0.0
    lag = 0.0
    desired_rate = 0.0  # hold zero rate

    is_fw = (cat == AirframeCat.FW)
    af_params = None
    if is_fw and af_filename in FW_AIRFRAMES:
        af_params = FW_AIRFRAMES[af_filename]
        I_roll, I_pitch = _compute_fw_inertia(af_params)
        inertia = I_pitch if axis_name == "Yaw" else I_roll
        model_max_rate = _compute_fw_max_rates(af_params)[AXIS_NAMES[axis_name]]
    elif is_fw and fw_model_idx >= 0:
        rp_i, y_i = FW_INERTIAS[fw_model_idx]
        inertia = y_i if axis_name == "Yaw" else rp_i
        model_max_rate = FW_MAX_RATES[fw_model_idx][AXIS_NAMES[axis_name]]
    elif own_inertia is not None:
        inertia = own_inertia
        model_max_rate = 100.0
    else:
        inertia = 1.0 / MR_INERTIA_R[AXIS_NAMES[axis_name]]
        model_max_rate = 100.0

    n = int(SIM_TIME / dT)
    total_ie = 0.0
    peak_dev = 0.0
    settle_time = SIM_TIME
    rates = []
    times = []

    gust_start_idx = int(gust.start_s / dT)
    gust_end_idx = int((gust.start_s + gust.duration_s) / dT)

    for i in range(n):
        t = i * dT
        pid.Desired = clamp(desired_rate, -pid.Max, pid.Max)
        out = run_rate_pd(pid, rate, dT)

        d_torque = gust.torque(t)

        if is_fw and fw_model_idx >= 0:
            _, rate, lag = run_physics_fw(0.0, rate, out, lag, dT,
                                           pid.Max, inertia, model_max_rate, axis_name,
                                           af_params=af_params)
        else:
            _, rate, lag = run_physics_mr(0.0, rate, out, lag, dT, axis_name)

        if d_torque != 0.0:
            rate += d_torque / inertia * dT
            rate = clamp(rate, -model_max_rate, model_max_rate) if is_fw else rate

        if i % 10 == 0:
            rates.append(rate)
            times.append(t)

        if gust_start_idx <= i <= gust_end_idx * 2:
            total_ie += abs(rate) * dT
        peak_dev = max(peak_dev, abs(rate))

    # Settling: time after gust end for rate to stay within ±5% of peak deviation
    settle_band = peak_dev * 0.05 + 1e-6
    after_gust = int((gust.start_s + gust.duration_s) / dT)
    settled = False
    for i in range(after_gust, n):
        t = i * dT
        if abs(rate) < settle_band:
            if not settled:
                settle_time = t
                settled = True
            break
        settled = False

    return DisturbanceMetrics(
        peak_deviation_rad=round(peak_dev, 6),
        settling_time_s=round(settle_time - gust.start_s - gust.duration_s if settle_time < SIM_TIME else SIM_TIME, 4),
        integrated_error=round(total_ie, 6),
        steady_rate_error=round(abs(rate), 6),
    )


def simulate_axis_coupled(af_filename: str, step_axis: str = "Roll",
                          dT: float = CONTROL_DT,
                          gusts: Optional[Dict[str, Gust]] = None) -> Dict[str, StepMetrics]:
    """Coupled 3-axis simulation with cross-coupling (dihedral, adverse yaw).

    Runs all three axes simultaneously so that:
      - Yaw rate → dihedral → roll torque
      - Aileron deflection → adverse yaw → yaw torque

    step_axis: which axis receives the step command (others hold zero)
    """
    af_params = FW_AIRFRAMES[af_filename]
    rho = AIR_DENSITY
    V = af_params["cruise_speed"]
    qbar = 0.5 * rho * V * V
    S = af_params["wing_area"]
    b = af_params["wingspan"]
    chord = S / b
    mass = af_params["mass"]

    I_roll  = mass * b * b / 12.0
    I_pitch = mass * (b * b + chord * chord) / 12.0
    I_yaw   = I_pitch

    C_l_ail = qbar * S * b * af_params.get("CL_D_AIL", 0.0)
    C_m_ele = qbar * S * chord * af_params.get("CM_D_ELE", 0.0)
    C_n_rud = qbar * S * b * af_params.get("CN_D_RUD", 0.0)

    C_d_roll  = abs(0.5 * rho * V * b * b * b * 0.04)
    C_d_pitch = abs(af_params.get("pitch_damp", 10.0))
    C_d_yaw   = abs(af_params.get("yaw_damp", 0.10)) if af_params.get("rudder_area", 0) > 0 else 0.2

    # Linear damping at low rates (bearing friction + low-speed aero)
    roll_damp_lin  = af_params.get("roll_damp_lin", 0.02)
    pitch_damp_lin = af_params.get("pitch_damp_lin", 0.04)
    yaw_damp_lin   = af_params.get("yaw_damp_lin", 0.015)

    # Static stability derivatives
    pitch_stab_coeff = af_params.get("pitch_stability", 0.0) * qbar * S * chord
    yaw_stab_coeff = af_params.get("yaw_stability", 0.0) * qbar * S * b

    adverse_yaw_coeff = af_params.get("adverse_yaw", 0.0)
    dihedral_coeff = af_params.get("dihedral_coeff", 0.0)
    servo_tau = af_params.get("servo_tau", EM_SERVO_TAU)

    ail_max_rad = af_params.get("aileron_max_deg", 0.0) * DEG_TO_RAD
    ele_max_rad = af_params.get("elevator_max_deg", 0.0) * DEG_TO_RAD
    rud_max_rad = af_params.get("rudder_max_deg", 0.0) * DEG_TO_RAD

    # Determine surface routing from FC mixer (DoServos)
    af_type = AF_FILES.get(af_filename)
    is_rudder_elevator = (af_type == AirframeType.RUDDER_ELEVATOR)

    # Load params and set up PIDs
    params = load_af_params(af_filename)

    pis = {}   # angle PIs
    pids = {}  # rate PIDs
    angles = {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
    rates   = {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
    lags    = {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}

    step_rads = {"Roll": TEST_STEPS["Roll"] * DEG_TO_RAD,
                 "Pitch": TEST_STEPS["Pitch"] * DEG_TO_RAD,
                 "Yaw": TEST_STEPS["Yaw"] * DEG_TO_RAD}

    for name in ["Roll", "Pitch", "Yaw"]:
        akp, aki, ail, ma, rkp, rkd, mr = get_axis_params(params, name)
        pis[name]  = PIStruct(Kp=akp, Ki=aki, IntLim=ail, Max=ma)
        pids[name] = PIDStruct(Kp=rkp, Kd=rkd, Max=mr)

    n = int(SIM_TIME / dT)
    n_warmup = int(WARMUP_TIME / dT)
    decimate = 5

    all_angles = {name: [] for name in ["Roll", "Pitch", "Yaw"]}
    all_times = []
    metrics = {}

    for name in ["Roll", "Pitch", "Yaw"]:
        pis[name].Max = step_rads[name] if name == step_axis else 0.523599

    for i in range(n):
        t = i * dT
        stick = 1.0 if i >= n_warmup and step_axis else 0.0
        step_s = stick if i >= n_warmup else 0.0

        # Angle loop → desired rate → rate PD → effort
        efforts = {}
        for name in ["Roll", "Pitch", "Yaw"]:
            s = step_s if name == step_axis else 0.0
            desired_rate = run_angle_loop(pis[name], angles[name], s, 0.0, dT)
            pids[name].Desired = clamp(desired_rate, -pids[name].Max, pids[name].Max)
            efforts[name] = run_rate_pd(pids[name], rates[name], dT)

        # Surface deflections from efforts — matches FC DoServos() mixing
        if is_rudder_elevator:
            # RudderElevatorAF: roll + yaw both summed onto rudder servo
            # TempAileron = Rl; PW[RudderC] = TempAileron + Yl
            rud_mixed = efforts["Roll"] + efforts["Yaw"]
            ail_rad = 0.0
            ele_rad = clamp(efforts["Pitch"], -1.0, 1.0) * ele_max_rad if ele_max_rad > 0 else 0.0
            rud_rad = clamp(rud_mixed, -1.0, 1.0) * rud_max_rad if rud_max_rad > 0 else 0.0
        else:
            # AileronAF / DeltaAF / SpoileronAF: roll→aileron, pitch→elevator, yaw→rudder
            ail_rad = clamp(efforts["Roll"], -1.0, 1.0) * ail_max_rad if ail_max_rad > 0 else 0.0
            ele_rad = clamp(efforts["Pitch"], -1.0, 1.0) * ele_max_rad if ele_max_rad > 0 else 0.0
            rud_rad = clamp(efforts["Yaw"], -1.0, 1.0) * rud_max_rad if rud_max_rad > 0 else 0.0

        # Servo lag
        alpha_s = dT / (servo_tau + dT)
        if is_rudder_elevator:
            lags["Roll"]  = lags["Roll"]  + alpha_s * (efforts["Roll"]  - lags["Roll"])
            lags["Pitch"] = lags["Pitch"] + alpha_s * (efforts["Pitch"] - lags["Pitch"])
            lags["Yaw"]   = lags["Yaw"]   + alpha_s * (rud_mixed       - lags["Yaw"])
        else:
            lags["Roll"]  = lags["Roll"]  + alpha_s * (efforts["Roll"]  - lags["Roll"])
            lags["Pitch"] = lags["Pitch"] + alpha_s * (efforts["Pitch"] - lags["Pitch"])
            lags["Yaw"]   = lags["Yaw"]   + alpha_s * (efforts["Yaw"]   - lags["Yaw"])

        # Aerodynamic torques
        L_ctrl = C_l_ail * ail_rad
        M_ctrl = C_m_ele * ele_rad
        N_ctrl = C_n_rud * rud_rad

        # Static stability restoring moments (passive airframe)
        M_stability = pitch_stab_coeff * angles["Pitch"]
        N_stability = -yaw_stab_coeff * angles["Yaw"]

        # Quadratic damping (opposes motion — C_d values are positive magnitudes)
        L_damp = C_d_roll  * rates["Roll"]  * abs(rates["Roll"])
        M_damp = C_d_pitch * rates["Pitch"] * abs(rates["Pitch"])
        N_damp = C_d_yaw   * rates["Yaw"]   * abs(rates["Yaw"])

        # Linear damping (opposes motion — also subtracted)
        L_damp += roll_damp_lin  * abs(rates["Roll"])
        M_damp += pitch_damp_lin * abs(rates["Pitch"])
        N_damp += yaw_damp_lin   * abs(rates["Yaw"])

        # Cross-coupling
        # Adverse yaw: aileron deflection → yaw torque
        N_adverse = adverse_yaw_coeff * L_ctrl if adverse_yaw_coeff > 0 and af_params.get("aileron_area", 0) > 0 else 0.0

        # Dihedral: yaw rate → roll torque (positive yaw rate → positive roll)
        L_dihedral = dihedral_coeff * rates["Yaw"] if dihedral_coeff > 0 else 0.0

        # Gusts
        L_gust = gusts["Roll"].torque(t)  if gusts and "Roll"  in gusts else 0.0
        M_gust = gusts["Pitch"].torque(t) if gusts and "Pitch" in gusts else 0.0
        N_gust = gusts["Yaw"].torque(t)   if gusts and "Yaw"   in gusts else 0.0

        # Total torques → angular accelerations
        alpha_r = (L_ctrl + L_dihedral - L_damp + L_gust)          / max(I_roll,  1e-6)
        alpha_p = (M_ctrl + M_stability - M_damp + M_gust)         / max(I_pitch, 1e-6)
        alpha_y = (N_ctrl + N_adverse + N_stability - N_damp + N_gust) / max(I_yaw,   1e-6)

        # Integrate
        rates["Roll"]  += alpha_r * dT
        rates["Pitch"] += alpha_p * dT
        rates["Yaw"]   += alpha_y * dT

        max_rates_roll  = _compute_fw_max_rates(af_params)[0]
        max_rates_pitch = _compute_fw_max_rates(af_params)[1]
        max_rates_yaw   = _compute_fw_max_rates(af_params)[2]
        rates["Roll"]  = clamp(rates["Roll"],  -max_rates_roll,  max_rates_roll)
        rates["Pitch"] = clamp(rates["Pitch"], -max_rates_pitch, max_rates_pitch)
        rates["Yaw"]   = clamp(rates["Yaw"],   -max_rates_yaw,   max_rates_yaw)

        angles["Roll"]  += rates["Roll"]  * dT
        angles["Pitch"] += rates["Pitch"] * dT
        angles["Yaw"]   += rates["Yaw"]   * dT

        if i % decimate == 0:
            all_times.append(t)
            for name in ["Roll", "Pitch", "Yaw"]:
                all_angles[name].append(angles[name])

    # Compute step metrics for each axis
    for name in ["Roll", "Pitch", "Yaw"]:
        a_list = all_angles[name]
        setpoint = step_rads[name] if name == step_axis else 0.0
        final_angle = a_list[-1] if a_list else 0.0
        peak_angle = max((abs(a) for a in a_list), default=0.0)
        max_rate = max((abs(r) for r in [0.0]), default=0.0)  # placeholder

        # Recompute max_rate from full data
        max_rate_val = 0.0
        for j in range(len(all_times)):
            if j > 0:
                dr = (a_list[j] - a_list[j-1]) / (all_times[j] - all_times[j-1]) if all_times[j] > all_times[j-1] else 0.0
                max_rate_val = max(max_rate_val, abs(dr))

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
            window = min(200, len(all_times))

            for j in range(len(all_times)):
                a = abs(a_list[j])
                if t10 is None and a >= lo_thresh:
                    t10 = all_times[j]
                if t90 is None and a >= hi_thresh:
                    t90 = all_times[j]
                if t90 is not None and not settled:
                    ok = True
                    for k in range(j, min(j + window, len(all_times))):
                        if abs(a_list[k] - setpoint) > settle_band:
                            ok = False
                            break
                    if ok:
                        settling = all_times[j]
                        settled = True
                        break

            if t10 is not None and t90 is not None:
                rise_time = t90 - t10

            peak_above = max((a - setpoint for a in a_list if a > setpoint), default=0.0)
            if setpoint > 0:
                overshoot = (peak_above / setpoint) * 100.0

        metrics[name] = StepMetrics(
            axis_name=name,
            rise_time_s=round(rise_time, 4),
            overshoot_pct=round(overshoot, 1),
            settling_time_s=round(settling, 4),
            steady_state_error=round(ss_error, 6),
            max_integrator=0.0,
            intlim=pis[name].IntLim,
            max_rate=round(max_rate_val, 4),
            peak_angle=round(peak_angle, 4),
            final_angle=round(final_angle, 4),
            setpoint=round(setpoint, 4),
            n_oscillations=0,
        )

    return metrics


@dataclass
class FreeFlightMetrics:
    """Results of open-loop turbulence response."""
    roll_peak_deg: float = 0.0
    pitch_peak_deg: float = 0.0
    yaw_peak_deg: float = 0.0
    roll_settle_s: float = 0.0
    pitch_settle_s: float = 0.0
    yaw_settle_s: float = 0.0
    roll_final_deg: float = 0.0
    pitch_final_deg: float = 0.0
    yaw_final_deg: float = 0.0
    roll_ie_deg: float = 0.0
    pitch_ie_deg: float = 0.0
    yaw_ie_deg: float = 0.0
    max_roll_rate: float = 0.0
    max_pitch_rate: float = 0.0
    max_yaw_rate: float = 0.0
    dihedral_roll_coupling: float = 0.0   # peak roll from yaw gust (dihedral effect)


def simulate_freeflight(af_filename: str, dT: float = CONTROL_DT,
                        sim_time: float = 15.0,
                        gust_axes: Optional[List[str]] = None) -> FreeFlightMetrics:
    """Open-loop free-flight with turbulence — no PID control.

    Tests intrinsic airframe stability: dihedral effect, damping, coupling.
    Reveals whether the airframe naturally recovers or diverges under gusts.
    """
    af_params = FW_AIRFRAMES[af_filename]
    rho = AIR_DENSITY
    V = af_params["cruise_speed"]
    qbar = 0.5 * rho * V * V
    S = af_params["wing_area"]
    b = af_params["wingspan"]
    chord = S / b
    mass = af_params["mass"]

    I_roll  = mass * b * b / 12.0
    I_pitch = mass * (b * b + chord * chord) / 12.0
    I_yaw   = I_pitch

    C_l_ail = qbar * S * b * af_params.get("CL_D_AIL", 0.0)
    C_m_ele = qbar * S * chord * af_params.get("CM_D_ELE", 0.0)
    C_n_rud = qbar * S * b * af_params.get("CN_D_RUD", 0.0)

    # Aerodynamic damping magnitudes (must be positive — subtracted in torque eq)
    C_d_roll  = abs(0.5 * rho * V * b * b * b * 0.04)
    C_d_pitch = abs(af_params.get("pitch_damp", 10.0))
    C_d_yaw   = abs(af_params.get("yaw_damp", 0.10)) if af_params.get("rudder_area", 0) > 0 else 0.2

    # Linear damping at low rates (bearing friction + low-speed aero)
    roll_damp_lin  = af_params.get("roll_damp_lin", 0.02)
    pitch_damp_lin = af_params.get("pitch_damp_lin", 0.04)
    yaw_damp_lin   = af_params.get("yaw_damp_lin", 0.015)

    # Static stability derivatives
    pitch_stab_coeff = af_params.get("pitch_stability", 0.0) * qbar * S * chord
    yaw_stab_coeff = af_params.get("yaw_stability", 0.0) * qbar * S * b

    adverse_yaw_coeff = af_params.get("adverse_yaw", 0.0)
    dihedral_coeff = af_params.get("dihedral_coeff", 0.0)

    af_type = AF_FILES.get(af_filename)
    is_rudder_elevator = (af_type == AirframeType.RUDDER_ELEVATOR)

    # Turbulence: broadband gusts (random walk filtered to ~0.5-2 Hz)
    import random
    random.seed(42)  # reproducible
    n = int(sim_time / dT)

    # Gust torque amplitudes — moderate turbulence
    gust_amp = {"Roll": 0.05, "Pitch": 0.04, "Yaw": 0.03}  # N·m
    gust_freq = {"Roll": 0.8, "Pitch": 1.0, "Yaw": 0.6}    # Hz
    if gust_axes:
        for ax in ["Roll", "Pitch", "Yaw"]:
            if ax not in gust_axes:
                gust_amp[ax] = 0.0

    angles = {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
    rates  = {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}

    decimate = 10
    all_angles = {"Roll": [], "Pitch": [], "Yaw": []}
    all_rates  = {"Roll": [], "Pitch": [], "Yaw": []}
    all_times = []

    for i in range(n):
        t = i * dT

        # Generate gust torques — sinusoidal at different frequencies with phase jitter
        gust_torque = {}
        for ax in ["Roll", "Pitch", "Yaw"]:
            if gust_amp[ax] > 0:
                phase = random.uniform(0, 2 * math.pi)
                # Sum 2 harmonics for richer spectrum
                g = gust_amp[ax] * (math.sin(2 * math.pi * gust_freq[ax] * t + phase)
                                  + 0.4 * math.sin(2 * math.pi * gust_freq[ax] * 2.1 * t + phase * 1.7))
                gust_torque[ax] = g
            else:
                gust_torque[ax] = 0.0

        # No PID — surfaces at zero (free flight)
        ail_rad = 0.0
        ele_rad = 0.0
        rud_rad = 0.0

        # Aerodynamic torques (zero — no surface deflection)
        L_ctrl = 0.0
        M_ctrl = 0.0
        N_ctrl = 0.0

        # Static stability restoring moments (passive airframe)
        M_stability = pitch_stab_coeff * angles["Pitch"]
        N_stability = -yaw_stab_coeff * angles["Yaw"]

        # Quadratic damping (opposes motion — C_d values are positive magnitudes)
        L_damp = C_d_roll  * rates["Roll"]  * abs(rates["Roll"])
        M_damp = C_d_pitch * rates["Pitch"] * abs(rates["Pitch"])
        N_damp = C_d_yaw   * rates["Yaw"]   * abs(rates["Yaw"])

        # Linear damping (opposes motion — also subtracted)
        L_damp += roll_damp_lin  * abs(rates["Roll"])
        M_damp += pitch_damp_lin * abs(rates["Pitch"])
        N_damp += yaw_damp_lin   * abs(rates["Yaw"])

        # Cross-coupling
        N_adverse = 0.0  # no aileron deflection
        L_dihedral = dihedral_coeff * rates["Yaw"] if dihedral_coeff > 0 else 0.0

        # Total torques → angular accelerations (damping subtracted)
        alpha_r = (L_ctrl + L_dihedral - L_damp + gust_torque["Roll"])         / max(I_roll,  1e-6)
        alpha_p = (M_ctrl + M_stability - M_damp + gust_torque["Pitch"])       / max(I_pitch, 1e-6)
        alpha_y = (N_ctrl + N_adverse + N_stability - N_damp + gust_torque["Yaw"]) / max(I_yaw,   1e-6)

        # Integrate
        rates["Roll"]  += alpha_r * dT
        rates["Pitch"] += alpha_p * dT
        rates["Yaw"]   += alpha_y * dT

        max_rr = _compute_fw_max_rates(af_params)[0]
        max_pr = _compute_fw_max_rates(af_params)[1]
        max_yr = _compute_fw_max_rates(af_params)[2]
        rates["Roll"]  = clamp(rates["Roll"],  -max_rr, max_rr)
        rates["Pitch"] = clamp(rates["Pitch"], -max_pr, max_pr)
        rates["Yaw"]   = clamp(rates["Yaw"],   -max_yr, max_yr)

        angles["Roll"]  += rates["Roll"]  * dT
        angles["Pitch"] += rates["Pitch"] * dT
        angles["Yaw"]   += rates["Yaw"]   * dT

        if i % decimate == 0:
            all_times.append(t)
            for ax in ["Roll", "Pitch", "Yaw"]:
                all_angles[ax].append(angles[ax])
                all_rates[ax].append(rates[ax])

    # Compute metrics
    def settle_time_5pct(ang_list, times):
        peak = max((abs(a) for a in ang_list), default=0.0)
        if peak < 0.01:
            return 0.0
        band = peak * 0.05
        peak_idx = max(range(len(ang_list)), key=lambda j: abs(ang_list[j]))
        for j in range(peak_idx, len(ang_list)):
            if abs(ang_list[j]) < band:
                return times[j] - times[peak_idx]
        return sim_time - times[peak_idx] if times else sim_time

    m = FreeFlightMetrics()
    for ax in ["Roll", "Pitch", "Yaw"]:
        a_list = all_angles[ax]
        r_list = all_rates[ax]
        suffix = ax.lower()
        setattr(m, f"{suffix}_peak_deg", round(max((abs(a) for a in a_list), default=0.0) * RAD_TO_DEG, 2))
        setattr(m, f"{suffix}_final_deg", round(abs(a_list[-1]) * RAD_TO_DEG, 2) if a_list else 0.0)
        setattr(m, f"{suffix}_settle_s", round(settle_time_5pct(a_list, all_times), 2))
        setattr(m, f"max_{suffix}_rate", round(max((abs(r) for r in r_list), default=0.0) * RAD_TO_DEG, 1))
        ie = sum(abs(a) * dT for a in a_list) * RAD_TO_DEG
        setattr(m, f"{suffix}_ie_deg", round(ie, 1))

    # Dihedral coupling metric: peak roll from yaw gust
    m.dihedral_roll_coupling = round(
        max((abs(a) for a in all_angles["Roll"]), default=0.0) * RAD_TO_DEG
        - max((abs(a) for a in all_angles["Yaw"]), default=0.0) * RAD_TO_DEG * dihedral_coeff * 0.1, 2)

    return m, all_times, all_angles


def format_freeflight(m: FreeFlightMetrics, af_filename: str) -> List[str]:
    """Format free-flight results as colored report lines."""
    lines = []
    lines.append(f"\n{B}{'='*60}{N}")
    lines.append(f"{B}  Free-Flight Turbulence Response (no PID control){N}")
    lines.append(f"{B}{'='*60}{N}")
    lines.append(f"  15s simulation, moderate turbulence, zero surface deflection")
    lines.append(f"  Dihedral coeff: {FW_AIRFRAMES[af_filename].get('dihedral_coeff', 0):.2f}  "
                 f"Adverse yaw: {FW_AIRFRAMES[af_filename].get('adverse_yaw', 0):.2f}")

    for ax, label in [("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")]:
        peak = getattr(m, f"{ax}_peak_deg")
        final = getattr(m, f"{ax}_final_deg")
        settle = getattr(m, f"{ax}_settle_s")
        ie = getattr(m, f"{ax}_ie_deg")
        maxr = getattr(m, f"max_{ax}_rate")

        peak_limit = 20.0 if ax != "yaw" else 30.0
        peak_ok = peak < peak_limit
        settle_ok = settle < 5.0
        final_ok = final < 3.0

        icon_p = f"{G}PASS{N}" if peak_ok else f"{R}FAIL{N}"
        icon_s = f"{G}PASS{N}" if settle_ok else f"{R}FAIL{N}"
        icon_f = f"{G}PASS{N}" if final_ok else f"{R}FAIL{N}"

        lines.append(f"\n  {B}{label}:{N}")
        lines.append(f"    [{icon_p}] Peak deviation:  {peak:5.1f} deg   (limit: <{peak_limit:.0f} deg)")
        lines.append(f"    [{icon_s}] Settle to 5%:    {settle:5.2f}s  (limit: <5.0s)")
        lines.append(f"    [{icon_f}] Final error:     {final:5.1f} deg   (limit: <3 deg)")
        lines.append(f"    Peak rate: {maxr:5.1f} deg/s   Integrated error: {ie:.1f} deg")

    if m.dihedral_roll_coupling > 0:
        lines.append(f"\n  {B}Dihedral coupling:{N} yaw gusts induced {m.dihedral_roll_coupling:.1f}° roll (expected for dihedral effect)")
    else:
        lines.append(f"\n  {B}Dihedral coupling:{N} minimal roll from yaw (low dihedral or no yaw coupling)")

    return lines
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

CRITERIA = {
    "Roll": Criteria(1.0, 10.0, 2.5, 1.0, 1.0, 0.3, 3),
    "Pitch": Criteria(1.0, 10.0, 2.5, 1.0, 1.0, 0.3, 3),
    "Yaw": Criteria(3.0, 15.0, 5.0, 2.0, 1.0, 0.2, 4),
}

FW_CRITERIA = {
    "Roll": Criteria(3.0, 15.0, 5.0, 2.0, 1.0, 0.1, 5),
    "Pitch": Criteria(3.0, 15.0, 5.0, 2.0, 1.0, 0.1, 5),
    "Yaw": Criteria(5.0, 20.0, 6.0, 3.0, 1.0, 0.05, 5),
}

DIST_CRITERIA = {
    "Roll": DisturbanceCriteria(15.0, 1.5, 10.0),
    "Pitch": DisturbanceCriteria(12.0, 1.5, 8.0),
    "Yaw": DisturbanceCriteria(12.0, 2.0, 10.0),
}

# ═══════════════════════════════════════════
#  Report / Critique
# ═══════════════════════════════════════════
G = "\033[92m"
Y = "\033[93m"
R = "\033[91m"
B = "\033[1m"
N = "\033[0m"

def check(label: str, v: float, limit: float, kind: str = "max", unit: str = "") -> Tuple[bool, str]:
    ok = (v <= limit) if kind == "max" else (v >= limit)
    icon = f"{G}PASS{N}" if ok else f"{R}FAIL{N}"
    return ok, f"  [{icon}] {label}: {v:.4g}{unit}  (limit: {kind} {limit:.4g}{unit})"

def critique(m: StepMetrics, c: Criteria, name: str) -> Tuple[List[str], bool]:
    lines = []
    lines.append(f"\n{B}── {name} @ {m.setpoint*RAD_TO_DEG:.0f}° step ──{N}")
    lines.append(f"     Final: {m.final_angle*RAD_TO_DEG:.1f}°   Peak: {m.peak_angle*RAD_TO_DEG:.1f}°   Max rate: {m.max_rate*RAD_TO_DEG:.0f}°/s")
    int_ratio = m.max_integrator / m.intlim if m.intlim > 0 else 0
    rate_ratio = m.max_rate / (abs(m.setpoint) * 2) if m.setpoint != 0 else 1.0
    results = [
        check("Rise time (10→90%)",          m.rise_time_s,         c.max_rise_time_s, "max", "s"),
        check("Overshoot",                    m.overshoot_pct,       c.max_overshoot_pct, "max", "%"),
        check("Settling time (±2%)",          m.settling_time_s,     c.max_settling_time_s, "max", "s"),
        check("Final error",                  m.steady_state_error * RAD_TO_DEG, c.max_ss_error_deg, "max", "°"),
        check("Integrator utilization",       int_ratio,             c.max_int_ratio, "max"),
        check("Rate headroom (peak/2×sp)",    rate_ratio,            c.min_rate_ratio, "min"),
        check("Zero crossings (oscillations)", m.n_oscillations,     c.max_oscillations, "max"),
    ]
    for ok, msg in results:
        lines.append(msg)
        if not ok:
            lines.append(f"    {R}╰─ recommend tuning{N}")
    all_pass = all(r[0] for r in results)
    lines.append(f"  → {name}: {G}PASS{N}" if all_pass else f"  → {name}: {R}ISSUES{N}")
    return lines, all_pass

def critique_disturbance(m: DisturbanceMetrics, c: DisturbanceCriteria, name: str) -> Tuple[List[str], bool]:
    lines = []
    lines.append(f"\n{B}── {name} disturbance rejection @ gust={c.max_peak_deviation_deg:.0f}°/s limit ──{N}")
    lines.append(f"     Peak deviation: {m.peak_deviation_rad*RAD_TO_DEG:.2f}°/s   Settle: {m.settling_time_s:.3f}s   IE: {m.integrated_error*RAD_TO_DEG:.3f}°")
    results = [
        check("Peak rate deviation", m.peak_deviation_rad*RAD_TO_DEG, c.max_peak_deviation_deg, "max", "°/s"),
        check("Settling after gust", m.settling_time_s,               c.max_settling_time_s,    "max", "s"),
        check("Integrated error",    m.integrated_error*RAD_TO_DEG,   c.max_integrated_error,   "max", "°"),
    ]
    for ok, msg in results:
        lines.append(msg)
        if not ok:
            lines.append(f"    {R}╰─ recommend tuning{N}")
    all_pass = all(r[0] for r in results)
    lines.append(f"  → {name} disturbance rejection: {G}PASS{N}" if all_pass else f"  → {name}: {R}ISSUES{N}")
    return lines, all_pass

def recommend(metrics: dict, is_fw: bool = False) -> List[str]:
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

# ═══════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════
def load_af_params(af_filename: str) -> dict:
    """Load parameters from .af file."""
    from airframes.airframes import parse_af_file
    from protocol_enums import ParamIndex
    # Files are in UAVXGS/uavx-python/src/airframes/
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    path = os.path.join(project_root, "uavx-python", "src", "airframes", af_filename)
    _, p = parse_af_file(path)
    return {ParamIndex(tag).name: val for tag, val in p.items()}

def get_params_for_af(af_filename: str) -> Tuple[dict, AirframeType, int, bool]:
    """Get PID params, category, FW model index, and is_fw flag for an airframe file."""
    af_type = AF_FILES.get(af_filename)
    if af_type is None:
        raise ValueError(f"No mapping for {af_filename}")

    cat = AF_CATEGORY[af_type]
    is_fw = (cat == AirframeCat.FW)
    fw_mid = FW_MODEL_IDX.get(af_type, -1) if is_fw else -1

    params = load_af_params(af_filename)
    return params, af_type, fw_mid, is_fw

def get_axis_params(params: dict, name: str) -> Tuple[float, float, float, float, float, float, float]:
    """Extract (angle_kp, angle_ki, angle_int_lim, max_angle, rate_kp, rate_kd, max_rate) for an axis."""
    from protocol_enums import ParamIndex
    up = name.upper()
    akp = params.get(f"{up}_ANGLE_KP", 0.0)
    aki = params.get(f"{up}_ANGLE_KI", 0.0)
    ail = params.get(f"{up}_ANGLE_INT_LIMIT", 0.0)
    ma  = params.get(f"MAX_{up}_ANGLE", 0.523599)
    rkp = params.get(f"{up}_RATE_KP", 0.0)
    rkd = params.get(f"{up}_RATE_KD", 0.0)
    mr  = params.get(f"MAX_{up}_RATE", 10.0)
    return akp, aki, ail, ma, rkp, rkd, mr

def run_tests_for_af(af_filename: str) -> Tuple[bool, List[str]]:
    """Run all tests for a single airframe file."""
    all_output = []
    all_pass = True

    try:
        params, af_type, fw_mid, is_fw = get_params_for_af(af_filename)
    except Exception as e:
        return False, [f"Error loading {af_filename}: {e}"]

    af_name = AIRFRAME_NAMES.get(af_type, af_type.name)
    model_lbl = af_type.name

    all_output.append(f"\n{'='*60}")
    all_output.append(f"{B}UAVX PID Critique — {af_filename}{N}")
    all_output.append(f"  Airframe: {af_name} ({model_lbl})")
    all_output.append(f"  Physics:  {'aerodynamic (qbar × S × C_ctrl)' if is_fw and af_filename in FW_AIRFRAMES else 'control-surface (emu.c FW)' if is_fw else 'motor-thrust (emu.c MR)'}")
    all_output.append(f"  Sim:      {SIM_TIME:.0f}s per axis   dt={CONTROL_DT*1000:.0f}ms")
    if is_fw and af_filename in FW_AIRFRAMES:
        af = FW_AIRFRAMES[af_filename]
        I_roll, I_pitch = _compute_fw_inertia(af)
        qbar = 0.5 * AIR_DENSITY * af["cruise_speed"] ** 2
        all_output.append(f"  Aero:     m={af['mass']}kg  b={af['wingspan']}m  S={af['wing_area']}m²  V={af['cruise_speed']}m/s  qbar={qbar:.1f}Pa")
        all_output.append(f"  Inertia:  I_roll={I_roll:.4f}  I_pitch={I_pitch:.4f} kg·m²")
    all_output.append(f"{'='*60}")

    axes = ["Roll", "Pitch", "Yaw"]
    metrics = {}

    if is_fw and af_filename in FW_AIRFRAMES:
        # Coupled 3-axis simulation for each step axis
        for step_ax in axes:
            gusts = {name: Gust(GUST_MAG[name], 2.0, 0.5, name) for name in axes}
            coupled_metrics = simulate_axis_coupled(af_filename, step_axis=step_ax, gusts=gusts)
            for name in axes:
                if name == step_ax:
                    crit = FW_CRITERIA[name]
                    m = coupled_metrics[name]
                    metrics[name] = m
                    lines, ok = critique(m, crit, f"{name} @ {TEST_STEPS[name]:.0f}°")
                    all_output.extend(lines)
                    if not ok:
                        all_pass = False
    else:
        for name in axes:
            akp, aki, ail, ma, rkp, rkd, mr = get_axis_params(params, name)
            pi = PIStruct(Kp=akp, Ki=aki, IntLim=ail, Max=ma)
            pid = PIDStruct(Kp=rkp, Kd=rkd, Max=mr)
            crit = CRITERIA[name]
            gust = Gust(GUST_MAG[name], 2.0, 0.5, name)
            m = simulate_axis(pi, pid, TEST_STEPS[name] * DEG_TO_RAD, ma,
                              cat=AF_CATEGORY[af_type], fw_model_idx=fw_mid,
                              axis_name=name, gust=gust,
                              af_filename=af_filename)
            metrics[name] = m
            lines, ok = critique(m, crit, f"{name} @ {TEST_STEPS[name]:.0f}°")
            all_output.extend(lines)
            if not ok:
                all_pass = False

    for l in recommend(metrics, is_fw):
        all_output.append(l)

    # Disturbance rejection test
    all_output.append(f"\n{B}{'='*60}{N}")
    all_output.append(f"{B}  Disturbance Rejection{N}")
    all_output.append(f"{B}{'='*60}{N}")

    for name in axes:
        akp, aki, ail, ma, rkp, rkd, mr = get_axis_params(params, name)
        pid = PIDStruct(Kp=rkp, Kd=rkd, Max=mr)
        gust = Gust(GUST_MAG[name], 2.0, 0.5, name)

        if is_fw and af_filename in FW_AIRFRAMES:
            I_roll, I_pitch = _compute_fw_inertia(FW_AIRFRAMES[af_filename])
            inertia = I_pitch if name == "Yaw" else I_roll
        elif is_fw and fw_mid >= 0:
            rp_i, y_i = FW_INERTIAS[fw_mid]
            inertia = y_i if name == "Yaw" else rp_i
        else:
            inertia = 1.0 / MR_INERTIA_R[AXIS_NAMES[name]]

        dm = simulate_rate_disturbance(pid, cat=AF_CATEGORY[af_type],
                                        fw_model_idx=fw_mid, axis_name=name,
                                        gust=gust, own_inertia=inertia,
                                        af_filename=af_filename)
        dc = DIST_CRITERIA[name]
        lines, ok = critique_disturbance(dm, dc, name)
        all_output.extend(lines)
        if not ok:
            all_pass = False

    # Free-flight turbulence test (FW only — tests intrinsic stability)
    if is_fw and af_filename in FW_AIRFRAMES:
        ff_metrics, ff_times, ff_angles = simulate_freeflight(af_filename)
        ff_lines = format_freeflight(ff_metrics, af_filename)
        all_output.extend(ff_lines)

    return all_pass, all_output


def main():
    if len(sys.argv) > 1:
        # Single airframe specified
        af_filename = sys.argv[1]
        if not af_filename.endswith('.af'):
            af_filename += '.af'
        pass_ok, output = run_tests_for_af(af_filename)
        for l in output:
            print(l)
        sys.exit(0 if pass_ok else 1)

    # Run all active _Tuned airframes
    tuned_files = [f for f in AF_FILES.keys() if f.endswith('_Tuned.af')]
    all_pass = True
    all_output = []

    for af_file in tuned_files:
        try:
            ok, output = run_tests_for_af(af_file)
            all_output.extend(output)
            if not ok:
                all_pass = False
        except Exception as e:
            all_pass = False
            all_output.append(f"Error testing {af_file}: {e}")

    for l in all_output:
        print(l)

    print(f"\n{'='*60}")
    if all_pass:
        print(f"{G}{B}All airframes PASS — tuning is acceptable{N}")
    else:
        print(f"{R}{B}Some airframes have issues — review recommendations above{N}")

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()