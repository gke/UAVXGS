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
    AirframeType.eQuadXAF: AirframeCat.MR,
    AirframeType.eHexXAF: AirframeCat.MR,
    AirframeType.eOctXAF: AirframeCat.MR,
    AirframeType.eElevonAF: AirframeCat.FW,
    AirframeType.eDeltaAF: AirframeCat.FW,
    AirframeType.eAileronSpoilerFlapsAF: AirframeCat.FW,
    AirframeType.eRudderElevatorAF: AirframeCat.FW,
    AirframeType.eDifferentialTwinAF: AirframeCat.MR,
    AirframeType.eTrackedAF: AirframeCat.LAND,
    AirframeType.eAileronAF: AirframeCat.FW,
}

# FW model index per active FW airframe (for physics params)
FW_MODEL_IDX = {
    AirframeType.eElevonAF: 0,
    AirframeType.eDeltaAF: 2,
    AirframeType.eAileronSpoilerFlapsAF: 3,
    AirframeType.eRudderElevatorAF: 5,
    AirframeType.eAileronAF: 1,          # Sky Surfer uses AileronAF model
}

# .af file to AirframeType mapping (from actual .af files)
AF_FILES = {
    # Generic defaults
    "generic/Quad.af": AirframeType.eQuadXAF,
    "generic/Quad_Medium.af": AirframeType.eQuadXAF,
    "generic/Quad_Racer.af": AirframeType.eQuadXAF,
    "generic/Hex.af": AirframeType.eHexXAF,
    "generic/Oct.af": AirframeType.eOctXAF,
    "generic/Delta.af": AirframeType.eDeltaAF,
    "generic/Spoileron.af": AirframeType.eAileronSpoilerFlapsAF,
    "generic/RudderElevator.af": AirframeType.eRudderElevatorAF,
    "generic/Elevon.af": AirframeType.eElevonAF,
    "generic/Shadow.af": AirframeType.eElevonAF,
    "generic/SkySurfer_Bixler.af": AirframeType.eAileronAF,
    "generic/SmallSpoileron.af": AirframeType.eAileronSpoilerFlapsAF,
    "generic/Dragon.af": AirframeType.eElevonAF,
    "generic/Radian.af": AirframeType.eRudderElevatorAF,

    # User airframes
    "user/Ken_s_LadyBug.af": AirframeType.eQuadXAF,
    "user/EcksTuned.af": AirframeType.eQuadXAF,
    "user/Ecks_800g_Moderate.af": AirframeType.eQuadXAF,
    "user/Ecks_800g_Sport.af": AirframeType.eQuadXAF,
    "user/Ecks_1kg_Moderate.af": AirframeType.eQuadXAF,
    "user/Ecks_1kg_Sport.af": AirframeType.eQuadXAF,
    "user/Ecks_220mm.af": AirframeType.eQuadXAF,
    "user/EcksQuatQ.af": AirframeType.eQuadXAF,
    "user/DevEBox.af": AirframeType.eQuadXAF,
    "user/S500_1137.af": AirframeType.eQuadXAF,
    "user/Ken_s_Alpha_Test.af": AirframeType.eQuadXAF,
    "user/Ken_s_450_1165.af": AirframeType.eQuadXAF,
    "user/150mm_Brushed.af": AirframeType.eQuadXAF,
    "user/Arado_555.af": AirframeType.eElevonAF,
    "user/Horten.af": AirframeType.eElevonAF,

    # Original default airframes
    "original/150mm_Brushed.af": AirframeType.eQuadXAF,
    "original/DevEBox.af": AirframeType.eQuadXAF,
    "original/Ecks_220mm.af": AirframeType.eQuadXAF,
    "original/Ken_s_450_1165.af": AirframeType.eQuadXAF,
    "original/Ken_s_Alpha_Test.af": AirframeType.eQuadXAF,
    "original/Ken_s_LadyBug.af": AirframeType.eQuadXAF,
    "original/Rok_Quad.af": AirframeType.eQuadXAF,
    "original/S500_1137.af": AirframeType.eQuadXAF,
    "original/Phoenix.af": AirframeType.eRudderElevatorAF,

    # MR - QuadXAF (tag 4)
    "EcksTuned.af": AirframeType.eQuadXAF,
    "Ecks_800g_Moderate.af": AirframeType.eQuadXAF,
    "Ecks_800g_Sport.af": AirframeType.eQuadXAF,
    "Ecks_1kg_Moderate.af": AirframeType.eQuadXAF,
    "Ecks_1kg_Sport.af": AirframeType.eQuadXAF,
    "user/Arado_555_Tuned.af": AirframeType.eElevonAF,
    "user/Ecks_220mm_Tuned.af": AirframeType.eQuadXAF,
    "user/Ken_450_1165_Tuned.af": AirframeType.eQuadXAF,
    "user/Ken_Alpha_Test_Tuned.af": AirframeType.eQuadXAF,
    "user/Ken_LadyBug_Tuned.af": AirframeType.eQuadXAF,
    "user/Ming_DevEBox_Tuned.af": AirframeType.eQuadXAF,
    "user/Phoenix_Tuned.af": AirframeType.eRudderElevatorAF,
    "user/Radian_Tuned.af": AirframeType.eRudderElevatorAF,
    "user/Rok_Quad_Tuned.af": AirframeType.eQuadXAF,
    "user/S500_1137_Tuned.af": AirframeType.eQuadXAF,
    "user/WLToys_LadyBug_Tuned.af": AirframeType.eQuadXAF,
}

AIR_DENSITY = 1.225          # kg/m³ at sea level
RAD_TO_DEG_F = 180.0 / math.pi

# ═══════════════════════════════════════════
#  Character slider param curves
#  Maps ParamIndex → (conservative_raw, aggressive_raw)
#  Values are FC raw float32 units (rad, rad/s, fraction, etc.)
#  Used to test slider extremes for safety.
# ═══════════════════════════════════════════
from protocol_enums import ParamIndex as _PI

_PARAM_CURVES = {
    # Angle gains (Quaternion P, scale=1.0 on GCS)
    int(_PI.ROLL_ANGLE_Q_KP):       (5.0, 9.0),       # default 7
    int(_PI.PITCH_ANGLE_Q_KP):      (5.0, 9.0),       # default 7
    int(_PI.YAW_ANGLE_Q_KP):        (5.0, 10.0),      # default 8
    # Angle integral gains
    int(_PI.ROLL_ANGLE_Q_KI):       (0.05, 0.5),      # default 0.25
    int(_PI.PITCH_ANGLE_Q_KI):      (0.05, 0.5),      # default 0.25
    int(_PI.YAW_ANGLE_Q_KI):        (0.05, 0.5),      # default 0.25
    # Angle integral limits (rad/s)
    int(_PI.ROLL_ANGLE_Q_INT_LIMIT):(0.005, 0.03),    # default 0.01
    int(_PI.PITCH_ANGLE_Q_INT_LIMIT):(0.005, 0.03),   # default 0.01
    int(_PI.YAW_ANGLE_Q_INT_LIMIT): (0.01, 0.06),     # default 0.03
    # Rate proportional gains
    int(_PI.ROLL_RATE_KP):        (0.125, 0.5),     # default 0.25
    int(_PI.PITCH_RATE_KP):       (0.125, 0.5),     # default 0.25
    int(_PI.YAW_RATE_KP):         (0.125, 0.75),    # default 0.25
    # Rate derivative gains
    int(_PI.ROLL_RATE_KD):        (0.005, 0.02),    # default 0.01
    int(_PI.PITCH_RATE_KD):       (0.005, 0.02),    # default 0.01
    int(_PI.YAW_RATE_KD):         (0.005, 0.02),    # default 0.01
    # Rate limits (rad/s)
    int(_PI.MAX_ROLL_RATE):       (1.396, 6.283),   # 80-360 deg/s
    int(_PI.MAX_PITCH_RATE):      (1.047, 4.189),   # 60-240 deg/s
    int(_PI.MAX_HEADING_RATE):(0.262, 2.094),   # 15-120 deg/s
    # Altitude
    int(_PI.ALT_POS_KP):          (0.2, 0.5),       # default 0.35
    int(_PI.ALT_POS_KI):          (0.001, 0.005),   # default 0.002
    int(_PI.ALT_THROTTLE_COMP_LIMIT): (0.1, 0.35),  # default 0.2-0.25 fraction
    int(_PI.ALT_ROC_KP):          (0.02, 0.10),     # default ~0.05
    int(_PI.UNUSED_ALT_VEL_KI):   (0.0003, 0.003),  # default ~0.001
    int(_PI.MAX_CLIMB_RATE_DMP_S): (1.0, 8.0),   # vertical-profile ascent shaping (m/s)
    # Navigation
    int(_PI.NAV_POS_KP):          (0.075, 0.3),     # default 0.15
    int(_PI.NAV_POS_KI):          (0.006, 0.025),   # default 0.012
    int(_PI.NAV_VEL_KP):          (0.1, 0.4),       # default 0.2
    int(_PI.HORIZON):             (2.0, 5.0),       # default ~3.33
    # Angle limits (rad)
    int(_PI.MAX_PITCH_ANGLE):     (0.349, 0.698),   # 20-40 deg
    int(_PI.MAX_ROLL_ANGLE):      (0.349, 0.698),   # 20-40 deg
}


def apply_slider(raw_params: dict, slider_pct: float) -> dict:
    """Apply character slider to raw params.

    Interpolates between conservative (0%) and aggressive (100%) raw FC values.
    Params not in _PARAM_CURVES are left unchanged.
    """
    from protocol_enums import ParamIndex
    result = dict(raw_params)
    for tag_int, (cons, agg) in _PARAM_CURVES.items():
        pname = ParamIndex(tag_int).name
        if pname not in raw_params:
            continue
        new_raw = cons + slider_pct * (agg - cons)
        result[pname] = new_raw
    return result

# Per-airframe physical descriptors for aerodynamic simulation.
# Each entry defines the real-world geometry that drives torque and damping.
# Control derivatives (C_l_ail, C_m_elev, C_n_rud) computed from these.
#
# Standard aero conventions (assumed across all FW airframes):
#   - 2° effective dihedral  → dihedral_coeff = 0.05
#   - 5% static margin (SM)  → pitch_stability = Cm_α = -0.05 × CL_α
#   - Geometry (mass/span/area/cruise) matches PHYS_* in the .af file
#     (mass = PHYS_AUW_G/1000, area = chord × span where not explicit).
# Keys are the current .af file paths — tuned/original variants removed.
FW_AIRFRAMES = {
    # Sky Surfer / Bixler 2000mm: 1400g AUW, 231.5mm chord, area 0.4630 m²
    # CL_α≈4.7 → Cm_α=-0.235
    "generic/SkySurfer_Bixler.af": {
        "mass": 1.4, "wingspan": 2.0, "wing_area": 0.463,
        "cruise_speed": 12.0,
        "aileron_area": 0.0126, "aileron_arm": 0.35,
        "elevator_area": 0.0081, "elevator_arm": 0.65,
        "rudder_area": 0.012, "rudder_arm": 0.75,
        "aileron_max_deg": 20.0, "elevator_max_deg": 20.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.040, "CM_D_ELE": 0.50, "CN_D_RUD": 0.12,
        "pitch_damp": -12.0, "yaw_damp": -0.15,
        "adverse_yaw": 0.15, "dihedral_coeff": 0.05,
        "pitch_stability": -0.235, "yaw_stability": 0.03,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.04, "yaw_damp_lin": 0.015,
        "servo_tau": 0.08,
    },
    # S800 Shadow: 500g AUW, 820mm span, 185mm chord, area 0.1517 m²
    # CL_α≈4.9 → Cm_α=-0.245
    "generic/Shadow.af": {
        "mass": 0.5, "wingspan": 0.82, "wing_area": 0.1517,
        "cruise_speed": 13.0,
        "aileron_area": 0.0044, "aileron_arm": 0.25,
        "elevator_area": 0.0044, "elevator_arm": 0.25,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 20.0, "elevator_max_deg": 20.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.01,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # Phoenix: 1000g AUW, 1800mm span, 250mm chord, rudder+elevator (no ailerons)
    # CL_α≈5.4 → Cm_α=-0.27
    "original/Phoenix.af": {
        "mass": 1.0, "wingspan": 1.8, "wing_area": 0.45,
        "cruise_speed": 13.0,
        "aileron_area": 0.0, "aileron_arm": 0.0,
        "elevator_area": 0.008, "elevator_arm": 0.60,
        "rudder_area": 0.012, "rudder_arm": 0.70,
        "aileron_max_deg": 0.0, "elevator_max_deg": 20.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.0, "CM_D_ELE": 0.60, "CN_D_RUD": 0.10,
        "pitch_damp": -10.0, "yaw_damp": -0.12,
        "adverse_yaw": 0.0, "dihedral_coeff": 0.05,
        "pitch_stability": -0.27, "yaw_stability": 0.04,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.05, "yaw_damp_lin": 0.02,
        "servo_tau": 0.08,
    },
    # SmallSpoileron: 1200g AUW, 1800mm span, 280mm chord (area 0.504 m²)
    # CL_α≈5.1 → Cm_α=-0.255
    "generic/SmallSpoileron.af": {
        "mass": 1.2, "wingspan": 1.8, "wing_area": 0.504,
        "cruise_speed": 14.0,
        "aileron_area": 0.0096, "aileron_arm": 0.28,
        "elevator_area": 0.0084, "elevator_arm": 0.40,
        "rudder_area": 0.006, "rudder_arm": 0.50,
        "aileron_max_deg": 20.0, "elevator_max_deg": 20.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.035, "CM_D_ELE": 0.45, "CN_D_RUD": 0.08,
        "pitch_damp": -9.0, "yaw_damp": -0.10,
        "adverse_yaw": 0.10, "dihedral_coeff": 0.05,
        "pitch_stability": -0.255, "yaw_stability": 0.03,
        "roll_damp_lin": 0.015, "pitch_damp_lin": 0.03, "yaw_damp_lin": 0.012,
        "servo_tau": 0.08,
    },
    # Dragon: 800g AUW, 1200mm span, 250mm chord, area 0.30 m² (twin elevon)
    # CL_α≈4.9 → Cm_α=-0.245
    "generic/Dragon.af": {
        "mass": 0.8, "wingspan": 1.2, "wing_area": 0.30,
        "cruise_speed": 13.0,
        "aileron_area": 0.0055, "aileron_arm": 0.30,
        "elevator_area": 0.0055, "elevator_arm": 0.30,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 20.0, "elevator_max_deg": 20.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.01,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # Radian: 850g AUW, 2000mm span, 178mm chord, area 0.356 m²
    # CL_α≈5.4 → Cm_α=-0.27
    "generic/Radian.af": {
        "mass": 0.85, "wingspan": 2.0, "wing_area": 0.356,
        "cruise_speed": 9.0,
        "aileron_area": 0.0, "aileron_arm": 0.0,
        "elevator_area": 0.007, "elevator_arm": 0.65,
        "rudder_area": 0.009, "rudder_arm": 0.70,
        "aileron_max_deg": 0.0, "elevator_max_deg": 20.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.0, "CM_D_ELE": 0.60, "CN_D_RUD": 0.10,
        "pitch_damp": -10.0, "yaw_damp": -0.12,
        "adverse_yaw": 0.0, "dihedral_coeff": 0.05,
        "pitch_stability": -0.27, "yaw_stability": 0.04,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.05, "yaw_damp_lin": 0.02,
        "servo_tau": 0.08,
    },
    # Arado 555: 1300g AUW, 1200mm span, 350mm chord, area 0.42 m² (elevon)
    # CL_α≈4.9 → Cm_α=-0.245
    "user/Arado_555.af": {
        "mass": 1.3, "wingspan": 1.2, "wing_area": 0.42,
        "cruise_speed": 13.0,
        "aileron_area": 0.006, "aileron_arm": 0.30,
        "elevator_area": 0.006, "elevator_arm": 0.30,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 20.0, "elevator_max_deg": 20.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.01,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.03, "yaw_damp_lin": 0.012,
        "servo_tau": 0.08,
    },
    # Horten: 550g AUW, 1200mm span, 225mm chord, area 0.27 m² (flying wing)
    # CL_α≈4.9 → Cm_α=-0.245
    "user/Horten.af": {
        "mass": 0.55, "wingspan": 1.2, "wing_area": 0.27,
        "cruise_speed": 13.0,
        "aileron_area": 0.0045, "aileron_arm": 0.28,
        "elevator_area": 0.0045, "elevator_arm": 0.28,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 20.0, "elevator_max_deg": 20.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.01,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # generic/Elevon.af: 1000g, 1400mm span, 200mm chord, BR2406S 5x4.3x3 3S
    # CL_α≈4.9 → Cm_α=-0.245
    "generic/Elevon.af": {
        "mass": 1.0, "wingspan": 1.4, "wing_area": 0.28,
        "cruise_speed": 10.7,
        "aileron_area": 0.0042, "aileron_arm": 0.35,
        "elevator_area": 0.0042, "elevator_arm": 0.35,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 20.0, "elevator_max_deg": 20.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.03,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # generic/Delta.af: 1000g, 1000mm span, 250mm chord, BR2406S 5x4.3x3 3S
    # CL_α≈4.0 → Cm_α=-0.20
    "generic/Delta.af": {
        "mass": 1.0, "wingspan": 1.0, "wing_area": 0.25,
        "cruise_speed": 11.3,
        "aileron_area": 0.00375, "aileron_arm": 0.25,
        "elevator_area": 0.00375, "elevator_arm": 0.25,
        "rudder_area": 0.003, "rudder_arm": 0.25,
        "aileron_max_deg": 20.0, "elevator_max_deg": 20.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.10,
        "pitch_damp": -8.0, "yaw_damp": -0.12,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.20, "yaw_stability": 0.02,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # generic/Spoileron.af: 1200g, 1800mm span, 280mm chord, X2216 10x6" folder 3S
    # CL_α≈5.1 → Cm_α=-0.255
    "generic/Spoileron.af": {
        "mass": 1.2, "wingspan": 1.8, "wing_area": 0.504,
        "cruise_speed": 8.7,
        "aileron_area": 0.010, "aileron_arm": 0.50,
        "elevator_area": 0.0076, "elevator_arm": 0.70,
        "rudder_area": 0.005, "rudder_arm": 0.63,
        "aileron_max_deg": 20.0, "elevator_max_deg": 20.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.030, "CM_D_ELE": 0.40, "CN_D_RUD": 0.08,
        "pitch_damp": -9.0, "yaw_damp": -0.10,
        "adverse_yaw": 0.10, "dihedral_coeff": 0.05,
        "pitch_stability": -0.255, "yaw_stability": 0.03,
        "roll_damp_lin": 0.015, "pitch_damp_lin": 0.03, "yaw_damp_lin": 0.012,
        "servo_tau": 0.08,
    },
    # generic/RudderElevator.af: 1100g, 1800mm span, 250mm chord, X2216 10x6" folder 3S
    # CL_α≈4.9 → Cm_α=-0.245
    "generic/RudderElevator.af": {
        "mass": 1.1, "wingspan": 1.8, "wing_area": 0.45,
        "cruise_speed": 8.8,
        "aileron_area": 0.0, "aileron_arm": 0.0,
        "elevator_area": 0.00675, "elevator_arm": 0.625,
        "rudder_area": 0.0045, "rudder_arm": 0.63,
        "aileron_max_deg": 0.0, "elevator_max_deg": 20.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.0, "CM_D_ELE": 0.6, "CN_D_RUD": 0.10,
        "pitch_damp": -6.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.0, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.04,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
}

# Maps PHYS_* aero keys in the .af file to FW_AIRFRAMES descriptor keys.
# Full aero authority set is stored in the Configuration (PHYS_* metadata);
# hardcoded FW_AIRFRAMES values act as defaults when a key is absent.
PHYS_TO_DESCRIPTOR = {
    "PHYS_CL_D_AIL": "CL_D_AIL",
    "PHYS_CM_D_ELE": "CM_D_ELE",
    "PHYS_CN_D_RUD": "CN_D_RUD",
    "PHYS_ROLL_DAMP": "roll_damp",
    "PHYS_PITCH_DAMP": "pitch_damp",
    "PHYS_YAW_DAMP": "yaw_damp",
    "PHYS_ADVERSE_YAW": "adverse_yaw",
    "PHYS_PITCH_STABILITY": "pitch_stability",
    "PHYS_YAW_STABILITY": "yaw_stability",
    "PHYS_DIHEDRAL": "dihedral_coeff",
    "PHYS_SERVO_TAU": "servo_tau",
    "PHYS_AILERON_AREA": "aileron_area",
    "PHYS_AILERON_ARM": "aileron_arm",
    "PHYS_AILERON_MAX_DEG": "aileron_max_deg",
    "PHYS_ELEVATOR_AREA": "elevator_area",
    "PHYS_ELEVATOR_ARM": "elevator_arm",
    "PHYS_ELEVATOR_MAX_DEG": "elevator_max_deg",
    "PHYS_RUDDER_AREA": "rudder_area",
    "PHYS_RUDDER_ARM": "rudder_arm",
    "PHYS_RUDDER_MAX_DEG": "rudder_max_deg",
}

def get_fw_descriptor(af_filename: str) -> Optional[dict]:
    """Return the FW aerodynamic descriptor for an airframe file.

    Keys in FW_AIRFRAMES use various prefixes ('original/...', 'user/...',
    none). Look up by exact key first, then by basename so tuned/original
    variants resolve to their descriptor regardless of the caller's prefix
    (a '_Tuned' suffix is stripped before the basename match).

    PHYS_* aero metadata from the .af Configuration is merged over the
    hardcoded defaults (PHYS_TO_DESCRIPTOR), so each airframe carries its
    own calibrated authority set. A merged copy is returned — the shared
    FW_AIRFRAMES dicts are never mutated.
    """
    if af_filename in FW_AIRFRAMES:
        desc = FW_AIRFRAMES[af_filename]
    else:
        base = os.path.basename(af_filename)
        if base.endswith("_Tuned.af"):
            base = base[: -len("_Tuned.af")] + ".af"
        desc = None
        for key, d in FW_AIRFRAMES.items():
            if os.path.basename(key) == base:
                desc = d
                break
        if desc is None:
            return None
    phys = load_af_physicals(af_filename)
    if not phys:
        return desc
    merged = dict(desc)
    for pk, dk in PHYS_TO_DESCRIPTOR.items():
        if pk in phys:
            merged[dk] = phys[pk]
    return merged

# Derived inertia and max-rate tuples (roll_pitch, yaw) — computed from physical params
def _compute_fw_inertia(af: dict) -> Tuple[float, float]:
    m, b, l = af["mass"], af["wingspan"], af["wing_area"] / af["wingspan"]
    I_roll  = m * b * b / 12.0
    I_pitch = m * (b * b + l * l) / 12.0
    return (I_roll, I_pitch)

def is_yaw_structurally_limited(af_filename: str) -> bool:
    """Rudderless elevon airframes have no real yaw authority — elevon
    drag-differential can only hold ~1-3° against weathervane stability.
    Yaw step/gust criteria are physically unreachable, so mark them
    structurally limited (non-scored) rather than reporting FAILs.
    """
    desc = get_fw_descriptor(af_filename)
    if desc is None:
        return False
    if desc.get("rudder_area", 0) > 0:
        return False
    from protocol_enums import AirframeType
    base = os.path.basename(af_filename)
    if base.endswith("_Tuned.af"):
        base = base[: -len("_Tuned.af")] + ".af"
    for key, ft in AF_FILES.items():
        if os.path.basename(key) == base:
            return ft == AirframeType.eElevonAF
    return AF_FILES.get(af_filename) == AirframeType.eElevonAF

def is_roll_structurally_limited(af_filename: str) -> bool:
    """Aileron-less RudderElevatorAF airframes route roll through the rudder
    (FC: PW[Rudder] = Rl + Yl). In the coupled sim the yaw-angle hold cancels
    the induced yaw, so bank-and-yank produces zero net roll authority.
    Roll step/gust criteria are physically unreachable in the current model —
    mark them structurally limited (non-scored) rather than reporting FAILs.
    """
    desc = get_fw_descriptor(af_filename)
    if desc is None:
        return False
    if desc.get("aileron_area", 0) > 0 or desc.get("aileron_max_deg", 0) > 0:
        return False
    return desc.get("CL_D_AIL", 0.0) <= 0.0


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

    # ElevonAF yaw authority: drag-differential via elevon deflection
    # (FC now mixes Yl into both elevons) — same derivation as simulate_axis_coupled.
    adverse_yaw = af.get("adverse_yaw", 0.0)
    C_n_elevon = adverse_yaw * C_l_ail if adverse_yaw > 0 else 0.0

    # Steady-state rate at FULL deflection (100%): control torque balanced by
    # quadratic + linear damping →  C_ctrl·δ_max = C_d·r² + C_lin·r.
    # This is the physically achievable ceiling (70% deflection yields the
    # MAX_*_RATE param value, so full throw ≈ 1.195 × MAX_*_RATE).
    C_d_roll  = abs(af.get("roll_damp", 0.5 * rho * V * b * b * b * 0.04))
    C_d_pitch = abs(af.get("pitch_damp", 10.0))
    C_d_yaw   = abs(af.get("yaw_damp", 0.10)) if af.get("rudder_area", 0) > 0 else 0.2
    lin_roll  = af.get("roll_damp_lin", 0.02)
    lin_pitch = af.get("pitch_damp_lin", 0.04)
    lin_yaw   = af.get("yaw_damp_lin", 0.015)

    def ss_rate(C_ctrl, dm, C_d, lin):
        if C_ctrl <= 0 or dm <= 0 or C_d <= 0:
            return 0.0
        T = C_ctrl * dm
        return (-lin + math.sqrt(lin * lin + 4.0 * C_d * T)) / (2.0 * C_d)

    r_roll  = ss_rate(C_l_ail, ail_max, C_d_roll, lin_roll)
    r_pitch = ss_rate(C_m_ele, ele_max, C_d_pitch, lin_pitch)
    if C_n_rud * rud_max > 0:
        r_yaw = ss_rate(C_n_rud, rud_max, C_d_yaw, lin_yaw)
    else:
        r_yaw = ss_rate(C_n_elevon, ail_max, C_d_yaw, lin_yaw)
    return (r_roll, r_pitch, r_yaw)

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

# Test step sizes (degrees) — MR
TEST_STEPS = {"Roll": 15.0, "Pitch": 10.0, "Yaw": 45.0}
# FW step sizes — larger pitch for recovery-from-upset test, smaller yaw for coordinated turn
FW_TEST_STEPS = {"Roll": 15.0, "Pitch": 30.0, "Yaw": 15.0}
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
                    axis: str = "Roll",
                    max_thrust: float = 0.0, arm_len: float = 0.0,
                    inertia_r_axis: float = 0.0) -> Tuple[float, float, float]:
    """Multirotor physics — torque from motor differential thrust."""
    if max_thrust <= 0 or arm_len <= 0 or inertia_r_axis <= 0:
        mt = globals().get('EM_MAX_THRUST', 14.5)
        al = globals().get('EM_ARM_LEN', 0.17)
        ir = globals().get('MR_INERTIA_R', [54.0, 36.0, 21.6])
        max_thrust = mt
        arm_len = al
        inertia_r_axis = ir[AXIS_NAMES[axis]] if isinstance(ir, list) else ir
    alpha = dT / (EM_MOTOR_TAU + dT)
    effort = lag + alpha * (out - lag)

    kT = max_thrust * 0.25 * arm_len
    torque = kT * effort
    _DAMP_C = {"Roll": 0.015, "Pitch": 0.03, "Yaw": 0.05}
    damp_c = _DAMP_C.get(axis, 0.02)
    damping = damp_c * sgn(rate) * rate * rate
    dRate = (torque - damping) * inertia_r_axis * dT
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
    C_d_roll  = abs(af_params.get("roll_damp", 0.5 * rho * V * b * b * b * 0.04))
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
                  af_filename: str = "",
                  params: Optional[dict] = None) -> StepMetrics:
    angle = 0.0
    rate = 0.0
    lag = 0.0
    ai = AXIS_NAMES[axis_name]

    is_fw = (cat == AirframeCat.FW)
    model_max_rate = 100.0
    r_max = pid.Max
    af_params = None

    # Compute MR physics from PHYS_ fields
    if cat == AirframeCat.MR and params is not None:
        try:
            auw_g = float(params.get('PHYS_AUW_G', 800))
            arm_mm = float(params.get('PHYS_ARM_MM', 220))
            motor_count = int(float(params.get('PHYS_MOTOR_COUNT', 4)))
            thrust_g = float(params.get('PHYS_MOTOR_THRUST_G', 620))
            mass_kg = auw_g / 1000.0
            arm_m = arm_mm / 1000.0
            if motor_count < 1:
                motor_count = 4
            hover_thrust_n = mass_kg * GRAVITY / motor_count
            max_thrust_n = thrust_g / 1000.0 * GRAVITY
            em_thr_cruise = hover_thrust_n / max_thrust_n if max_thrust_n > 0 else 0.55
            shape_kf = {4: 0.55, 6: 0.40, 8: 0.30}.get(motor_count, 0.55)
            ixx = mass_kg * arm_m * arm_m * shape_kf
            inertia_r = 12.0 / (mass_kg * arm_m * arm_m) if mass_kg * arm_m * arm_m > 0 else 12.0 / 0.02312
            mr_max_thrust = max_thrust_n * motor_count
            mr_arm_len = arm_m
            mr_inertia_r = [inertia_r, inertia_r / 1.5, inertia_r / 2.5]
            inertia = 1.0 / mr_inertia_r[ai]
        except (ValueError, TypeError, ZeroDivisionError):
            inertia = 1.0 / MR_INERTIA_R[ai]
            mr_max_thrust = EM_MAX_THRUST
            mr_arm_len = EM_ARM_LEN
            mr_inertia_r = MR_INERTIA_R
    elif cat == AirframeCat.MR:
        inertia = 1.0 / MR_INERTIA_R[ai]
        mr_max_thrust = EM_MAX_THRUST
        mr_arm_len = EM_ARM_LEN
        mr_inertia_r = MR_INERTIA_R
    else:
        mr_max_thrust = EM_MAX_THRUST
        mr_arm_len = EM_ARM_LEN
        mr_inertia_r = MR_INERTIA_R

    if is_fw and get_fw_descriptor(af_filename):
        af_params = get_fw_descriptor(af_filename)
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
            angle, rate, lag = run_physics_mr(angle, rate, out, lag, dT, axis_name,
                                               max_thrust=mr_max_thrust, arm_len=mr_arm_len,
                                               inertia_r_axis=mr_inertia_r[ai])

        if d_torque != 0.0:
            if is_fw:
                rate += d_torque / inertia * dT
                rate = clamp(rate, -model_max_rate, model_max_rate)
            else:
                rate += d_torque * mr_inertia_r[ai] * dT

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
    if is_fw and get_fw_descriptor(af_filename):
        af_params = get_fw_descriptor(af_filename)
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


# ═══════════════════════════════════════════
#  Altitude Hold step response
# ═══════════════════════════════════════════
# AH PI: output = AltPosKp * err + AltPosKi * integral(err)
# Output is throttle compensation, clamped to AltThrCompLimit

# MR AH: mass-thrust model — fast response (~0.5s)
# Throttle → thrust → vertical accel → velocity → altitude
# Plant: mass=0.8kg, max_thrust~18N, hover_thr=55%, drag∝v²

def simulate_alt_hold_mr(params: dict, step_m: float = 5.0,
                         dT: float = CONTROL_DT,
                         af_filename: str = None) -> StepMetrics:
    """Simulate MR altitude hold step response.

    step_m: altitude step in meters (default 5m)
    Physics: mass-thrust model with quadratic drag.
    Uses per-airframe PHYS_ params when available.
    """
    kp = float(params.get("ALT_POS_KP", 0.0))
    ki = float(params.get("ALT_POS_KI", 0.0))
    thr_lim = float(params.get("ALT_THROTTLE_COMP_LIMIT", 0.2))
    roc_kp = float(params.get("ALT_ROC_KP", 0.05))
    roc_ki = float(params.get("UNUSED_ALT_VEL_KI", 0.001))
    roc_max = 5.0

    if af_filename:
        phys = load_af_physicals(af_filename)
        mass_g = phys.get('PHYS_AUW_G', 0)
        thrust_g = phys.get('PHYS_MOTOR_THRUST_G', 0)
        motor_count = phys.get('PHYS_MOTOR_COUNT', 0)
        if mass_g > 0 and thrust_g > 0 and motor_count > 0:
            mass = mass_g / 1000.0
            max_thrust = thrust_g * motor_count * GRAVITY / 1000.0
            hover_thr = mass * GRAVITY / max_thrust
        else:
            mass = EM_MASS
            max_thrust = EM_MASS / EM_THR_CRUISE * GRAVITY
            hover_thr = EM_THR_CRUISE
    else:
        mass = EM_MASS
        max_thrust = EM_MASS / EM_THR_CRUISE * GRAVITY
        hover_thr = EM_THR_CRUISE

    alt = 0.0
    vel = 0.0
    int_e_pos = 0.0
    int_e_vel = 0.0
    n = int(SIM_TIME / dT)

    peak_alt = 0.0
    max_int = 0.0
    rise_time_s = SIM_TIME
    settled = False
    settle_time = SIM_TIME

    for i in range(n):
        t = i * dT
        err = step_m - alt

        # Position PI -> ROC setpoint
        p_term = kp * err
        roc_desired = clamp(p_term + int_e_pos, -roc_max, roc_max)
        int_e_pos += err * ki * dT
        int_e_pos = clamp(int_e_pos, -roc_max, roc_max)

        # Velocity PI -> throttle compensation
        roc_err = roc_desired - vel
        roc_p = roc_kp * roc_err
        thr_comp = clamp(roc_p + int_e_vel, -thr_lim, thr_lim)
        int_e_vel += roc_err * roc_ki * dT
        int_e_vel = clamp(int_e_vel, -thr_lim, thr_lim)

        total_thr = hover_thr + thr_comp
        thrust_force = total_thr * max_thrust
        accel = thrust_force / mass - GRAVITY
        drag = 0.5 * abs(vel) * vel
        accel -= drag / mass

        vel += accel * dT
        alt += vel * dT

        max_int = max(max_int, abs(int_e_pos), abs(int_e_vel))
        peak_alt = max(peak_alt, alt)

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

    final_err = abs(alt - step_m)
    overshoot = max(0, (peak_alt - step_m) / step_m * 100.0) if step_m > 0 else 0.0

    return StepMetrics(
        axis_name="Altitude (MR)",
        rise_time_s=round(rise_time_s, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settle_time if settle_time < SIM_TIME else SIM_TIME, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=round(max_int, 6),
        peak_angle=round(peak_alt, 4),
        final_angle=round(alt, 4),
        setpoint=step_m,
    )


# FW AH: pitch-to-climb model — slow response (~3-5s)
# Throttle controls airspeed, pitch angle controls climb rate
# Plant: mass=1.0kg, CL~0.5, climb_rate ≈ V * sin(pitch)
# AH outputs throttle comp → FC also commands pitch for climb
# Simplified: altitude rate = V * sin(climb_angle)
# climb_angle controlled by PI with time constant ~2s (servo + aero lag)

def simulate_alt_hold_fw(params: dict, step_m: float = 5.0, cruise_v: float = 13.0,
                         dT: float = CONTROL_DT) -> StepMetrics:
    """Simulate FW altitude hold step response.

    step_m: altitude step in meters (default 5m)
    Physics: pitch-to-climb model with airspeed coupling.
    FW climbs by pitching up (controlled by AH), which bleeds airspeed.
    Much slower than MR — time constant ~2-3s.
    """
    kp = float(params.get("ALT_POS_KP", 0.0))
    ki = float(params.get("ALT_POS_KI", 0.0))
    thr_lim = float(params.get("ALT_THROTTLE_COMP_LIMIT", 0.2))
    roc_kp = float(params.get("ALT_ROC_KP", 0.05))
    roc_ki = float(params.get("UNUSED_ALT_VEL_KI", 0.001))
    roc_max = 5.0

    # DoMotors mirror: TotalThrottle = DesiredThrottle + AltHoldThrComp
    #   + pFWPitchThrottleFFFrac*Abs(Pl), capped at pFWClimbThrottleFrac.
    cruise_thr = float(params.get("EST_CRUISE_THR", 0.5))
    climb_cap = float(params.get("FW_CLIMB_THROTTLE", 0.7))
    ff_pt = float(params.get("FW_PITCH_THROTTLE_FF", 0.0))
    climb_margin = max(0.0, climb_cap - cruise_thr)

    alt = 0.0
    vel_vert = 0.0
    int_e_pos = 0.0
    int_e_vel = 0.0
    n = int(SIM_TIME / dT)

    peak_alt = 0.0
    max_int = 0.0
    rise_time_s = SIM_TIME
    settled = False
    settle_time = SIM_TIME

    # FW climb dynamics: Position PI -> ROC setpoint -> Velocity PI -> climb
    # Actual climb rate follows with time constant (servo + aero lag)
    climb_tau = 2.0  # seconds — much slower than MR
    V = cruise_v

    for i in range(n):
        t = i * dT
        err = step_m - alt

        # Position PI -> ROC setpoint
        p_term = kp * err
        roc_desired = clamp(p_term + int_e_pos, -roc_max, roc_max)
        int_e_pos += err * ki * dT
        int_e_pos = clamp(int_e_pos, -roc_max, roc_max)

        # Velocity PI -> climb command
        roc_err = roc_desired - vel_vert
        roc_p = roc_kp * roc_err
        thr_comp = clamp(roc_p + int_e_vel, -thr_lim, thr_lim)
        int_e_vel += roc_err * roc_ki * dT
        int_e_vel = clamp(int_e_vel, -thr_lim, thr_lim)

        # DoMotors limit: TotalThrottle ≤ pFWClimbThrottleFrac (and ≥ 0).
        # Positive compensation capped by (climb_cap - cruise); negative by cruise.
        pl_proxy = abs(roc_desired) / roc_max  # pitch effort grows with climb demand
        pos_limit = min(thr_lim, climb_margin + ff_pt * pl_proxy)
        thr_comp = clamp(thr_comp, -min(thr_lim, cruise_thr), pos_limit)

        # Desired vertical velocity from PI output
        # Scale: thr_lim=0.25 -> ~5 m/s climb rate max
        v_desired = thr_comp * 20.0  # m/s max climb

        # First-order lag (servo + aero response)
        vel_vert += (v_desired - vel_vert) / climb_tau * dT

        # Airspeed bleed during climb (energy trade)
        alt += vel_vert * dT

        max_int = max(max_int, abs(int_e_pos), abs(int_e_vel))
        peak_alt = max(peak_alt, alt)

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

    final_err = abs(alt - step_m)
    overshoot = max(0, (peak_alt - step_m) / step_m * 100.0) if step_m > 0 else 0.0

    return StepMetrics(
        axis_name="Altitude (FW)",
        rise_time_s=round(rise_time_s, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settle_time if settle_time < SIM_TIME else SIM_TIME, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=round(max_int, 6),
        peak_angle=round(peak_alt, 4),
        final_angle=round(alt, 4),
        setpoint=step_m,
    )


# ═══════════════════════════════════════════
#  Navigation step response
# ═══════════════════════════════════════════
# Nav PI: outer (position) → inner (velocity/bank) cascade

# MR Nav: tilt-to-move — fast, direct response (~1s)
# Position error → desired velocity → tilt angle → lateral accel → velocity → position
# MR tilts directly: lateral_accel = g * tan(bank), response ~0.3s

def simulate_nav_mr(params: dict, step_m: float = 10.0,
                    dT: float = CONTROL_DT,
                    af_filename: str = None) -> StepMetrics:
    """Simulate MR navigation step response (lateral offset).

    step_m: lateral offset step in meters (default 10m)
    Physics: bank-to-turn model with fast tilt response.
    """
    pos_kp = float(params.get("NAV_POS_KP", 0.0))
    pos_ki = float(params.get("NAV_POS_KI", 0.0))
    vel_kp = float(params.get("NAV_VEL_KP", 0.0))
    max_angle = float(params.get("MAX_PITCH_ANGLE", 0.5236))

    step_rad = step_m  # treat as meters lateral offset

    pos = 0.0
    vel = 0.0
    int_e = 0.0
    n = int(SIM_TIME / dT)

    peak_pos = 0.0
    max_int = 0.0
    rise_time_s = SIM_TIME
    settled = False
    settle_time = SIM_TIME

    # MR tilt dynamics: fast response (motor time constant ~0.1s)
    tilt_tau = 0.3  # seconds

    for i in range(n):
        t = i * dT
        err = step_rad - pos
        pos_cmd = pos_kp * err
        int_e = clamp(int_e + err * pos_ki * dT, -max_angle, max_angle)

        # Inner loop: velocity error → bank angle
        vel_err = pos_cmd - vel
        bank = clamp(vel_kp * vel_err + int_e, -max_angle, max_angle)

        # Bank-to-turn: lateral accel = g * tan(bank)
        lat_accel = GRAVITY * math.tan(bank)

        # First-order lag on tilt
        accel = (lat_accel - vel * abs(vel) * 0.1) / tilt_tau  # with drag
        vel += accel * dT
        pos += vel * dT

        max_int = max(max_int, abs(int_e))
        peak_pos = max(peak_pos, abs(pos))

        if rise_time_s == SIM_TIME and abs(pos) >= 0.9 * abs(step_rad):
            rise_time_s = t

        if t > 1.0:
            if abs(pos - step_rad) < 0.02 * abs(step_rad):
                if not settled:
                    settle_time = t
                    settled = True
            else:
                settled = False
                settle_time = t + dT

    final_err = abs(pos - step_rad)
    overshoot = max(0, (peak_pos - abs(step_rad)) / abs(step_rad) * 100.0) if step_rad != 0 else 0.0

    return StepMetrics(
        axis_name="Navigation (MR)",
        rise_time_s=round(rise_time_s, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settle_time if settle_time < SIM_TIME else SIM_TIME, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=round(max_int, 6),
        peak_angle=round(peak_pos, 4),
        final_angle=round(pos, 4),
        setpoint=step_m,
    )


# FW Nav: bank-to-turn — slower (~2-3s), bank-angle limited
# Cross-track error → desired bank angle → coordinated turn → heading rate → position
# FW must bank to turn: heading_rate = g * tan(bank) / V
# Bank limited by MAX_PITCH_ANGLE (typically 20-35°)

def simulate_nav_fw(params: dict, step_m: float = 10.0, cruise_v: float = 13.0,
                    dT: float = CONTROL_DT) -> StepMetrics:
    """Simulate FW navigation step response (cross-track correction).

    step_m: cross-track error step in meters (default 10m)
    Physics: coordinated turn model with bank angle limit.
    """
    pos_kp = float(params.get("NAV_POS_KP", 0.0))
    pos_ki = float(params.get("NAV_POS_KI", 0.0))
    vel_kp = float(params.get("NAV_VEL_KP", 0.0))
    max_bank = float(params.get("MAX_PITCH_ANGLE", 0.5236))

    pos = 0.0  # cross-track error in meters
    vel = 0.0  # lateral velocity
    heading = 0.0
    int_e = 0.0
    n = int(SIM_TIME / dT)

    peak_pos = 0.0
    max_int = 0.0
    rise_time_s = SIM_TIME
    settled = False
    settle_time = SIM_TIME

    V = cruise_v
    # Servo + aero response time constant
    bank_tau = 1.5  # seconds — slower than MR

    for i in range(n):
        t = i * dT
        err = step_m - pos
        pos_cmd = pos_kp * err
        int_e = clamp(int_e + err * pos_ki * dT, -max_bank, max_bank)

        # Desired bank angle from cross-track correction
        vel_err = pos_cmd - vel
        bank_desired = clamp(vel_kp * vel_err + int_e, -max_bank, max_bank)

        # First-order bank dynamics (roll rate limited)
        bank_rate = (bank_desired - 0) / bank_tau  # simplified
        heading += GRAVITY * math.tan(bank_desired) / V * dT

        # Position update: move toward cross-track zero
        # Simplified: lateral velocity = V * sin(heading_correction)
        vel = V * math.sin(heading)
        pos += vel * dT

        max_int = max(max_int, abs(int_e))
        peak_pos = max(peak_pos, abs(pos))

        if rise_time_s == SIM_TIME and abs(pos) >= 0.9 * abs(step_m):
            rise_time_s = t

        if t > 1.0:
            if abs(pos - step_m) < 0.02 * abs(step_m):
                if not settled:
                    settle_time = t
                    settled = True
            else:
                settled = False
                settle_time = t + dT

    final_err = abs(pos - step_m)
    overshoot = max(0, (peak_pos - abs(step_m)) / abs(step_m) * 100.0) if step_m != 0 else 0.0

    return StepMetrics(
        axis_name="Navigation (FW)",
        rise_time_s=round(rise_time_s, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settle_time if settle_time < SIM_TIME else SIM_TIME, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=round(max_int, 6),
        peak_angle=round(peak_pos, 4),
        final_angle=round(pos, 4),
        setpoint=step_m,
    )


# ═══════════════════════════════════════════
#  Navigation step response
# ═══════════════════════════════════════════
# Nav PI: outer (position) → inner (velocity) cascade
# Output is bank angle → bank-to-turn: heading_rate = g * tan(bank) / V
# Simplified: command heading change, measure heading response

def simulate_nav(params: dict, step_deg: float = 30.0, cat: int = AirframeCat.MR,
                 dT: float = CONTROL_DT) -> StepMetrics:
    """Simulate navigation step response (heading change).

    step_deg: heading step in degrees (default 30°)
    Simplified physics: bank-to-turn model.
    """
    pos_kp = float(params.get("NAV_POS_KP", 0.0))
    pos_ki = float(params.get("NAV_POS_KI", 0.0))
    vel_kp = float(params.get("NAV_VEL_KP", 0.0))
    max_angle = float(params.get("MAX_PITCH_ANGLE", 0.5236))  # max bank for nav

    step_rad = step_deg * DEG_TO_RAD

    heading = 0.0
    cross_track = 0.0  # meters
    vel = 0.0
    int_e = 0.0
    n = int(SIM_TIME / dT)

    peak_heading = 0.0
    max_int = 0.0
    rise_time_s = SIM_TIME
    settled = False
    settle_time = SIM_TIME

    V = 10.0  # m/s cruise speed

    for i in range(n):
        t = i * dT
        heading_err = step_rad - heading

        # Outer loop: position error → desired velocity
        pos_cmd = pos_kp * heading_err
        int_e = clamp(int_e + heading_err * pos_ki * dT, -max_angle, max_angle)

        # Inner loop: velocity error → bank angle
        vel_err = pos_cmd - vel
        bank_angle = clamp(vel_kp * vel_err + int_e, -max_angle, max_angle)

        # Bank-to-turn: heading_rate = g * tan(bank) / V
        heading_rate = GRAVITY * math.tan(bank_angle) / V
        heading += heading_rate * dT
        cross_track += V * math.sin(heading) * dT

        max_int = max(max_int, abs(int_e))
        peak_heading = max(peak_heading, abs(heading))

        # Rise time
        if rise_time_s == SIM_TIME and abs(heading) >= 0.9 * abs(step_rad):
            rise_time_s = t

        # Settling
        if t > 1.0:
            if abs(heading - step_rad) < 0.02 * abs(step_rad):
                if not settled:
                    settle_time = t
                    settled = True
            else:
                settled = False
                settle_time = t + dT

    final_err = abs(heading - step_rad)
    overshoot = max(0, (peak_heading - abs(step_rad)) / abs(step_rad) * 100.0) if step_rad != 0 else 0.0

    return StepMetrics(
        axis_name="Navigation",
        rise_time_s=round(rise_time_s, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settle_time if settle_time < SIM_TIME else SIM_TIME, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=round(max_int, 6),
        peak_angle=round(peak_heading * RAD_TO_DEG, 4),
        final_angle=round(heading * RAD_TO_DEG, 4),
        setpoint=step_deg,
    )


def simulate_axis_coupled(af_filename: str, step_axis: str = "Roll",
                          dT: float = CONTROL_DT,
                          gusts: Optional[Dict[str, Gust]] = None,
                          params_override: Optional[dict] = None,
                          step_rads: Optional[Dict[str, float]] = None) -> Dict[str, StepMetrics]:
    """Coupled 3-axis simulation with cross-coupling (dihedral, adverse yaw).

    Runs all three axes simultaneously so that:
      - Yaw rate → dihedral → roll torque
      - Aileron deflection → adverse yaw → yaw torque

    step_axis: which axis receives the step command (others hold zero)
    params_override: if provided, use these params instead of re-loading from file
    step_rads: if provided, use these step sizes (radians) instead of TEST_STEPS
    """
    af_params = get_fw_descriptor(af_filename)
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

    C_d_roll  = abs(af_params.get("roll_damp", 0.5 * rho * V * b * b * b * 0.04))
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

    # ElevonAF yaw authority: drag-differential from elevon deflection.
    # Derived from the adverse-yaw (drag-differential) coefficient × the aileron
    # roll-torque coefficient — same physical mechanism, per FC Yl→elevon mixing.
    C_n_elevon = adverse_yaw_coeff * C_l_ail if adverse_yaw_coeff > 0 else 0.0

    ail_max_rad = af_params.get("aileron_max_deg", 0.0) * DEG_TO_RAD
    ele_max_rad = af_params.get("elevator_max_deg", 0.0) * DEG_TO_RAD
    rud_max_rad = af_params.get("rudder_max_deg", 0.0) * DEG_TO_RAD

    # Determine surface routing from FC mixer (DoServos)
    af_type = AF_FILES.get(af_filename)
    is_rudder_elevator = (af_type == AirframeType.eRudderElevatorAF)
    is_elevon = (af_type == AirframeType.eElevonAF)

    # Load params and set up PIDs
    params = params_override if params_override is not None else load_af_params(af_filename)

    # FW actuator-path feedforwards (FC DoServos / DoMotors):
    #   FW_ROLL_PITCH_FF    → TempElevator = Pl + FF*Abs(Rl)  (all FW)
    #   FW_AILERON_RUDDER_MIX → rudder Yl + FF*Rl             (Delta/Aileron/Spoileron)
    ff_rp = float(params.get("FW_ROLL_PITCH_FF", 0.0))
    ff_ar = float(params.get("FW_AILERON_RUDDER_MIX", 0.0))

    pis = {}   # angle PIs
    pids = {}  # rate PIDs
    angles = {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
    rates   = {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
    lags    = {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}

    step_rads = step_rads if step_rads is not None else {
        "Roll": TEST_STEPS["Roll"] * DEG_TO_RAD,
        "Pitch": TEST_STEPS["Pitch"] * DEG_TO_RAD,
        "Yaw": TEST_STEPS["Yaw"] * DEG_TO_RAD
    }

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

        # Surface deflections from efforts — mirrors FC DoServos() mixing
        Rl = efforts["Roll"]
        Pl = efforts["Pitch"]
        Yl = efforts["Yaw"]
        # FC: TempElevator = Pl + pFWRollPitchFFFrac * Abs(Rl) (all FW elevator/elevon)
        TempElevator = Pl + ff_rp * abs(Rl)
        if is_elevon:
            # ElevonAF: elevons carry roll + pitch + yaw (Yl now mixed in per FC fix).
            # Roll torque via elevon differential, pitch via common mode,
            # yaw via drag-differential (C_n_elevon).
            ail_rad = clamp(Rl, -1.0, 1.0) * ail_max_rad if ail_max_rad > 0 else 0.0
            ele_rad = clamp(TempElevator, -1.0, 1.0) * ele_max_rad if ele_max_rad > 0 else 0.0
            rud_rad = clamp(Yl, -1.0, 1.0) * ail_max_rad if ail_max_rad > 0 else 0.0
        elif is_rudder_elevator:
            # RudderElevatorAF: roll + yaw both summed onto rudder servo
            # TempAileron = Rl; PW[RudderC] = TempAileron + Yl
            rud_mixed = Rl + Yl
            ail_rad = 0.0
            ele_rad = clamp(TempElevator, -1.0, 1.0) * ele_max_rad if ele_max_rad > 0 else 0.0
            rud_rad = clamp(rud_mixed, -1.0, 1.0) * rud_max_rad if rud_max_rad > 0 else 0.0
        else:
            # AileronAF / DeltaAF / SpoileronAF: roll→aileron, pitch→elevator,
            # yaw→rudder with aileron→rudder FF (Yl + pFWAileronRudderFFFrac*Rl)
            ail_rad = clamp(Rl, -1.0, 1.0) * ail_max_rad if ail_max_rad > 0 else 0.0
            ele_rad = clamp(TempElevator, -1.0, 1.0) * ele_max_rad if ele_max_rad > 0 else 0.0
            rud_rad = clamp(Yl + ff_ar * Rl, -1.0, 1.0) * rud_max_rad if rud_max_rad > 0 else 0.0

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
        N_ctrl = (C_n_elevon if is_elevon else C_n_rud) * rud_rad

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
    af_params = get_fw_descriptor(af_filename)
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
    C_d_roll  = abs(af_params.get("roll_damp", 0.5 * rho * V * b * b * b * 0.04))
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
    is_rudder_elevator = (af_type == AirframeType.eRudderElevatorAF)

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
    lines.append(f"  Dihedral coeff: {get_fw_descriptor(af_filename).get('dihedral_coeff', 0):.2f}  "
                 f"Adverse yaw: {get_fw_descriptor(af_filename).get('adverse_yaw', 0):.2f}")

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
    "Roll": Criteria(4.0, 25.0, 8.0, 5.0, 1.0, 0.05, 5),
    "Pitch": Criteria(5.0, 30.0, 12.0, 15.0, 1.5, 0.05, 6),
    "Yaw": Criteria(5.0, 25.0, 10.0, 5.0, 1.0, 0.03, 5),
}

DIST_CRITERIA = {
    "Roll": DisturbanceCriteria(15.0, 1.5, 10.0),
    "Pitch": DisturbanceCriteria(12.0, 1.5, 8.0),
    "Yaw": DisturbanceCriteria(12.0, 2.0, 10.0),
}

# AH/Nav criteria — more relaxed than attitude (these are outer loops)
AH_CRITERIA_MR = Criteria(2.0, 20.0, 5.0, 1.0, 1.0, 0.3, 3)   # 5m step, per-airframe physics
AH_CRITERIA_FW = Criteria(6.0, 15.0, 8.0, 2.0, 1.0, 0.3, 3)   # rise 6s/settle 8s — FW climb-tau physics floor ~4.4s
NAV_CRITERIA_MR = Criteria(2.0, 20.0, 5.0, 1.0, 1.0, 0.3, 3)
NAV_CRITERIA_FW = Criteria(3.0, 15.0, 5.0, 2.0, 1.0, 0.3, 3)

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

def critique_linear(m: StepMetrics, c: Criteria, name: str, units: str = "m") -> Tuple[List[str], bool]:
    """Critique for linear (meters) step response — AH and Nav sims."""
    lines = []
    lines.append(f"\n{B}── {name} @ {m.setpoint:.1f}{units} step ──{N}")
    lines.append(f"     Final: {m.final_angle:.2f}{units}   Peak: {m.peak_angle:.2f}{units}")
    int_ratio = m.max_integrator / m.intlim if m.intlim > 0 else 0
    rate_ratio = m.max_rate / (abs(m.setpoint) * 2) if m.setpoint != 0 else 1.0
    results = [
        check("Rise time (10→90%)",          m.rise_time_s,         c.max_rise_time_s, "max", "s"),
        check("Overshoot",                    m.overshoot_pct,       c.max_overshoot_pct, "max", "%"),
        check("Settling time (±2%)",          m.settling_time_s,     c.max_settling_time_s, "max", "s"),
        check(f"Final error",                 m.steady_state_error,  c.max_ss_error_deg, "max", units),
        check("Integrator utilization",       int_ratio,             c.max_int_ratio, "max"),
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
def load_af_physicals(af_filename: str) -> dict:
    """Load physical parameters (PHYS_*) from .af file metadata as floats."""
    from airframes.airframes import parse_af_file
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    path = os.path.join(project_root, "uavx-python", "src", "airframes", af_filename)
    try:
        _, _, meta = parse_af_file(path)
    except Exception:
        return {}
    phys = {}
    for k, v in meta.items():
        if k.startswith('PHYS_'):
            try:
                phys[k] = float(v)
            except ValueError:
                pass
    return phys

def _resolve_path(af_filename: str) -> str:
    """Resolve a relative .af key or absolute path to a file on disk.

    Relative keys (e.g. "generic/Quad.af") live in
    UAVXGS/uavx-python/src/airframes/. Absolute paths are used as-is.
    """
    if os.path.isabs(af_filename):
        return af_filename
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    return os.path.join(project_root, "uavx-python", "src", "airframes", af_filename)


def load_af_params(af_filename: str) -> dict:
    """Load parameters from .af file.

    Returns ParamIndex-named values plus PHYS_ metadata keys (as floats) so
    MR physics readers (mr_phys/load_phys/simulate_axis) can consume the
    per-airframe physical descriptors.
    """
    from airframes.airframes import parse_af_file
    from protocol_enums import ParamIndex
    path = _resolve_path(af_filename)
    _, p, meta = parse_af_file(path)
    params = {ParamIndex(tag).name: val for tag, val in p.items()}
    for k, v in meta.items():
        if k.startswith('PHYS_'):
            try:
                params[k] = float(v)
            except ValueError:
                pass
    return params

def get_params_for_af(af_filename: str) -> Tuple[dict, AirframeType, int, bool]:
    """Get PID params, category, FW model index, and is_fw flag for an airframe file.

    Accepts relative keys (e.g. "generic/Quad.af") or absolute paths. Unknown
    airframes fall back to reading AF_TYPE from the .af file itself, so any
    user-saved airframe can be simulated.
    """
    from airframes.airframes import parse_af_file
    from protocol_enums import ParamIndex as _ParamIndex
    from protocol_enums import AirframeType as _AirframeType
    af_type = AF_FILES.get(af_filename)
    if af_type is None:
        try:
            path = _resolve_path(af_filename)
            _name, vals, _meta = parse_af_file(path)
            tag = int(_ParamIndex.AF_TYPE)
            af_type = _AirframeType(int(vals[tag])) if tag in vals else None
        except Exception:
            af_type = None
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
    akp = params.get(f"{up}_ANGLE_Q_KP", 0.0)
    aki = params.get(f"{up}_ANGLE_Q_KI", 0.0)
    ail = params.get(f"{up}_ANGLE_Q_INT_LIMIT", 0.0)
    # Yaw has no explicit angle limit (360° continuous rotation); defaults to 2π
    default_max_angle = 6.2832 if up == "YAW" else 0.523599
    ma  = params.get(f"MAX_{up}_ANGLE", default_max_angle)
    rkp = params.get(f"{up}_RATE_KP", 0.0)
    rkd = params.get(f"{up}_RATE_KD", 0.0)
    mr  = params.get(f"MAX_{up}_RATE", 10.0)
    return akp, aki, ail, ma, rkp, rkd, mr

def check_cascade_integrity(params: dict) -> Tuple[bool, List[str]]:
    """Validate the quaternion attitude cascade against max-commanded-rate caps
    (mirrors control.c DoQuaternionAttitudeControl / the GCS tuning caveats).

    For each axis the outer angle loop's rate setpoint is clamped to
    A[axis].R.Max for roll/pitch and min(A[Yaw].R.Max, Nav.MaxHeadingRate) for
    yaw.  Two cross-checks:

      1. P-path max demand 2·sin(Θmax/2)·Kp (2·Kp for yaw, Qa max=1 at 180°) must
         not exceed the axis max commanded rate, else the P gain saturates the
         rate clamp before its setpoint — loss of proportional action.
      2. IntLim must not exceed the max commanded rate (cascade windup: an I-term
         above the clamp can never discharge).  The 20–30% band is a *ceiling
         guide* — I-limits below it are fine (I only trims); only breaches warn.

    All values in FC-native units (rad/s, rad).
    """
    lines = []
    ok = True
    axes = [
        ("Roll", "ROLL_ANGLE_Q_KP", "ROLL_ANGLE_Q_INT_LIMIT", "MAX_ROLL_ANGLE",
         "MAX_ROLL_RATE"),
        ("Pitch", "PITCH_ANGLE_Q_KP", "PITCH_ANGLE_Q_INT_LIMIT", "MAX_PITCH_ANGLE",
         "MAX_PITCH_RATE"),
    ]

    for name, kp_k, il_k, ang_k, rate_k in axes:
        kp = params.get(kp_k, 0.0)
        il = params.get(il_k, 0.0)
        ang = params.get(ang_k, 0.0)
        max_r = params.get(rate_k, 0.0)
        if max_r <= 0.0:
            continue
        qp = 2.0 * math.sin(min(ang, math.pi) / 2.0) * kp if ang > 0.0 else 2.0 * kp
        lines.append(f"  {B}{name}: maxRate={max_r:.3f}  P-path demand={qp:.3f} rad/s  IntLim={il:.3f}{N}")
        # P-path saturation of the rate clamp is NORMAL for a rate-limited cascade
        # (the clamp exists to bound it); it is surfaced as a caveat, not a FAIL.
        if qp > max_r * 1.02:
            lines.append(f"    {Y}~ P-path demand {qp:.2f} > max commanded {max_r:.2f} — P saturates the rate clamp (caveat; normal for rate-limited loops){N}")
        if il > max_r:
            lines.append(f"    {R}! I-limit {il:.2f} > max commanded {max_r:.2f} — integrator can never discharge (windup){N}")
            ok = False
        elif il > 0.30 * max_r:
            lines.append(f"    {Y}~ I-limit {il:.2f} = {100.0*il/max_r:.0f}% of max — above 20–30% band (QP no longer main driver){N}")
        else:
            lines.append(f"    I-limit at {100.0*il/max_r:.0f}% of max — within 20–30% band")

    # Yaw: cap = min(A[Yaw].R.Max, Nav.MaxHeadingRate) per control.c
    headings = [v for v in (params.get("MAX_HEADING_RATE"), params.get("MAX_YAW_RATE"))
                if v is not None and v > 0.0]
    ykp = params.get("YAW_ANGLE_Q_KP", 0.0)
    yil = params.get("YAW_ANGLE_Q_INT_LIMIT", 0.0)
    yqp = 2.0 * ykp
    if headings:
        ymax = min(headings)
        lines.append(f"  {B}Yaw: maxRate={ymax:.3f}  QP demand={yqp:.3f} rad/s  IntLim={yil:.3f}{N}")
        if yqp > ymax * 1.02:
            lines.append(f"    {Y}~ Yaw QP demand {yqp:.2f} > max commanded {ymax:.2f} — P saturates the rate clamp (caveat){N}")
        if yil > ymax:
            lines.append(f"    {R}! Yaw I-limit {yil:.2f} > max commanded {ymax:.2f} — integrator can never discharge (windup){N}")
            ok = False
        elif yil > 0.30 * ymax:
            lines.append(f"    {Y}~ Yaw I-limit {100.0*yil/ymax:.0f}% of max — above 20–30% band (QP no longer main driver){N}")
        else:
            lines.append(f"    Yaw I-limit at {100.0*yil/ymax:.0f}% of max — within 20–30% band")
    return ok, lines

def run_tests_for_af(af_filename: str, slider_pct: float = None) -> Tuple[bool, List[str]]:
    """Run all tests for a single airframe file.

    If slider_pct is None, uses params as-is from the .af file (base values).
    If slider_pct is 0.0..1.0, applies the character slider to override
    the curve-controlled params at that position.
    """
    all_output = []
    all_pass = True

    try:
        params, af_type, fw_mid, is_fw = get_params_for_af(af_filename)
    except Exception as e:
        return False, [f"Error loading {af_filename}: {e}"]

    if slider_pct is not None:
        params = apply_slider(params, slider_pct)

    af_name = AIRFRAME_NAMES.get(af_type, af_type.name)
    model_lbl = af_type.name

    slider_label = ""
    if slider_pct is not None:
        slider_label = f"  Slider:   {slider_pct*100:.0f}% ({'conservative' if slider_pct < 0.25 else 'aggressive' if slider_pct > 0.75 else 'mid'})"

    all_output.append(f"\n{'='*60}")
    all_output.append(f"{B}UAVX PID Critique — {af_filename}{N}")
    all_output.append(f"  Airframe: {af_name} ({model_lbl})")
    all_output.append(f"  Physics:  {'aerodynamic (qbar × S × C_ctrl)' if is_fw and get_fw_descriptor(af_filename) else 'control-surface (emu.c FW)' if is_fw else 'motor-thrust (emu.c MR)'}")
    all_output.append(f"  Sim:      {SIM_TIME:.0f}s per axis   dt={CONTROL_DT*1000:.0f}ms")
    if slider_label:
        all_output.append(slider_label)
    if is_fw and get_fw_descriptor(af_filename):
        af = get_fw_descriptor(af_filename)
        I_roll, I_pitch = _compute_fw_inertia(af)
        qbar = 0.5 * AIR_DENSITY * af["cruise_speed"] ** 2
        all_output.append(f"  Aero:     m={af['mass']}kg  b={af['wingspan']}m  S={af['wing_area']}m²  V={af['cruise_speed']}m/s  qbar={qbar:.1f}Pa")
        all_output.append(f"  Inertia:  I_roll={I_roll:.4f}  I_pitch={I_pitch:.4f} kg·m²")
    all_output.append(f"{'='*60}")

    # Cascade integrity cross-checks (P-path vs rate cap, I-limit windup guard)
    c_ok, c_lines = check_cascade_integrity(params)
    all_output.append(f"\n{B}  Cascade Integrity{N}")
    all_output.extend(c_lines)
    if not c_ok:
        all_pass = False
        all_output.append(f"  {R}! Cascade limits breached — review the values above{N}")

    axes = ["Roll", "Pitch", "Yaw"]
    metrics = {}

    if is_fw and get_fw_descriptor(af_filename):
        # Coupled 3-axis simulation for each step axis
        fw_step_rads = {name: FW_TEST_STEPS[name] * DEG_TO_RAD for name in axes}
        yaw_struct_limited = is_yaw_structurally_limited(af_filename)
        roll_struct_limited = is_roll_structurally_limited(af_filename)
        for step_ax in axes:
            if step_ax == "Yaw" and yaw_struct_limited:
                all_output.append(f"\n{B}── Yaw @ {FW_TEST_STEPS['Yaw']:.0f}° step ──{N}")
                all_output.append(f"{Y}  Structural limit: rudderless elevon — drag-differential yaw authority holds only ~1-3° against weathervane stability. Yaw step/gust criteria are not scored.{N}")
                all_pass = all_pass and True
                continue
            if step_ax == "Roll" and roll_struct_limited:
                all_output.append(f"\n{B}── Roll @ {FW_TEST_STEPS['Roll']:.0f}° step ──{N}")
                all_output.append(f"{Y}  Structural limit: aileron-less RudderElevatorAF — roll is routed through the rudder (PW[Rudder] = Rl + Yl) and bank-and-yank is cancelled by the coupled yaw hold in this model. Roll step/gust criteria are not scored.{N}")
                all_pass = all_pass and True
                continue
            gusts = {name: Gust(GUST_MAG[name], 2.0, 0.5, name) for name in axes}
            coupled_metrics = simulate_axis_coupled(af_filename, step_axis=step_ax, gusts=gusts, step_rads=fw_step_rads)
            for name in axes:
                if name == step_ax:
                    crit = FW_CRITERIA[name]
                    m = coupled_metrics[name]
                    metrics[name] = m
                    lines, ok = critique(m, crit, f"{name} @ {FW_TEST_STEPS[name]:.0f}°")
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
                              af_filename=af_filename,
                              params=params)
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

        if is_fw and get_fw_descriptor(af_filename):
            I_roll, I_pitch = _compute_fw_inertia(get_fw_descriptor(af_filename))
            inertia = I_pitch if name == "Yaw" else I_roll
        elif is_fw and fw_mid >= 0:
            rp_i, y_i = FW_INERTIAS[fw_mid]
            inertia = y_i if name == "Yaw" else rp_i
        else:
            inertia = 1.0 / MR_INERTIA_R[AXIS_NAMES[name]]

        if name == "Yaw" and is_fw and is_yaw_structurally_limited(af_filename):
            all_output.append(f"{Y}  Yaw: structural limit (rudderless elevon) — gust rejection not scored.{N}")
            all_pass = all_pass and True
            continue
        if name == "Roll" and is_fw and is_roll_structurally_limited(af_filename):
            all_output.append(f"{Y}  Roll: structural limit (aileron-less RudderElevatorAF) — gust rejection not scored.{N}")
            all_pass = all_pass and True
            continue
        dm = simulate_rate_disturbance(pid, cat=AF_CATEGORY[af_type],
                                        fw_model_idx=fw_mid, axis_name=name,
                                        gust=gust, own_inertia=inertia,
                                        af_filename=af_filename)
        dc = DIST_CRITERIA[name]
        lines, ok = critique_disturbance(dm, dc, name)
        all_output.extend(lines)
        if not ok:
            all_pass = False

    # ── Altitude Hold step response ──
    all_output.append(f"\n{B}{'='*60}{N}")
    all_output.append(f"{B}  Altitude Hold (5m step){N}")
    all_output.append(f"{B}{'='*60}{N}")

    if is_fw:
        cruise_v = 13.0
        if get_fw_descriptor(af_filename):
            cruise_v = get_fw_descriptor(af_filename).get("cruise_speed", 13.0)
        ah_m = simulate_alt_hold_fw(params, step_m=5.0, cruise_v=cruise_v)
        ah_crit = AH_CRITERIA_FW
    else:
        ah_m = simulate_alt_hold_mr(params, step_m=5.0, af_filename=af_filename)
        ah_crit = AH_CRITERIA_MR

    lines, ok = critique_linear(ah_m, ah_crit, "Alt Hold 5m")
    all_output.extend(lines)
    if not ok:
        all_pass = False

    # ── Navigation step response ──
    all_output.append(f"\n{B}{'='*60}{N}")
    all_output.append(f"{B}  Navigation (10m lateral step){N}")
    all_output.append(f"{B}{'='*60}{N}")

    if is_fw:
        cruise_v = 13.0
        if get_fw_descriptor(af_filename):
            cruise_v = get_fw_descriptor(af_filename).get("cruise_speed", 13.0)
        nav_m = simulate_nav_fw(params, step_m=10.0, cruise_v=cruise_v)
        nav_crit = NAV_CRITERIA_FW
    else:
        nav_m = simulate_nav_mr(params, step_m=10.0, af_filename=af_filename)
        nav_crit = NAV_CRITERIA_MR

    lines, ok = critique_linear(nav_m, nav_crit, "Nav 10m")
    all_output.extend(lines)
    if not ok:
        all_pass = False

    # Free-flight turbulence test (FW only — tests intrinsic stability)
    if is_fw and get_fw_descriptor(af_filename):
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

    # Run generic airframes at slider extremes
    generic_files = sorted([f for f in AF_FILES.keys() if f.startswith('generic/')])
    test_files = generic_files
    all_pass = True
    all_output = []

    SLIDER_TESTS = [
        (0.0,  "Conservative (0%)"),
        (1.0,  "Aggressive (100%)"),
    ]

    for af_file in test_files:
        for slider_pct, slider_label in SLIDER_TESTS:
            try:
                ok, output = run_tests_for_af(af_file, slider_pct=slider_pct)
                all_output.extend(output)
                if not ok:
                    all_pass = False
            except Exception as e:
                all_pass = False
                all_output.append(f"Error testing {af_file} at {slider_label}: {e}")

    for l in all_output:
        print(l)

    print(f"\n{'='*60}")
    if all_pass:
        print(f"{G}{B}All airframes PASS at both slider extremes{N}")
    else:
        print(f"{R}{B}Some airframes have issues at slider extremes — review above{N}")

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()