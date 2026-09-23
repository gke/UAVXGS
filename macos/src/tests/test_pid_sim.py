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
import cmath
import sys
import os
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Callable, Sequence

# Find project root (UAVXGS directory)
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AIRFRAMES_DIR = os.path.join(PROJECT_ROOT, "uavx-python", "src", "airframes")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Shared critic: measurement definitions + pass/fail criteria, so the FC
# (trace.c captures) and the sim judge a response identically.
from critic.metrics import (  # noqa: E402
    StepMetrics,
    DisturbanceMetrics,
    count_zero_crossings,
    step_metrics_window,
    step_metrics_persist,
    disturbance_metrics,
)
from critic.criteria import (  # noqa: E402
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

RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0
GRAVITY = 9.80665

# ── Control-authority calibration knob ─────────────────────────────────────
# The FW plant's deflection→rate gain is UNCALIBRATED (the sim assumes
# qbar·S·b·C_ctrl vs the machine's real surface effectiveness).  FW_AUTHORITY_SCALE
# multiplies the control-torque coefficients (CL_D_AIL, CM_D_ELE, CN_D_RUD).
#
# 2026-09-13 FIELD-DERIVED FUDGE — authority ×4 (Greg, Shadow flight):
#   The real Shadow (1000 g plank) was stable/controllable in rough air with the
#   Ch10 RateGainScale pot at its low end (scale 0.25, 4^(2p-1)), i.e. an
#   EFFECTIVE rate Kp of 0.1 × 0.25 = 0.025.  The sim, at 1× authority, tuned
#   the same frame to Kp = 0.1.  A rate loop's closed-loop pole scales with
#   (CE · Kp / I); the airframe needed 4× LESS gain than the sim for a
#   comparable response, so the sim's implicit control effectiveness is 4× too
#   SMALL.  Multiplying CE by 4 makes the sim reproduce the field: stored
#   Kp=0.1 becomes too hot (as flown) and the retuned gain drops toward 0.025.
#   Direction is NOT authority /4 — that would make the sim MORE docile and
#   push recommended gains UP, the opposite of what the aircraft demanded.
#
# PROVISIONAL: derived from one aircraft in one air mass; a correctly-working
#   trace dump (missing today — the 2026-09-13 dump was a 32-byte no-capture
#   placeholder) is the intended per-airframe calibration instrument.  This is
#   "the best fudge we have", not truth.  Override for sweeps/sensitivity:
#   UAVX_FW_AUTHORITY=2.0 python3 src/tests/test_pid_sim.py generic/Shadow.af
#   (1.0 restores the pre-2026-09-13 sim so old report numbers are reproducible).
FW_AUTHORITY_SCALE = float(os.environ.get("UAVX_FW_AUTHORITY", "4.0"))

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
from protocol_enums import AirframeCategory, category_of as _af_category_of

# AF type -> category, derived from the single authority (protocol_enums
# AIRFRAME_CATEGORY / FC ClassifyAFType()), filled for every enum member so
# dict indexing never misses. Do NOT add a fork here.
AF_CATEGORY = {af: _af_category_of(af) for af in AirframeType}

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
    # Shadow2 = working template for encoding user knowledge: edit the FW_AIRFRAMES
    # descriptor geometry keys (sweep_deg/dihedral_deg/anhedral_deg/anhedral_start/
    # taper_ratio) — the .af file itself can only hold the scalar PHYS_DIHEDRAL.
    "generic/Shadow2.af": AirframeType.eElevonAF,
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
# Sweep contribution to effective dihedral: C_lβ,sweep = K_SWEEP · CL · tan(Λ) ·
# Fλ.  Rolls over the spanwise-lift-leverage of a yawed swept wing (∝ total
# lift); NASA NTRS 19930080953 states roll-due-to-sideslip for sweep is only
# ~1/3–1/6 that of an equal dihedral angle, which places K_SWEEP near unity
# (swept planks like Shadow 30°/Horten 40° read ~4–6° EDA at cruise CL).
K_SWEEP = 1.0

# ── Lateral-directional mode analysis (linearized, β-p-r-φ) ────────────────
# Dutch-roll / spiral / roll-subsidence eigenvalues of the small-perturbation
# lateral-directional rigid-body model about level cruise.  Diagnostic only —
# not a tuning input and not read by simulate_axis_coupled.  The nonlinear
# sim's quadratic damping is represented here as an equivalent LINEAR term
# evaluated at a representative oscillation amplitude (LIN_EQ_RATE), making
# the mode analysis a conservative worst-case (amplitude-dependent damping
# only adds more at larger excursions).  Fin-derived sideforce Y_β and yaw
# damping N_r use a fin lift-slope assumption A_V_FIN (typical RC fin AR≈1-2).
A_V_FIN = 2.5            # fin lift slope a_v, /rad
LIN_EQ_RATE = 0.25       # representative |rate| for eq-linear damping, rad/s
DR_ZETA_MIN = 0.05       # FAIL: Dutch-roll damping ratio below this
SPIRAL_T_DOUBLE_MIN = 8.0  # FAIL: unstable spiral with time-to-double below, s

# ═══════════════════════════════════════════
#  Character slider param curves
#  Maps ParamIndex → (conservative_raw, aggressive_raw)
#  Values are FC raw float32 units (rad, rad/s, fraction, etc.)
#  Used to test slider extremes for safety.
# ═══════════════════════════════════════════
from protocol_enums import ParamIndex as _PI

# Character-slider curve tables (cons → agg, raw FC values).
#
# Two layers:
#   _PARAM_CURVES — vehicle-shared envelope: rate/angle limits, altitude
#                   hold, angle limits, horizon. Applies to every airframe.
#   MR_CURVES / FW_CURVES — per-category attitude + nav gains. MR and FW are
#                   different plants: a single global curve cannot fit both
#                   (MR nav needs NAV_POS_KP ≥ ~2.5 to meet its 2s rise with a
#                   20° bank; FW nav overshoots past its 15% bar above ~2.0 and
#                   is happy at 1.0-1.5. MR rate Kp ~0.2 gives a tight clean
#                   attitude; FW needs ~0.6+ or the rate error can never elicit
#                   enough elevator for a 30° pitch step). Mirrors the existing
#                   FW-vs-MR criteria split.
_PARAM_CURVES = {
    # Angle integral limits (rad/s)
    int(_PI.ROLL_ANGLE_Q_INT_LIMIT):(0.005, 0.03),    # default 0.01
    int(_PI.PITCH_ANGLE_Q_INT_LIMIT):(0.005, 0.03),   # default 0.01
    int(_PI.YAW_ANGLE_Q_INT_LIMIT): (0.01, 0.06),     # default 0.03
    # Rate limits (rad/s)
    int(_PI.MAX_ROLL_RATE):       (1.396, 6.283),   # 80-360 deg/s
    int(_PI.MAX_PITCH_RATE):      (1.047, 4.189),   # 60-240 deg/s
    int(_PI.MAX_HEADING_RATE):(0.262, 2.094),   # 15-120 deg/s
    # Altitude (drag-limited vertical plant — CONS must still climb 5m in ~2s:
    # ALT_POS_KP/ALT_ROC_KP/comp-limit were genuinely too weak to do so).
# comp-limit: pinned to the FC max (PARAM_LIMITS[102] = 0.0..0.25), which
     # the generic .af bases now carry (0.25) so the sim actually binds it;
     # at 0.25 CONS the heavy-quad AH rise is ~1.86s (was 2.01 at the 0.2 sim
     # default the suite silently fell back to while the param was absent).
     int(_PI.ALT_POS_KP):          (2.0, 2.4),       # default 2.0
     int(_PI.ALT_POS_KI):          (0.002, 0.005),   # default 0.002
     int(_PI.ALT_THROTTLE_COMP_LIMIT): (0.25, 0.25), # default 0.25, FC max 0.25
    int(_PI.ALT_ROC_KP):          (1.3, 1.6),       # default 1.2
    int(_PI.UNUSED_ALT_VEL_KI):   (0.0008, 0.0015), # default 0.001
    int(_PI.MAX_CLIMB_RATE_MP_S): (1.0, 8.0),   # vertical-profile ascent shaping (m/s)
    # Navigation — shared defaults; per-category sets below override for the
    # param tags present in MR_CURVES / FW_CURVES.
    int(_PI.HORIZON):             (2.0, 5.0),       # default ~3.33
    # Angle limits (rad)
    int(_PI.MAX_PITCH_ANGLE):     (0.349, 0.698),   # 20-40 deg
    int(_PI.MAX_ROLL_ANGLE):      (0.349, 0.698),   # 20-40 deg
}

# MR attitude / nav curve envelope.
MR_CURVES = {
    # Angle gains (Quaternion P, scale=1.0 on GCS)
    int(_PI.ROLL_ANGLE_Q_KP):       (5.0, 9.0),       # default 7
    int(_PI.PITCH_ANGLE_Q_KP):      (5.0, 9.0),       # default 7
    int(_PI.YAW_ANGLE_Q_KP):        (5.0, 10.0),      # default 8
    # Angle integral gains
    int(_PI.ROLL_ANGLE_Q_KI):       (0.05, 0.5),      # default 0.25
    int(_PI.PITCH_ANGLE_Q_KI):      (0.05, 0.5),      # default 0.25
    int(_PI.YAW_ANGLE_Q_KI):        (0.05, 0.5),      # default 0.25
    # Rate proportional gains
    int(_PI.ROLL_RATE_KP):        (0.28, 0.5),     # default 0.25
    int(_PI.PITCH_RATE_KP):       (0.28, 0.5),     # default 0.25
    int(_PI.YAW_RATE_KP):         (0.20, 0.75),    # default 0.25
    # Rate derivative gains
    int(_PI.ROLL_RATE_KD):        (0.008, 0.02),    # default 0.01
    int(_PI.PITCH_RATE_KD):       (0.008, 0.02),    # default 0.01
    int(_PI.YAW_RATE_KD):         (0.005, 0.02),    # default 0.01
    # Navigation (bank-to-turn kinematic plant — needs a high position gain
    # at the 20° CONS bank to cross 9m in ≤2s)
    int(_PI.NAV_POS_KP):          (2.5, 3.0),     # default 2.5
    int(_PI.NAV_POS_KI):          (0.02, 0.03),   # default 0.02
    int(_PI.NAV_VEL_KP):          (1.0, 2.0),     # default 1.0
}

# FW attitude / nav curve envelope (see docblock above _PARAM_CURVES).
FW_CURVES = {
    # Angle gains (Quaternion P, scale=1.0 on GCS)
    # Pitch CONS 11.0: at 60°/s demand the elevator effort saturates against
    # the stiffness equilibrium (M_ctrl == M_stab) before reaching 27°; a P
    # of ~11 crosses it in the 5s window. Roll CONS 3.0 keeps the aileron
    # demand below the roll-rate clamp for the big-authority frames
    # (Spoileron/Elevon peak ~19°=27% OS at any higher gain — gain-insensitive
    # momentum overshoot, so the generation is capped, not the damping).
    int(_PI.ROLL_ANGLE_Q_KP):       (3.0, 4.5),      # default 7
    int(_PI.PITCH_ANGLE_Q_KP):      (11.0, 10.0),    # default 7
    int(_PI.YAW_ANGLE_Q_KP):        (7.0, 11.0),     # default 8
    # Angle integral gains
    int(_PI.ROLL_ANGLE_Q_KI):       (0.05, 0.5),     # default 0.25
    int(_PI.PITCH_ANGLE_Q_KI):      (0.05, 0.5),     # default 0.25
    int(_PI.YAW_ANGLE_Q_KI):        (0.05, 0.5),     # default 0.25
    # Angle integral limits (rad/s) — FW needs real authority here: the tiny
    # shared 0.005 cap cannot push past the pitch stiffness equilibrium.
    int(_PI.ROLL_ANGLE_Q_INT_LIMIT):  (0.15, 0.15),  # rad/s
    int(_PI.PITCH_ANGLE_Q_INT_LIMIT): (0.15, 0.15),  # rad/s
    # Rate proportional gains — FW needs enough gain to elicit meaningful
    # elevator for a 30° pitch step (rate error only reaches ~1 rad/s before
    # the MAX_*_RATE clamp; Kp must turn that into ≥0.5-0.7 effort)
    int(_PI.ROLL_RATE_KP):        (1.2, 1.8),     # default 0.6
    int(_PI.PITCH_RATE_KP):       (0.9, 1.5),     # default 0.6
    int(_PI.YAW_RATE_KP):         (1.0, 1.8),     # default 0.8
    # Rate derivative gains
    int(_PI.ROLL_RATE_KD):        (0.050, 0.12),  # default 0.01
    int(_PI.PITCH_RATE_KD):       (0.010, 0.03),  # default 0.01
    int(_PI.YAW_RATE_KD):         (0.015, 0.04),  # default 0.01
    # Per-category rate limits — the shared CONS clamp (80°/s roll, 60°/s
    # pitch) does not fit the FW generation demand transfer: high-authority
    # frames overshoot if the demand pins the clamp. FW CONS wants a gentler
    # roll demand (~40°/s) yet a pitch generation that can actually climb.
    int(_PI.MAX_ROLL_RATE):       (0.7, 0.9),     # 40-52 deg/s
    int(_PI.MAX_PITCH_RATE):      (1.047, 1.57),  # 60-90 deg/s
    # Navigation (heading-integrator bank-to-turn: high position gain
    # over-drives the heading slew and overshoots the 15% bar)
    int(_PI.NAV_POS_KP):          (1.0, 1.5),     # default 1.0
    int(_PI.NAV_POS_KI):          (0.005, 0.015), # default 0.005
    int(_PI.NAV_VEL_KP):          (0.6, 1.2),     # default 0.6
}

# Merge order: shared table first, then the category table overrides for the
# attitude/nav param tags that legitimately differ MR vs FW.
_CATEGORY_CURVES = {AirframeCat.MR: MR_CURVES, AirframeCat.FW: FW_CURVES}


def apply_slider(raw_params: dict, slider_pct: float,
                 cat: Optional[int] = None) -> dict:
    """Apply character slider to raw params.

    Interpolates between conservative (0%) and aggressive (100%) raw FC values.
    Params not in the curve tables are left unchanged.

    cat: airframe category (AF_CATEGORY value). Selects the per-category
    attitude/nav envelope (MR_CURVES / FW_CURVES) layered over the shared
    _PARAM_CURVES. If None, both category tables merge (MR then FW) so direct
    callers without a category still get the full envelope.
    """
    from protocol_enums import ParamIndex
    curves = dict(_PARAM_CURVES)
    if cat in _CATEGORY_CURVES:
        curves.update(_CATEGORY_CURVES[cat])
    else:
        for cset in _CATEGORY_CURVES.values():
            curves.update(cset)
    result = dict(raw_params)
    for tag_int, (cons, agg) in curves.items():
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
    # Conventional aircraft: long tail arm, mass distributed along span+chord.
    # rf_roll=0.8 (wing mass near fuselage), rf_pitch=0.9 (tail at distance).
    "generic/SkySurfer_Bixler.af": {
        "mass": 1.4, "wingspan": 2.0, "wing_area": 0.463,
        "cruise_speed": 12.0,
        "roll_mass_frac": 0.8, "pitch_mass_frac": 0.9,
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
    # Shadow2: Represents a flat constant-chord plank (cf. the "Reptile") —
    # ~2° physical dihedral, NO sweep.  Template for encoding geometry: every
    # key is optional; omit them to inherit the flat dihedral_coeff (≈3°).
    # Common encodings:
    #   plank 2°:               {sweep_deg: 0, dihedral_deg: 2}
    #   swept 30° / 2°:         {sweep_deg: 30, dihedral_deg: 2}
    #   nulled-tip (Arado):     {sweep_deg:45, dihedral_deg:5,
    #                            anhedral_deg:5, anhedral_start:0.667}
    "generic/Shadow2.af": {
        "mass": 0.5, "wingspan": 0.82, "wing_area": 0.1517,
        "cruise_speed": 13.0,
        "sweep_deg": 0.0, "dihedral_deg": 2.0,
        "roll_mass_frac": 0.4, "pitch_mass_frac": 0.3,
        "aileron_area": 0.0044, "aileron_arm": 0.25,
        "elevator_area": 0.0044, "elevator_arm": 0.25,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 30.0, "elevator_max_deg": 12.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.01,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # Shadow: 1000g AUW, 1800mm span, swept plank (30° sweep, ~2° dihedral).
    # REF4 tuning baseline (overwrote the 2026-08-28 0.5kg/820mm descriptor).
    # AUTHORITY RE-BASELINE 2026-09-10 (wiki/Session_Report_ShadowAuthorityTuning_Sep10.md):
    #   CL_D_AIL 0.025 -> 0.18  — external aileron/elevon power band 0.12–0.32
    #     per rad (flight-ID: Grillo & Montano 0.173, Tecnam flight test ~0.21);
    #     the old 0.025 was 5–13× below the whole measured band (assumed, never
    #     calibrated).  0.18 = defensible mid-band (report-recommended 0.15–0.20).
    #   CM_D_ELE 0.6 KEPT — external elevator Cmδe band 0.33–0.56, our 0.6 is
    #     already in/near band; pitch authority was never the deficit.
    #   INERTIA: concentrated-mass model — plank with mass near CG.
    #     rf_roll=0.4, rf_pitch=0.3 (roll ~6×, pitch ~30× higher angular accel
    #     than uniform thin-rod).  Surfaces 40/16° (Greg: "40 deg roll, 16 pitch
    #     — planks typically have 40% as much control deflection on pitch as on roll
    #     because of the short moment arm").
    "generic/Shadow.af": {
        "mass": 1.0, "wingspan": 1.8, "wing_area": 0.45,
        "cruise_speed": 8.4,
        "sweep_deg": 30.0, "dihedral_deg": 2.0,
        "roll_mass_frac": 0.4, "pitch_mass_frac": 0.3,
        "aileron_area": 0.0132, "aileron_arm": 0.55,
        "elevator_area": 0.0132, "elevator_arm": 0.55,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 40.0, "elevator_max_deg": 16.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.18, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -0.04, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.01,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.18, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # Phoenix: 1000g AUW, 1800mm span, 250mm chord, rudder+elevator (no ailerons)
    # CL_α≈5.4 → Cm_α=-0.27
    "original/Phoenix.af": {
        "mass": 1.0, "wingspan": 1.8, "wing_area": 0.45,
        "cruise_speed": 13.0,
        "roll_mass_frac": 0.85, "pitch_mass_frac": 0.95,
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
    # Conventional tail; mixed spar/inner mass → rf 0.75-0.85.
    "generic/SmallSpoileron.af": {
        "mass": 1.2, "wingspan": 1.8, "wing_area": 0.504,
        "cruise_speed": 14.0,
        "roll_mass_frac": 0.75, "pitch_mass_frac": 0.85,
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
    # Plank: mass near CG → rf 0.4.
    "generic/Dragon.af": {
        "mass": 0.8, "wingspan": 1.2, "wing_area": 0.30,
        "cruise_speed": 13.0,
        "roll_mass_frac": 0.4, "pitch_mass_frac": 0.3,
        "aileron_area": 0.0055, "aileron_arm": 0.30,
        "elevator_area": 0.0055, "elevator_arm": 0.30,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 30.0, "elevator_max_deg": 12.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.01,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # Radian: 850g AUW, 2000mm span, 178mm chord, area 0.356 m²
    # CL_α≈5.4 → Cm_α=-0.27
    # Conventional glider: massive tail + long moment arm → rf 0.9.
    "generic/Radian.af": {
        "mass": 0.85, "wingspan": 2.0, "wing_area": 0.356,
        "cruise_speed": 9.0,
        "roll_mass_frac": 0.9, "pitch_mass_frac": 0.95,
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
    # CL_α≈4.9 → Cm_α=-0.245; geometry per user specs (2026-08-28):
    # 45° sweep, +5° dihedral inboard 2/3 span, −5° anhedral tip panels.
    # Plank: mass near CG → rf 0.4; 40/16 surface throw (plank ratio).
    "user/Arado_555.af": {
        "mass": 1.3, "wingspan": 1.2, "wing_area": 0.42,
        "cruise_speed": 13.0,
        "sweep_deg": 45.0, "dihedral_deg": 5.0,
        "anhedral_deg": 5.0, "anhedral_start": 0.667,
        "roll_mass_frac": 0.4, "pitch_mass_frac": 0.3,
        "aileron_area": 0.006, "aileron_arm": 0.30,
        "elevator_area": 0.006, "elevator_arm": 0.30,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 30.0, "elevator_max_deg": 12.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.01,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.03, "yaw_damp_lin": 0.012,
        "servo_tau": 0.08,
    },
    # Horten: 550g AUW, 1200mm span, 225mm chord, area 0.27 m² (flying wing)
    # CL_α≈4.9 → Cm_α=-0.245; 40° sweep per user specs (2026-08-28).
    # Plank: mass near CG → rf 0.4; 40/16 surface throw.
    "user/Horten.af": {
        "mass": 0.55, "wingspan": 1.2, "wing_area": 0.27,
        "cruise_speed": 13.0,
        "sweep_deg": 40.0, "dihedral_deg": 2.0,
        "roll_mass_frac": 0.4, "pitch_mass_frac": 0.3,
        "aileron_area": 0.0045, "aileron_arm": 0.28,
        "elevator_area": 0.0045, "elevator_arm": 0.28,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 30.0, "elevator_max_deg": 12.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.01,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # generic/Elevon.af: 1000g, 1400mm span, 200mm chord, BR2406S 5x4.3x3 3S
    # CL_α≈4.9 → Cm_α=-0.245
    # Plank: mass near CG → rf 0.4; 40/16 surface throw.
    "generic/Elevon.af": {
        "mass": 1.0, "wingspan": 1.4, "wing_area": 0.28,
        "cruise_speed": 10.7,
        "roll_mass_frac": 0.4, "pitch_mass_frac": 0.3,
        "aileron_area": 0.0042, "aileron_arm": 0.35,
        "elevator_area": 0.0042, "elevator_arm": 0.35,
        "rudder_area": 0.0, "rudder_arm": 0.0,
        "aileron_max_deg": 30.0, "elevator_max_deg": 12.0, "rudder_max_deg": 0.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.0,
        "pitch_damp": -8.0, "yaw_damp": -0.08,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.245, "yaw_stability": 0.03,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # generic/Delta.af: 1000g, 1000mm span, 250mm chord, BR2406S 5x4.3x3 3S
    # CL_α≈4.0 → Cm_α=-0.20
    # Plank: mass near CG → rf 0.45; 40/16 surface throw.
    "generic/Delta.af": {
        "mass": 1.0, "wingspan": 1.0, "wing_area": 0.25,
        "cruise_speed": 11.3,
        "roll_mass_frac": 0.45, "pitch_mass_frac": 0.35,
        "aileron_area": 0.00375, "aileron_arm": 0.25,
        "elevator_area": 0.00375, "elevator_arm": 0.25,
        "rudder_area": 0.003, "rudder_arm": 0.25,
        "aileron_max_deg": 30.0, "elevator_max_deg": 12.0, "rudder_max_deg": 25.0,
        "CL_D_AIL": 0.025, "CM_D_ELE": 0.6, "CN_D_RUD": 0.10,
        "pitch_damp": -8.0, "yaw_damp": -0.12,
        "adverse_yaw": 0.05, "dihedral_coeff": 0.05,
        "pitch_stability": -0.20, "yaw_stability": 0.02,
        "roll_damp_lin": 0.02, "pitch_damp_lin": 0.02, "yaw_damp_lin": 0.01,
        "servo_tau": 0.08,
    },
    # generic/Spoileron.af: 1200g, 1800mm span, 280mm chord, X2216 10x6" folder 3S
    # CL_α≈5.1 → Cm_α=-0.255
    # Conventional tail: rf 0.75-0.85.
    "generic/Spoileron.af": {
        "mass": 1.2, "wingspan": 1.8, "wing_area": 0.504,
        "cruise_speed": 8.7,
        "roll_mass_frac": 0.75, "pitch_mass_frac": 0.85,
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
    # Conventional tail: rf 0.8.
    "generic/RudderElevator.af": {
        "mass": 1.1, "wingspan": 1.8, "wing_area": 0.45,
        "cruise_speed": 8.8,
        "roll_mass_frac": 0.8, "pitch_mass_frac": 0.8,
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
        merged = dict(desc)
        # Apply the control-authority calibration scale to the torque coeffs
        merged["CL_D_AIL"] = merged.get("CL_D_AIL", 0.0) * FW_AUTHORITY_SCALE
        merged["CM_D_ELE"] = merged.get("CM_D_ELE", 0.0) * FW_AUTHORITY_SCALE
        merged["CN_D_RUD"] = merged.get("CN_D_RUD", 0.0) * FW_AUTHORITY_SCALE
        return merged
    merged = dict(desc)
    for pk, dk in PHYS_TO_DESCRIPTOR.items():
        if pk in phys:
            merged[dk] = phys[pk]
    # Apply the control-authority calibration scale to the torque coeffs
    merged["CL_D_AIL"] = merged.get("CL_D_AIL", 0.0) * FW_AUTHORITY_SCALE
    merged["CM_D_ELE"] = merged.get("CM_D_ELE", 0.0) * FW_AUTHORITY_SCALE
    merged["CN_D_RUD"] = merged.get("CN_D_RUD", 0.0) * FW_AUTHORITY_SCALE
    return merged

# Derived inertia and max-rate tuples (roll_pitch, yaw) — computed from physical params
# Component build-up inertia fractions (AUW-based, literature: wing 50-64% of
# sailplane empty mass — Hoff, Tech.Soaring; empennage ≈ 6%)
FW_MASS_FRAC_WING   = 0.50
FW_MASS_FRAC_TAIL   = 0.06
FW_LEVER_TAIL       = 0.85   # aero centre vs mass-centre offset on the tail arm

def _compute_fw_inertia(af: dict) -> Tuple[float, float]:
    """Component build-up (parallel-axis) inertia for fixed-wing aircraft.

    Literature: Raymer Class-II component/geometry build-up; NASA "representative
    geometric figures" method (Rein Inge Hoff, "Estimating Sailplane Mass
    Properties", Technical Soaring) — component masses at their CG offsets summed
    via the parallel-axis (Huygens–Steiner) theorem.  Component fractions are
    AUW-based: wing ≈ 0.50 (sailplane data: 50–64% of empty mass), empennage ≈ 0.06,
    fuselage+pyload lumped on a slender rod of length L.

    Roll (about the nose axis): spanwise mass distribution of the wing plate:
        I_roll = m_wing·b²/12         (fuselage sits on the axis — no lever)
    Pitch (about the span axis): wing chord + tail-arm lever + fuselage rod:
        I_pitch = m_wing·c²/12 + m_tail·(k·l_tail)² + m_fuse·L²/12
        plank:   c is tiny, no boom (l_tail≈0, L≈2c) → ultra pitch-sensitive
        conventional: long boom + tail point mass at reality arm
    Yaw = I_roll + I_pitch (thin-body identity).
    rf_roll/rf_pitch kept for backward compatibility (ignored — geometry now
    derives pitch from the stored elevator/rudder arms).
    """
    m = af["mass"]
    b = af["wingspan"]
    chord = af["wing_area"] / af["wingspan"]

    plank = af.get("rudder_area", 0) <= 0
    m_wing  = FW_MASS_FRAC_WING * m
    m_tail  = FW_MASS_FRAC_TAIL * m
    m_fuse  = m - m_wing - m_tail

    I_roll = m_wing * b * b / 12.0

    if plank:
        l_tail = 0.0                # tail surfaces live on the wing — no boom
        L = 2.0 * chord            # short pod
    else:
        l_tail = max(af.get("elevator_arm", 0.0), af.get("rudder_arm", 0.0))
        L = 2.0 * l_tail           # boom extends ~arm behind CG, nose ahead

    I_pitch = (m_wing * chord * chord / 12.0   # wing chord
               + m_tail * (FW_LEVER_TAIL * l_tail) ** 2   # empennage point mass
               + m_fuse * L * L / 12.0)        # fuselage rod
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

# Model cruise throttle per FW model (mirrors emu.c EmuModels[].CruiseThrottle
# for MODEL_ELEVON..MODEL_VTOL — the value InitEmulation seeds into
# Config.CruiseThrottleFF on every emu boot). The AH sim uses this as the
# DoMotors-mirror baseline instead of the legacy static UNUSED_20 (=0.5) seed,
# so climb/descent authority reflects the airframe's real cruise setting.
# 2026-09-24: seeded sims previously assumed 0.5 for every frame (the .af
# legacy default) while the emu/flight baseline is per-model.
FW_CRUISE_THR = [
    0.45,   # 0 ElevonAF
    0.35,   # 1 AileronAF (SkySurfer)
    0.50,   # 2 DeltaAF
    0.35,   # 3 AileronSpoilerFlapsAF
    0.40,   # 4 AileronVTailAF
    0.35,   # 5 RudderElevatorAF
    0.50,   # 6 VTOLAF
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
# FW step sizes — pitch already max sustainable climb (~15 deg), yaw smaller for coordinated turn.
# 30 deg sustained pitch step dropped 2026-09-12: not a realistic FW demand (climb attitude is small,
# and pitch-up in a FW comes with throttle increase that boosts elevator authority via propwash —
# the clamped fixed-qbar model here does not capture that slipstream gain).
FW_TEST_STEPS = {"Roll": 15.0, "Pitch": 15.0, "Yaw": 15.0}
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
    _DAMP_C = {"Roll": 0.015, "Pitch": 0.03, "Yaw": 0.05}
    damp_c = _DAMP_C.get(axis, 0.02)

    # Sub-step the rate integration: explicit Euler is unstable at CONTROL_DT
    # when inertia_r_axis is large (tiny m*a² micro frames expose a rate mode
    # far faster than the controller step). Keep the motor lag + rate ODE at
    # an inertia-adaptive dt that honours the Euler bound for the quadratic
    # damping, while the 1 kHz control loop above is unchanged.
    stiff = 2.0 * damp_c * 2.0 * inertia_r_axis
    sub = max(1, int(dT * stiff / 0.5) + 1)
    dts = dT / sub
    alphas = dts / (EM_MOTOR_TAU + dts)
    effort = lag
    for _ in range(sub):
        effort += alphas * (out - effort)
        torque = kT * effort
        damping = damp_c * sgn(rate) * rate * rate
        rate += (torque - damping) * inertia_r_axis * dts
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

    # Moment of inertia — component build-up (see _compute_fw_inertia)
    I_roll, I_pitch = _compute_fw_inertia(af_params)
    I_yaw   = I_roll + I_pitch

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

    # FW angle loop is pure-P by design — control.c DoAngleControl gates the
    # quaternion integral accumulation on pAFTypeCategory != eCatFw (=:671), so
    # a FW airframe NEVER runs its angle-I no matter what Ki the .af stores.
    # Mirror that gate here so sim results reflect flight (derived FW Ki is
    # dormant: shipped in the file, never applied).
    if is_fw:
        pi.Ki = 0.0

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

    angles, times, intes, angles_full = [], [], [], []

    max_int = max_rate = peak_angle = 0.0

    pi.Max = max_angle_rad
    stick = step_rad / max_angle_rad if max_angle_rad > 0 else 0.0

    for i in range(n):
        t = i * dT
        s = stick if i >= n_warmup else 0.0

        desired_rate = run_angle_loop(pi, angle, s, 0.0, dT)
        pid.Desired = clamp(desired_rate, -pid.Max, pid.Max)
        out = run_rate_pd(pid, rate, dT)

        d_torque = gust.torque(t) if gust else 0.0

        if is_fw:
            if fw_model_idx < 0:
                raise ValueError(
                    f"{axis_name}: FW airframe routed without a FW model index "
                    f"(FW_MODEL_IDX missing for this AF type). Focus-1 class boundary: "
                    f"a FW airframe must NEVER run the MR plant — refuse rather than "
                    f"silently cross-class.")
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
                rate += d_torque * MR_INERTIA_R[ai] * dT

        if i % decimate == 0:
            angles.append(angle)
            times.append(t)
            intes.append(pi.IntE)

        max_int = max(max_int, abs(pi.IntE))
        max_rate = max(max_rate, abs(rate))
        peak_angle = max(peak_angle, abs(angle))
        angles_full.append(angle)

    final_angle = angles[-1] if angles else 0.0
    setpoint = step_rad
    ss_error = abs(final_angle - setpoint)

    rise_time, overshoot, settling = step_metrics_window(times, angles, setpoint)
    zero_crossings = count_zero_crossings(angles_full, n_warmup * 3 + 1)

    return StepMetrics(
        rise_time_s=round(rise_time, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settling, 4),
        steady_state_error=round(ss_error, 6),
        max_integrator=max_int,
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
    rates = []
    times = []
    rates_full = []
    times_full = []

    for i in range(n):
        t = i * dT
        pid.Desired = clamp(desired_rate, -pid.Max, pid.Max)
        out = run_rate_pd(pid, rate, dT)

        d_torque = gust.torque(t)

        if is_fw:
            if fw_model_idx < 0:
                raise ValueError(
                    f"{axis_name}: FW disturbance routed without a FW model index "
                    f"(FW_MODEL_IDX missing for this AF type). Focus-1 class boundary: "
                    f"a FW airframe must NEVER run the MR plant — refuse rather than "
                    f"silently cross-class.")
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

        rates_full.append(rate)
        times_full.append(t)

    peak_dev, settle_rel, total_ie, steady = disturbance_metrics(
        times_full, rates_full, gust.start_s, gust.duration_s, dT)

    return DisturbanceMetrics(
        peak_deviation_rad=round(peak_dev, 6),
        settling_time_s=round(settle_rel, 4),
        integrated_error=round(total_ie, 6),
        steady_rate_error=round(steady, 6),
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
                         af_filename: str = None,
                         descend: bool = False) -> StepMetrics:
    """Simulate MR altitude hold step response.

    step_m: altitude step in meters (default 5m)
    Physics: mass-thrust model with quadratic drag.
    Uses per-airframe PHYS_ params when available.
    descend=True runs the reverse leg: hold at +step_m, command 0 m, and
    score the descent-progress series (step_m - alt), so the same criteria
    apply to "how fast can it come down". Exercises the comp's NEGATIVE
    throttle authority bound (DesiredThrottle can't go below 0 — a real
    rotor spins down, it can't push) — the too-fast-descent flip side of the
    climb ceiling.
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

    alt = step_m if descend else 0.0
    vel = 0.0
    int_e_pos = 0.0
    int_e_vel = 0.0
    n = int(SIM_TIME / dT)

    peak_alt = 0.0
    max_int = 0.0
    atimes = []
    alts = []
    target = 0.0 if descend else step_m

    for i in range(n):
        t = i * dT
        err = target - alt

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
        # Descent tracks progress (step_m - alt) so the same 0->step_m
        # series drives the shared rise/settle metrics.
        track = step_m - alt if descend else alt

        max_int = max(max_int, abs(int_e_pos), abs(int_e_vel))
        peak_alt = max(peak_alt, track)
        atimes.append(t)
        alts.append(track)

    final_err = abs(alt - target)
    rise_time, overshoot, settling = step_metrics_persist(atimes, alts, step_m, dt=dT)

    return StepMetrics(
        axis_name="Altitude (MR)" + (" descent" if descend else ""),
        rise_time_s=round(rise_time, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settling, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=max_int,
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
                         dT: float = CONTROL_DT, descend: bool = False,
                         cruise_thr: float = None) -> StepMetrics:
    """Simulate FW altitude hold step response.

    step_m: altitude step in meters (default 5m)
    Physics: pitch-to-climb model with airspeed coupling.
    FW climbs by pitching up (controlled by AH), which bleeds airspeed.
    Much slower than MR — time constant ~2-3s.
    descend=True: hold at +step_m, command 0 m, score descent-progress
    (step_m - alt). FW descends by throttling back and letting the wing
    glide down — the descent-rate authority is the THROTTLE floor (negative
    compensation bound −min(thr_lim, cruise_thr)), not the +climb ceiling.
    cruise_thr: model cruise throttle baseline (FW_CRUISE_THR[fw_mid]),
    mirroring the emu's Config.CruiseThrottleFF seed. None → legacy static
    UNUSED_20 (=0.5) fallback.
    """
    kp = float(params.get("ALT_POS_KP", 0.0))
    ki = float(params.get("ALT_POS_KI", 0.0))
    thr_lim = float(params.get("ALT_THROTTLE_COMP_LIMIT", 0.2))
    roc_kp = float(params.get("ALT_ROC_KP", 0.05))
    roc_ki = float(params.get("UNUSED_ALT_VEL_KI", 0.001))
    roc_max = 5.0

    # DoMotors mirror: TotalThrottle = DesiredThrottle + AltHoldThrComp
    #   + pFWPitchThrottleFFFrac*Abs(Pl), capped at pFWClimbThrottleFrac.
    if cruise_thr is None:
        cruise_thr = float(params.get("UNUSED_20", 0.5))
    climb_cap = float(params.get("FW_CLIMB_THROTTLE", 0.7))
    ff_pt = float(params.get("FW_PITCH_THROTTLE_FF", 0.0))
    climb_margin = max(0.0, climb_cap - cruise_thr)

    alt = step_m if descend else 0.0
    vel_vert = 0.0
    int_e_pos = 0.0
    int_e_vel = 0.0
    n = int(SIM_TIME / dT)

    peak_alt = 0.0
    max_int = 0.0
    atimes = []
    alts = []
    target = 0.0 if descend else step_m

    # FW climb dynamics: Position PI -> ROC setpoint -> Velocity PI -> climb
    # Actual climb rate follows with time constant (servo + aero lag)
    climb_tau = 2.0  # seconds — much slower than MR
    V = cruise_v

    for i in range(n):
        t = i * dT
        err = target - alt

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
        # Descent tracks progress (step_m - alt); rise == commanded -x m leg.
        track = step_m - alt if descend else alt

        max_int = max(max_int, abs(int_e_pos), abs(int_e_vel))
        peak_alt = max(peak_alt, track)
        atimes.append(t)
        alts.append(track)

    final_err = abs(alt - target)
    rise_time, overshoot, settling = step_metrics_persist(atimes, alts, step_m, dt=dT)

    return StepMetrics(
        axis_name="Altitude (FW)" + (" descent" if descend else ""),
        rise_time_s=round(rise_time, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settling, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=max_int,
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
    ntimes = []
    npos = []

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
        ntimes.append(t)
        npos.append(abs(pos))

    final_err = abs(pos - step_rad)
    rise_time, overshoot, settling = step_metrics_persist(ntimes, npos, abs(step_rad), dt=dT)

    return StepMetrics(
        axis_name="Navigation (MR)",
        rise_time_s=round(rise_time, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settling, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=max_int,
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
    # FW bank = ROLL angle (bank-to-turn); pitch max is climb attitude, NOT the
    # bank limit. FC clamps FW roll setpoint at A[eRoll].P.Max (control.c).
    max_bank = float(params.get("MAX_ROLL_ANGLE", 0.7854))

    pos = 0.0  # cross-track error in meters
    vel = 0.0  # lateral velocity
    heading = 0.0
    int_e = 0.0
    n = int(SIM_TIME / dT)

    peak_pos = 0.0
    max_int = 0.0
    ntimes = []
    npos = []

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
        ntimes.append(t)
        npos.append(abs(pos))

    final_err = abs(pos - step_m)
    rise_time, overshoot, settling = step_metrics_persist(ntimes, npos, abs(step_m), dt=dT)

    return StepMetrics(
        axis_name="Navigation (FW)",
        rise_time_s=round(rise_time, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settling, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=max_int,
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
    # FW bank = ROLL angle (bank-to-turn); see note in simulate_nav_fw_cross_track.
    max_angle = float(params.get("MAX_ROLL_ANGLE", 0.7854))  # max bank for nav

    step_rad = step_deg * DEG_TO_RAD

    heading = 0.0
    cross_track = 0.0  # meters
    vel = 0.0
    int_e = 0.0
    n = int(SIM_TIME / dT)

    peak_heading = 0.0
    max_int = 0.0
    ntimes = []
    nheading = []

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
        ntimes.append(t)
        nheading.append(abs(heading))

    final_err = abs(heading - step_rad)
    rise_time, overshoot, settling = step_metrics_persist(ntimes, nheading, abs(step_rad), dt=dT)

    return StepMetrics(
        axis_name="Navigation",
        rise_time_s=round(rise_time, 4),
        overshoot_pct=round(overshoot, 1),
        settling_time_s=round(settling, 4),
        steady_state_error=round(final_err, 4),
        max_integrator=max_int,
        peak_angle=round(peak_heading * RAD_TO_DEG, 4),
        final_angle=round(heading * RAD_TO_DEG, 4),
        setpoint=step_deg,
    )


def simulate_axis_coupled(af_filename: str, step_axis: str = "Roll",
                          dT: float = CONTROL_DT,
                          gusts: Optional[Dict[str, Gust]] = None,
                          params_override: Optional[dict] = None,
                          step_rads: Optional[Dict[str, float]] = None,
                          demand_frac: Optional[Callable[[float], float]] = None,
                          return_series: bool = False,
                          sim_time_s: Optional[float] = None) -> Dict[str, StepMetrics]:
    """Coupled 3-axis simulation with cross-coupling (dihedral, adverse yaw).

    Runs all three axes simultaneously so that:
      - Yaw rate → dihedral → roll torque
      - Aileron deflection → adverse yaw → yaw torque

    step_axis: which axis receives the step command (others hold zero)
    params_override: if provided, use these params instead of re-loading from file
    step_rads: if provided, use these step sizes (radians) instead of TEST_STEPS
    demand_frac: if provided, the step demand for step_axis is this callable of t
        (seconds) returning a stick fraction (-1..1) instead of the instant step.
        Default None = instant step after warmup (identification probe). Used by
        the finite-rise and sawtooth process tests (Greg 2026-09-13 foci 5/6).
    return_series: if True, also return (times, decimated angles of step_axis).
        sim_time_s: override the 8 s wall for sawtooth runs that need full periods.
    """
    af_params = get_fw_descriptor(af_filename)
    rho = AIR_DENSITY
    V = af_params["cruise_speed"]
    qbar = 0.5 * rho * V * V
    S = af_params["wing_area"]
    b = af_params["wingspan"]
    chord = S / b
    mass = af_params["mass"]

    # Moment of inertia — component build-up (see _compute_fw_inertia)
    I_roll, I_pitch = _compute_fw_inertia(af_params)
    I_yaw   = I_roll + I_pitch

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
    is_fw = (AF_CATEGORY.get(af_type) == AirframeCat.FW)

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
        # FW angle loop is pure-P by design — control.c DoAngleControl gates the
        # quaternion integral accumulation on pAFTypeCategory != eCatFw (=:671),
        # so FW never runs its angle-I (derived FW Ki is dormant in the file).
        if is_fw:
            aki = 0.0
        pis[name]  = PIStruct(Kp=akp, Ki=aki, IntLim=ail, Max=ma)
        pids[name] = PIDStruct(Kp=rkp, Kd=rkd, Max=mr)

    n = int((sim_time_s if sim_time_s is not None else SIM_TIME) / dT)
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
            if name == step_axis and demand_frac is not None:
                s = demand_frac(t)
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
        rise_time, overshoot, settling = step_metrics_window(all_times, a_list, setpoint)

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

    if return_series:
        return {name: m for name, m in metrics.items()}, all_times, all_angles[step_axis]
    return metrics


def _finite_rise_demand(rise_s: float, t_step: float) -> Callable[[float], float]:
    """Return a demand_frac(t) ramping 0 → 1 over rise_s after t_step (focus 5).

    A smooth first-order approach would need a time constant; the study directive
    is a finite rise/decay "fractions of a second to seconds" mirroring real stick
    motion, so a linear ramp over rise_s is used (hard step when rise_s == 0).
    """
    if rise_s <= 0:
        return lambda t: (1.0 if t >= t_step else 0.0)
    def frac(t):
        return 0.0 if t < t_step else min(1.0, (t - t_step) / rise_s)
    return frac


def _sawtooth_demand(amp: float, period_s: float, t_start: float) -> Callable[[float], float]:
    """Return a demand_frac(t): triangular wave, amplitude `amp` (fraction ±amp),
    period `period_s`, starting at 0 at t_start and rising (focus 6).

    Period and amplitude are chosen by the caller to mimic human stick inputs or
    WP-navigation command shapes (per-airframe, see run_process_steps).
    """
    if amp <= 0 or period_s <= 0:
        return lambda t: 0.0
    def frac(t):
        if t < t_start:
            return 0.0
        tt = (t - t_start) % period_s
        q = tt / period_s * 4.0
        if q < 1.0:
            v = q
        elif q < 3.0:
            v = 2.0 - q
        else:
            v = q - 4.0
        return amp * v
    return frac


@dataclass
class TrackMetrics:
    """Finite-rise / sawtooth tracking outcome."""
    axis_name: str = ""
    demand_kind: str = ""        # "step" | "finite-rise" | "sawtooth"
    ndemand_ms: int = 0          # number of demand samples in the tracking window
    rms_err_deg: float = 0.0
    peak_err_deg: float = 0.0
    lag_s: float = 0.0
    reached_90_pct: bool = False  # finite-rise: demand tracked to 90% of target


def _tracking_metrics(axis_name: str, kind: str,
                      times: List[float], angle_deg: List[float],
                      demand_deg: List[float],
                      t_start: float, t_end: float) -> TrackMetrics:
    """Error/lag of angle-deg versus demand-deg over the active [t_start, t_end] window."""
    t_start = max(t_start, times[0] if times else 0.0)
    t_end = min(t_end, times[-1] if times else t_start)
    errs, n = [], 0
    reached = False
    win_a, win_d = [], []
    for t, a, d in zip(times, angle_deg, demand_deg):
        if t < t_start:
            continue
        if t >= t_end:
            break
        n += 1
        errs.append(a - d)
        if d != 0.0 and abs(a) >= 0.9 * abs(d):
            reached = True
        win_a.append(a)
        win_d.append(d)
    if not errs:
        return TrackMetrics(axis_name=axis_name, demand_kind=kind)
    peak = max((abs(e) for e in errs), default=0.0)
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    # lag = time shift of angle vs demand that maximises correlation (angle lags
    # demand → angle is a delayed copy → best_k negative → report +best_k·step).
    lag = 0.0
    if len(win_a) > 10:
        step = (times[1] - times[0]) if len(times) > 1 else 0.01
        lim = int(1.0 / step) if step > 0 else 0
        def corr(k):
            lo, hi = max(0, -k), min(len(win_a), len(win_a) - k)
            if hi - lo < 4:
                return -1e18
            return sum(x * y for x, y in zip(win_a[lo:hi], win_d[lo + k:hi + k]))
        best_k = max(range(-lim, lim + 1), key=corr)
        lag = round(-best_k * step, 3)
    return TrackMetrics(
        axis_name=axis_name,
        demand_kind=kind,
        ndemand_ms=n,
        rms_err_deg=round(rms, 3),
        peak_err_deg=round(peak, 3),
        lag_s=lag,
        reached_90_pct=reached,
    )


def run_process_steps(af_filename: str,
                      finite_rise_ss: Sequence[float] = (0.5, 1.0, 2.0),
                      sawtooth_amp_deg: float = 3.0,
                      sawtooth_period_s: float = 3.0,
                      wp_amp_deg: float = 10.0,
                      wp_period_s: float = 10.0,
                      t_run_s: float = 7.5) -> Tuple[bool, List[str]]:
    """Process-step battery (foci 5 & 6, Greg 2026-09-13): realish demands.

    finite_rise_ss: rise times to step the same FW_TEST_STEPS demand (focus 5)
    sawtooth_*:     human-stick-like triangular tracking demand (focus 6a)
    wp_*:           WP-navigation-like triangular tracking demand (focus 6b)

    All demand-command shapes are applied to the SAME coupled 3-axis plant used
    by the identification battery (run_tests_for_af) — no separate plant, no
    shared-body across MC/FW classes (FW-only here). Generic-frames only.
    """
    lines = []
    ok = True
    axes = ["Roll", "Pitch", "Yaw"]
    step_deg = FW_TEST_STEPS
    yaw_sl = is_yaw_structurally_limited(af_filename)
    roll_sl = is_roll_structurally_limited(af_filename)
    struct_skip = {"Roll": roll_sl, "Pitch": False, "Yaw": yaw_sl}

    # Sawtooth runs need >= 2 full periods of a periodic command for a clean
    # steady-state lag estimate: extend the sim wall beyond the 8 s identification
    # default. The finite-rise step runs keep the identification 8 s wall.
    saw_wall_s = min(2.0 * max(sawtooth_period_s, wp_period_s) + 1.0, 60.0)

    lines.append(f"\n{'='*60}")
    lines.append(f"{B}Process-step runs (foci 5/6) — {af_filename}{N}")
    lines.append(f"  Finite-rise step rises: {finite_rise_ss}")
    lines.append(f"  Sawtooth stick:  amp {sawtooth_amp_deg}°  period {sawtooth_period_s}s")
    lines.append(f"  Sawtooth WP-nav: amp {wp_amp_deg}°  period {wp_period_s}s")
    lines.append(f"  plant = coupled FW aero (same as ident battery)   saw sim wall {saw_wall_s:.0f}s")
    lines.append(f"{'='*60}")

    for ax in axes:
        if struct_skip[ax]:
            lines.append(f"\n{B}── {ax} ──{N}  {Y}structural skip (no scored demand){N}")
            continue
        t_start = 0.5
        angles = step_deg[ax] * DEG_TO_RAD
        lines.append(f"\n{B}── {ax} @ {step_deg[ax]:.0f}° ──{N}")

        # Focus 5: finite-rise step (vs the ident battery's sharp step)
        for rise in finite_rise_ss:
            df = _finite_rise_demand(rise, t_start)
            m, ts, an = simulate_axis_coupled(
                af_filename, step_axis=ax, step_rads={a: step_deg[a] * DEG_TO_RAD for a in axes},
                demand_frac=df, return_series=True)
            dem = [df(t) * angles * RAD_TO_DEG for t in ts]
            ag = [a * RAD_TO_DEG for a in an]
            tm = _tracking_metrics(ax, f"finite-rise {rise:.1f}s", ts, ag, dem, t_start + rise, t_run_s)
            r90 = "yes" if tm.reached_90_pct else "no"
            lines.append(f"  rise {rise:.1f}s: 90% follower={r90:>3}  rms err {tm.rms_err_deg:5.2f}°  "
                         f"peak err {tm.peak_err_deg:5.2f}°  lag {tm.lag_s:4.2f}s")
            if rise == finite_rise_ss[0]:
                crit = FW_CRITERIA[ax]
                real_m = m[ax]
                lines.append(f"    step metrics on finite-rise demand: rise {real_m.rise_time_s}s "
                             f"settle {real_m.settling_time_s}s fe {real_m.steady_state_error*RAD_TO_DEG:.2f}°")
                if real_m.settling_time_s > crit.max_settling_time_s or \
                   real_m.steady_state_error * RAD_TO_DEG > 2 * crit.max_ss_error_deg:
                    ok = False
                    lines.append(f"    {R}! settle/fe out of identification bounds on finite-rise demand{N}")
        # Focus 6a: human-stick-like sawtooth (>= 2 full periods; analyse the last)
        df = _sawtooth_demand(sawtooth_amp_deg / step_deg[ax] if step_deg[ax] else 0.0,
                              sawtooth_period_s, t_start)
        _, ts, an = simulate_axis_coupled(
            af_filename, step_axis=ax, step_rads={a: step_deg[a] * DEG_TO_RAD for a in axes},
            demand_frac=df, return_series=True, sim_time_s=saw_wall_s)
        dem = [df(t) * angles * RAD_TO_DEG for t in ts]
        ag = [a * RAD_TO_DEG for a in an]
        tm = _tracking_metrics(ax, "saw(stick)", ts, ag, dem,
                               saw_wall_s - sawtooth_period_s - t_start, saw_wall_s - t_start)
        lag_tol = 0.40 * sawtooth_period_s
        sfail = tm.lag_s > lag_tol
        lines.append(f"  sawstick ±{sawtooth_amp_deg}° / {sawtooth_period_s}s: "
                     f"rms err {tm.rms_err_deg:5.2f}°  peak {tm.peak_err_deg:5.2f}°  lag {tm.lag_s:4.2f}s"
                     + (f"  {R}! lag > {lag_tol:.2f}s (40% of period, provisional){N}" if sfail else ""))
        if sfail:
            ok = False
        # Focus 6b: WP-nav-like sawtooth
        df = _sawtooth_demand(wp_amp_deg / step_deg[ax] if step_deg[ax] else 0.0,
                              wp_period_s, t_start)
        _, ts, an = simulate_axis_coupled(
            af_filename, step_axis=ax, step_rads={a: step_deg[a] * DEG_TO_RAD for a in axes},
            demand_frac=df, return_series=True, sim_time_s=saw_wall_s)
        dem = [df(t) * angles * RAD_TO_DEG for t in ts]
        ag = [a * RAD_TO_DEG for a in an]
        tm = _tracking_metrics(ax, "saw(wp)", ts, ag, dem,
                               saw_wall_s - wp_period_s - t_start, saw_wall_s - t_start)
        wp_tol = 0.40 * wp_period_s
        # The corner-following peak error is slew×response-transient, not steady
        # lag — the phase-lag gate is the defensible criterion (provisional).
        wfail = tm.lag_s > wp_tol
        lines.append(f"  saw-WP  ±{wp_amp_deg}° / {wp_period_s}s: "
                     f"rms err {tm.rms_err_deg:5.2f}°  peak {tm.peak_err_deg:5.2f}°  lag {tm.lag_s:4.2f}s"
                     + (f"  {R}! lag > {wp_tol:.2f}s (40% of period, provisional){N}" if wfail else ""))
        if wfail:
            ok = False

    return ok, lines


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


def _effective_dihedral(af_params: Dict, b: float, S: float, mass: float,
                        qbar: float) -> Tuple[float, float]:
    """Roll-due-to-sideslip C_lβ from geometric dihedral + sweepback, plus the
    equivalent dihedral angle (EDA) that produces it — the
    reconcile-with-Panknin figure.  Composition:
      dihedral C_lβ : panels averaged via Δα = arctan(sinβ·tanΓ), weighted by
          the panel span fraction, times the wing lift slope a_w (lifting-line)
          and the taper factor Fλ = (1+2λ)/(3(1+λ)).
      sweep C_lβ    : K_SWEEP · CL_cruise · tan(Λ), times Fλ (span-lift
          leverage ∝ total lift — zero at CL=0, grows with CL).
    Per-frame geometry lives in the descriptor as optional keys:
      sweep_deg, dihedral_deg, anhedral_deg, anhedral_start (tip panels; the
      remaining span holds +dihedral_deg), taper_ratio (default 0.5).
    Frames without explicit geometry keep the existing dihedral_coeff (rad)
    as their mean geometric dihedral.
    """
    dihedral_coeff = af_params.get("dihedral_coeff", 0.0)
    sweep_deg = af_params.get("sweep_deg", 0.0)
    lam = af_params.get("taper_ratio", 0.5)
    if af_params.get("dihedral_deg") is not None:
        g_start = float(af_params["dihedral_deg"])
        if af_params.get("anhedral_deg"):
            start = min(max(af_params.get("anhedral_start", 2.0 / 3.0), 0.0), 1.0)
            g_eff_deg = g_start * start - abs(af_params["anhedral_deg"]) * (1.0 - start)
        else:
            g_eff_deg = g_start
    else:
        g_eff_deg = math.degrees(dihedral_coeff)
    F_lambda = (1.0 + 2.0 * lam) / (3.0 * (1.0 + lam))
    ar = b * b / max(S, 1e-9)
    a_w = 2.0 * math.pi * ar / (2.0 + math.sqrt(4.0 + ar * ar))
    cl_cruise = mass * GRAVITY / max(qbar * S, 1e-9)
    C_l_beta = (a_w * math.tan(math.radians(g_eff_deg))
                + K_SWEEP * cl_cruise * math.tan(math.radians(sweep_deg))) * F_lambda
    eda_deg = math.degrees(math.atan(C_l_beta / max(a_w * F_lambda, 1e-9)))
    return C_l_beta, eda_deg


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

    # Moment of inertia — component build-up (see _compute_fw_inertia)
    I_roll, I_pitch = _compute_fw_inertia(af_params)
    I_yaw   = I_roll + I_pitch

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

    # Effective dihedral → roll-due-to-sideslip (C_lβ) — composition and
    # per-frame geometry handling live in _effective_dihedral (shared with the
    # lateral-mode analysis).  Prior versions had NO angle-based roll restoring
    # at all (only rate damping + yaw coupling), so open-loop gust tests
    # randomly walked off / spiralled on frames with a modest dihedral moment
    # while their closed-loop sims (and real flying) were fine — a model gap,
    # not an airframe defect.
    C_l_beta, eda_deg = _effective_dihedral(af_params, b, S, mass, qbar)

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
        L_restore = -C_l_beta * qbar * S * b * math.sin(angles["Roll"])
        alpha_r = (L_ctrl + L_dihedral + L_restore - L_damp + gust_torque["Roll"]) / max(I_roll,  1e-6)
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
class LateralModeMetrics:
    """Linearized lateral-directional modes (β, p, r, φ small-perturbation)."""
    dutch_roll_zeta: float = 0.0       # damping ratio (negative = unstable)
    dutch_roll_period_s: float = 0.0
    dutch_roll_wn_rads: float = 0.0
    pair_real: bool = False            # True: no oscillatory DR (overdamped)
    spiral_time_to_double_s: float = 0.0  # > 0 only when spiral is unstable
    spiral_t_half_s: float = 0.0          # stable spiral: time to halve, s
    roll_sub_tau_s: float = 0.0
    dutch_roll_ok: bool = True
    spiral_ok: bool = True
    eda_deg: float = 0.0

def _mat_mul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    n = len(A)
    return [[sum(A[i][k] * B[k][j] for k in range(n)) for j in range(n)] for i in range(n)]

def _mat_trace(M: List[List[float]]) -> float:
    return sum(M[i][i] for i in range(len(M)))

def _char_poly(A: List[List[float]]) -> List[float]:
    """Characteristic polynomial det(λI − A) = λⁿ + a₁λⁿ⁻¹ + … + aₙ,
    via Faddeev–LeVerrier."""
    n = len(A)
    B = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    coeffs = [1.0]
    for k in range(1, n + 1):
        M = _mat_mul(A, B)
        a = -_mat_trace(M) / k
        coeffs.append(a)
        B = [[M[i][j] + (a if i == j else 0.0) for j in range(n)] for i in range(n)]
    return coeffs

def _poly_eval(coeffs: List[float], z: complex) -> complex:
    acc = complex(coeffs[0], 0.0)
    for c in coeffs[1:]:
        acc = acc * z + c
    return acc

def _cbrt(x: complex) -> complex:
    """Cube root.  Python's ** (1/3) returns the complex principal root of a
    negative real, which breaks Cardano (picks a complex root where a real one
    exists); use a sign-preserving real cube root for real radicands."""
    if abs(x.imag) < 1e-12:
        re = x.real
        return complex(-((-re) ** (1.0 / 3.0)), 0.0) if re < 0.0 else complex(re ** (1.0 / 3.0), 0.0)
    return x ** (1.0 / 3.0)

def _cubic_real_root(A: float, B: float, C: float) -> complex:
    """One root of z³ + A z² + B z + C = 0 (Cardano; real for real inputs).
    The depressed form t³ + pt + q = 0 yields a real root as the sum of a
    conjugate pair of cube roots — valid also for Δ < 0 (3 real)."""
    p = B - A * A / 3.0
    q = 2.0 * A * A * A / 27.0 - A * B / 3.0 + C
    disc = cmath.sqrt((q / 2.0) ** 2 + (p / 3.0) ** 3)
    u = _cbrt(-q / 2.0 + disc)
    v = _cbrt(-q / 2.0 - disc)
    return u + v - A / 3.0

def _quartic_roots(coeffs: List[float]) -> List[complex]:
    """Roots of monic quartic x⁴ + ax³ + bx² + cx + d = 0 via the closed-form
    Ferrari solution (resolvent cubic → two quadratics).  Deterministic — the
    alternative Durand–Kerner iteration stalls into non-root fixed points
    whenever all four roots are real, which is the common case here."""
    a, b, c, d = coeffs[1], coeffs[2], coeffs[3], coeffs[4]
    p = b - 3.0 * a * a / 8.0
    q = c - a * b / 2.0 + a * a * a / 8.0
    r = d - a * c / 4.0 + a * a * b / 16.0 - 3.0 * a ** 4 / 256.0

    # Resolvent cubic in u = α²: u³ + 2p·u² + (p² − 4r)·u − q² = 0
    u0 = _cubic_real_root(2.0 * p, p * p - 4.0 * r, -q * q)
    alpha = cmath.sqrt(u0)

    if abs(alpha) < 1e-9 and abs(q) > 1e-9:
        # Degenerate α≈0 is only consistent when q≈0; guard by solving the
        # α=0 factor system (β+γ=p, βγ=r) directly.
        beta = (p + cmath.sqrt(p * p - 4.0 * r)) / 2.0
        gamma = (p - cmath.sqrt(p * p - 4.0 * r)) / 2.0
    else:
        beta = (u0 + p - q / alpha) / 2.0
        gamma = (u0 + p + q / alpha) / 2.0

    # The two quadratics are y² + αy + β = 0  and  y² − αy + γ = 0
    roots = []
    for l, t2 in [(alpha, beta), (-alpha, gamma)]:
        discr = cmath.sqrt(alpha * alpha - 4.0 * t2)
        roots.append((-l + discr) / 2.0 - a / 4.0)
        roots.append((-l - discr) / 2.0 - a / 4.0)
    return roots

def _poly_roots(coeffs: List[float]) -> List[complex]:
    """Roots of a monic quartic (closed-form, see _quartic_roots)."""
    return _quartic_roots(coeffs)

def analyze_lateral_modes(af_filename: str) -> LateralModeMetrics:
    """Lateral-directional mode analysis (Dutch roll / spiral / roll
    subsidence) from the FW descriptor aero data.

    Forms the 4-state small-perturbation system (β, p, r, φ) about level
    cruise (θ₀ = 0):
        β̇ = (Yβ/mV)β − r + (g/V)φ
        ṗ = (Lβ/Ixx)β + (Lp/Ixx)p + (Lr/Ixx)r
        ṙ = (Nβ/Izz)β + (Nr/Izz)r
        φ̇ = p
    Derivatives use the same C_lβ / yaw-stability / damping conventions as
    simulate_freeflight; quadratic damping is applied as an equivalent linear
    term at |rate| = LIN_EQ_RATE.  Fin sideforce (Y_β) and yaw damping (N_r)
    are estimated from rudder_area / rudder_arm with the A_V_FIN fin
    lift-slope assumption.  N_p (roll→yaw) is deliberately 0 — the non-linear
    sim has no roll-rate yaw coupling either.  Diagnostic only.
    """
    af_params = get_fw_descriptor(af_filename)
    rho = AIR_DENSITY
    V = af_params["cruise_speed"]
    qbar = 0.5 * rho * V * V
    S = af_params["wing_area"]
    b = af_params["wingspan"]
    chord = S / b
    mass = af_params["mass"]

    # Moment of inertia — component build-up (see _compute_fw_inertia)
    I_roll, I_pitch = _compute_fw_inertia(af_params)
    Ixx = I_roll
    Izz = I_roll + I_pitch

    C_l_beta, eda_deg = _effective_dihedral(af_params, b, S, mass, qbar)
    L_beta = C_l_beta * qbar * S * b
    N_beta = af_params.get("yaw_stability", 0.0) * qbar * S * b

    C_d_roll = abs(af_params.get("roll_damp", 0.5 * rho * V * b * b * b * 0.04))
    C_d_yaw  = abs(af_params.get("yaw_damp", 0.10)) if af_params.get("rudder_area", 0) > 0 else 0.2
    roll_damp_lin = af_params.get("roll_damp_lin", 0.02)
    yaw_damp_lin  = af_params.get("yaw_damp_lin", 0.015)

    # Dihedral yaw-rate→roll coupling (same term as the non-linear sim)
    L_r = af_params.get("dihedral_coeff", 0.0) / Ixx

    # Equivalent-linear damping at representative oscillation amplitude
    L_p = -(C_d_roll * LIN_EQ_RATE + roll_damp_lin) / Ixx

    s_v = af_params.get("rudder_area", 0.0)
    l_v = af_params.get("rudder_arm", 0.0)
    if s_v > 0:
        n_r_fin = A_V_FIN * qbar * s_v * l_v * l_v / V
        y_beta  = A_V_FIN * qbar * s_v / (mass * V)
        N_r = -(C_d_yaw * LIN_EQ_RATE + yaw_damp_lin + n_r_fin) / Izz
    else:
        N_r = -(C_d_yaw * LIN_EQ_RATE + yaw_damp_lin) / Izz
        y_beta = 0.0

    gv = GRAVITY / V

    A = [
        [y_beta,              0.0,      -1.0,      gv],
        [L_beta / Ixx,        L_p,       L_r,      0.0],
        [N_beta / Izz,        0.0,       N_r,      0.0],
        [0.0,                 1.0,       0.0,      0.0],
    ]

    roots = _poly_roots(_char_poly(A))

    m = LateralModeMetrics()
    m.eda_deg = eda_deg

    pairs = sorted((z for z in roots if abs(z.imag) >= 1e-6),
                   key=lambda z: abs(z.imag), reverse=True)
    reals = [z for z in roots if abs(z.imag) < 1e-6]

    # Dutch roll: the oscillatory pair (0 or 1 → overdamped, no oscillation)
    if pairs:
        lam = pairs[0]
        wn = abs(lam)
        if wn > 1e-9:
            m.dutch_roll_wn_rads = wn
            m.dutch_roll_period_s = 2.0 * math.pi / wn
            m.dutch_roll_zeta = -lam.real / wn
        m.dutch_roll_ok = m.dutch_roll_zeta >= DR_ZETA_MIN
    else:
        m.pair_real = True
        m.dutch_roll_ok = True  # no oscillation ⇒ no unstable oscillation

    # Spiral: the slowest mode — smallest |Re| among the real roots and any
    # second (low-frequency) complex pair.  Time-to-double for Re>0 (i.e.
    # unstable growth), time-to-half otherwise.
    slow = list(reals)
    if len(pairs) >= 2:
        slow.append(pairs[1])
    if slow:
        l_spiral = min(slow, key=lambda z: abs(z.real))
        re_spiral = l_spiral.real
        if re_spiral > 0.0:
            m.spiral_time_to_double_s = math.log(2.0) / re_spiral
            m.spiral_ok = m.spiral_time_to_double_s >= SPIRAL_T_DOUBLE_MIN
        else:
            m.spiral_t_half_s = math.log(2.0) / abs(re_spiral)

    # Roll subsidence: the fast large-negative real root (or the second pair's
    # decay rate when the system is fully oscillatory).
    if reals:
        reals_neg = [z.real for z in reals if z.real < 0.0]
        if reals_neg:
            m.roll_sub_tau_s = 1.0 / abs(min(reals_neg))
    elif len(pairs) >= 2 and pairs[1].real < 0.0:
        m.roll_sub_tau_s = 1.0 / abs(pairs[1].real)

    return m

def format_lateral_modes(m: LateralModeMetrics, af_filename: str) -> List[str]:
    """Format lateral-directional mode analysis as report lines."""
    lines = [f"\n  {B}Lateral modes:{N} (linearized β-p-r-φ)   "
             f"EDA {m.eda_deg:.1f}°"]
    if m.pair_real:
        lines.append(f"    Dutch roll:   overdamped (no oscillation)   [{G}PASS{N}]")
    else:
        icon = f"{G}PASS{N}" if m.dutch_roll_ok else f"{R}FAIL{N}"
        lines.append(f"    Dutch roll:   ζ={m.dutch_roll_zeta:+.2f}  "
                     f"T={m.dutch_roll_period_s:.1f}s  ωn={m.dutch_roll_wn_rads:.2f} rad/s  "
                     f"(limit ζ ≥ {DR_ZETA_MIN:.2f})  [{icon}]")
    if m.spiral_time_to_double_s > 0.0:
        icon = f"{G}PASS{N}" if m.spiral_ok else f"{R}FAIL{N}"
        lines.append(f"    Spiral:       UNSTABLE  T½={m.spiral_time_to_double_s:.1f}s  "
                     f"(limit ≥ {SPIRAL_T_DOUBLE_MIN:.0f}s)  [{icon}]")
    else:
        lines.append(f"    Spiral:       stable  T½={m.spiral_t_half_s:.1f}s  [{G}PASS{N}]")
    lines.append(f"    Roll subs.:   τ={m.roll_sub_tau_s:.1f}s  (informational)")
    return lines

# ═══════════════════════════════════════════
#  Report / Critique — shared with FC-side measurement
# ═══════════════════════════════════════════
# Criteria tables, verdict helpers, kritik/report renderers, and the
# metric extractors live in src/critic/ (critic/metrics.py + critic/criteria.py)
# so the FC in-flight trace tool can measure identically. All of it is
# imported at the top of this file — nothing is defined locally anymore.
# ═══════════════════════════════════════════

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

def yaw_rate_max_mr(params: dict) -> float:
    """MR yaw angle-loop rate clamp — mirrors control.c DoQuaternionAttitudeControl:
    rate demand is capped at min(A[Yaw].R.Max, Nav.MaxHeadingRate). The quaternion
    attitude step/disturbance paths must clamp with the same bound, else yaw
    authority is over-estimated (MaxYawRate is typically 2x MaxHeadingRate)."""
    vals = [v for v in (params.get("MAX_YAW_RATE"), params.get("MAX_HEADING_RATE"))
            if v is not None and v > 0.0]
    return min(vals) if vals else 10.0

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

    cat = AF_CATEGORY[af_type]

    if slider_pct is not None:
        params = apply_slider(params, slider_pct, cat=cat)

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
            coupled_metrics = simulate_axis_coupled(af_filename, step_axis=step_ax,
                                                    gusts=gusts, step_rads=fw_step_rads,
                                                    params_override=params)
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
            if name == "Yaw":
                mr = yaw_rate_max_mr(params)
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
        if name == "Yaw" and not is_fw:
            mr = yaw_rate_max_mr(params)
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
        cruise_thr = FW_CRUISE_THR[fw_mid] if fw_mid >= 0 else None
        ah_m = simulate_alt_hold_fw(params, step_m=5.0, cruise_v=cruise_v,
                                    cruise_thr=cruise_thr)
        ah_crit = AH_CRITERIA_FW
    else:
        ah_m = simulate_alt_hold_mr(params, step_m=5.0, af_filename=af_filename)
        ah_crit = AH_CRITERIA_MR

    lines, ok = critique_linear(ah_m, ah_crit, "Alt Hold 5m")
    all_output.extend(lines)
    if not ok:
        all_pass = False

    # Descent leg: hold at +5 m, command 0 m. FW and MR both come down by
    # throttling back -> the descent rate is bound by the NEGATIVE comp
    # authority (-thr_lim, floor at -min(thr_lim, cruise_thr)), the
    # counterpart of the +climb ceiling. Same Criteria, scored on
    # descent-progress so the shared rise/settle metrics apply.
    all_output.append(f"\n{B}{'='*60}{N}")
    all_output.append(f"{B}  Altitude Hold (5m descent){N}")
    all_output.append(f"{B}{'='*60}{N}")

    if is_fw:
        ah_d = simulate_alt_hold_fw(params, step_m=5.0, cruise_v=cruise_v,
                                    descend=True, cruise_thr=cruise_thr)
    else:
        ah_d = simulate_alt_hold_mr(params, step_m=5.0, af_filename=af_filename,
                                    descend=True)

    lines, ok = critique_linear(ah_d, ah_crit, "Alt Descent 5m")
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

        # Lateral-directional mode analysis (Dutch roll / spiral recovery)
        lm = analyze_lateral_modes(af_filename)
        lm_lines = format_lateral_modes(lm, af_filename)
        all_output.extend(lm_lines)
        if not (lm.dutch_roll_ok and lm.spiral_ok):
            all_pass = False

    return all_pass, all_output


def main():
    # Process-step battery (foci 5/6): python3 src/tests/test_pid_sim.py process <af>
    if len(sys.argv) > 2 and sys.argv[1] == "process":
        af_filename = sys.argv[2]
        if not af_filename.endswith('.af'):
            af_filename += '.af'
        pass_ok, output = run_process_steps(af_filename)
        for l in output:
            print(l)
        sys.exit(0 if pass_ok else 1)

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