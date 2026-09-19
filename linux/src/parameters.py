# ParamTable-derived parameter definitions
# Auto-generated from UAVXArm32F4/src/params.c

from typing import Dict, Tuple
from dataclasses import dataclass

from protocol_enums import ParamIndex, RC_MAP_INDICES, RC_MAP_EXPECTED, RC_MAPPING_NAMES


@dataclass(frozen=True)
class Parameter:
    name: str
    description: str = ""


PARAMETER_DEFS: Dict[int, Parameter] = {
    0: Parameter("RollRateKp", "Roll rate P gain"),
    1: Parameter("AltPosKi", "Altitude integral gain"),
    2: Parameter("RollAngleQKp", "Roll angle P gain"),
    3: Parameter("ArmingMode", "Arming mode"),
    4: Parameter("RollAngleQIntLimit", "Roll angle integral limit"),
    5: Parameter("PitchRateKp", "Pitch rate P gain"),
    6: Parameter("AltPosKp", "Altitude position P gain"),
    7: Parameter("PitchAngleQKp", "Pitch angle P gain"),
    8: Parameter("RFSensorType", "Rangefinder sensor type"),
    9: Parameter("PitchAngleQIntLimit", "Pitch angle integral limit"),
    10: Parameter("YawRateKp", "Yaw rate P gain"),
    11: Parameter("RollRateKd", "Roll rate D gain"),
    12: Parameter("IMUFiltType", "IMU filter type"),
    13: Parameter("BBLogType", "Blackbox log type"),
    14: Parameter("RxType", "RX type"),
    15: Parameter("Config1Bits", "Configuration bitfield 1 (Emulation)"),
    16: Parameter("RxThrottleCh", "Throttle channel mapping"),
    17: Parameter("LowVoltThres", "Low voltage threshold (V)"),
    18: Parameter("RollCamKp", "Roll camera gain"),
    19: Parameter("Unused20", "Unused -- was EstCruiseThr (moved to FC Config.CruiseThrottleFF)"),
    20: Parameter("StickHysteresis", "Stick hysteresis"),
    21: Parameter("FWClimbThrottle", "FW climb throttle"),
    22: Parameter("PercentIdleThr", "Idle throttle percent"),
    23: Parameter("RollAngleQKi", "Roll angle integral gain"),
    24: Parameter("PitchAngleQKi", "Pitch angle integral gain"),
    25: Parameter("PitchCamKp", "Pitch camera gain"),
    26: Parameter("ServoLPFHz", "Servo LPF Hz"),
    27: Parameter("PitchRateKd", "Pitch rate D gain"),
    28: Parameter("NavVelKp", "Nav velocity gain"),
    29: Parameter("AltVelKd", "Alt velocity damping (FW rate damping term)"),
    30: Parameter("Horizon", "Was Horizon (still active)"),
    31: Parameter("MadgwickKpMag", "Madgwick magnetic gain"),
    32: Parameter("NavRTHAlt", "RTH altitude (m)"),
    33: Parameter("NavMagVar", "Magnetic variation"),
    34: Parameter("UnusedSensorHint", "Sensor type hint"),
    35: Parameter("ESCType", "ESC type"),
    36: Parameter("RCChannels", "Number of RC channels"),
    37: Parameter("RxRollCh", "Roll channel mapping"),
    38: Parameter("MadgwickKpAcc", "Madgwick accel gain"),
    39: Parameter("RollCamTrim", "Roll camera trim"),
    40: Parameter("NavPosIntLimit", "Nav max velocity (m/s)"),
    41: Parameter("RxPitchCh", "Pitch channel mapping"),
    42: Parameter("RxYawCh", "Yaw channel mapping"),
    43: Parameter("AFType", "Airframe type"),
    44: Parameter("TelemetryType", "Telemetry type"),
    45: Parameter("MaxDescentRateMpS", "Max descent rate (m/s) — vertical-profile descent shaping"),
    46: Parameter("DescentDelayS", "Descent delay (s)"),
    47: Parameter("GyroLPFSel", "Gyro LPF selection"),
    48: Parameter("NavCrossTrackKp", "Cross-track gain"),
    49: Parameter("RxGearCh", "Gear channel mapping"),
    50: Parameter("RxAux1Ch", "Aux1 channel mapping"),
    51: Parameter("ServoSense", "Servo sense"),
    52: Parameter("AccConfSD", "Accel confidence SD"),
    53: Parameter("BatteryCapacity", "Battery capacity (mAh)"),
    54: Parameter("RxAux2Ch", "Aux2 channel mapping"),
    55: Parameter("RxAux3Ch", "Aux3 channel mapping"),
    56: Parameter("NavPosKp", "Nav position gain"),
    57: Parameter("AltLPF", "Altitude LPF -- unused (KF provides denoising)"),
    58: Parameter("Balance", "Balance"),
    59: Parameter("RxAux4Ch", "Aux4 channel mapping"),
    60: Parameter("NavPosKi", "Nav position integral gain"),
    61: Parameter("UnusedGPSProtocol", "Unused GPS protocol"),
    62: Parameter("NavPosIntLim", "Nav position integral limit"),
    63: Parameter("MaxYawRate", "Max yaw rate"),
    64: Parameter("FWRollPitchFF", "FW roll-pitch feedforward"),
    65: Parameter("FWPitchThrottleFF", "FW pitch-throttle feedforward"),
    66: Parameter("UnusedAltVelIntLimit", "[unused] AltVelIntLimit"),
    67: Parameter("FWMaxClimbAngle", "FW max climb angle"),
    68: Parameter("NavMaxAngle", "Nav max bank angle"),
    69: Parameter("FWSpoilerDecayPercentPS", "FW spoiler decay (%/s)"),
    70: Parameter("FWAileronDifferential", "FW aileron differential"),
    71: Parameter("ASSensorType", "Airspeed sensor type"),
    72: Parameter("Unused73", "Unused -- was KFAccUBiasVar (tracked live in state.c)"),
    73: Parameter("Config2Bits", "Config 2 bitfield"),
    74: Parameter("MaxPitchAngle", "Max pitch angle"),
    75: Parameter("Unused76", "Unused"),
    76: Parameter("MaxRollAngle", "Max roll angle"),
    77: Parameter("YawLPFHz", "Yaw LPF Hz"),
    78: Parameter("NavHeadingTurnout", "Nav heading turnout"),
    79: Parameter("AltHoldThrCompDecayPercentPS", "Alt hold throttle comp decay (%/s)"),
    80: Parameter("UnusedAltPosKd", "Unused -- was Nav vel Kd"),
    81: Parameter("FWBoardPitchAngle", "FW board pitch angle"),
    82: Parameter("MaxRollRate", "Max roll rate"),
    83: Parameter("MaxPitchRate", "Max pitch rate"),
    84: Parameter("CurrentScale", "Current scale (A)"),
    85: Parameter("VoltScale", "Volt scale (V)"),
    86: Parameter("FWAileronRudderMix", "FW aileron-rudder mix"),
    87: Parameter("FWAltSpoilerFF", "FW altitude spoiler FF"),
    88: Parameter("MaxHeadingRate", "Max heading rate"),
    89: Parameter("AccLPFSel", "Acc LPF selection"),
    90: Parameter("YawRateKd", "Yaw rate D gain"),
    91: Parameter("Unused92", "Unused -- was GyroSlewRate"),
    92: Parameter("ThrottleGainRate", "Throttle gain rate"),
    93: Parameter("RxAux5Ch", "Aux5 channel mapping"),
    94: Parameter("RxAux6Ch", "Aux6 channel mapping (PassThru)"),
    95: Parameter("DiveRC", "Dive RC channel mapping"),
    96: Parameter("YawAngleQKp", "Yaw angle P gain"),
    97: Parameter("YawAngleQKi", "Yaw angle integral gain"),
    98: Parameter("YawAngleQIntLimit", "Yaw angle integral limit"),
    99: Parameter("AltPosIntLimit", "Alt position integral limit"),
    100: Parameter("MotorStopSel", "Motor stop selection"),
     101: Parameter("AltROCKi", "ROC velocity integral gain (I term)"),
    102: Parameter("AltThrottleCompLimit", "Alt throttle comp limit"),
    103: Parameter("VRSROC", "VRS ROC (m/s)"),
    104: Parameter("Unused105", "Unused -- was BootDiag (moved to FC Config.BootDiag)"),
    105: Parameter("AHROCWindowMPS", "AH ROC window (m/s)"),
    106: Parameter("NavProxAltM", "Nav proximity altitude (m)"),
    107: Parameter("NavProxRadiusM", "Nav proximity radius (m)"),
108: Parameter("YawRateKi", "Yaw rate loop integral gain (heading hold)"),
     109: Parameter("YawRateIntLim", "Yaw rate loop integral limit (windup)"),
110: Parameter("Unused111", "Unused -- was KFBaroVar (tracked live in state.c)"),
     111: Parameter("Unused112", "Unused -- was KFAccUVar (tracked live in state.c)"),
    112: Parameter("FWStickScale", "FW stick scale"),
    113: Parameter("Unused114", "Unused -- was FW roll control pitch limit"),
    114: Parameter("AHThrottleMovingTrigger", "AH throttle moving trigger"),
    115: Parameter("NavFenceRadiusM", "Nav fence radius (m)"),
    116: Parameter("DiveROC", "Dive ROC (m/s)"),
    117: Parameter("Unused118", "Unused"),
    118: Parameter("Unused119", "Unused"),
    119: Parameter("Unused120", "Unused"),
120: Parameter("AltROCKp", "ROC velocity proportional gain (P term)"),
121: Parameter("MaxClimbRateMpS", "Max climb rate (m/s) — vertical-profile ascent shaping"),
     122: Parameter("RudderMotorFF", "Rudder Motor Feedforward (%)"),
     123: Parameter("SpiralDescentBandM", "Spiral descent trigger band (m) — residual altitude that engages MR spiral-orbit descent"),
    124: Parameter("Unused125", "Unused"),
    125: Parameter("TraceType", "Trace capture selector (U8, TraceTypes enum): 0=None 1=Rate 2=Attitude 3=AltHold 4=Actuator 5=IMU"),
    126: Parameter("Unused127", "Unused"),
    127: Parameter("PowerResetCause", "Reset cause"),
}

# FC-native default values from ParamTable
PARAM_DEFAULTS: Dict[int, float] = {
    0: 0.3,
    1: 0.001,
    2: 1.75,
    3: 0.0,
    4: 0.01,
    5: 0.45,
    6: 0.35,
    7: 1.75,
    8: 5.0,
    9: 0.01,
    10: 0.75,
    11: 0.015,
    12: 0.0,
    13: 0.0,
    14: 0.0,
    15: 18.0,
    16: 0.0,
    17: 9.6,
    18: 1.0,
    19: 0.55,
    20: 0.02,
    21: 0.0,
    22: 0.05,
    23: 0.0125,
    24: 0.0125,
    25: 1.0,
    26: 25.0,
    27: 0.0225,
    28: 0.5,
    29: 0.16,
    30: 0.3,
    31: 0.5,
    32: 10.0,
    33: 0.2059488517353309,
    34: 4.0,
    35: 2.0,
    36: 7.0,
    37: 1.0,
    38: 0.4,
    39: 0.0,
    40: 5.0,
    41: 2.0,
    42: 3.0,
    43: 4.0,
    44: 2.0,
    45: 3.0,
    46: 15.0,
    47: 2.0,
    48: 0.04,
    49: 4.0,
    50: 5.0,
    51: 0.0,
    52: 10.0,
    53: 1500.0,
    54: 6.0,
    55: 10.0,
    56: 0.25,
    57: 10.0,
    58: 0.0,
    59: 8.0,
60: 0.02,
    61: 0.0,
    62: 1.0,
    63: 2.0943951023931953,
    64: 0.0,
    65: 0.0,
    66: 0.5,
    67: 1.0471975511965976,
    68: 0.4363323129985824,
    69: 0.01,
    70: 0.0,
    71: 4.0,
    72: 0.0001,
    73: 1.0,
    74: 0.5235987755982988,
    75: 0.0,
    76: 0.5235987755982988,
    77: 50.0,
    78: 0.3490658503988659,
    79: 0.03,
    80: 0.8,
    81: 0.0,
    82: 4.1887902047863905,
    83: 2.0943951023931953,
    84: 0.0,
    85: 18.5,
    86: 0.0,
    87: 0.0,
    88: 0.5235987755982988,
    89: 4.0,
    90: 0.0375,
    91: 5.0,
    92: 0.0,
    93: 9.0,
    94: 7.0,
    95: 11.0,
    96: 0.75,
    97: 0.0125,
    98: 0.03,
    99: 0.5,
    100: 0.0,
     101: 0.001,

    102: 1.0,
    103: -1.5,
    104: 0.0,
    105: 1.0,
    106: 4.0,
    107: 3.0,
    108: 0.0,
    109: 0.03,
    110: 0.04,
    111: 2.0,
    112: 0.4,
    113: 0.0,
    114: 0.02,
    115: 200.0,
    116: -3.0,
    117: 0.0,
    118: 0.0,
    119: 0.0,
     120: 0.05,

    121: 3.0,
    122: 0.0,
    123: 3.0,
124: 0.0,
     125: 1.0,
     126: 0.0,
    127: 0.0,
}

# Display multiplier: convert FC-native float -> GCS display value
# Derived from ParamTable.scale (inverse) for unit-conversion params
PARAM_DISPLAY_MULT: Dict[int, float] = {
    0: 1.0,
    1: 1.0,
    2: 1.0,
    3: 1.0,
    4: 1.0,
    5: 1.0,
    6: 1.0,
    7: 1.0,
    8: 1.0,
    9: 1.0,
    10: 1.0,
    11: 1.0,
    12: 1.0,
    13: 1.0,
    14: 1.0,
    15: 1.0,
    16: 1.0,
    17: 1.0,
    18: 1.0,
19: 1.0,
    20: 100.0,
    21: 100.0,
    22: 100.0,
    23: 1.0,
    24: 1.0,
    25: 1.0,
    26: 1.0,
    27: 1.0,
    28: 1.0,
    29: 1.0,
    30: 100.0,
    31: 100.0,
    32: 1.0,
    33: 57.29577951308232,
    34: 1.0,
    35: 1.0,
    36: 1.0,
    37: 1.0,
    38: 1.0,
    39: 100.0,
    40: 1.0,
    41: 1.0,
    42: 1.0,
    43: 1.0,
    44: 1.0,
    45: 1.0,
    46: 1.0,
    47: 1.0,
    48: 100.0,
    49: 1.0,
    50: 1.0,
    51: 1.0,
    52: 1.0,
    53: 1.0,
    54: 1.0,
    55: 1.0,
    56: 1.0,
    57: 1.0,
    58: 100.0,
    59: 1.0,
    60: 1.0,
    61: 1.0,
    62: 1.0,
    63: 57.29577951308232,
    64: 1.0,
    65: 100.0,
    66: 1.0,
    67: 57.29577951308232,
    68: 57.29577951308232,
    69: 1000.0,
    70: 100.0,
    71: 1.0,
    72: 1.0,
    73: 1.0,
    74: 57.29577951308232,
    75: 1.0,
    76: 57.29577951308232,
    77: 1.0,
    78: 57.29577951308232,
    79: 1.0,
    80: 1.0,
    81: 57.29577951308232,
    82: 57.29577951308232,
    83: 57.29577951308232,
    84: 1.0,
    85: 1.0,
    86: 100.0,
    87: 100.0,
    88: 57.29577951308232,
    89: 1.0,
    90: 1.0,
    91: 1.0,
    92: 1.0,
    93: 1.0,
    94: 1.0,
    95: 1.0,
    96: 1.0,
    97: 1.0,
    98: 1.0,
    99: 1.0,
    100: 1.0,
    101: 1.0,
    102: 100.0,
    103: 1.0,
    104: 1.0,
    105: 1.0,
    106: 1.0,
    107: 1.0,
    108: 1.0,
    109: 100.0,
    110: 1.0,
    111: 1.0,
    112: 100.0,
    113: 1.0,
    114: 100.0,
    115: 1.0,
    116: 1.0,
    117: 1.0,
    118: 1.0,
    119: 100.0,
    120: 1.0,
    121: 1.0,
    122: 100.0,
    123: 1.0,
    124: 1.0,
    125: 1.0,
    126: 1.0,
    127: 1.0,
}

# ---------------------------------------------------------------------------
# Class-based parameter bounds — mirrors FC UAVXArmQ/src/params.c:
#   - PARAM_CLASS_BOUNDS: raw-unit ceiling per ParamClass (ParamClass[] table)
#   - PARAM_CLASS_OF: tag -> class (ParamTable[tag].cls column)
#   - PARAM_EXPLICIT_LIMITS: authoritative per-tag (lo, hi) for eClassExplicit tags
# PARAM_LIMITS is DERIVED from these, so it can never drift from the FC clamp.
# All bounds are RAW FC units (display = raw * PARAM_DISPLAY_MULT).
# Gain ceilings are deliberately WIDE (~10x typical) so field tuning can type
# gains well above the default window; the tight per-frame guidance lives in the
# .af [LIMITS] block, not here.  They still catch fat-finger 100x slips.
# ---------------------------------------------------------------------------
PARAM_CLASS_BOUNDS: Dict[str, Tuple[float, float]] = {
    "eClassExplicit":      (0.0, 0.0),           # not indexed — entry carries bounds
    "eClassGainRateP":   (0.0, 30.0),          # rate P gains
    "eClassGainRateD":   (0.0, 2.0),           # rate D gains
    "eClassGainRateI":   (0.0, 2.0),           # yaw rate I
    "eClassGainAngleQ":  (0.0, 200.0),         # quaternion angle P
    "eClassGainAngleI":  (0.0, 250.0),         # angle integral
    "eClassGainAlt":      (0.0, 3.66),          # alt position/vel/ROC loop
    "eClassGainNav":      (0.0, 15.0),          # nav position/vel/crosstrack
    "eClassGainCam":      (0.0, 20.0),          # camera gimbal
    "eClassGainKf":       (0.0, 20.0),          # KF variances
    "eClassRateIntLim":  (0.0, 30.0),          # rad/s integral clamps
    "eClassAngle":         (0.0, 1.047198),   # rad — 60 deg (FC ParamTable literal)
    "eClassAngleBipolar": (-0.349066, 0.349066),  # rad — +/-20 deg (FC ParamTable literal)
    "eClassRate":          (0.0, 10.471976),   # rad/s — 600 deg/s (FC ParamTable literal)
    "eClassPct":           (0.0, 1.0),           # fraction — 100%
    "eClassPctS":         (0.0, 0.5),           # fraction/s — 50%/s
    "eClassHz":            (0.0, 255.0),         # Hz
    "eClassVolts":             (0.0, 255.0),         # V
    "eClassAmps":             (0.0, 255.0),         # A
    "eClassMetres":             (0.0, 1000.0),        # m
    "eClassMs":            (0.0, 50.0),          # m/s
    "eClassMah":           (1500.0, 10000.0),    # mAh
    "eClassSeconds":             (0.0, 60.0),          # s
}

# Tag -> ParamClass — mirror of ParamTable[tag].cls (index == tag)
PARAM_CLASS_OF: Dict[int, str] = {
    0: "eClassGainRateP", 1: "eClassGainAlt", 2: "eClassGainAngleQ",
    3: "eClassExplicit", 4: "eClassRateIntLim", 5: "eClassGainRateP",
    6: "eClassGainAlt", 7: "eClassGainAngleQ", 8: "eClassExplicit",
    9: "eClassRateIntLim", 10: "eClassGainRateP", 11: "eClassGainRateD",
    12: "eClassExplicit", 13: "eClassExplicit", 14: "eClassExplicit",
    15: "eClassExplicit", 16: "eClassExplicit", 17: "eClassExplicit",
    18: "eClassGainCam", 19: "eClassExplicit", 20: "eClassPct",
    21: "eClassPct", 22: "eClassPct", 23: "eClassGainAngleI",
    24: "eClassGainAngleI", 25: "eClassGainCam", 26: "eClassHz",
    27: "eClassGainRateD", 28: "eClassGainNav", 29: "eClassGainAlt",
    30: "eClassExplicit", 31: "eClassExplicit", 32: "eClassExplicit",
    33: "eClassAngle", 34: "eClassExplicit", 35: "eClassExplicit",
    36: "eClassExplicit", 37: "eClassExplicit", 38: "eClassExplicit",
    39: "eClassPct", 40: "eClassGainNav", 41: "eClassExplicit",
    42: "eClassExplicit", 43: "eClassExplicit", 44: "eClassExplicit",
    45: "eClassExplicit", 46: "eClassExplicit", 47: "eClassExplicit",
    48: "eClassGainNav", 49: "eClassExplicit", 50: "eClassExplicit",
    51: "eClassExplicit", 52: "eClassExplicit", 53: "eClassMah",
    54: "eClassExplicit", 55: "eClassExplicit", 56: "eClassGainNav",
    57: "eClassExplicit", 58: "eClassExplicit", 59: "eClassExplicit",
    60: "eClassGainNav", 61: "eClassExplicit", 62: "eClassGainNav",
    63: "eClassRate", 64: "eClassPct", 65: "eClassPct",
    66: "eClassExplicit", 67: "eClassAngle", 68: "eClassAngle",
    69: "eClassPctS", 70: "eClassPct", 71: "eClassExplicit",
    72: "eClassExplicit", 73: "eClassExplicit", 74: "eClassAngle",
    75: "eClassExplicit", 76: "eClassAngle", 77: "eClassExplicit",
    78: "eClassAngle", 79: "eClassExplicit", 80: "eClassPct",
    81: "eClassAngleBipolar", 82: "eClassRate", 83: "eClassRate",
    84: "eClassAmps", 85: "eClassVolts", 86: "eClassPct",
    87: "eClassPct", 88: "eClassRate", 89: "eClassExplicit",
    90: "eClassExplicit", 91: "eClassExplicit", 92: "eClassExplicit",
    93: "eClassExplicit", 94: "eClassExplicit", 95: "eClassExplicit",
    96: "eClassGainAngleQ", 97: "eClassGainAngleI", 98: "eClassRateIntLim",
    99: "eClassGainAlt", 100: "eClassExplicit", 101: "eClassGainAlt",
    102: "eClassExplicit", 103: "eClassExplicit", 104: "eClassExplicit",
    105: "eClassExplicit", 106: "eClassExplicit", 107: "eClassExplicit",
    108: "eClassGainRateI", 109: "eClassExplicit", 110: "eClassExplicit",
    111: "eClassExplicit", 112: "eClassExplicit", 113: "eClassExplicit",
    114: "eClassPct", 115: "eClassExplicit", 116: "eClassExplicit",
    117: "eClassExplicit", 118: "eClassExplicit", 119: "eClassExplicit",
    120: "eClassGainAlt", 121: "eClassExplicit", 122: "eClassExplicit",
    123: "eClassExplicit", 124: "eClassExplicit", 125: "eClassExplicit",
    126: "eClassExplicit", 127: "eClassExplicit",
}

# The 21 PID terms (P, I, I-limit, and D) across Roll/Pitch/Yaw/Altitude/Nav.
# These are the classic tuning gains; spinboxes must keep 4 decimal places
# so fine rad/s values (e.g. yaw rate D 0.0003375) round-trip without
# being quantized to a misleading 0.000 display.
PID_GAIN_TAGS: frozenset = frozenset({
    0, 11, 23, 4, 2,            # Roll: RateKp, RateKd, AngleQKi, AngleQIntLimit, AngleQKp
    5, 27, 24, 9, 7,            # Pitch
    10, 90, 97, 98, 96,         # Yaw
    6, 1, 99,                   # Altitude: PosKp, PosKi, IntLimit
    56, 60, 28, 48,              # Nav (gains only; limits 40/62 are 1-dec m/s)
})

# Authoritative per-tag bounds for eClassExplicit params (raw units)
PARAM_EXPLICIT_LIMITS: Dict[int, Tuple[float, float]] = {
    3: (0.0, 1.0),            # ArmingMode TxArming..SwitchArming
    8: (0.0, 5.0),            # RFSensorType MaxSonarcm..noRF
    12: (0.0, 5.0),           # IMUFiltType LP2Filt..F4
    13: (0.0, 4.0),           # BBLogType logUAVX..logYaw
    14: (0.0, 5.0),           # RxType CPPMRx..UnknownRx
    15: (0.0, 255.0),         # Config1Bits (bits 0..7)
    16: (0.0, 15.0),          # RxThrottleCh
    17: (9.0, 20.0),          # LowVoltThres
    19: (0.0, 255.0),         # Unused20 (was EstCruiseThr — moved to FC Config.CruiseThrottleFF)
    30: (0.2, 0.5),          # Horizon (stick fraction)
    31: (0.0, 0.55),          # MadgwickKpMag
    32: (0.0, 30.0),          # NavRTHAlt
    34: (0.0, 255.0),         # UnusedSensorHint
    35: (0.0, 2.0),           # ESCType ESCPWM..MotorsOff
    36: (0.0, 16.0),          # RCChannels
    37: (0.0, 15.0),          # RxRollCh
    38: (0.0, 0.5),           # MadgwickKpAcc
    41: (0.0, 15.0),          # RxPitchCh
    42: (0.0, 15.0),          # RxYawCh
    43: (0.0, 26.0),          # AFType 0..AFUnknown
    44: (0.0, 7.0),           # TelemetryType UAVXDJT..u8Telemetry
    45: (0.0, 50.0),          # MaxDescentRateMpS
    46: (0.0, 30.0),          # DescentDelayS
    47: (0.0, 7.0),           # GyroLPFSel 0..GYRO_LPF_SEL_MAX
    49: (0.0, 15.0),          # RxGearCh
    50: (0.0, 15.0),          # RxAux1Ch
    51: (0.0, 127.0),         # ServoSense
    52: (0.0, 50.0),          # AccConfSD
    54: (0.0, 15.0),          # RxAux2Ch
    55: (0.0, 15.0),          # RxAux3Ch
    57: (0.0, 255.0),         # Unused58
    58: (-0.5, 0.5),         # CGOffset
    59: (0.0, 15.0),          # RxAux4Ch
    61: (0.0, 255.0),         # Unused62
    66: (0.0, 255.0),         # tag 66
    71: (0.0, 4.0),           # ASSensorType MS4525D0I2C..noAS
    72: (0.0, 255.0),         # Unused73 (was KFAccUBiasVar — tracked live in state.c)
    73: (0.0, 255.0),         # Config2Bits (bits 0..7)
    75: (0.0, 255.0),         # Unused76
    77: (25.0, 255.0),        # YawLPFHz
    79: (0.0, 0.2),           # AltHoldThrCompDecayPercentPS
    89: (0.0, 5.0),           # AccLPFSel 0..ACC_LPF_SEL_MAX
    90: (0.0, 0.05),          # YawRateKd
    91: (0.0, 255.0),         # Unused92
    92: (0.0, 100.0),         # ThrottleGainRate
    93: (0.0, 15.0),          # RxAux5Ch
    94: (0.0, 15.0),          # RxAux6Ch
    95: (0.0, 15.0),          # RxAux7Ch
    100: (0.0, 4.0),          # MotorStopSel landNoStop..landDescentRateAndAccU
    102: (0.0, 0.25),         # MaxAltHoldThrComp
    103: (-3.0, 0.0),        # VRSROC — crown consensus descent 1.5
    104: (0.0, 255.0),        # Unused105 (was BootDiag — moved to FC Config.BootDiag)
    105: (0.0, 30.0),         # AHROCWindowMPS
    106: (0.0, 10.0),         # NavProxAltM
    107: (0.0, 30.0),         # NavProxRadiusM
    109: (0.0, 2.0),          # YawRateIntLim
    110: (0.0, 255.0),        # Unused111 (was KFBaroVar — tracked live in state.c)
    111: (0.0, 255.0),        # Unused112 (was KFAccUVar — tracked live in state.c)
    112: (0.15, 1.0),         # FWStickScaleFrac
    113: (0.0, 255.0),         # Unused114 (was FWRollControlPitchLimit)
    115: (0.0, 1000.0),       # NavFenceRadiusM
    116: (0.0, 25.0),       # DiveRecoverAlt (0 = disabled/not commissioned; active range 10..25)
    117: (0.0, 2.0),          # FailsafeAction eFsRth..eFsMotorsOff
    118: (0.0, 60.0),         # FailsafeDelay
    119: (0.0, 0.5),         # BatteryAlarmPct (0 = disabled)
    121: (0.0, 50.0),        # MaxClimbRateMpS
    122: (0.0, 20.0),         # RudderMotorFF
123: (0.0, 15.0),        # SpiralDescentBandM (0 = spiral for any residual)
124: (0.0, 255.0),        # Unused124
	125: (0.0, 5.0),        # TraceType (U8, eTraceNone..eTraceIMU — mirrors FC ParamTable U8 enum bounds)
    126: (0.0, 255.0),        # Unused126
    127: (0.0, 7.0),          # PowerResetCause UNKNOWN..BROWNOUT
}

# FC-native limits from ParamTable — derived from the class tables above
# (raw lo, hi — GCS scales by display mult)
PARAM_LIMITS: Dict[int, Tuple[float, float]] = {
    tag: (PARAM_EXPLICIT_LIMITS[tag] if cls == "eClassExplicit"
          else PARAM_CLASS_BOUNDS[cls])
    for tag, cls in PARAM_CLASS_OF.items()
}

# Parameter type from ParamTable
PARAM_TYPES: Dict[int, str] = {
    0: 'FLOAT',
    1: 'FLOAT',
    2: 'FLOAT',
    3: 'U8',
    4: 'FLOAT',
    5: 'FLOAT',
    6: 'FLOAT',
    7: 'FLOAT',
    8: 'U8',
    9: 'FLOAT',
    10: 'FLOAT',
    11: 'FLOAT',
    12: 'U8',
    13: 'U8',
    14: 'U8',
    15: 'U8',
    16: 'U8',
    17: 'FLOAT',
    18: 'FLOAT',
    19: 'FLOAT',
    20: 'FLOAT',
    21: 'FLOAT',
    22: 'FLOAT',
    23: 'FLOAT',
    24: 'FLOAT',
    25: 'FLOAT',
    26: 'FLOAT',
    27: 'FLOAT',
    28: 'FLOAT',
    29: 'FLOAT',
    30: 'FLOAT',
    31: 'FLOAT',
    32: 'FLOAT',
    33: 'FLOAT',
    34: 'U8',
    35: 'U8',
    36: 'U8',
    37: 'U8',
    38: 'FLOAT',
    39: 'FLOAT',
    40: 'FLOAT',
    41: 'U8',
    42: 'U8',
    43: 'U8',
    44: 'U8',
    45: 'FLOAT',
    46: 'U8',
    47: 'U8',
    48: 'FLOAT',
    49: 'U8',
    50: 'U8',
    51: 'U8',
    52: 'FLOAT',
    53: 'FLOAT',
    54: 'U8',
    55: 'U8',
    56: 'FLOAT',
    57: 'FLOAT',
    58: 'FLOAT',
    59: 'U8',
    60: 'FLOAT',
    61: 'U8',
    62: 'FLOAT',
    63: 'FLOAT',
    64: 'FLOAT',
    65: 'FLOAT',
    66: 'FLOAT',
    67: 'FLOAT',
    68: 'FLOAT',
    69: 'FLOAT',
    70: 'FLOAT',
    71: 'U8',
    72: 'FLOAT',
    73: 'U8',
    74: 'FLOAT',
    75: 'U8',
    76: 'FLOAT',
    77: 'FLOAT',
    78: 'FLOAT',
    79: 'FLOAT',
    80: 'FLOAT',
    81: 'FLOAT',
    82: 'FLOAT',
    83: 'FLOAT',
    84: 'FLOAT',
    85: 'FLOAT',
    86: 'FLOAT',
    87: 'FLOAT',
    88: 'FLOAT',
    89: 'U8',
    90: 'FLOAT',
    91: 'U8',
    92: 'U8',
    93: 'U8',
    94: 'U8',
    95: 'U8',
    96: 'FLOAT',
    97: 'FLOAT',
    98: 'FLOAT',
    99: 'FLOAT',
    100: 'U8',
    101: 'FLOAT',
    102: 'FLOAT',
    103: 'FLOAT',
    104: 'U8',
    105: 'FLOAT',
    106: 'FLOAT',
    107: 'FLOAT',
    108: 'FLOAT',
     109: 'FLOAT',
    110: 'FLOAT',
    111: 'FLOAT',
    112: 'FLOAT',
    113: 'FLOAT',
    114: 'FLOAT',
    115: 'FLOAT',
    116: 'FLOAT',
    117: 'U8',
    118: 'U8',
    119: 'U8',
     120: 'FLOAT',
121: 'FLOAT',
     122: 'FLOAT',
     123: 'FLOAT',
    124: 'U8',
    125: 'U8',
    126: 'U8',
    127: 'U8',
}

# Legacy-tagged params (stored as raw float32, displayed without conversion)
LEGACY_TAGS: set = {
    0, 1, 4, 5, 6, 9, 10, 11,
    18, 23, 24, 25, 27, 28, 29, 31,
    40, 48, 56, 60, 66, 90,
    97, 98, 99, 101,
}

# FC ParamTable scale factors (display->FC conversion)
PARAM_SCALES: Dict[int, float] = {
    0: 0.005,
    1: 0.00046,
    2: 1.0,
    3: 1,
    4: 0.015,
    5: 0.005,
    6: 0.0183,
    7: 1.0,
    8: 1,
    9: 0.015,
    10: 0.005,
    11: 0.0001,
    12: 1,
    13: 1,
    14: 1,
    15: 1,
    16: 1,
    17: 1,
    18: 0.1,
    19: 0.01,
    20: 0.01,
    21: 0.01,
    22: 0.01,
    23: 0.05,
    24: 0.05,
    25: 0.1,
    26: 1,
    27: 0.0001,
    28: 0.06,
    29: 0.001,
    30: 0.01,
    31: 0.01,
    32: 1,
    33: 0.017453292519943295,
    34: 1,
    35: 1,
    36: 1,
    37: 1,
    38: 1,
    39: 0.01,
    40: 1,
    41: 1,
    42: 1,
    43: 1,
    44: 1,
    45: 1,
    46: 1,
    47: 1,
    48: 0.01,
    49: 1,
    50: 1,
    51: 1,
    52: 1,
    53: 1,
    54: 1,
    55: 1,
    56: 0.0165,
    57: 1,
    58: 0.01,
    59: 1,
    60: 0.004,
    61: 1,
    62: 1.0,
    63: 0.017453292519943295,
    64: 1,
    65: 0.01,
    66: 0.05,
    67: 0.017453292519943295,
    68: 0.017453292519943295,
    69: 0.001,
    70: 0.01,
    71: 1,
    72: 1.0,
    73: 1,
    74: 0.017453292519943295,
    75: 1,
    76: 0.017453292519943295,
    77: 1,
    78: 0.017453292519943295,
    79: 0.001,
    80: 1,
    81: 0.017453292519943295,
    82: 0.017453292519943295,
    83: 0.017453292519943295,
    84: 1.0,
    85: 1.0,
    86: 0.01,
    87: 0.01,
    88: 0.017453292519943295,
    89: 1,
    90: 2.5e-05,
    91: 1,
    92: 1,
    93: 1,
    94: 1,
    95: 1,
    96: 1.0,
    97: 0.05,
    98: 0.0008726646259971648,
    99: 0.05,
    100: 1,
    101: 0.00027,
    102: 0.01,
    103: 1,
    104: 1,
    105: 1,
    106: 1,
    107: 1,
    108: 1,
    109: 1,
    110: 1.0,
    111: 1.0,
    112: 0.01,
    113: 1,
    114: 0.01,
    115: 1,
    116: 1,
    117: 1,
    118: 1,
    119: 1,
     120: 0.001,
    121: 1,
    122: 0,
    123: 1,
    124: 1,
    125: 1,
    126: 1,
    127: 1,
}

# Boot-scoped params — consumed ONLY by FC boot-time init (RX protocol + ISR,
# inertial/attitude filter setup, ESC/PWM drive pin config, sensor init). A
# live write updates the FC RAM image and sets ConfigChanged, but the effect
# cannot appear until the next power-on; every other param applies instantly.
# Mirrors the FC ParamTable BootRequiredParameterTags list (params.c) — keep
# both in sync. When one of these is live-written while connected, the GCS
# auto-offers "Apply & Reboot" (persist + tag-72 reboot); normal edits never
# reboot. 15 (Config1Bits) and 73 (Config2Bits) are included because config
# bits are latched into F-flags / globals at boot (DoConfigBits in
# ConditionParameters) and some (e.g. HaveGPS, eUseGPS) only take effect at
# boot-time init — Greg: force the apply+reboot offer whenever any config
# bit is changed rather than relying on another boot-scoped edit.
PARAM_BOOT_REQUIRED: set = {
    8, 12, 13, 14, 15, 35, 43, 44, 47, 71, 73, 89,
}
