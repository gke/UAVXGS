"""
Complete UAVX Protocol Enums
Mirrors the C header files (params.h, telem.h, protocol_constants.h, etc.)
All enums start at 0 as per C convention
"""

from enum import IntEnum, Enum, IntFlag
from typing import Dict


# ============================================================================
# Packet Tags (from telem.h PacketTags enum)
# ============================================================================
class PacketTag(IntEnum):
    """UAVX packet tags - matches telem.h PacketTags"""
    UNKNOWN = 0
    LEV = 1
    NAV = 2
    MICROPILOT = 3
    WAY = 4
    AIRFRAME = 5
    NAV_UPDATE = 6
    BASIC = 7
    RESTART = 8
    TRIMBLE = 9
    MESSAGE = 10
    ENVIRONMENT = 11
    BEACON = 12
    # UAVX packets start at 13
    FLIGHT = 13
    NAV_UAVX = 14
    STATS = 15
    CONTROL = 16
    PARAM = 17
    MIN = 18
    ORIGIN = 19
    WP = 20
    MISSION = 21
    RC_CHANNELS = 22

    PARAM_TAGGED_READ = 71
    PARAM_COMMIT = 72
    REQUEST = 50
    ACK = 51
    MISC = 52
    NOISE = 53
    BB = 54
    INERTIAL = 55
    MINIMOSD = 56
    TUNING = 57
    UKF = 58
    GUIDANCE = 59
    ALTITUDE_CONTROL = 60
    SOARING = 61
    CALIBRATION = 62
    AFNAME = 63
    WIND = 64
    TRACK = 65
    SERIAL_PORTS = 66
    EXECUTION_TIME = 67
    ATTITUDE_CONTROL = 68
    LINK_STATS = 69
    FRSKY = 99


# ============================================================================
# MISC Commands (from telem.h MiscComms enum)
# ============================================================================
class MiscCommand(IntEnum):
    """MISC packet commands - matches telem.h MiscComms"""
    CAL_IMU = 0           # Calibrate IMU (Gyro + Accel)
    CAL_MAG = 1           # Calibrate Magnetometer
    LB = 2                # Loopback test
    UNUSED = 3            # Unused
    BB_DUMP = 4           # Dump Black Box
    GPS_PASS_THRU = 5     # GPS Pass-through
    CAL_ACC = 6           # Calibrate Accelerometer only
    CAL_GYRO = 7          # Calibrate Gyro only
    BOOTLOADER = 8        # Enter bootloader
    GPS_INIT = 9          # Force GPS re-initialization
    FORCE_DEFAULTS = 10   # Reset all params to embedded defaults
    SET_NAV_MODE = 11     # Set NavMode RC channel value: 0=Low(PIC), 1=Middle(Hold/Mission), 2=High(RTH)
    CYCLE_BB_LOG = 12     # Cycle black box log type


# ============================================================================
# Telemetry Types (from telem.h TelemetryTypes enum)
# ============================================================================
class TelemetryType(IntEnum):
    """Telemetry output types - matches telem.h TelemetryTypes"""
    UAVX_DJT = 0
    INAV_LUA = 1
    NO_TELEMETRY = 2
    U4 = 3
    U5 = 4
    U6 = 5
    U7 = 6
    U8 = 7


# ============================================================================
# In-Flight Logs (from telem.h inflightLogs enum)
# ============================================================================
class InflightLog(IntEnum):
    """In-flight log types - matches telem.h inflightLogs"""
    UAVX = 0
    ALTITUDE = 1
    PITCH = 2
    ROLL = 3
    YAW = 4


# ============================================================================
# RC Controls (from params.h RCControls enum)
# ============================================================================
class RCControl(IntEnum):
    """RC channel functions - maps to parameter values"""
    THROTTLE = 0
    ROLL = 1
    PITCH = 2
    YAW = 3
    NAV_MODE = 4
    ATTITUDE_MODE = 5
    NAV_QUALIFICATION = 6
    AUX1_CAM_PITCH = 7
    AUX2 = 8
    TRANSITION = 9
    PASS_THRU = 10
    UNUSED11 = 11
    NULL = 12


# ============================================================================
# Airframe Types (from params.h AFs enum)
# ============================================================================
class AirframeType(IntEnum):
    """Airframe types - matches params.h AFs"""
    TRI = 0
    TRI_COAX = 1          # Y6
    VTAIL = 2
    QUAD = 3
    QUAD_X = 4
    QUAD_COAX = 5         # OctoCoax
    QUAD_COAX_X = 6
    HEX = 7
    HEX_X = 8
    OCT = 9
    OCT_X = 10
    HELI_90 = 11
    BI = 12
    ELEVON = 13
    DELTA = 14
    AILERON = 15
    AILERON_SPOILER_FLAPS = 16
    AILERON_VTAIL = 17
    RUDDER_ELEVATOR = 18
    DIFFERENTIAL_TWIN = 19
    VTOL = 20
    VTOL2 = 21
    TRACKED = 22
    FOUR_WHEEL = 23
    TWO_WHEEL = 24
    GIMBAL = 25
    INSTRUMENTATION = 26
    UNKNOWN = 27


# ============================================================================
# ESC Types (from params.h)
# ============================================================================
class ESCType(IntEnum):
    """Drive / ESC types"""
    FAST_PWM = 0
    DC_MOTORS = 1
    DRIVES_DISARMED = 2


# ============================================================================
# Rx Types (from params.h RxTypes enum)
# ============================================================================
class RxType(IntEnum):
    """Receiver types"""
    CPPM = 0
    FUTABA_SBUS = 1
    SPEKTRUM_1024 = 2
    SPEKTRUM_2048 = 3
    CRSF = 4          # Crossfire/ExpressLRS
    UNKNOWN = 5


# ============================================================================
# Arming Modes (from params.h ArmingModes enum)
# ============================================================================
class ArmingMode(IntEnum):
    """Arming modes"""
    TX_ARMING = 0
    SWITCH_ARMING = 1
    UNUSED_ARMING_2 = 2
    UNUSED_ARMING_3 = 3


# ============================================================================
# GPS Protocols
# ============================================================================
class GPSProtocol(IntEnum):
    """GPS protocol types"""
    U_BLOX_M8 = 0
    U_BLOX_LEGACY = 1
    U_BLOX = 2
    NO_GPS = 3


# ============================================================================
# Rangefinder Types (from params.h RFs enum)
# ============================================================================
class RangefinderType(IntEnum):
    """Rangefinder types"""
    MAX_SONAR_CM = 0
    SRF_I2C_CM = 1
    MAX_SONAR_I2C_CM = 2
    SHARP_IR_GP2Y0A02YK = 3
    SHARP_IR_GP2Y0A710K = 4
    NO_RF = 5


# ============================================================================
# Airspeed Sensor Types (from params.h ASSensorTypes enum)
# ============================================================================
class AirspeedSensorType(IntEnum):
    """Airspeed sensor types"""
    MS4525D0_I2C = 0
    MPXV7002DP_ANALOG = 1
    AS_THERMOPILE_ANALOG = 2
    AS_GPS_DERIVED = 3
    NO_AS = 4


# ============================================================================
# IMU Filter Types
# ============================================================================
class IMUFilterType(IntEnum):
    """IMU filter types"""
    LPFILT = 0
    MPUFILT = 1
    PT1FILT = 2
    IMU_FILT_3 = 3
    IMU_FILT_4 = 4
    IMU_FILT_5 = 5


# ============================================================================
# Flight States (from protocol_constants.py)
# ============================================================================
class FlightState(IntEnum):
    """Flight state machine states"""
    STARTING = 0
    WARMUP = 1
    LANDING = 2
    LANDED = 3
    SHUTDOWN = 4
    FLYING = 5
    IREMULATE = 6
    PREFLIGHT = 7
    READY = 8
    THROTTLE_OPEN_CHECK = 9
    ERECTING_GYROS = 10
    MONITOR_INSTRUMENTS = 11
    INITIALISING_GPS = 12

    @classmethod
    def get_name(cls, state: int) -> str:
        names = {
            cls.STARTING: "Starting",
            cls.WARMUP: "Warmup",
            cls.LANDING: "Landing",
            cls.LANDED: "Landed",
            cls.SHUTDOWN: "Shutdown",
            cls.FLYING: "Flying",
            cls.IREMULATE: "IREmulate",
            cls.PREFLIGHT: "Preflight",
            cls.READY: "Ready",
            cls.THROTTLE_OPEN_CHECK: "Throttle",
            cls.ERECTING_GYROS: "Erect Gyros",
            cls.MONITOR_INSTRUMENTS: "Instrumentation",
            cls.INITIALISING_GPS: "Init GPS"
        }
        return names.get(state, f"Unknown({state})")


# ============================================================================
# Nav States (from protocol_constants.py)
# ============================================================================
class NavState(IntEnum):
    """Navigation state machine states"""
    HOLDING_STATION = 0
    RETURNING_HOME = 1
    AT_HOME = 2
    DESCENDING = 3
    TOUCHDOWN = 4
    TRANSITING = 5
    LOITERING = 6
    ORBITING_POI = 7
    PERCHING = 8
    TAKEOFF = 9
    PIC = 10
    ACQUIRING_ALTITUDE = 11
    USING_THERMAL = 12
    USING_RIDGE = 13
    USING_WAVE = 14
    BOOST_CLIMB = 15
    ALTITUDE_LIMITING = 16
    JUST_GLIDING = 17
    WP_ALT_FAIL = 18
    WP_PROXIMITY_FAIL = 19

    @classmethod
    def get_name(cls, state: int) -> str:
        names = {
            cls.HOLDING_STATION: "Holding",
            cls.RETURNING_HOME: "Returning",
            cls.AT_HOME: "@Home",
            cls.DESCENDING: "Descending",
            cls.TOUCHDOWN: "Touchdown",
            cls.TRANSITING: "Transiting",
            cls.LOITERING: "Loitering",
            cls.ORBITING_POI: "Orbiting",
            cls.PERCHING: "Perching",
            cls.TAKEOFF: "Takeoff",
            cls.PIC: "PIC",
            cls.ACQUIRING_ALTITUDE: "Acquire Alt",
            cls.USING_THERMAL: "Thermal",
            cls.USING_RIDGE: "Ridge",
            cls.USING_WAVE: "Wave",
            cls.BOOST_CLIMB: "Boost",
            cls.ALTITUDE_LIMITING: "Alt Lim",
            cls.JUST_GLIDING: "Gliding",
            cls.WP_ALT_FAIL: "ALTFAIL",
            cls.WP_PROXIMITY_FAIL: "PROXFAIL"
        }
        return names.get(state, f"Unknown({state})")


# ============================================================================
# Config Bit Masks (from params.h)
# ============================================================================
class Config1Bits(IntFlag):
    """Config1 bit masks - matches params.h Config1"""
    USE_INVERT_MAG = 0x01           # bit 0
    USE_RTH_DESCEND = 0x02          # bit 1
    DISABLE_LEDS_IN_FLIGHT = 0x04   # bit 2
    EMULATION_ENABLE = 0x08         # bit 3
    USE_ALT_HOLD_ALARM = 0x10       # bit 4
    USE_OFFSET_HOME = 0x20          # bit 5
    USE_RAPID_DESCENT = 0x40        # bit 6
    ENFORCE_DRIVE_SYMMETRY = 0x80   # bit 7

    # Default Config1 = EmulationEnableMask | EnforceDriveSymmetryMask
    DEFAULT = EMULATION_ENABLE | ENFORCE_DRIVE_SYMMETRY


class Config2Bits(IntFlag):
    """Config2 bit masks - matches params.h Config2"""
    USE_BATTERY_COMP = 0x01         # bit 0
    USE_FAST_START = 0x02           # bit 1
    USE_BLHELI = 0x04               # bit 2
    USE_GLIDER_STRATEGY = 0x08      # bit 3
    USE_PROP_SENSE = 0x10           # bit 4
    USE_TURN_TO_WP = 0x20           # bit 5
    USE_NAV_BEEP = 0x40             # bit 6
    # bit 7 unusable

    # Default Config2 = UseFastStartMask
    DEFAULT = USE_FAST_START


# ============================================================================
# Flag Bit Positions (from main_window.py FLAG_* constants)
# ============================================================================
class FlagBits(IntEnum):
    """Flight flag bit positions"""
    ALT_HOLD = 0
    GPS_ALT = 1
    VRS = 2
    LAND_SW = 3
    LEVEL = 4
    LOW_BATT = 5
    GPS_OK = 6
    ORIGIN = 7
    BARO_FAIL = 8
    IMU_FAIL = 9
    MAG_FAIL = 10
    GPS_FAIL = 11
    ATT_HOLD = 12
    THR_MOVE = 13
    HOLDING_ALT = 14
    NAV = 15
    RTH = 16
    WP_ACH = 17
    WP_CENT = 18
    ORBIT = 19
    AUTO_LAND = 20
    BARO = 21
    RF = 22
    USE_RF = 23
    POI = 24
    PASS_THRU = 25
    ANGLE = 26
    EMUL = 27
    OFFSET = 28
    ARMED = 29
    ACC_Z_BUMP = 30
    DC_MOTORS = 31
    SAT = 32
    DUMP_BB = 33
    PARAM = 34
    SIGNAL = 35
    WP_NAV = 36
    IMU = 37
    MAG = 38
    ARMED_2 = 39
    FIXED_WING = 40
    THR_OPEN = 41
    MAG_CAL = 42
    RC_MAP_FAIL = 43
    NEW_ALT = 44
    IMU_CAL = 45
    FENCE_ALARM = 46


# ============================================================================
# Parameter Indices (from params.h Params enum)
# ============================================================================
class ParamIndex(IntEnum):
    """Parameter indices - matches params.h Params enum exactly"""
    ROLL_RATE_KP = 0           # 01
    ALT_POS_KI = 1             # 02
    ROLL_ANGLE_KP = 2          # 03
    ARMING_MODE = 3            # 04
    ROLL_ANGLE_INT_LIMIT = 4   # 05
    PITCH_RATE_KP = 5          # 06
    ALT_POS_KP = 6             # 07
    PITCH_ANGLE_KP = 7         # 08
    RF_SENSOR_TYPE = 8         # 09
    PITCH_ANGLE_INT_LIMIT = 9  # 10
    YAW_RATE_KP = 10           # 11
    ROLL_RATE_KD = 11          # 12
    IMU_FILT_TYPE = 12         # 13
    BB_LOG_TYPE = 13           # 14
    RX_TYPE = 14               # 15
    CONFIG1_BITS = 15          # 16
    RX_THROTTLE_CH = 16        # 17
    LOW_VOLT_THRES = 17        # 18
    ROLL_CAM_KP = 18           # 19
    EST_CRUISE_THR = 19        # 20
    STICK_HYSTERESIS = 20      # 21
    FW_CLIMB_THROTTLE = 21     # 22
    PERCENT_IDLE_THR = 22      # 23
    ROLL_ANGLE_KI = 23         # 24
    PITCH_ANGLE_KI = 24        # 25
    PITCH_CAM_KP = 25          # 26
    SERVO_LPF_HZ = 26          # 27
    PITCH_RATE_KD = 27         # 28
    NAV_VEL_KP = 28            # 29
    UNUSED_ALT_VEL_KP = 29     # 30
    HORIZON = 30               # 31
    MADGWICK_KP_MAG = 31       # 32
    NAV_RTH_ALT = 32           # 33
    NAV_MAG_VAR = 33           # 34
    UNUSED_SENSOR_HINT = 34    # 35
    ESC_TYPE = 35              # 36
    RC_CHANNELS = 36           # 37
    RX_ROLL_CH = 37            # 38
    MADGWICK_KP_ACC = 38       # 39
    ROLL_CAM_TRIM = 39         # 40
    NAV_POS_INT_LIMIT = 40     # 41
    RX_PITCH_CH = 41           # 42
    RX_YAW_CH = 42             # 43
    AF_TYPE = 43               # 44
    TELEMETRY_TYPE = 44        # 45
    MAX_DESCENT_RATE_DMP_S = 45 # 46
    DESCENT_DELAY_S = 46       # 47
    GYRO_LPF_SEL = 47          # 48
    NAV_CROSS_TRACK_KP = 48    # 49
    RX_GEAR_CH = 49            # 50
    RX_AUX1_CH = 50            # 51 - RCMap1
    SERVO_SENSE = 51           # 52 - RCMap2
    ACC_CONF_SD = 52           # 53
    BATTERY_CAPACITY = 53      # 54 (0.1 AH)
    RX_AUX2_CH = 54            # 55 - RCMap3
    RX_AUX3_CH = 55            # 56 - RCMap4
    NAV_POS_KP = 56            # 57
    ALT_LPF = 57               # 58
    BALANCE = 58               # 59
    RX_AUX4_CH = 59            # 60 - RCMap5
    NAV_POS_KI = 60            # 61
    UNUSED_GPS_PROTOCOL = 61   # 62 (auto-detected)
    TILT_THROTTLE_FF = 62      # 63
    MAX_YAW_RATE = 63          # 64
    FW_ROLL_PITCH_FF = 64      # 65
    FW_PITCH_THROTTLE_FF = 65  # 66
    UNUSED_ALT_VEL_INT_LIMIT = 66  # 67
    FW_MAX_CLIMB_ANGLE = 67    # 68
    NAV_MAX_ANGLE = 68         # 69
    FW_SPOILER_DECAY_PERCENT_PS = 69  # 70
    FW_AILERON_DIFFERENTIAL = 70       # 71
    AS_SENSOR_TYPE = 71        # 72
    KF_ACC_U_BIAS_VAR = 72     # 73
    CONFIG2_BITS = 73          # 74
    MAX_PITCH_ANGLE = 74       # 75
    QUAT_GAIN = 75            # 76
    MAX_ROLL_ANGLE = 76        # 77
    YAW_LPF_HZ = 77            # 78
    NAV_HEADING_TURNOUT = 78   # 79
    ALT_HOLD_THR_COMP_DECAY_PERCENT_PS = 79  # 80
    YAW_SYMMETRY_FACTOR = 80       # 81
    FW_BOARD_PITCH_ANGLE = 81  # 82
    MAX_ROLL_RATE = 82         # 83
    MAX_PITCH_RATE = 83        # 84
    CURRENT_SCALE = 84          # 85
    VOLT_SCALE = 85             # 86
    FW_AILERON_RUDDER_MIX = 86 # 87
    FW_ALT_SPOILER_FF = 87     # 88
    MAX_COMPASS_YAW_RATE = 88  # 89
    ACC_LPF_SEL = 89           # 90
    YAW_RATE_KD = 90           # 91
    UNUSED_92 = 91             # 92
    THROTTLE_GAIN_RATE = 92    # 93
    RX_AUX5_CH = 93            # 94 - RCMap6
    RX_AUX6_CH = 94            # 95 - RCMap7
    RX_AUX7_CH = 95            # 96 - RCMap8
    YAW_ANGLE_KP = 96          # 97
    YAW_ANGLE_KI = 97          # 98
    YAW_ANGLE_INT_LIMIT = 98   # 99
    ALT_POS_INT_LIMIT = 99     # 100
    MOTOR_STOP_SEL = 100       # 101
    UNUSED_ALT_VEL_KI = 101    # 102
    ALT_THROTTLE_COMP_LIMIT = 102  # 103
    VRS_ROC = 103     # 104
    UNUSED_105 = 104           # 105
    AH_ROC_WINDOW_MPS = 105    # 106
    NAV_PROX_ALT_M = 106       # 107
    NAV_PROX_RADIUS_M = 107    # 108
    UNUSED_109 = 108           # 109
    UNUSED_110 = 109           # 110
    KF_BARO_VAR = 110          # 111
    KF_ACC_U_VAR = 111         # 112
    FW_STICK_SCALE = 112       # 113
    FW_ROLL_CONTROL_PITCH_LIMIT = 113  # 114
    AH_THROTTLE_MOVING_TRIGGER = 114   # 115
    NAV_FENCE_RADIUS_M = 115   # 116
    DIVE_RECOVER_ALT = 116     # 117 — auto pull-out altitude AGL (m)
    UNUSED_118 = 117           # 118
    UNUSED_ALT_ROC_KP = 118    # 119
    UNUSED_120 = 119           # 120
    UNUSED_121 = 120           # 121
    UNUSED_122 = 121           # 122
    UNUSED_123 = 122           # 123
    UNUSED_124 = 123           # 124
    UNUSED_125 = 124           # 125
    UNUSED_126 = 125           # 126
    UNUSED_127 = 126           # 127
    POWER_RESET_CAUSE = 127    # 128


# ============================================================================
# RC Map Constants
# ============================================================================
# RC Map indices (the parameters that define RC channel mapping)
RC_MAP_INDICES = [
    ParamIndex.RX_AUX1_CH,    # 50 - RCMap1 (Throttle)
    ParamIndex.SERVO_SENSE,   # 51 - RCMap2 (Roll)
    ParamIndex.RX_AUX3_CH,    # 55 - RCMap3 (Pitch)
    ParamIndex.NAV_POS_KP,    # 56 - RCMap4 (Yaw)
    ParamIndex.NAV_POS_KI,    # 60 - RCMap5 (Gear)
    ParamIndex.RX_AUX6_CH,    # 94 - RCMap6 (Aux1)
    ParamIndex.RX_AUX7_CH,    # 95 - RCMap7 (Aux2)
    ParamIndex.YAW_ANGLE_KP,  # 96 - RCMap8 (Aux3)
]

# Expected RC Map values (0-7 in order, matching RCControl enum)
RC_MAP_EXPECTED = [
    RCControl.THROTTLE,
    RCControl.ROLL,
    RCControl.PITCH,
    RCControl.YAW,
    RCControl.NAV_MODE,
    RCControl.ATTITUDE_MODE,
    RCControl.NAV_QUALIFICATION,
    RCControl.AUX1_CAM_PITCH,
]

# RC Mapping: param index -> function name (for display)
RC_MAPPING_NAMES = {
    ParamIndex.RX_THROTTLE_CH: "Thr",
    ParamIndex.RX_ROLL_CH: "Roll",
    ParamIndex.RX_PITCH_CH: "Pitch",
    ParamIndex.RX_YAW_CH: "Yaw",
    ParamIndex.RX_GEAR_CH: "Gear",
    ParamIndex.RX_AUX1_CH: "Aux1",
    ParamIndex.RX_AUX2_CH: "Aux2",
    ParamIndex.RX_AUX3_CH: "Aux3",
    ParamIndex.RX_AUX4_CH: "Aux4",
    ParamIndex.RX_AUX5_CH: "Aux5",
    ParamIndex.RX_AUX6_CH: "Aux6",
    ParamIndex.RX_AUX7_CH: "Aux7",
}


# ============================================================================
# Helper Functions
# ============================================================================
def get_param_name(index: int) -> str:
    """Get parameter name from index"""
    try:
        return ParamIndex(index).name
    except ValueError:
        return f"UNKNOWN_{index}"


def get_packet_name(tag: int) -> str:
    """Get packet name from tag"""
    try:
        return PacketTag(tag).name
    except ValueError:
        return f"UNKNOWN_{tag}"


def get_airframe_name(af_type: int) -> str:
    """Get airframe name from type"""
    try:
        return AirframeType(af_type).name
    except ValueError:
        return f"UNKNOWN_{af_type}"


# ============================================================================
# Type Mapping Dictionaries (for UI display)
# ============================================================================
AIRFRAME_NAMES = {
    AirframeType.TRI: "Tricopter",
    AirframeType.TRI_COAX: "CoaxTri Y6",
    AirframeType.VTAIL: "VTail Y4",
    AirframeType.QUAD: "Quadcopter",
    AirframeType.QUAD_X: "X Quadcopter",
    AirframeType.QUAD_COAX: "Coax Quad",
    AirframeType.QUAD_COAX_X: "X Coax Quad",
    AirframeType.HEX: "Hexacopter",
    AirframeType.HEX_X: "X Hexacopter",
    AirframeType.OCT: "Octocopter",
    AirframeType.OCT_X: "X Octocopter",
    AirframeType.HELI_90: "Heli90",
    AirframeType.BI: "Heli120",
    AirframeType.ELEVON: "Flying Wing",
    AirframeType.DELTA: "Delta",
    AirframeType.AILERON: "Aileron",
    AirframeType.AILERON_SPOILER_FLAPS: "Spoilerons",
    AirframeType.AILERON_VTAIL: "VTail",
    AirframeType.RUDDER_ELEVATOR: "Rudder Elevator",
    AirframeType.DIFFERENTIAL_TWIN: "Differential Twin",
    AirframeType.VTOL: "VTOL",
    AirframeType.VTOL2: "VTOL2",
    AirframeType.TRACKED: "Tracked",
    AirframeType.FOUR_WHEEL: "Four Wheel",
    AirframeType.TWO_WHEEL: "Two Wheel",
    AirframeType.GIMBAL: "Gimbal",
    AirframeType.INSTRUMENTATION: "Instrumentation",
    AirframeType.UNKNOWN: "Unknown Aircraft",
}

# Airframe types marked as redacted (legacy/rarely used) — matches FC RedactedAF[28]
REDACTED_AIRFRAMES = frozenset({
    AirframeType.TRI,              # 0
    AirframeType.TRI_COAX,         # 1
    AirframeType.VTAIL,            # 2
    AirframeType.QUAD,             # 3
    AirframeType.QUAD_COAX,        # 5
    AirframeType.QUAD_COAX_X,      # 6
    AirframeType.HEX,              # 7
    AirframeType.OCT,              # 9
    AirframeType.HELI_90,          # 11
    AirframeType.BI,               # 12
    AirframeType.ELEVON,           # 13
    AirframeType.AILERON,          # 15
    AirframeType.AILERON_VTAIL,    # 17
    AirframeType.VTOL,             # 20
    AirframeType.VTOL2,            # 21
    AirframeType.FOUR_WHEEL,       # 23
    AirframeType.TWO_WHEEL,        # 24
    AirframeType.UNKNOWN,          # 27
})

# Active airframe types (non-redacted) — for pulldown display
# Derived from REDACTED_AIRFRAMES — single source of truth
ACTIVE_AIRFRAMES = frozenset({
    af for af in AirframeType
    if af not in REDACTED_AIRFRAMES and af != AirframeType.UNKNOWN
})

ESC_TYPE_NAMES = {
    ESCType.FAST_PWM: "FastPWM",
    ESCType.DC_MOTORS: "DCMotors",
    ESCType.DRIVES_DISARMED: "MotorsOff",
}

RX_TYPE_NAMES = {
    RxType.CPPM: "CPPM",
    RxType.FUTABA_SBUS: "SBus",
    RxType.SPEKTRUM_1024: "Spektrum 1024",
    RxType.SPEKTRUM_2048: "Spektrum 2048",
    RxType.CRSF: "CRFS/ExpressLRS",
    RxType.UNKNOWN: "Unknown Rx",
}

ARMING_MODE_NAMES = {
    ArmingMode.TX_ARMING: "Tx Arming",
    ArmingMode.SWITCH_ARMING: "Sw Arming",
    ArmingMode.UNUSED_ARMING_2: "u2",
    ArmingMode.UNUSED_ARMING_3: "u3",
}

TELEMETRY_TYPE_NAMES = {
    TelemetryType.UAVX_DJT: "DJT Decoder",
    TelemetryType.INAV_LUA: "LUA Script",
    TelemetryType.NO_TELEMETRY: "No Telemetry",
    TelemetryType.U4: "u4",
    TelemetryType.U5: "u5",
    TelemetryType.U6: "u6",
    TelemetryType.U7: "u7",
    TelemetryType.U8: "u8",
}

GPS_PROTOCOL_NAMES = {
    GPSProtocol.U_BLOX_M8: "u-blox M8",
    GPSProtocol.U_BLOX_LEGACY: "u-blox Legacy",
    GPSProtocol.U_BLOX: "u-blox",

    GPSProtocol.NO_GPS: "No GPS",
}

RF_TYPE_NAMES = {
    RangefinderType.MAX_SONAR_CM: "MaxSonar cm",
    RangefinderType.SRF_I2C_CM: "SRF I2C cm",
    RangefinderType.MAX_SONAR_I2C_CM: "MaxSonar I2C cm",
    RangefinderType.SHARP_IR_GP2Y0A02YK: "Sharp IR GP2Y0A02YK",
    RangefinderType.SHARP_IR_GP2Y0A710K: "Sharp IR GP2Y0A710K",
    RangefinderType.NO_RF: "No RF",
}

AS_SENSOR_TYPE_NAMES = {
    AirspeedSensorType.MS4525D0_I2C: "MPXV7002DP",
    AirspeedSensorType.MPXV7002DP_ANALOG: "MS425D0 (I2C)",
    AirspeedSensorType.AS_THERMOPILE_ANALOG: "Thermopile",
    AirspeedSensorType.AS_GPS_DERIVED: "GPS Derived",
    AirspeedSensorType.NO_AS: "No Airspeed",
}

IMU_FILTER_NAMES = {
    IMUFilterType.LPFILT: "LPFilt",
    IMUFilterType.MPUFILT: "MPUFilt",
    IMUFilterType.PT1FILT: "PT1Filt",
    IMUFilterType.IMU_FILT_3: "F3",
    IMUFilterType.IMU_FILT_4: "F4",
    IMUFilterType.IMU_FILT_5: "F5",
}

GYRO_LPF_NAMES = {
    0: "250",
    1: "184",
    2: "98",
    3: "41",
    4: "20",
    5: "10",
    6: "5",
}

ACC_LPF_NAMES = {
    0: "480",
    1: "184",
    2: "92",
    3: "41",
    4: "20",
    5: "10",
    6: "5",
}

BB_LOG_NAMES = {
    0: "Off",
    1: "Altitude",
    2: "Pitch",
    3: "Roll",
    4: "Yaw",
    5: "Attitude",
}

MOTOR_STOP_NAMES = {
    0: "No Stop",
    1: "Contact Switch",
    2: "Descent Rate",
    3: "Accel Bump",
    4: "Rate + Bump",
}
