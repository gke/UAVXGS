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
    """UAVX packet tags - matches telem.h PacketTags (reserved slots kept)"""
    UNKNOWN = 0
    UNUSED_LEV = 1
    UNUSED_NAV = 2
    UNUSED_MICROPILOT = 3
    UNUSED_WAY = 4
    UNUSED_AIRFRAME = 5
    UNUSED_NAV_UPDATE = 6
    UNUSED_BASIC = 7
    UNUSED_RESTART = 8
    UNUSED_TRIMBLE = 9
    UNUSED_MESSAGE = 10
    UNUSED_ENVIRONMENT = 11
    UNUSED_BEACON = 12
    # UAVX packets start at 13
    FLIGHT = 13
    NAV_UAVX = 14
    UNUSED_STATS = 15
    CONTROL = 16
    PARAM = 17
    MIN = 18
    ORIGIN = 19
    WP = 20
    UNUSED_MISSION = 21
    RC_CHANNELS = 22

    PARAM_TAGGED_READ = 71
    PARAM_COMMIT = 72
    TEST_REQUEST = 73
    TEST_RESPONSE = 74
    AFNAME_SET = 75
    I2C_ERRORS = 76
    REQUEST = 50
    ACK = 51
    MISC = 52
    UNUSED_NOISE = 53
    BB = 54
    UNUSED_INERTIAL = 55
    UNUSED_MINIMOSD = 56
    TUNING = 57
    UNUSED_UKF = 58
    GUIDANCE = 59
    UNUSED_ALTITUDE_CONTROL = 60
    UNUSED_SOARING = 61
    CALIBRATION = 62
    AFNAME = 63
    WIND = 64
    UNUSED_TRACK = 65
    SERIAL_PORTS = 66
    EXECUTION_TIME = 67
    UNUSED_ATTITUDE_CONTROL = 68
    LINK_STATS = 69
    INIT_STATE = 70
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
    GPS_PASS_THRU_UNUSED = 5   # Slot retained: GPS pass-through removed
    CAL_ACC = 6           # Calibrate Accelerometer only
    CAL_GYRO = 7          # Calibrate Gyro only
    BOOTLOADER = 8        # Enter bootloader
    # Slot 9 = GPS_INIT reserved (removed: blocked the FC main loop ~9.4s → IWDG reset)
    FORCE_DEFAULTS = 10   # Reset all params to embedded defaults
    SET_NAV_MODE = 11     # Set NavMode RC channel value: 0=Low(PIC), 1=Middle(Hold/Mission), 2=High(RTH)
    CYCLE_BB_LOG = 12     # Cycle black box log type
    ESC_PROG = 13         # Enter ESC programming (4way/MSP runtime feed-through)


# ============================================================================
# Telemetry Types (from telem.h TelemetryTypes enum)
# ============================================================================
class TelemetryType(IntEnum):
    """Reserved telemetry slots - matches telem.h TelemetryTypes (param 44)"""
    # Telemetry is now auto-selected by RxType: CRSF -> CRSF telemetry (no
    # entry here), CPPM -> D8 in fixed DJT format (iNavLUA-over-D8 retired).
    # This enum is a reserved param slot only, kept for .af/flash index
    # stability. UAVX_DJT/INAV_LUA are always D8; NoTelemetry=2 is the fallback.
    eUAVXDJTTelemetry = 0
    eINavLUATelemetry = 1
    eNoTelemetry = 2
    eU4Telemetry = 3
    eU5Telemetry = 4
    eU6Telemetry = 5
    eU7Telemetry = 6
    eU8Telemetry = 7


# ============================================================================
# In-Flight Logs (from telem.h inflightLogs enum)
# ============================================================================
class InflightLog(IntEnum):
    """In-flight log types - matches telem.h inflightLogs"""
    eLogUAVX = 0
    eLogAltitude = 1
    eLogPitch = 2
    eLogRoll = 3
    eLogYaw = 4


# ============================================================================
# RC Controls (from params.h RCControls enum)
# ============================================================================
class RCControl(IntEnum):
    """RC channel functions - maps to parameter values"""
    eThrottleRC = 0
    eRollRC = 1
    ePitchRC = 2
    eYawRC = 3
    eArmingRC = 4
    eAttitudeModeRC = 5
    eNavModeRC = 6
    ePassThruRC = 7
    eDiveRC = 8
    eTraceRC = 9
    eTransitionRC = 10
    eAux1CamPitchRC = 11
    eNullRC = 12


# ============================================================================
# Airframe Types (from params.h AFs enum)
# ============================================================================
class AirframeType(IntEnum):
    """Airframe types - matches params.h AFs"""
    eTriAF = 0
    eTriCoaxAF = 1          # Y6
    eVTailAF = 2
    eQuadAF = 3
    eQuadXAF = 4
    eQuadCoaxAF = 5         # OctoCoax
    eQuadCoaxXAF = 6
    eHexAF = 7
    eHexXAF = 8
    eOctAF = 9
    eOctXAF = 10
    eHeli90AF = 11
    eBiAF = 12
    eElevonAF = 13
    eDeltaAF = 14
    eAileronAF = 15
    eAileronSpoilerFlapsAF = 16
    eAileronVTailAF = 17
    eRudderElevatorAF = 18
    eDifferentialTwinAF = 19
    eVTOLAF = 20
    eVTOL2AF = 21
    eTrackedAF = 22
    eFourWheelAF = 23
    eTwoWheelAF = 24
    eInstrumentation = 25
    eAFUnknown = 26


# ============================================================================
# Airframe Category (single authority — mirror of FC ClassifyAFType())
# ============================================================================
# params.h AirframeCategory enum (eCatMr..eCatLand) and ClassifyAFType() are
# the ORIGINAL truth on the FC.  Every GCS Python consumer of the AF->category
# mapping MUST use AIRFRAME_CATEGORY / category_of() below instead of defining
# its own map — drift between mirrors already silently misclassified
# eAileronVTailAF as a multirotor in the sim (test_pid_sim AF_CATEGORY omitted
# it).  Anything not listed here falls to eCatMr by construction (the FC's
# else branch).
class AirframeCategory(IntEnum):
    """Matches params.h AirframeCategory enum."""
    eCatMr = 0   # multirotor
    eCatFw = 1   # fixed wing
    eCatVtol = 2 # VTOL / quadplane
    eCatLand = 3 # ground vehicles


# AF type -> category, exactly mirroring FC ClassifyAFType() (params.c).
AIRFRAME_CATEGORY = {
    AirframeType.eElevonAF: AirframeCategory.eCatFw,
    AirframeType.eDeltaAF: AirframeCategory.eCatFw,
    AirframeType.eAileronAF: AirframeCategory.eCatFw,
    AirframeType.eAileronSpoilerFlapsAF: AirframeCategory.eCatFw,
    AirframeType.eAileronVTailAF: AirframeCategory.eCatFw,
    AirframeType.eRudderElevatorAF: AirframeCategory.eCatFw,
    AirframeType.eVTOLAF: AirframeCategory.eCatVtol,
    AirframeType.eVTOL2AF: AirframeCategory.eCatVtol,
    AirframeType.eTrackedAF: AirframeCategory.eCatLand,
    AirframeType.eFourWheelAF: AirframeCategory.eCatLand,
    AirframeType.eTwoWheelAF: AirframeCategory.eCatLand,
}


def category_of(af_type):
    """Return the AirframeCategory for an AF (enum value or int).

    Mirror of FC ClassifyAFType() in params.c — anything unknown/illegal is
    eCatMr (the FC else branch), never an exception.
    """
    try:
        af = AirframeType(af_type)
    except (ValueError, TypeError):
        return AirframeCategory.eCatMr
    return AIRFRAME_CATEGORY.get(af, AirframeCategory.eCatMr)


# ============================================================================
# ESC Types (from params.h)
# ============================================================================
class ESCType(IntEnum):
    """Drive / ESC types"""
    eESCPWM = 0
    eDCMotors = 1
    eMotorsOff = 2


# ============================================================================
# Rx Types (from params.h RxTypes enum)
# ============================================================================
class RxType(IntEnum):
    """Receiver types"""
    eCPPMRx = 0
    eFutabaSBusRx = 1
    eSpektrum1024Rx = 2
    eSpektrum2048Rx = 3
    eCRSFRx = 4          # CRSF/ExpressLRS
    eUnknownRx = 5


# ============================================================================
# Arming Modes (from params.h ArmingModes enum)
# ============================================================================
class ArmingMode(IntEnum):
    """Arming modes"""
    eTxArming = 0
    eSwitchArming = 1
    eUnusedArming2 = 2
    eUnusedArming3 = 3


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
    eMaxSonarcm = 0
    eSRFI2Ccm = 1
    eMaxSonarI2Ccm = 2
    eSharpIRGP2Y0A02YK = 3
    eSharpIRGP2Y0A710K = 4
    eNoRF = 5


# ============================================================================
# Airspeed Sensor Types (from params.h ASSensorTypes enum)
# ============================================================================
class AirspeedSensorType(IntEnum):
    """Airspeed sensor types"""
    eMS4525D0I2C = 0
    eMPXV7002DPAnalog = 1
    eASThermopileAnalog = 2
    eASGPSDerived = 3
    eNoAS = 4


# ============================================================================
# IMU Filter Types
# ============================================================================
class IMUFilterType(IntEnum):
    """IMU filter types"""
    eLP2Filt = 0
    eHDLPFilt = 1
    eF1 = 2
    eF2 = 3
    eF3 = 4
    eF4 = 5


# ============================================================================
# Flight States (from protocol_constants.py)
# ============================================================================
class FlightState(IntEnum):
    """Flight state machine states"""
    eStarting = 0
    eWarmup = 1
    eLanding = 2
    eLanded = 3
    eShutdown = 4
    eInFlight = 5
    eIREmulateUNUSED = 6
    ePreflight = 7
    eReady = 8
    eThrottleOpenCheck = 9
    eErectingGyros = 10
    eMonitorInstruments = 11
    eInitialisingGPS = 12
    eStepClocks = 13
    eStepMisc = 14
    eStepParams = 15
    eStepHarness = 16
    eStepOLED = 17
    eStepBattery = 18
    eStepLEDs = 19
    eStepIMU = 20
    eStepMag = 21
    eStepMadgwick = 22
    eStepAlt = 23
    eStepNVMem = 24
    eStepTemp = 25
    eStepCtrl = 26
    eStepNav = 27
    eStepEmu = 28
    eStepSPI = 29
    eStepESCProg = 30

    @classmethod
    def get_name(cls, state: int) -> str:
        names = {
            cls.eStarting: "Starting",
            cls.eWarmup: "Warmup",
            cls.eLanding: "Landing",
            cls.eLanded: "Landed",
            cls.eShutdown: "Shutdown",
            cls.eInFlight: "Flying",
            cls.eIREmulateUNUSED: "IREmulate",
            cls.ePreflight: "Preflight",
            cls.eReady: "Ready",
            cls.eThrottleOpenCheck: "Throttle",
            cls.eErectingGyros: "Erect Gyros",
            cls.eMonitorInstruments: "Instrumentation",
            cls.eInitialisingGPS: "Init GPS",
            cls.eStepClocks: "Init Clocks",
            cls.eStepMisc: "Init Misc",
            cls.eStepParams: "Load Params",
            cls.eStepHarness: "Init Harness",
            cls.eStepOLED: "Init OLED",
            cls.eStepBattery: "Init Battery",
            cls.eStepLEDs: "Init LEDs",
            cls.eStepIMU: "Init IMU",
            cls.eStepMag: "Init Mag",
            cls.eStepMadgwick: "Init Madgwick",
            cls.eStepAlt: "Init Alt",
            cls.eStepNVMem: "Init NVMem",
            cls.eStepTemp: "Init Temp",
            cls.eStepCtrl: "Init Ctrl",
            cls.eStepNav: "Init Nav",
            cls.eStepEmu: "Init Emu",
            cls.eStepSPI: "Init SPI",
            cls.eStepESCProg: "Init ESC Prog",
        }
        return names.get(state, f"Unknown({state})")


# ============================================================================
# Nav States (from protocol_constants.py)
# ============================================================================
class NavState(IntEnum):
    """Navigation state machine states"""
    eHoldingStation = 0
    eReturningHome = 1
    eAtHome = 2
    eDescending = 3
    eTouchdown = 4
    eTransiting = 5
    eLoitering = 6
    eOrbitingPOI = 7
    ePerching = 8
    eTakeoff = 9
    ePIC = 10
    eAcquiringAltitude = 11
    eUsingThermal = 12
    eUsingRidge = 13
    eUsingWave = 14
    eBoostClimb = 15
    eAltitudeLimiting = 16
    eJustGliding = 17
    eWPAltFail = 18
    eWPProximityFail = 19

    @classmethod
    def get_name(cls, state: int) -> str:
        names = {
            cls.eHoldingStation: "Holding",
            cls.eReturningHome: "Returning",
            cls.eAtHome: "@Home",
            cls.eDescending: "Descending",
            cls.eTouchdown: "Touchdown",
            cls.eTransiting: "Transiting",
            cls.eLoitering: "Loitering",
            cls.eOrbitingPOI: "Orbiting",
            cls.ePerching: "Perching",
            cls.eTakeoff: "Takeoff",
            cls.ePIC: "PIC",
            cls.eAcquiringAltitude: "Acquire Alt",
            cls.eUsingThermal: "Thermal",
            cls.eUsingRidge: "Ridge",
            cls.eUsingWave: "Wave",
            cls.eBoostClimb: "Boost",
            cls.eAltitudeLimiting: "Alt Lim",
            cls.eJustGliding: "Gliding",
            cls.eWPAltFail: "ALTFAIL",
            cls.eWPProximityFail: "PROXFAIL"
        }
        return names.get(state, f"Unknown({state})")


# ============================================================================
# Config Bit Masks (from params.h)
# ============================================================================
class Config1Bits(IntFlag):
    """Config1 bit masks - matches params.h Config1"""
    eUseInvertMag = 0x01           # bit 0
    eUseRTHDescend = 0x02          # bit 1
    eUsingMag = 0x04              # bit 2
    eEmulationEnable = 0x08         # bit 3
    eUseAltHoldAlarm = 0x10       # bit 4
    eUseOffsetHome = 0x20          # bit 5
    eEnforceDriveSymmetry = 0x80   # bit 7

    # bit 6 — FREE (was Test WP mission, folded into emulation 2026-09-23:
    #   F.Emulation implies the 4-WP test mission). eTestMission kept as an
    #   alias so legacy .af files still load without clobbering the bit.
    eUnused1_6 = 0x40
    eTestMission = eUnused1_6      # legacy .af token — do not re-use

    # Default Config1 = FC DEFAULT_CONFIG1 (params.c): RTHDescend|AltHoldAlarm|Mag
    DEFAULT = eUseRTHDescend | eUseAltHoldAlarm | eUsingMag


class Config2Bits(IntFlag):
    """Config2 bit masks - matches params.h Config2"""
    eUseBatteryComp = 0x01         # bit 0
    eUseFastStart = 0x02           # bit 1
    eUnused2_2 = 0x04              # bit 2 — FREE (was UseESCProg). ESC-Prog is now
                               # runtime-triggered via the GCS ESC button (miscESCProg),
                               # so it needs no config bit. Available for a future feature.
    eUseGPS = 0x08             # bit 3
    ePropsInwards = 0x10           # bit 4 — SET = front two props rotate INWARDS (usual convention)
    eUseTurnToWP = 0x20           # bit 5
    eUseNavBeep = 0x40             # bit 6
    eUnused2_7 = 0x80              # bit 7 — FREE. Available for a future feature.

    # Default Config2 = FC DEFAULT_CONFIG2 (params.c): BattComp|FastStart|GPS|NavBeep
    DEFAULT = eUseBatteryComp | eUseFastStart | eUseGPS | eUseNavBeep


# ============================================================================
# Failsafe Action Enum (from params.h FailsafeActions)
# ============================================================================
class FailsafeAction(IntEnum):
    """Failsafe action on RC loss"""
    eFsRth = 0
    eFsLand = 1
    eFsMotorsOff = 2


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
    ROLL_ANGLE_Q_KP = 2          # 03
    ARMING_MODE = 3            # 04
    ROLL_ANGLE_Q_INT_LIMIT = 4   # 05
    PITCH_RATE_KP = 5          # 06
    ALT_POS_KP = 6             # 07
    PITCH_ANGLE_Q_KP = 7         # 08
    RF_SENSOR_TYPE = 8         # 09
    PITCH_ANGLE_Q_INT_LIMIT = 9  # 10
    YAW_RATE_KP = 10           # 11
    ROLL_RATE_KD = 11          # 12
    IMU_FILT_TYPE = 12         # 13
    BB_LOG_TYPE = 13           # 14
    RX_TYPE = 14               # 15
    CONFIG1_BITS = 15          # 16
    RX_THROTTLE_CH = 16        # 17
    LOW_VOLT_THRES = 17        # 18
    ROLL_CAM_KP = 18           # 19
    UNUSED_20 = 19            # 20 — was EST_CRUISE_THR (moved to Config.CruiseThrottleFF)
    STICK_HYSTERESIS = 20      # 21
    FW_CLIMB_THROTTLE = 21     # 22
    PERCENT_IDLE_THR = 22      # 23
    ROLL_ANGLE_Q_KI = 23         # 24
    PITCH_ANGLE_Q_KI = 24        # 25
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
    MAX_DESCENT_RATE_MP_S = 45  # 46
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
    NAV_POS_INT_LIM = 62       # 63
    MAX_YAW_RATE = 63          # 64
    FW_ROLL_PITCH_FF = 64      # 65
    FW_PITCH_THROTTLE_FF = 65  # 66
    UNUSED_ALT_VEL_INT_LIMIT = 66  # 67
    FW_MAX_CLIMB_ANGLE = 67    # 68
    NAV_MAX_ANGLE = 68         # 69
    FW_SPOILER_DECAY_PERCENT_PS = 69  # 70
    FW_AILERON_DIFFERENTIAL = 70       # 71
    AS_SENSOR_TYPE = 71        # 72
    UNUSED_73 = 72            # 73 — was KFAccUBiasVar (tracked live in state.c)
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
    MAX_HEADING_RATE = 88  # 89
    ACC_LPF_SEL = 89           # 90
    YAW_RATE_KD = 90           # 91
    UNUSED_92 = 91             # 92
    THROTTLE_GAIN_RATE = 92    # 93
    RX_AUX5_CH = 93            # 94 - RCMap6
    RX_AUX6_CH = 94            # 95 - RCMap7
    RX_AUX7_CH = 95            # 96 - RCMap8
    YAW_ANGLE_Q_KP = 96          # 97
    YAW_ANGLE_Q_KI = 97          # 98
    YAW_ANGLE_Q_INT_LIMIT = 98   # 99
    ALT_POS_INT_LIMIT = 99     # 100
    MOTOR_STOP_SEL = 100       # 101
    UNUSED_ALT_VEL_KI = 101    # 102
    ALT_THROTTLE_COMP_LIMIT = 102  # 103
    VRS_ROC = 103     # 104
    UNUSED_105 = 104           # 105
    AH_ROC_WINDOW_MPS = 105    # 106
    NAV_PROX_ALT_M = 106       # 107
    NAV_PROX_RADIUS_M = 107    # 108
    YAW_RATE_KI = 108          # 109 — yaw rate-loop integral gain (heading hold)
    YAW_RATE_INT_LIM = 109     # 110 — yaw rate-loop integral limit
    UNUSED_111 = 110          # 111 — was KFBaroVar (tracked live in state.c)
    UNUSED_112 = 111         # 112 — was KFAccUVar (tracked live in state.c)
    FW_STICK_SCALE = 112       # 113
    UNUSED_114 = 113            # 114 — was FW roll control pitch limit (Euler-era FW guard; unified quaternion loop is singularity-free)
    AH_THROTTLE_MOVING_TRIGGER = 114   # 115
    NAV_FENCE_RADIUS_M = 115   # 116
    DIVE_RECOVER_ALT = 116     # 117 — auto pull-out altitude AGL (m)
    FAILSAFE_ACTION = 117          # 118
    FAILSAFE_DELAY = 118          # 119 — failsafe trigger delay (seconds)
    BATTERY_ALARM_PCT = 119       # 120
    ALT_ROC_KP = 120              # 121
    MAX_CLIMB_RATE_MP_S = 121   # 122 — vertical-profile ascent shaping rate (m/s)
    RUDDER_MOTOR_FF = 122          # 123
    SPIRAL_DESCENT_BAND_M = 123      # 124 — residual altitude (m) that triggers MR spiral-orbit descent
    UNUSED_124 = 124     # 125 — was FS_CONFIG_BITS; VRS protection is always armed
    TRACE_TYPE = 125   # trace capture selector (TraceTypes: 0=None 1=Rate 2=Attitude 3=AltHold 4=Actuator 5=IMU)
    UNUSED_126 = 126
    POWER_RESET_CAUSE = 127


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
    ParamIndex.YAW_ANGLE_Q_KP,  # 96 - RCMap8 (Aux3)
]

# Expected RC Map values (0-7 in order, matching RCControl enum)
RC_MAP_EXPECTED = [
    RCControl.eThrottleRC,
    RCControl.eRollRC,
    RCControl.ePitchRC,
    RCControl.eYawRC,
    RCControl.eArmingRC,
    RCControl.eAttitudeModeRC,
    RCControl.eNavModeRC,
    RCControl.ePassThruRC,
]

# RC Mapping: param index -> function name (for display)
RC_MAPPING_NAMES = {
    ParamIndex.RX_THROTTLE_CH: "Thr",
    ParamIndex.RX_ROLL_CH: "Roll",
    ParamIndex.RX_PITCH_CH: "Pitch",
    ParamIndex.RX_YAW_CH: "Yaw",
    ParamIndex.RX_GEAR_CH: "NavMode",
    ParamIndex.RX_AUX1_CH: "AttMode",
    ParamIndex.RX_AUX2_CH: "Arming",
    ParamIndex.RX_AUX3_CH: "CamPitch",
    ParamIndex.RX_AUX4_CH: "Trace",
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
    AirframeType.eTriAF: "Tricopter",
    AirframeType.eTriCoaxAF: "CoaxTri Y6",
    AirframeType.eVTailAF: "VTail Y4",
    AirframeType.eQuadAF: "Quadcopter",
    AirframeType.eQuadXAF: "X Quadcopter",
    AirframeType.eQuadCoaxAF: "Coax Quad",
    AirframeType.eQuadCoaxXAF: "X Coax Quad",
    AirframeType.eHexAF: "Hexacopter",
    AirframeType.eHexXAF: "X Hexacopter",
    AirframeType.eOctAF: "Octocopter",
    AirframeType.eOctXAF: "X Octocopter",
    AirframeType.eHeli90AF: "Heli90",
    AirframeType.eBiAF: "Heli120",
    AirframeType.eElevonAF: "Elevon",
    AirframeType.eDeltaAF: "Delta",
    AirframeType.eAileronAF: "Aileron",
    AirframeType.eAileronSpoilerFlapsAF: "Spoilerons",
    AirframeType.eAileronVTailAF: "VTail",
    AirframeType.eRudderElevatorAF: "Rudder Elevator",
    AirframeType.eDifferentialTwinAF: "Differential Twin",
    AirframeType.eVTOLAF: "VTOL",
    AirframeType.eVTOL2AF: "VTOL2",
    AirframeType.eTrackedAF: "Tracked",
    AirframeType.eFourWheelAF: "Four Wheel",
    AirframeType.eTwoWheelAF: "Two Wheel",
    AirframeType.eInstrumentation: "Instrumentation",
    AirframeType.eAFUnknown: "Unknown Aircraft",
}

# Airframe types marked as redacted (legacy/rarely used) — matches FC RedactedAF[28]
REDACTED_AIRFRAMES = frozenset({
    AirframeType.eTriAF,              # 0
    AirframeType.eTriCoaxAF,         # 1
    AirframeType.eVTailAF,            # 2
    AirframeType.eQuadAF,             # 3
    AirframeType.eQuadCoaxAF,        # 5
    AirframeType.eQuadCoaxXAF,      # 6
    AirframeType.eHexAF,              # 7
    AirframeType.eOctAF,              # 9
    AirframeType.eHeli90AF,          # 11
    AirframeType.eBiAF,               # 12
    AirframeType.eAileronAF,          # 15
    AirframeType.eAileronVTailAF,    # 17
    AirframeType.eVTOLAF,             # 20
    AirframeType.eVTOL2AF,            # 21
    AirframeType.eFourWheelAF,       # 23
    AirframeType.eTwoWheelAF,        # 24
    AirframeType.eAFUnknown,          # 27
})

# Active airframe types (non-redacted) — for pulldown display
# Derived from REDACTED_AIRFRAMES — single source of truth
ACTIVE_AIRFRAMES = frozenset({
    af for af in AirframeType
    if af not in REDACTED_AIRFRAMES and af != AirframeType.eAFUnknown
})

# ALL selectable airframe types — exhaustive, name-sorted (single source for the
# GCS AF_TYPE combos). Includes REDACTED_AIRFRAMES members so that any value the
# FC or an .af file can legitimately produce resolves via findData() — e.g.
# generic/SkySurfer_Bixler.af uses eAileronAF (15), which is in REDACTED but
# MUST round-trip. Without the full set, a load/readback whose findData() misses
# fell back to treating the raw value as a positional index, landing on the
# WRONG airframe (FW -> X Quadcopter, Elevon -> Delta). eAFUnknown stays out.
ALL_AIRFRAMES = tuple(sorted(
    (af for af in AirframeType if af != AirframeType.eAFUnknown),
    key=lambda x: AIRFRAME_NAMES[x].lower(),
))

ESC_TYPE_NAMES = {
    ESCType.eESCPWM: "FastPWM",
    ESCType.eDCMotors: "DCMotors",
    ESCType.eMotorsOff: "MotorsOff",
}

RX_TYPE_NAMES = {
    RxType.eCPPMRx: "CPPM",
    RxType.eFutabaSBusRx: "SBus",
    RxType.eSpektrum1024Rx: "Spektrum 1024",
    RxType.eSpektrum2048Rx: "Spektrum 2048",
    RxType.eCRSFRx: "CRSF/ELRS",
    RxType.eUnknownRx: "Unknown Rx",
}

ARMING_MODE_NAMES = {
    ArmingMode.eTxArming: "Tx Arming",
    ArmingMode.eSwitchArming: "Sw Arming",
    ArmingMode.eUnusedArming2: "u2",
    ArmingMode.eUnusedArming3: "u3",
}

TELEMETRY_TYPE_NAMES = {
    TelemetryType.eUAVXDJTTelemetry: "DJT Decoder",
    TelemetryType.eINavLUATelemetry: "LUA Script",
    TelemetryType.eNoTelemetry: "No Telemetry",
    TelemetryType.eU4Telemetry: "u4",
    TelemetryType.eU5Telemetry: "u5",
    TelemetryType.eU6Telemetry: "u6",
    TelemetryType.eU7Telemetry: "u7",
    TelemetryType.eU8Telemetry: "u8",
}

GPS_PROTOCOL_NAMES = {
    GPSProtocol.U_BLOX_M8: "u-blox M8",
    GPSProtocol.U_BLOX_LEGACY: "u-blox Legacy",
    GPSProtocol.U_BLOX: "u-blox",

    GPSProtocol.NO_GPS: "No GPS",
}

RF_TYPE_NAMES = {
    RangefinderType.eMaxSonarcm: "MaxSonar cm",
    RangefinderType.eSRFI2Ccm: "SRF I2C cm",
    RangefinderType.eMaxSonarI2Ccm: "MaxSonar I2C cm",
    RangefinderType.eSharpIRGP2Y0A02YK: "Sharp IR GP2Y0A02YK",
    RangefinderType.eSharpIRGP2Y0A710K: "Sharp IR GP2Y0A710K",
    RangefinderType.eNoRF: "No RF",
}

AS_SENSOR_TYPE_NAMES = {
    AirspeedSensorType.eMS4525D0I2C: "MPXV7002DP",
    AirspeedSensorType.eMPXV7002DPAnalog: "MS425D0 (I2C)",
    AirspeedSensorType.eASThermopileAnalog: "Thermopile",
    AirspeedSensorType.eASGPSDerived: "GPS Derived",
    AirspeedSensorType.eNoAS: "No Airspeed",
}

IMU_FILTER_NAMES = {
    IMUFilterType.eLP2Filt: "LPFilt",
    IMUFilterType.eHDLPFilt: "MPUFilt",
    IMUFilterType.eF1: "PT1Filt",
    IMUFilterType.eF2: "F3",
    IMUFilterType.eF3: "F4",
    IMUFilterType.eF4: "F5",
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

FAILSAFE_ACTION_NAMES = {
    0: "RTH",
    1: "Land",
    2: "Motors Off",
}
