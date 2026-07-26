# protocol_constants.py - Updated with enums
"""
UAVX Protocol Constants - Using Enums
From FormMain.cs and params.h
"""

from protocol_enums import (
    PacketTag,
    RCControl,
    AirframeType,
    ESCType,
    RxType,
    ArmingMode,
    TelemetryType,
    RangefinderType,
    AirspeedSensorType,
    IMUFilterType,
    FlightState,
    NavState,
    Config1Bits,
    Config2Bits,
    FlagBits,
    ParamIndex,
    RC_MAP_INDICES,
    RC_MAP_EXPECTED,
    RC_MAPPING_NAMES,
    get_param_name,
    get_packet_name,
    get_airframe_name,
    AIRFRAME_NAMES,
    ESC_TYPE_NAMES,
    RX_TYPE_NAMES,
    ARMING_MODE_NAMES,
    TELEMETRY_TYPE_NAMES,
    RF_TYPE_NAMES,
    AS_SENSOR_TYPE_NAMES,
    IMU_FILTER_NAMES,
    GYRO_LPF_NAMES,
    ACC_LPF_NAMES,
    BB_LOG_NAMES,
)

# Protocol framing constants
SOH = 0x01
EOT = 0x04
ACK = 0x06
NAK = 0x15
ESC = 0x1B
CR = 0x0D
LF = 0x0A
NUL = 0x00
HT = 0x09

# ============================================================================
# RC Channel Scaling
# ============================================================================
RC_RAW_MIN = 0
RC_RAW_MAX = 65535
RC_US_MIN = 1000
RC_US_MAX = 2000
RC_US_CENTER = 1500

def raw_to_us(raw_value: int) -> int:
    """Convert raw ADC value to microseconds (1000-2000)"""
    clamped = max(RC_RAW_MIN, min(raw_value, RC_RAW_MAX))
    return int(RC_US_MIN + (clamped / RC_RAW_MAX) * (RC_US_MAX - RC_US_MIN))

def raw_to_percent(raw_value: int) -> float:
    """Convert raw ADC value to percentage (-100 to +100)"""
    us = raw_to_us(raw_value)
    return ((us - RC_US_CENTER) / (RC_US_MAX - RC_US_CENTER)) * 100

# ============================================================================
# Backward Compatibility - Flight State Names (use FlightState.get_name())
# ============================================================================
FLIGHT_STATE_NAMES = {i: FlightState.get_name(i) for i in range(13)}
NAV_STATE_NAMES = {i: NavState.get_name(i) for i in range(20)}

# Keep old constants for backward compatibility
FLIGHT_STARTING = FlightState.STARTING
FLIGHT_WARMUP = FlightState.WARMUP
FLIGHT_LANDING = FlightState.LANDING
FLIGHT_LANDED = FlightState.LANDED
FLIGHT_SHUTDOWN = FlightState.SHUTDOWN
FLIGHT_FLYING = FlightState.FLYING
FLIGHT_IREMULATE = FlightState.IREMULATE
FLIGHT_PREFLIGHT = FlightState.PREFLIGHT
FLIGHT_READY = FlightState.READY
FLIGHT_THROTTLE_OPEN_CHECK = FlightState.THROTTLE_OPEN_CHECK
FLIGHT_ERECTING_GYROS = FlightState.ERECTING_GYROS
FLIGHT_MONITOR_INSTRUMENTS = FlightState.MONITOR_INSTRUMENTS
FLIGHT_INITIALISING_GPS = FlightState.INITIALISING_GPS

NAV_HOLDING_STATION = NavState.HOLDING_STATION
NAV_RETURNING_HOME = NavState.RETURNING_HOME
NAV_AT_HOME = NavState.AT_HOME
NAV_DESCENDING = NavState.DESCENDING
NAV_TOUCHDOWN = NavState.TOUCHDOWN
NAV_TRANSITING = NavState.TRANSITING
NAV_LOITERING = NavState.LOITERING
NAV_ORBITING_POI = NavState.ORBITING_POI
NAV_PERCHING = NavState.PERCHING
NAV_TAKEOFF = NavState.TAKEOFF
NAV_PIC = NavState.PIC
NAV_ACQUIRING_ALTITUDE = NavState.ACQUIRING_ALTITUDE
NAV_USING_THERMAL = NavState.USING_THERMAL
NAV_USING_RIDGE = NavState.USING_RIDGE
NAV_USING_WAVE = NavState.USING_WAVE
NAV_BOOST_CLIMB = NavState.BOOST_CLIMB
NAV_ALTITUDE_LIMITING = NavState.ALTITUDE_LIMITING
NAV_JUST_GLIDING = NavState.JUST_GLIDING
NAV_WP_ALT_FAIL = NavState.WP_ALT_FAIL
NAV_WP_PROXIMITY_FAIL = NavState.WP_PROXIMITY_FAIL

# Re-export all enums and helpers
__all__ = [
    # Protocol framing
    'SOH', 'EOT', 'ACK', 'NAK', 'ESC', 'CR', 'LF', 'NUL', 'HT',
    # RC scaling
    'RC_RAW_MIN', 'RC_RAW_MAX', 'RC_US_MIN', 'RC_US_MAX', 'RC_US_CENTER',
    'raw_to_us', 'raw_to_percent',
    # Enums
    'PacketTag', 'RCControl', 'AirframeType', 'ESCType', 'RxType',
    'ArmingMode', 'TelemetryType',     'RangefinderType',
    'AirspeedSensorType', 'IMUFilterType', 'FlightState', 'NavState',
    'Config1Bits', 'Config2Bits', 'FlagBits', 'ParamIndex',
    # Constants
    'RC_MAP_INDICES', 'RC_MAP_EXPECTED', 'RC_MAPPING_NAMES',
    # Helpers
    'get_param_name', 'get_packet_name', 'get_airframe_name',
    # Display names
    'AIRFRAME_NAMES', 'ESC_TYPE_NAMES', 'RX_TYPE_NAMES',
    'ARMING_MODE_NAMES', 'TELEMETRY_TYPE_NAMES',
    'RF_TYPE_NAMES', 'AS_SENSOR_TYPE_NAMES', 'IMU_FILTER_NAMES',
    'GYRO_LPF_NAMES', 'ACC_LPF_NAMES', 'BB_LOG_NAMES',
    # State names (backward compat)
    'FLIGHT_STATE_NAMES', 'NAV_STATE_NAMES',
    # Flight state constants (backward compat)
    'FLIGHT_STARTING', 'FLIGHT_WARMUP', 'FLIGHT_LANDING', 'FLIGHT_LANDED',
    'FLIGHT_SHUTDOWN', 'FLIGHT_FLYING', 'FLIGHT_IREMULATE',
    'FLIGHT_PREFLIGHT', 'FLIGHT_READY', 'FLIGHT_THROTTLE_OPEN_CHECK',
    'FLIGHT_ERECTING_GYROS', 'FLIGHT_MONITOR_INSTRUMENTS',
    'FLIGHT_INITIALISING_GPS',
    # Nav state constants (backward compat)
    'NAV_HOLDING_STATION', 'NAV_RETURNING_HOME', 'NAV_AT_HOME',
    'NAV_DESCENDING', 'NAV_TOUCHDOWN', 'NAV_TRANSITING', 'NAV_LOITERING',
    'NAV_ORBITING_POI', 'NAV_PERCHING', 'NAV_TAKEOFF', 'NAV_PIC',
    'NAV_ACQUIRING_ALTITUDE', 'NAV_USING_THERMAL', 'NAV_USING_RIDGE',
    'NAV_USING_WAVE', 'NAV_BOOST_CLIMB', 'NAV_ALTITUDE_LIMITING',
    'NAV_JUST_GLIDING', 'NAV_WP_ALT_FAIL', 'NAV_WP_PROXIMITY_FAIL',
]
