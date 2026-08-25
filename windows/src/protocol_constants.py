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
FLIGHT_STARTING = FlightState.eStarting
FLIGHT_WARMUP = FlightState.eWarmup
FLIGHT_LANDING = FlightState.eLanding
FLIGHT_LANDED = FlightState.eLanded
FLIGHT_SHUTDOWN = FlightState.eShutdown
FLIGHT_FLYING = FlightState.eInFlight
FLIGHT_IREMULATE = FlightState.eIREmulateUNUSED
FLIGHT_PREFLIGHT = FlightState.ePreflight
FLIGHT_READY = FlightState.eReady
FLIGHT_THROTTLE_OPEN_CHECK = FlightState.eThrottleOpenCheck
FLIGHT_ERECTING_GYROS = FlightState.eErectingGyros
FLIGHT_MONITOR_INSTRUMENTS = FlightState.eMonitorInstruments
FLIGHT_INITIALISING_GPS = FlightState.eInitialisingGPS

NAV_HOLDING_STATION = NavState.eHoldingStation
NAV_RETURNING_HOME = NavState.eReturningHome
NAV_AT_HOME = NavState.eAtHome
NAV_DESCENDING = NavState.eDescending
NAV_TOUCHDOWN = NavState.eTouchdown
NAV_TRANSITING = NavState.eTransiting
NAV_LOITERING = NavState.eLoitering
NAV_ORBITING_POI = NavState.eOrbitingPOI
NAV_PERCHING = NavState.ePerching
NAV_TAKEOFF = NavState.eTakeoff
NAV_PIC = NavState.ePIC
NAV_ACQUIRING_ALTITUDE = NavState.eAcquiringAltitude
NAV_USING_THERMAL = NavState.eUsingThermal
NAV_USING_RIDGE = NavState.eUsingRidge
NAV_USING_WAVE = NavState.eUsingWave
NAV_BOOST_CLIMB = NavState.eBoostClimb
NAV_ALTITUDE_LIMITING = NavState.eAltitudeLimiting
NAV_JUST_GLIDING = NavState.eJustGliding
NAV_WP_ALT_FAIL = NavState.eWPAltFail
NAV_WP_PROXIMITY_FAIL = NavState.eWPProximityFail

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
