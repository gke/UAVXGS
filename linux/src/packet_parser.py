# packet_parser.py
"""
UAVX Packet Parser
Based on telem.c and FormMain.cs
All ESC bytes are removed before this parser receives data!
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any
import struct

from protocol_constants import *


def extract_byte(data: bytes, offset: int) -> int:
    """Extract a byte from buffer"""
    if offset >= len(data):
        return 0
    return data[offset] & 0xFF


def extract_short(data: bytes, offset: int) -> int:
    """Extract little-endian 16-bit value"""
    if offset + 1 >= len(data):
        return 0
    return (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8)


def extract_int24(data: bytes, offset: int) -> int:
    """Extract little-endian 24-bit value"""
    if offset + 2 >= len(data):
        return 0
    return (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8) | ((data[offset + 2] & 0xFF) << 16)


def extract_int32(data: bytes, offset: int) -> int:
    """Extract little-endian 32-bit value (unsigned)"""
    if offset + 3 >= len(data):
        return 0
    return (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8) | \
           ((data[offset + 2] & 0xFF) << 16) | ((data[offset + 3] & 0xFF) << 24)


def extract_signed_short(data: bytes, offset: int) -> int:
    """Extract signed little-endian 16-bit value"""
    val = extract_short(data, offset)
    if val > 32767:
        val -= 65536
    return val


def extract_signed_byte(data: bytes, offset: int) -> int:
    """Extract signed byte"""
    val = extract_byte(data, offset)
    if val > 127:
        val -= 256
    return val


def extract_signed_int24(data: bytes, offset: int) -> int:
    """Extract signed little-endian 24-bit value (two's complement)"""
    val = extract_int24(data, offset)
    if val > 8388607:  # 2^23 - 1
        val -= 16777216  # 2^24
    return val


def extract_signed_int32(data: bytes, offset: int) -> int:
    """Extract signed little-endian 32-bit value (two's complement)"""
    val = extract_int32(data, offset)
    if val > 2147483647:
        val -= 4294967296
    return val


def extract_real32(data: bytes, offset: int) -> float:
    """Extract raw IEEE-754 little-endian float32"""
    if len(data) < offset + 4:
        return 0.0
    return struct.unpack('<f', data[offset:offset+4])[0]


# Scaling constants - from telem.c
SCALE_BATTERY_VOLTS = 0.01
SCALE_BATTERY_CURRENT = 0.01
SCALE_BATTERY_CHARGE = 1.0
SCALE_ALTITUDE = 0.01
SCALE_ROC = 0.01
SCALE_HEADING = 0.001
SCALE_ANGLE = 0.001
SCALE_RATE = 0.001
SCALE_ACC = 0.001
SCALE_BARO_TEMP = 0.01
SCALE_BARO_PRESSURE = 0.001
SCALE_MPU_TEMP = 0.1
SCALE_GPS_HACC = 0.01
SCALE_GPS_VACC = 0.01
SCALE_GPS_SACC = 0.01
SCALE_GPS_CACC = 0.01
SCALE_GPS_VEL = 0.1
SCALE_GPS_ALTITUDE = 0.01
SCALE_GPS_LATLON = 1e-7
SCALE_WP_BEARING = 0.001
SCALE_CROSS_TRACK = 0.1
SCALE_RANGEFINDER = 0.01
SCALE_TILT_FF = 0.001
SCALE_BATT_FF = 0.001
SCALE_ALT_COMP = 0.1
SCALE_ACC_CONFIDENCE = 1.0


def scale_rc(raw: int) -> int:
    """Scale raw ADC (0-65535) to µs (1000-2000)"""
    if raw == 0:
        return 1500
    clamped = max(0, min(raw, 65535))
    return int(1000 + (clamped / 65535.0) * 1000.0)


@dataclass
class FlightData:
    flags: List[int] = field(default_factory=list)
    flag_bits: List[bool] = field(default_factory=list)
    flight_state: int = 0
    nav_state: int = 0
    alarm_state: int = 0
    battery_volts_raw: int = 0
    battery_volts: float = 0.0
    battery_current: float = 0.0
    battery_charge: float = 0.0
    rc_glitches: int = 0
    q0: float = 1.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    angle_roll: float = 0.0
    angle_pitch: float = 0.0
    angle_yaw: float = 0.0
    rate_roll: float = 0.0
    rate_pitch: float = 0.0
    rate_yaw: float = 0.0
    acc_lr: float = 0.0
    acc_fb: float = 0.0
    acc_du: float = 0.0
    desired_throttle: float = 0.0
    desired_roll: float = 0.0
    desired_pitch: float = 0.0
    desired_yaw: float = 0.0
    desired_rate_roll: float = 0.0
    desired_rate_pitch: float = 0.0
    desired_rate_yaw: float = 0.0
    altitude: float = 0.0
    desired_altitude: float = 0.0
    roc: float = 0.0
    baro_altitude: float = 0.0
    gps_altitude: float = 0.0
    rangefinder_altitude: float = 0.0
    heading: float = 0.0
    desired_heading: float = 0.0
    mag_heading: float = 0.0
    gps_lat: float = 0.0
    gps_lon: float = 0.0
    gps_sats: int = 0
    gps_fix: int = 0
    gps_vel: float = 0.0
    gps_heading: float = 0.0
    gps_hacc: float = 0.0
    gps_vacc: float = 0.0
    gps_sacc: float = 0.0
    gps_cacc: float = 0.0
    curr_wp: int = 0
    wp_bearing: float = 0.0
    distance_to_wp: float = 0.0
    cross_track_error: float = 0.0
    north_pos_e: float = 0.0
    east_pos_e: float = 0.0
    nav_state_timeout: float = 0.0
    gps_type: int = 0
    gps_update_rate: float = 0.0
    nav_sensitivity: float = 0.0
    mag_var_wmm: float = 0.0
    nav_p_corr: float = 0.0
    nav_r_corr: float = 0.0
    nav_y_corr: float = 0.0
    uplink_lq: int = 0
    uplink_rssi: int = 0
    uplink_snr: int = 0
    rc_failsafes: int = 0
    rc_signal_losses: int = 0
    pwm: List[int] = field(default_factory=list)
    rc_raw: List[int] = field(default_factory=list)
    rc_channels: List[int] = field(default_factory=list)
    pwm_diag: List[int] = field(default_factory=list)
    motors_and_servos: int = 0
    tilt_ff_comp: float = 0.0
    batt_ff_comp: float = 0.0
    alt_comp: float = 0.0
    cruise_throttle: float = 0.0
    acc_confidence: float = 0.0
    baro_temp: float = 0.0
    baro_pressure: float = 0.0
    mpu_temp: float = 0.0
    mission_time: float = 0.0
    airframe_type: int = 0
    fw_rate_energy: float = 0.0
    fw_glide_offset: float = 0.0
    baro_variance: float = 0.0
    accu_variance: float = 0.0
    accu_bias_variance: float = 0.0
    # Min packet fields
    min_packet: bool = False


@dataclass
class ExecTimeData:
    """Execution time packet data (Tag=67)"""
    exec_percent: int = 0
    exec_peak_percent: int = 0
    revision: int = 0


@dataclass
class LinkStatsData:
    """RC Link stats packet data (Tag=69)"""
    uplink_lq: int = 0
    uplink_snr: int = 0
    uplink_rssi: int = 0
    rc_signal_losses: int = 0
    rc_failsafes: int = 0


@dataclass
class ParameterData:
    """Parameter data (Tag=17) — flat float32 array format"""
    base_idx: int = 0
    count: int = 0
    entries: Dict[int, Tuple[int, Any]] = field(default_factory=dict)
    version_name: str = ''


@dataclass
class OriginData:
    """Origin packet data (Tag=19)"""
    num_waypoints: int = 0
    max_velocity: float = 0.0
    fence_radius: int = 0
    home_altitude: int = 0
    home_lat: int = 0
    home_lon: int = 0


@dataclass
class GuidanceData:
    """Guidance packet data (Tag=59) - from telem.c SendGuidancePacket()
    Body: distance(2) + bearing(2) + elevation(1) + hint(1) = 6 bytes
    """
    distance: float = 0.0
    bearing: float = 0.0
    elevation: float = 0.0
    hint: float = 0.0


@dataclass
class AltitudeControlData:
    """Altitude control packet data (Tag=60) - from telem.c SendAltitudeControlPacket()
    Body: 25 bytes
    """
    acceleration: float = 0.0
    acc_bias: float = 0.0
    baro_density_alt: float = 0.0
    kf_altitude: float = 0.0
    desired_alt: float = 0.0
    baro_roc: float = 0.0
    kf_roc: float = 0.0
    battery_volts: float = 0.0
    cruise_throttle: float = 0.0
    desired_throttle: float = 0.0
    tilt_ff_comp: float = 0.0
    batt_ff_comp: float = 0.0
    alt_hold_comp: float = 0.0
    total_throttle: float = 0.0


@dataclass
class TuningData:
    """Minimal tuning telemetry (Tag=57) - from telem.c SendTuningPacket()
    Body: flags(1) + thr(1) + 3 axes×6 = 22 bytes
    """
    flags: int = 0
    throttle_pct: float = 0.0
    rate_desired_roll: float = 0.0
    rate_actual_roll: float = 0.0
    out_roll: float = 0.0
    rate_desired_pitch: float = 0.0
    rate_actual_pitch: float = 0.0
    out_pitch: float = 0.0
    rate_desired_yaw: float = 0.0
    rate_actual_yaw: float = 0.0
    out_yaw: float = 0.0

    @property
    def armed(self) -> bool:
        return bool(self.flags & 0x01)
    @property
    def in_flight(self) -> bool:
        return bool(self.flags & 0x02)
    @property
    def angle_mode(self) -> bool:
        return bool(self.flags & 0x04)
    @property
    def alt_hold(self) -> bool:
        return bool(self.flags & 0x08)


@dataclass
class AttitudeControlData:
    """Attitude control packet data (Tag=68) - from telem.c
    Short form (3B): rate(2) + out(1)
    Quaternion form (21B): 3×rate(6) + 3×out(3) + 4×q(8) + 2×angle(4)
    """
    axis: int = 0
    rate: float = 0.0
    out: float = 0.0
    # Quaternion form fields
    rate_roll: float = 0.0
    rate_pitch: float = 0.0
    rate_yaw: float = 0.0
    out_roll: float = 0.0
    out_pitch: float = 0.0
    out_yaw: float = 0.0
    q0: float = 0.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    angle_roll: float = 0.0
    angle_pitch: float = 0.0
    is_quaternion: bool = False


@dataclass
class ControlData:
    """Control packet data (Tag=16)"""
    desired_throttle: float = 0.0
    angle_roll: float = 0.0
    angle_pitch: float = 0.0
    angle_yaw: float = 0.0
    rate_roll: float = 0.0
    rate_pitch: float = 0.0
    rate_yaw: float = 0.0
    acc_lr: float = 0.0
    acc_fb: float = 0.0
    acc_du: float = 0.0
    desired_roll: float = 0.0
    desired_pitch: float = 0.0
    desired_yaw: float = 0.0
    desired_rate_roll: float = 0.0
    desired_rate_pitch: float = 0.0
    desired_rate_yaw: float = 0.0
    airframe: int = 0
    pwm: List[int] = field(default_factory=list)
    pwm_diag: List[int] = field(default_factory=list)
    mission_time: float = 0.0
    firmware_revision: int = 0


@dataclass
class WindData:
    """Wind packet data (Tag=64) - from telem.c SendWindPacket()
    Body: speed(2) + direction(2) + Est[WN](2) + Est[WE](2) + Est[WD](2) = 10 bytes
    """
    speed: float = 0.0
    direction: float = 0.0
    wind_n: float = 0.0
    wind_e: float = 0.0
    wind_d: float = 0.0


def parse_flight_packet(data: bytes, voltage_trim: float = 1.0) -> Optional[FlightData]:
    """
    Parse UAVXFlightPacket (Tag=13)
    Based on telem.c SendFlightPacket()
    """
    if len(data) < 111:
        return None
    
    f = FlightData()
    
    # Flags (body[0-5])
    f.flags = [extract_byte(data, i) for i in range(6)]
    f.flag_bits = []
    for byte_val in f.flags:
        for bit in range(8):
            f.flag_bits.append((byte_val & (1 << bit)) != 0)
    
    # State (body[6])
    f.flight_state = extract_byte(data, 6)
    
    # Battery (body[7-12])
    f.battery_volts_raw = extract_short(data, 7)
    f.battery_volts = f.battery_volts_raw * SCALE_BATTERY_VOLTS * voltage_trim
    f.battery_current = extract_short(data, 9) * SCALE_BATTERY_CURRENT
    f.battery_charge = extract_short(data, 11) * SCALE_BATTERY_CHARGE
    f.rc_glitches = extract_short(data, 13)
    
    # Desired throttle (body[15])
    f.desired_throttle = extract_short(data, 15) * 0.1

    # Attitude data - starts at body[33]
    # C firmware sends via ShowAttitude(): quaternion (16 bytes), then
    # for a = Pitch(0); a <= Yaw(2); a++
    # So axis order in the packet is: Pitch(0), Roll(1), Yaw(2)
    # Each axis: Desired, Angle, Acc, DesiredRate, Rate (10 bytes)
    pitch_start = 33
    f.desired_pitch = extract_signed_short(data, pitch_start) * SCALE_ANGLE
    f.angle_pitch = extract_signed_short(data, pitch_start + 2) * SCALE_ANGLE
    f.acc_fb = extract_signed_short(data, pitch_start + 4) * SCALE_ACC
    f.desired_rate_pitch = extract_signed_short(data, pitch_start + 6) * SCALE_RATE
    f.rate_pitch = extract_signed_short(data, pitch_start + 8) * SCALE_RATE
    
    # Roll (axis 1) - offset +10
    roll_start = pitch_start + 10
    f.desired_roll = extract_signed_short(data, roll_start) * SCALE_ANGLE
    f.angle_roll = extract_signed_short(data, roll_start + 2) * SCALE_ANGLE
    f.acc_lr = extract_signed_short(data, roll_start + 4) * SCALE_ACC
    f.desired_rate_roll = extract_signed_short(data, roll_start + 6) * SCALE_RATE
    f.rate_roll = extract_signed_short(data, roll_start + 8) * SCALE_RATE
    
    # Yaw (axis 2) - offset +20
    yaw_start = pitch_start + 20
    f.desired_yaw = extract_signed_short(data, yaw_start) * SCALE_ANGLE
    f.angle_yaw = extract_signed_short(data, yaw_start + 2) * SCALE_ANGLE
    f.acc_du = extract_signed_short(data, yaw_start + 4) * SCALE_ACC
    f.desired_rate_yaw = extract_signed_short(data, yaw_start + 6) * SCALE_RATE
    f.rate_yaw = extract_signed_short(data, yaw_start + 8) * SCALE_RATE

    # ROC (body[63])
    f.roc = extract_signed_short(data, 63) * SCALE_ROC

    # Altitude (body[65]) - 24-bit
    f.altitude = extract_signed_int24(data, 65) * SCALE_ALTITUDE

    # Cruise throttle (body[68])
    f.cruise_throttle = extract_short(data, 68) * 0.1

    # Rangefinder altitude (body[70])
    f.rangefinder_altitude = extract_short(data, 70) * SCALE_RANGEFINDER

    # Desired altitude (body[72]) - 24-bit
    f.desired_altitude = extract_signed_int24(data, 72) * SCALE_ALTITUDE

    # Heading (body[75,77])
    f.heading = extract_short(data, 75) * SCALE_HEADING
    f.desired_heading = extract_short(data, 77) * SCALE_HEADING

    # Compensation (body[79,81,83])
    f.tilt_ff_comp = extract_signed_short(data, 79) * SCALE_TILT_FF
    f.batt_ff_comp = extract_signed_short(data, 81) * SCALE_BATT_FF
    f.alt_comp = extract_signed_short(data, 83) * SCALE_ALT_COMP

    # Acc confidence (body[85])
    f.acc_confidence = extract_byte(data, 85) * SCALE_ACC_CONFIDENCE

    # Sensors (body[86-105])
    f.baro_temp = extract_signed_short(data, 86) * SCALE_BARO_TEMP
    f.baro_pressure = extract_int24(data, 88) * SCALE_BARO_PRESSURE
    f.baro_altitude = extract_signed_int24(data, 91) * SCALE_ALTITUDE
    f.baro_variance = extract_short(data, 94) * 0.001
    f.accu_variance = extract_short(data, 96) * 0.001
    f.accu_bias_variance = extract_short(data, 98) * 0.00001
    f.mag_heading = extract_short(data, 100) * SCALE_HEADING
    f.mpu_temp = extract_signed_short(data, 102) * SCALE_MPU_TEMP

    # Fixed wing (body[104,106])
    f.fw_rate_energy = extract_short(data, 104) * 0.01
    f.fw_glide_offset = extract_signed_short(data, 106) * 0.1

    # Nav state (body[108])
    f.nav_state = extract_byte(data, 108)

    # PWM outputs - ShowDrives() (body[109+])
    f.motors_and_servos = extract_byte(data, 109)
    pwm_start = 110
    for i in range(min(f.motors_and_servos, 10)):
        f.pwm.append(extract_short(data, pwm_start + i * 2))
    
    # PWM diagnostics
    diag_start = pwm_start + f.motors_and_servos * 2
    for i in range(min(f.motors_and_servos, 10)):
        f.pwm_diag.append(extract_signed_byte(data, diag_start + i))
    
    # Mission time
    f.mission_time = extract_int24(data, diag_start + f.motors_and_servos)
    
    return f


MAG_VAR_WMM_SCALE = 0.1

def parse_nav_packet(data: bytes) -> Optional[FlightData]:
    """
    Parse UAVXNavPacket (Tag=14)
    Based on telem.c SendNavPacket()
    Body: 59 bytes (57 base + 2 mag_var_wmm)
    """
    if len(data) < 57:
        return None
    
    f = FlightData()
    
    # Nav state (body[0])
    f.nav_state = extract_byte(data, 0)
    f.alarm_state = extract_byte(data, 1)
    f.gps_sats = extract_byte(data, 2)
    f.gps_fix = extract_byte(data, 3)
    f.curr_wp = extract_byte(data, 4)
    
    # GPS accuracy (body[5-12])
    f.gps_hacc = extract_short(data, 5) * SCALE_GPS_HACC
    f.gps_vacc = extract_short(data, 7) * SCALE_GPS_VACC
    f.gps_sacc = extract_short(data, 9) * SCALE_GPS_SACC
    f.gps_cacc = extract_short(data, 11) * SCALE_GPS_CACC
    
    # Navigation (body[13-20])
    f.wp_bearing = extract_short(data, 13) * SCALE_WP_BEARING
    f.cross_track_error = extract_short(data, 15) * SCALE_CROSS_TRACK
    f.gps_vel = extract_short(data, 17) * SCALE_GPS_VEL
    f.gps_heading = extract_short(data, 19) * SCALE_HEADING
    
    # GPS altitude - SIGNED 24-bit value (body[21-23])
    f.gps_altitude = extract_signed_int24(data, 21) * SCALE_GPS_ALTITUDE
    
    # GPS coordinates - SIGNED 32-bit values (body[24-31])
    f.gps_lat = extract_signed_int32(data, 24) * SCALE_GPS_LATLON
    f.gps_lon = extract_signed_int32(data, 28) * SCALE_GPS_LATLON
    
    # Position and timeout (body[32-42])
    f.north_pos_e = extract_int32(data, 32) * 0.1
    f.east_pos_e = extract_int32(data, 36) * 0.1
    f.nav_state_timeout = extract_int24(data, 40) * 0.001
    
    # Extended fields (body[43-56])
    if len(data) > 49:
        dT_raw = extract_short(data, 43)
        f.gps_update_rate = 1000.0 / dT_raw if dT_raw > 0 else 0.0
        # body[45-48] = GPS.hwVersion (uint32, not used)
        f.gps_type = extract_byte(data, 49)  # CurrGPSType
        # body[50] = padding
        f.nav_p_corr = extract_short(data, 51) * SCALE_HEADING
        f.nav_r_corr = extract_short(data, 53) * SCALE_HEADING
        f.nav_y_corr = extract_short(data, 55) * SCALE_HEADING
    
    if len(data) >= 59:
        f.mag_var_wmm = extract_short(data, 57) * MAG_VAR_WMM_SCALE
    
    return f


def parse_param_packet(data: bytes) -> Optional[ParameterData]:
    """Parameter packet (Tag=17)
    Two wire formats:
      - Single-param: [param_idx(1), float32(4)] (5 bytes — echo ACK)
      - Bulk: [base_idx(1), count(1), N × float32(4)] (>=6 bytes — readback)
    Bulk may also include a 2-byte revision trailer: [... , revision(2)]
    """
    if len(data) < 2:
        return None
    p = ParameterData()
    if len(data) == 5:
        p.base_idx = extract_byte(data, 0)
        p.count = 1
        val = struct.unpack('<f', data[1:5])[0]
        p.entries[p.base_idx] = (3, val)
    else:
        p.base_idx = extract_byte(data, 0)
        p.count = extract_byte(data, 1)
        for i in range(p.count):
            idx = p.base_idx + i
            offset = 2 + i * 4
            if offset + 4 > len(data):
                break
            val = struct.unpack('<f', data[offset:offset+4])[0]
            p.entries[idx] = (3, val)
        # Check for 2-byte revision trailer after the float32 array
        tail_start = 2 + p.count * 4
        if len(data) >= tail_start + 2:
            revision = extract_short(data, tail_start)
            if revision > 0:
                p.version_name = f"r{revision}"
    return p


def parse_rc_packet(data: bytes) -> Optional[FlightData]:
    """Parse UAVXRCChannelsPacket (Tag=22)
    Body: frameInterval(2) + numChannels(1) + Nx channels(2 each)
    Firmware sends channel values as int16 in µs (1000-2000).
    """
    if len(data) < 5:
        return None
    
    # Parse as many 2-byte channel values as the body contains (after 3-byte header)
    num_channels = (len(data) - 3) // 2
    if num_channels < 1:
        return None
    
    f = FlightData()
    f.rc_channels = []
    f.rc_raw = []
    
    for i in range(num_channels):
        offset = 3 + i * 2
        us_value = extract_short(data, offset)
        f.rc_channels.append(us_value)
        f.rc_raw.append(us_value)
    
    return f


def parse_guidance_packet(data: bytes) -> Optional[GuidanceData]:
    """Parse UAVXGuidancePacket (Tag=59) - from telem.c SendGuidancePacket()
    Body: distance(2) + bearing(2) + elevation(1) + hint(1) = 6 bytes
    """
    if len(data) < 6:
        return None

    g = GuidanceData()
    g.distance = extract_short(data, 0)
    g.bearing = extract_signed_short(data, 2) * SCALE_HEADING
    g.elevation = extract_signed_byte(data, 4)
    g.hint = extract_signed_byte(data, 5)

    return g


def parse_altitude_control_packet(data: bytes) -> Optional[AltitudeControlData]:
    """Parse UAVXAltitudeControlPacket (Tag=60) - from telem.c SendAltitudeControlPacket()
    Body: 25 bytes
    """
    if len(data) < 25:
        return None

    a = AltitudeControlData()
    a.acceleration = extract_signed_short(data, 0) * 0.001
    a.acc_bias = extract_signed_short(data, 2) * 0.001
    a.baro_density_alt = extract_signed_short(data, 4) * 0.01
    a.kf_altitude = extract_signed_short(data, 6) * 0.01
    a.desired_alt = extract_signed_short(data, 8) * 0.01
    a.baro_roc = extract_signed_short(data, 10) * 0.001
    a.kf_roc = extract_signed_short(data, 12) * 0.001
    a.battery_volts = extract_byte(data, 14) * 0.1
    a.cruise_throttle = extract_byte(data, 15) * 0.005
    a.desired_throttle = extract_byte(data, 16) * 0.005
    a.tilt_ff_comp = extract_signed_short(data, 17) * 0.0001
    a.batt_ff_comp = extract_signed_short(data, 19) * 0.0001
    a.alt_hold_comp = extract_signed_short(data, 21) * 0.001
    a.total_throttle = extract_signed_short(data, 23) * 0.0001

    return a


def parse_attitude_control_packet(data: bytes, axis: int = 0) -> Optional[AttitudeControlData]:
    """Parse UAVXAttitudeControlPacket (Tag=68) - from telem.c
    Short form (3B): rate(2) + out(1)
    Quaternion form (21B): 3×rate + 3×out + 4×q + 2×angle
    """
    a = AttitudeControlData()

    if len(data) >= 21:
        a.is_quaternion = True
        a.rate_roll = extract_signed_short(data, 0) * 0.001
        a.rate_pitch = extract_signed_short(data, 2) * 0.001
        a.rate_yaw = extract_signed_short(data, 4) * 0.001
        a.out_roll = extract_signed_byte(data, 6) * 0.005
        a.out_pitch = extract_signed_byte(data, 7) * 0.005
        a.out_yaw = extract_signed_byte(data, 8) * 0.005
        a.q0 = extract_signed_short(data, 9) * 0.0001
        a.q1 = extract_signed_short(data, 11) * 0.0001
        a.q2 = extract_signed_short(data, 13) * 0.0001
        a.q3 = extract_signed_short(data, 15) * 0.0001
        a.angle_roll = extract_signed_short(data, 17) * 0.001
        a.angle_pitch = extract_signed_short(data, 19) * 0.001
        return a

    if len(data) < 3:
        return None

    a.axis = axis
    a.rate = extract_signed_short(data, 0) * SCALE_RATE
    a.out = extract_signed_byte(data, 2) * 0.005

    return a


SCALE_WIND_SPEED = 0.01
SCALE_WIND_DIR = 0.001
SCALE_WIND_EST = 0.01


def parse_wind_packet(data: bytes) -> Optional[WindData]:
    """Parse UAVXWindPacket (Tag=64) - from telem.c SendWindPacket()
    Body: speed(2) + direction(2) + Est[WN](2) + Est[WE](2) + Est[WD](2) = 10 bytes
    """
    if len(data) < 10:
        return None

    w = WindData()
    w.speed = extract_short(data, 0) * SCALE_WIND_SPEED
    w.direction = extract_signed_short(data, 2) * SCALE_WIND_DIR
    w.wind_n = extract_signed_short(data, 4) * SCALE_WIND_EST
    w.wind_e = extract_signed_short(data, 6) * SCALE_WIND_EST
    w.wind_d = extract_signed_short(data, 8) * SCALE_WIND_EST
    return w


def parse_link_stats(data: bytes) -> Optional[LinkStatsData]:
    """Parse UAVXLinkStatsPacket (Tag=69)
    Body: lq(1) + snr(1) + rssi(2) + signalLosses(2) + failsafes(2) = 8 bytes
    """
    if len(data) < 8:
        return None
    
    f = LinkStatsData()
    f.uplink_lq = extract_byte(data, 0)
    f.uplink_snr = extract_byte(data, 1)
    f.uplink_rssi = extract_short(data, 2)
    f.rc_signal_losses = extract_short(data, 4)
    f.rc_failsafes = extract_short(data, 6)
    
    return f


def parse_ack_packet(data: bytes) -> Optional[dict]:
    """
    Parse UAVXAckPacket (Tag=51)
    Based on telem.c SendAckPacket()
    Body: ackTag(1) + reason(1) = 2 bytes
    """
    if len(data) < 2:
        return None
    
    tag = extract_byte(data, 0)      # body[0] = Tag being ACKed
    reason = extract_byte(data, 1)   # body[1] = Reason (255 = success, 0 = failure)
    
    return {
        'ack_tag': tag,
        'ack_success': reason != 0,
        'ack_reason': reason,
        'ack_text': 'ACK' if reason != 0 else 'NACK'
    }


def parse_waypoint_packet(data: bytes) -> Optional[dict]:
    """Parse UAVXWPPacket (Tag=20) - from telem.c SendWPPacket()
    Body: wpIdx(1) + lat(4) + lon(4) + alt(2) + vel(2) + loiter(2)
          + orbitRadius(2) + orbitAlt(2) + orbitVel(2) + pulseWidth(4)
          + pulsePeriod(4) + action(1) = 30 bytes
    """
    if len(data) < 30:
        return None
    
    result = {
        'wp_index': extract_byte(data, 0),
        'wp_lat': extract_signed_int32(data, 1) * SCALE_GPS_LATLON,
        'wp_lon': extract_signed_int32(data, 5) * SCALE_GPS_LATLON,
        'wp_alt': extract_short(data, 9),
        'wp_velocity': extract_short(data, 11) * 0.1,  # dM/S
        'wp_loiter': extract_short(data, 13),  # seconds
        'wp_orbit_radius': extract_short(data, 15),
        'wp_orbit_alt': extract_short(data, 17),
        'wp_orbit_velocity': extract_short(data, 19) * 0.1,
        'wp_pulse_width': extract_int32(data, 21),  # mS
        'wp_pulse_period': extract_int32(data, 25),  # mS
        'wp_action': extract_byte(data, 29),
    }
    
    return result


def parse_airframe_name(data: bytes) -> Optional[dict]:
    """Parse UAVXConfigPacket (Tag=63) - from telem.c SendConfigPacket()
    Body: revision string (null-terminated) + AFType(1)
    """
    # Find null terminator
    null_idx = data.find(b'\x00')
    if null_idx < 0 or null_idx + 1 >= len(data):
        return None
    revision = data[:null_idx].decode('ascii', errors='replace')
    af_type = data[null_idx + 1]
    return {
        'revision': revision,
        'af_type': af_type,
    }


def parse_execution_time(data: bytes) -> Optional[ExecTimeData]:
    """Parse UAVXExecutionTimePacket (Tag=67) - from telem.c SendExecutionTimeStatus()
    Body: execPercent(2) + execPeakPercent(2) + revision(2) = 6 bytes
    """
    if len(data) < 6:
        return None
    
    f = ExecTimeData()
    f.exec_percent = extract_short(data, 0)
    f.exec_peak_percent = extract_short(data, 2)
    f.revision = extract_short(data, 4)
    return f


def parse_tuning_packet(data: bytes) -> Optional[TuningData]:
    """Parse UAVXTuningPacket (Tag=57) - from telem.c SendTuningPacket()"""
    if len(data) < 21:
        return None
    t = TuningData()
    t.flags = data[0]
    t.throttle_pct = data[1] * 0.5
    off = 2
    t.rate_desired_roll = extract_short(data, off) * 0.001;    off += 2
    t.rate_actual_roll  = extract_short(data, off) * 0.001;    off += 2
    t.out_roll          = extract_short(data, off) * 0.0001;   off += 2
    t.rate_desired_pitch = extract_short(data, off) * 0.001;   off += 2
    t.rate_actual_pitch  = extract_short(data, off) * 0.001;   off += 2
    t.out_pitch          = extract_short(data, off) * 0.0001;  off += 2
    t.rate_desired_yaw  = extract_short(data, off) * 0.001;    off += 2
    t.rate_actual_yaw   = extract_short(data, off) * 0.001;    off += 2
    t.out_yaw           = extract_short(data, off) * 0.0001;   off += 2
    return t


def parse_calibration_packet(data: bytes) -> Optional[dict]:
    """Parse UAVXCalibrationPacket (Tag=62) - from telem.c SendCalibrationPacket()
    Body: flags(6) + refTemp(2) + 32 cal values(2 each) = 72 bytes
    
    Cal values are raw int16. Display scaling (e.g. *0.001 for gyros, *0.1 for temp)
    is applied by the consumer based on index.
    Indices 23-30 are mag calibration octant sample counts (positive ints, 0-55).
    """
    if len(data) < 72:
        return None
    
    result = {
        'flags': [extract_byte(data, i) for i in range(6)],
        'cal_ref_temp': extract_short(data, 6) * 0.1,
        'cal_data': []
    }
    
    # Extract 32 raw int16 calibration values starting at body[8]
    for i in range(32):
        offset = 8 + i * 2
        if offset + 1 < len(data):
            result['cal_data'].append(extract_short(data, offset))
        else:
            result['cal_data'].append(0)
    
    return result


def parse_serial_ports_packet(data: bytes) -> Optional[dict]:
    """Parse UAVXSerialPortsPacket (Tag=66) - from telem.c SendSerialPortStatus()
    Body: numPorts(1) + bufSize(2)
          + telemetry_flags(1) + telemetry_tx(2) + telemetry_rx(2)
          + gps_flags(1) + gps_tx(2) + gps_rx(2)
          + softserial_flags(1) + softserial_tx(2) + softserial_rx(2)
          = 18 bytes
    """
    if len(data) < 18:
        return None

    def _port(name: str, off: int):
        flags = extract_byte(data, off)
        return {
            'name': name,
            'tx_overflow': bool(flags & 2),
            'rx_overflow': bool(flags & 1),
            'tx_q': extract_short(data, off + 1),
            'rx_q': extract_short(data, off + 3),
        }

    return {
        'num_ports': extract_byte(data, 0),
        'buffer_size': extract_short(data, 1),
        'telemetry': _port('Telemetry', 3),
        'gps': _port('GPS', 8),
        'softserial': _port('SoftSerial', 13),
    }


def parse_min_packet(data: bytes) -> Optional[FlightData]:
    """
    Parse UAVXMinPacket (Tag=18)
    Based on telem.c SendMinPacket()
    """
    if len(data) < 39:
        return None
    
    f = FlightData()
    f.min_packet = True
    
    # Flags (body[0-5])
    f.flags = [extract_byte(data, i) for i in range(6)]
    f.flag_bits = []
    for byte_val in f.flags:
        for bit in range(8):
            f.flag_bits.append((byte_val & (1 << bit)) != 0)
    
    # State (body[6])
    f.flight_state = extract_byte(data, 6)
    f.nav_state = extract_byte(data, 7)
    f.alarm_state = extract_byte(data, 8)
    
    # Battery (body[9-14])
    f.battery_volts = extract_short(data, 9) * SCALE_BATTERY_VOLTS
    f.battery_current = extract_short(data, 11) * SCALE_BATTERY_CURRENT
    f.battery_charge = extract_short(data, 13) * SCALE_BATTERY_CHARGE
    
    # Attitude (body[15-18])
    # C SendMinPacket(): Angle[Roll] first, then Angle[Pitch]
    f.angle_roll = extract_signed_short(data, 15) * SCALE_ANGLE
    f.angle_pitch = extract_signed_short(data, 17) * SCALE_ANGLE
    
    # Altitude (body[19-21]) - 24-bit
    f.altitude = extract_signed_int24(data, 19) * SCALE_ALTITUDE
    
    # ROC (body[22-23])
    f.roc = extract_signed_short(data, 22) * SCALE_ROC
    
    # Heading (body[24-25])
    f.heading = extract_short(data, 24) * SCALE_HEADING
    
    # GPS (body[26-33])
    f.gps_lat = extract_signed_int32(data, 26) * SCALE_GPS_LATLON
    f.gps_lon = extract_signed_int32(data, 30) * SCALE_GPS_LATLON
    
    # Airframe (body[34])
    f.airframe_type = extract_byte(data, 34)
    
    # Mission time (body[35-37]) - 24-bit
    f.mission_time = extract_int24(data, 35)

    return f


def parse_origin_packet(data: bytes) -> Optional[OriginData]:
    """
    Parse UAVXOriginPacket (Tag=19)
    Based on telem.c SendOriginPacket()
    Body: numWP(1) + maxVel(1) + fenceRadius(2) + homeAlt(2) + homeLat(4) + homeLon(4) = 14 bytes
    """
    if len(data) < 14:
        return None
    
    o = OriginData()
    o.num_waypoints = extract_byte(data, 0)
    o.max_velocity = extract_byte(data, 1) * 0.1
    o.fence_radius = extract_short(data, 2)
    o.home_altitude = extract_short(data, 4)
    o.home_lat = extract_signed_int32(data, 6)
    o.home_lon = extract_signed_int32(data, 10)
    
    return o


def parse_control_packet(data: bytes) -> Optional[ControlData]:
    """
    Parse UAVXControlPacket (Tag=16)
    Based on telem.c SendControlPacket()
    Body: desiredThrottle(2) + ShowAttitude(quat16+att30=46) + airframe(1) + drives + mSClock(3)
    """
    if len(data) < 49:
        return None
    
    c = ControlData()
    
    # Desired throttle (body[0-1])
    c.desired_throttle = extract_short(data, 0) * 0.1
    
    # Attitude data - starts at body[18]
    # C firmware sends via ShowAttitude(): quaternion(16), then Pitch(0), Roll(1), Yaw(2)
    pitch_start = 18
    c.desired_pitch = extract_signed_short(data, pitch_start) * SCALE_ANGLE
    c.angle_pitch = extract_signed_short(data, pitch_start + 2) * SCALE_ANGLE
    c.acc_fb = extract_signed_short(data, pitch_start + 4) * SCALE_ACC
    c.desired_rate_pitch = extract_signed_short(data, pitch_start + 6) * SCALE_RATE
    c.rate_pitch = extract_signed_short(data, pitch_start + 8) * SCALE_RATE
    
    # Roll (axis 1) - offset +10
    roll_start = pitch_start + 10
    c.desired_roll = extract_signed_short(data, roll_start) * SCALE_ANGLE
    c.angle_roll = extract_signed_short(data, roll_start + 2) * SCALE_ANGLE
    c.acc_lr = extract_signed_short(data, roll_start + 4) * SCALE_ACC
    c.desired_rate_roll = extract_signed_short(data, roll_start + 6) * SCALE_RATE
    c.rate_roll = extract_signed_short(data, roll_start + 8) * SCALE_RATE
    
    # Yaw (axis 2) - offset +20
    yaw_start = pitch_start + 20
    c.desired_yaw = extract_signed_short(data, yaw_start) * SCALE_ANGLE
    c.angle_yaw = extract_signed_short(data, yaw_start + 2) * SCALE_ANGLE
    c.acc_du = extract_signed_short(data, yaw_start + 4) * SCALE_ACC
    c.desired_rate_yaw = extract_signed_short(data, yaw_start + 6) * SCALE_RATE
    c.rate_yaw = extract_signed_short(data, yaw_start + 8) * SCALE_RATE
    
    # Airframe (body[48])
    c.airframe = extract_byte(data, 48)
    
    # PWM outputs - if available
    # body[49] = motors_and_servos count, body[50+] = PWM values (2 bytes each)
    num_drives = extract_byte(data, 49) if len(data) > 49 else 0
    pwm_start = 50
    for i in range(min(num_drives, 10)):
        offset = pwm_start + i * 2
        if offset + 1 < len(data):
            c.pwm.append(extract_short(data, offset))
    
    # PWM diagnostics (1 byte each)
    diag_start = pwm_start + num_drives * 2
    for i in range(min(num_drives, 10)):
        offset = diag_start + i
        if offset < len(data):
            c.pwm_diag.append(extract_signed_byte(data, offset))
    
    # Mission time (last 3 bytes)
    if len(data) >= 3:
        time_start = len(data) - 3
        c.mission_time = extract_int24(data, time_start)
    
    return c


def parse_packet(data: bytes, voltage_trim: float = 1.0) -> Tuple[Optional[object], int]:
    """
    Parse a packet based on its tag.
    Input: full buffer [SOH, tag, len, body..., checksum, EOT]
    Strips framing and validates checksum (XOR of tag + len + body).
    """
    if len(data) < 5:
        return None, 0

    if data[0] != 0x01:  # SOH
        return None, 0

    tag = extract_byte(data, 1)
    body_length = extract_byte(data, 2)

    if len(data) < 5:
        return None, 0

    # Tag 17 (ParamPacket) and tag 71 (TaggedReadResponse) have variable-length float32
    # payloads that can exceed 255 bytes. Use remaining data minus trailing checksum byte.
    if tag in (17, 71):
        body = data[3:-1]
    else:
        if len(data) < 3 + body_length + 1:
            return None, 0
        body = data[3:3 + body_length]

    match tag:
        case 13:
            return parse_flight_packet(body, voltage_trim), tag
        case 14:
            return parse_nav_packet(body), tag
        case 16:
            return parse_control_packet(body), tag
        case 17:
            return parse_param_packet(body), tag
        case 18:
            return parse_min_packet(body), tag
        case 19:
            return parse_origin_packet(body), tag
        case 20:
            return parse_waypoint_packet(body), tag
        case 22:
            return parse_rc_packet(body), tag
        case 51:
            return parse_ack_packet(body), tag
        case 57:
            return parse_tuning_packet(body), tag
        case 59:
            return parse_guidance_packet(body), tag
        case 60:
            return parse_altitude_control_packet(body), tag
        case 62:
            return parse_calibration_packet(body), tag
        case 63:
            return parse_airframe_name(body), tag
        case 64:
            return parse_wind_packet(body), tag
        case 66:
            return parse_serial_ports_packet(body), tag
        case 67:
            return parse_execution_time(body), tag
        case 68:
            return parse_attitude_control_packet(body), tag
        case 69:
            return parse_link_stats(body), tag
        case 71:
            return parse_param_packet(body), tag
        case _:
            return body, tag
