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


def extract_long(data: bytes, offset: int) -> int:
    """Extract little-endian 32-bit value"""
    if offset + 3 >= len(data):
        return 0
    return ((data[offset] & 0xFF)
            | ((data[offset + 1] & 0xFF) << 8)
            | ((data[offset + 2] & 0xFF) << 16)
            | ((data[offset + 3] & 0xFF) << 24))


def extract_float32(data: bytes, offset: int) -> float:
    """Extract little-endian IEEE-754 float32 value"""
    if offset + 3 >= len(data):
        return 0.0
    return struct.unpack('<f', data[offset:offset + 4])[0]


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


def extract_half(data: bytes, offset: int) -> float:
    """Extract IEEE-754 binary16 (half) little-endian, widened to float"""
    if len(data) < offset + 2:
        return 0.0
    return struct.unpack('<e', data[offset:offset+2])[0]


# Scaling constants - from telem.c
RAD_TO_DEG = 57.29578  # FC-native angles are radians
RATE_GYRO_SCALE = 0.001064225154  # raw gyro counts -> rad/s (2048 LSB/(deg/s))
SCALE_GPS_LATLON = 1e-7


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
    battery_time_remaining_sec: int = 0
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
    discovered_channels: int = 0
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
    fw_glide_offset: float = 0.0
    baro_variance: float = 0.0
    accu_variance: float = 0.0
    accu_bias_variance: float = 0.0


@dataclass
class ExecTimeData:
    """Execution time packet data (Tag=67)"""
    exec_us: int = 0
    exec_peak_us: int = 0
    revision: int = 0
    imu_rejects: int = 0
    wdt_mark: int = 0
    imu_fault_code: int = 0
    imu_fault_recoveries: int = 0
    # Newtonian slew-window peaks per axis, order Roll/Pitch/Yaw,
    # FC-native units:
    rate_slew_max_rs: tuple = (0.0, 0.0, 0.0)     # rad/s
    acc_slew_max_mps2: tuple = (0.0, 0.0, 0.0)    # m/s^2
    # IMU reject-class counters (cumulative). The four sum to imu_rejects:
    imu_rej_bus_wedge: int = 0
    imu_rej_free_fall: int = 0
    imu_rej_gyro_slew: int = 0
    imu_rej_acc_slew: int = 0


@dataclass
class LinkStatsData:
    """RC Link stats packet data (Tag=69)"""
    uplink_lq: int = 0
    uplink_snr: int = 0
    uplink_rssi: int = 0
    rc_signal_losses: int = 0
    rc_failsafes: int = 0
    rx_type: int = -1
    usart1_sr: int = 0


@dataclass
class TestResponseData:
    """Diagnostic test response (Tag=74) from tests.c.

    Wire body: testId(1) phase(1) + int32 data[16].
    data[0] carries the running sub-phase id column.
    """
    test_id: int = 0
    phase: int = 0
    data: List[int] = field(default_factory=lambda: [0] * 16)


@dataclass
class ParameterData:
    """Parameter data (Tag=17) — flat float32 array format"""
    base_idx: int = 0
    count: int = 0
    entries: Dict[int, Tuple[int, Any]] = field(default_factory=dict)
    version_name: str = ''


@dataclass
class I2CErrorData:
    """I2C bus-health packet (Tag=76) from telem.c SendI2CErrorsPacket().
    Emitted by PollDiag on counter change and as a 1 Hz refresh.

    Wire body: imuBusNo(1) magBusNo(1) baroBusNo(1) imuErrors(2)
    magErrors(2) baroErrors(2) imuAvgUs(2) imuPeakUs(2) magAvgUs(2)
    magPeakUs(2) baroAvgUs(2) baroPeakUs(2) = 21 bytes.
    Counts are the per-device SIO error counters (whichever bus each device
    sits on — I2C or SPI, which may run concurrently).
    Avg/peak are per-device SIO transaction durations in us, accumulated from
    after preflight only (so init/calibration delays are excluded).
    """
    imu_bus: int = 0
    mag_bus: int = 0
    baro_bus: int = 0
    imu_errors: int = 0
    mag_errors: int = 0
    baro_errors: int = 0
    imu_avg_us: int = 0
    imu_peak_us: int = 0
    mag_avg_us: int = 0
    mag_peak_us: int = 0
    baro_avg_us: int = 0
    baro_peak_us: int = 0
    bus1_census_count: int = 0
    bus1_census_fails: int = 0
    bus1_census: tuple = ()
    bus2_census_count: int = 0
    bus2_census_fails: int = 0
    bus2_census: tuple = ()
    regs: dict = None
    spl_diag: int = -1   # SPL06 pipeline bits (1=active 2=coeff 4=cal 8=id)
    spl_id: int = -1     # raw ID-reg read (0x10 good) or failure sentinel
    spl_coeff: int = -1  # last raw MEAS_CFG while polling coeff-ready
    spl_stage: int = -1  # 0 ok, 10 coeff stuck, 11..19 failed cal pair
    spl_coeffs: dict = None  # decoded coefficient block (c0..c30)
    spl_raw: tuple = ()      # (rawP, rawT) last inputs fed to Compensate()
    spl_raw_p: int = 0       # scalar copy - logger records scalars only
    spl_raw_t: int = 0
    spl_cfg: tuple = ()      # (prs_cfg, tmp_cfg, cfg_reg) echo-backs
    spl_tcalc: float = None  # recomputed temperature from ground truth (C)
    spl_pcalc: float = None  # recomputed pressure from ground truth (Pa)


@dataclass
class OriginData:
    """Origin packet data (Tag=19)"""
    num_waypoints: int = 0
    max_velocity: float = 0.0
    fence_radius: float = 0.0  # m (was int16 m)
    home_altitude: float = 0.0  # m (was int16 m)
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
@dataclass
class TuningData:
    """Tuning telemetry (Tag=57) - from telem.c SendTuningPacket()
    Body: flags(1) + thr(1) + 3 axes×3×f32(36) + ident block 3×(4×f32+u32)(60) = 98
    Rates in rad/s; axis output in native mixer units.
    Ident fields (Roll/Pitch/Yaw order): a, b (ARX params), sigA, sigB (variance), n (samples)."""
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
    ident_a_roll: float = 0.0
    ident_b_roll: float = 0.0
    ident_sig_a_roll: float = 0.0
    ident_sig_b_roll: float = 0.0
    ident_n_roll: int = 0
    ident_a_pitch: float = 0.0
    ident_b_pitch: float = 0.0
    ident_sig_a_pitch: float = 0.0
    ident_sig_b_pitch: float = 0.0
    ident_n_pitch: int = 0
    ident_a_yaw: float = 0.0
    ident_b_yaw: float = 0.0
    ident_sig_a_yaw: float = 0.0
    ident_sig_b_yaw: float = 0.0
    ident_n_yaw: int = 0
    has_ident: bool = False

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
    quaternion: List[float] = field(default_factory=list)
    pwm: List[float] = field(default_factory=list)
    pwm_diag: List[float] = field(default_factory=list)
    mission_time: float = 0.0
    firmware_revision: int = 0


@dataclass
class WindData:
    """Wind packet data (Tag=64) - from telem.c SendWindPacket()
    Body: speed(2) + direction(2) + Est[WN](2) + Est[WE](2) + Est[WD](2)
          + confidence(1) = 11 bytes
    """
    speed: float = 0.0
    direction: float = 0.0
    wind_n: float = 0.0
    wind_e: float = 0.0
    wind_d: float = 0.0
    confidence: float = 0.0


def parse_flight_packet(data: bytes, voltage_trim: float = 1.0) -> Optional[FlightData]:
    """
    Parse UAVXFlightPacket (Tag=13) — f16 telemetry format
    Based on telem.c SendFlightPacket(). Display-grade values ship as
    IEEE-754 binary16 (extract_half), the filter estimates and pressure
    keep f32 precision. Byte layout (body, all little-endian):
      [0-11]    Flags (12)
      [12]      State
      [13-14] BatteryVolts f16   [15-16] BatteryCurrent f16
      [17-18] mAH i16            [19-20] TimeRemaining i16
      [21-22] DesiredThrottle f16 (fraction)
      ShowAttitude: q0-q3 (4x2) + Pitch/Roll/Yaw each 5 x f16 (10 B):
        P.Desired, Angle(rad), Acc(m/s^2), R.Desired, Rate(rad/s)
      [23-30] q0..q3
      [31-40] pitch   [41-50] roll   [51-60] yaw
      [61-62] ROC m/s             [63-64] Altitude m
      [65-66] CruiseThr           [67-68] RF.Altitude m [69-70] DesiredAlt m
      [71-72] Heading rad         [73-74] DesiredHeading rad
      [75-76] TiltComp            [77-78] BattComp       [79-80] AltHoldComp
      [81-82] AccConfidence f16 fraction 0..1
      [83-84] BaroTemp C          [85-88] BaroPressure Pa f32
      [89-92] KF.Altitude m f32   [93-96][97-100][101-104] KF variances f32
      [105-106] MagHeading rad    [107-108] IMUTemp C
      [109-110] RateEnergy slot (reserved, 0.0 since 2026-09-17)  [111-112] GlideOffsetRad f16
      [113]    NavState
      [114]    CurrMaxPWMOutputs, then RawPW[] f16 (normalized, 1.0=1000uS),
               then drive balance[] f16 fraction, then mSClock(3)
    """
    if len(data) < 115:
        return None

    f = FlightData()

    # Flags (body[0-11])
    f.flags = [extract_byte(data, i) for i in range(12)]
    f.flag_bits = []
    for byte_val in f.flags:
        for bit in range(8):
            f.flag_bits.append((byte_val & (1 << bit)) != 0)

    f.flight_state = extract_byte(data, 12)

    f.battery_volts = extract_half(data, 13) * voltage_trim
    f.battery_current = extract_half(data, 15)
    f.battery_charge = extract_short(data, 17)
    f.battery_time_remaining_sec = extract_short(data, 19)

    f.desired_throttle = extract_half(data, 21)

    # Quaternion (body[23-30])
    f.q0 = extract_half(data, 23)
    f.q1 = extract_half(data, 25)
    f.q2 = extract_half(data, 27)
    f.q3 = extract_half(data, 29)

    # Quaternion occupies 23..30; axis blocks follow at 31/41/51.
    # All values arrive as f16 in FC-native units
    # (radians, rad/s, m/s^2) - no scaling applied.
    att = 31
    f.desired_pitch = extract_half(data, att)
    f.angle_pitch = extract_half(data, att + 2)
    f.acc_fb = extract_half(data, att + 4)
    f.desired_rate_pitch = extract_half(data, att + 6)
    f.rate_pitch = extract_half(data, att + 8)

    f.desired_roll = extract_half(data, att + 10)
    f.angle_roll = extract_half(data, att + 12)
    f.acc_lr = extract_half(data, att + 14)
    f.desired_rate_roll = extract_half(data, att + 16)
    f.rate_roll = extract_half(data, att + 18)

    f.desired_yaw = extract_half(data, att + 20)
    f.angle_yaw = extract_half(data, att + 22)
    f.acc_du = extract_half(data, att + 24)
    f.desired_rate_yaw = extract_half(data, att + 26)
    f.rate_yaw = extract_half(data, att + 28)

    f.roc = extract_half(data, 61)
    f.altitude = extract_half(data, 63)
    f.cruise_throttle = extract_half(data, 65)
    f.rangefinder_altitude = extract_half(data, 67)
    f.desired_altitude = extract_half(data, 69)

    f.heading = extract_half(data, 71)
    f.desired_heading = extract_half(data, 73)

    f.tilt_ff_comp = extract_half(data, 75)
    f.batt_ff_comp = extract_half(data, 77)
    f.alt_comp = extract_half(data, 79)

    f.acc_confidence = extract_half(data, 81)  # fraction 0..1

    f.baro_temp = extract_half(data, 83)
    f.baro_pressure = extract_real32(data, 85) * 0.01  # Pa -> hPa
    f.baro_altitude = extract_real32(data, 89)
    f.baro_variance = extract_real32(data, 93)
    f.accu_variance = extract_real32(data, 97)
    f.accu_bias_variance = extract_real32(data, 101)
    f.mag_heading = extract_half(data, 105)
    f.mpu_temp = extract_half(data, 107)

    f.fw_glide_offset = extract_half(data, 111) * RAD_TO_DEG

    f.nav_state = extract_byte(data, 113)

    # Drive outputs (body[114+]) - RawPW normalized (1.0 == 1000uS),
    # drive balance as a fraction of the fleet average, both f16
    f.motors_and_servos = extract_byte(data, 114)
    pwm_start = 115
    for i in range(min(f.motors_and_servos, 10)):
        if pwm_start + i * 2 + 1 < len(data):
            f.pwm.append(extract_half(data, pwm_start + i * 2))

    diag_start = pwm_start + f.motors_and_servos * 2
    for i in range(min(f.motors_and_servos, 10)):
        if diag_start + i * 2 + 1 < len(data):
            f.pwm_diag.append(extract_half(data, diag_start + i * 2))

    time_start = diag_start + f.motors_and_servos * 2
    if time_start + 2 < len(data):
        f.mission_time = extract_int24(data, time_start)

    return f



def parse_nav_packet(data: bytes) -> Optional[FlightData]:
    """
    Parse UAVXNavPacket (Tag=14)
    Based on telem.c SendNavPacket() — analog values raw float32.
    Body: 86 bytes
      [0-4]   navState, alarm, sats, fix, currWp
      [5-20]  hAcc/vAcc/sAcc/cAcc f32 (m)
      [21-24] WPBearing rad       [25-28] CrossTrack m
      [29-32] gspeed m/s          [33-36] GPS.heading rad
      [37-40] GPS.Altitude m
      [41-44] lat i32 raw         [45-48] lon i32 raw
      [49-52] NorthPosE m         [53-56] EastPosE m
      [57-59] navStateTimeout ms int24
      [60-63] GPS.dT s f32
      [64-67] hwVersion u32       [68] ubxMajor [69] pad
      [70-73] NavCorr pitch       [74-77] NavCorr roll
      [78-81] Yaw P.Error rad     [82-85] MagVar deg
    """
    if len(data) < 86:
        return None

    f = FlightData()

    f.nav_state = extract_byte(data, 0)
    f.alarm_state = extract_byte(data, 1)
    f.gps_sats = extract_byte(data, 2)
    f.gps_fix = extract_byte(data, 3)
    f.curr_wp = extract_byte(data, 4)

    # GPS accuracy (m)
    f.gps_hacc = extract_real32(data, 5)
    f.gps_vacc = extract_real32(data, 9)
    f.gps_sacc = extract_real32(data, 13)
    f.gps_cacc = extract_real32(data, 17)

    f.wp_bearing = extract_real32(data, 21)
    f.cross_track_error = extract_real32(data, 25)
    f.gps_vel = extract_real32(data, 29)
    f.gps_heading = extract_real32(data, 33)

    f.gps_altitude = extract_real32(data, 37)

    f.gps_lat = extract_signed_int32(data, 41) * SCALE_GPS_LATLON
    f.gps_lon = extract_signed_int32(data, 45) * SCALE_GPS_LATLON

    f.north_pos_e = extract_real32(data, 49)
    f.east_pos_e = extract_real32(data, 53)
    f.nav_state_timeout = extract_int24(data, 57) * 0.001

    dT_s = extract_real32(data, 60)
    f.gps_update_rate = 1.0 / dT_s if dT_s > 0 else 0.0
    f.gps_type = extract_byte(data, 68)

    # Correction terms arrive in radians; the yaw P error is published in
    # degrees for the tuning view (legacy behaviour, now actually correct).
    f.nav_p_corr = extract_real32(data, 70)
    f.nav_r_corr = extract_real32(data, 74)
    f.nav_y_corr = extract_real32(data, 78) * RAD_TO_DEG

    if len(data) >= 86:
        f.mag_var_wmm = extract_real32(data, 82)

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
    Body: frameInterval(2) + discovered(1) + flags(1) + controls u16 uS x 4
    (THR/ROL/PIT/YAW mapped slots) + physical RCInp[] u16 uS x discovered.
    Older bodies are handled by length:
      - 2026-09 format: interval(2)+discovered(1)+flags(1)+logical f32 x 12
        + physical u16 x discovered
      - original: interval(2)+discovered(1)+RC[] f32 x N
    Physical RCInp[] are raw uS as received.
    """
    if len(data) < 5:
        return None

    f = FlightData()
    f.rc_channels = []
    f.rc_raw = []
    f.discovered_channels = extract_byte(data, 2)
    f.rc_flags = 0
    f.rc_physical = []

    new_len = 4 + 4 * 2 + 2 * f.discovered_channels
    prev_len = 4 + 12 * 4 + 2 * f.discovered_channels
    if f.discovered_channels >= 1 and len(data) == new_len:
        f.rc_flags = extract_byte(data, 3)
        f.rc_channels = [max(0, extract_short(data, 4 + 2 * i)) for i in range(4)]
        base = 4 + 4 * 2
        for i in range(f.discovered_channels):
            f.rc_physical.append(extract_short(data, base + i * 2))
        f.rc_raw = list(f.rc_physical)
    elif f.discovered_channels >= 1 and len(data) == prev_len:
        f.rc_flags = extract_byte(data, 3)
        logical = [extract_real32(data, 4 + i * 4) for i in range(12)]
        base = 4 + 12 * 4
        for i in range(f.discovered_channels):
            f.rc_physical.append(extract_short(data, base + i * 2))
        f.rc_channels = [round((frac + 1.0) * 1000.0) for frac in logical]
        f.rc_raw = list(f.rc_channels)
    else:
        # Legacy: logical-only f32, no flags, no physical channels
        num_channels = (len(data) - 3) // 4
        if num_channels < 1:
            return None
        logical = [extract_real32(data, 3 + i * 4) for i in range(num_channels)]
        f.rc_channels = [round((frac + 1.0) * 1000.0) for frac in logical]
        f.rc_raw = list(f.rc_channels)

    if not f.rc_physical:
        f.rc_physical = list(f.rc_channels)

    return f


def parse_guidance_packet(data: bytes) -> Optional[GuidanceData]:
    """Parse UAVXGuidancePacket (Tag=59) - from telem.c SendGuidancePacket()
    Body: distance(2) + bearing f32(4) + elevation f32(4) + hint f32(4)
    = 14 bytes; angles arrive as raw radians
    """
    if len(data) < 14:
        return None

    g = GuidanceData()
    g.distance = extract_short(data, 0)
    g.bearing = extract_real32(data, 2) * RAD_TO_DEG
    g.elevation = extract_real32(data, 6) * RAD_TO_DEG
    g.hint = extract_real32(data, 10) * RAD_TO_DEG

    return g




def parse_test_response(data: bytes) -> Optional[TestResponseData]:
    """Parse UAVXTestResponsePacket (Tag=74) from tests.c.

    Body: testId(1) + phase(1) + int32 data[16]. data[0] is echoed phase.
    """
    if len(data) < 2:
        return None
    r = TestResponseData(test_id=data[0], phase=data[1])
    count = (len(data) - 2) // 4
    for i in range(min(count, 16)):
        r.data[i] = extract_signed_int32(data, 2 + 4 * i)
    return r


def parse_wind_packet(data: bytes) -> Optional[WindData]:
    """Body: speed f32 m/s + direction f32 rad + N/E/D f32 m/s + conf byte"""
    if len(data) < 21:
        return None

    w = WindData()
    w.speed = extract_real32(data, 0)
    w.direction = extract_real32(data, 4)
    w.wind_n = extract_real32(data, 8)
    w.wind_e = extract_real32(data, 12)
    w.wind_d = extract_real32(data, 16)
    w.confidence = extract_byte(data, 20) / 255.0
    return w


def parse_i2c_errors(data: bytes) -> Optional[I2CErrorData]:
    """Parse UAVXI2CErrorsPacket (Tag=76) from telem.c SendI2CErrorsPacket().

    Body: imuBusNo(1) magBusNo(1) baroBusNo(1) imuErrors(2) magErrors(2)
    baroErrors(2) imuAvgUs(2) imuPeakUs(2) magAvgUs(2) magPeakUs(2)
    baroAvgUs(2) baroPeakUs(2) = 21 bytes, plus an optional 12-byte census
    trailer: per bus (buses 1..2): ackCount(1) startFails(1) + up to 4
    ACKing 7-bit addresses. acks=0 with startFails=0 → alive but empty bus;
    high startFails → the bus never clocks (peripheral/wiring/power dead).
    Counts/avg/peak saturated to int16 on the FC. Timings are SIO
    (serial-I/O) transaction times — the device's bus-access cost whichever
    bus it sits on.
    """
    if len(data) < 21:
        return None
    d = I2CErrorData()
    d.imu_bus = extract_byte(data, 0)
    d.mag_bus = extract_byte(data, 1)
    d.baro_bus = extract_byte(data, 2)
    d.imu_errors = extract_short(data, 3)
    d.mag_errors = extract_short(data, 5)
    d.baro_errors = extract_short(data, 7)
    d.imu_avg_us = extract_short(data, 9)
    d.imu_peak_us = extract_short(data, 11)
    d.mag_avg_us = extract_short(data, 13)
    d.mag_peak_us = extract_short(data, 15)
    d.baro_avg_us = extract_short(data, 17)
    d.baro_peak_us = extract_short(data, 19)
    if len(data) >= 33:
        d.bus1_census_count = extract_byte(data, 21)
        d.bus1_census_fails = extract_byte(data, 22)
        d.bus1_census = tuple(
            extract_byte(data, 23 + i)
            for i in range(min(d.bus1_census_count, 4)))
        d.bus2_census_count = extract_byte(data, 27)
        d.bus2_census_fails = extract_byte(data, 28)
        d.bus2_census = tuple(
            extract_byte(data, 29 + i)
            for i in range(min(d.bus2_census_count, 4)))
    if len(data) >= 47:
        names = ("MODER", "OTYPER", "PUPDR", "AFRH", "CR1", "CCR", "TRISE")
        d.regs = {
            n: extract_short(data, 33 + 2 * i) & 0xFFFF
            for i, n in enumerate(names)
        }
    if len(data) >= 49:
        d.spl_diag = extract_byte(data, 47)
        d.spl_id = extract_byte(data, 48)
    if len(data) >= 51:
        d.spl_coeff = extract_byte(data, 49)
        d.spl_stage = extract_byte(data, 50)
    if len(data) >= 84:
        d.spl_coeffs = {
            "c0": extract_signed_short(data, 51),
            "c1": extract_signed_short(data, 53),
            "c01": extract_signed_short(data, 55),
            "c11": extract_signed_short(data, 57),
            "c20": extract_signed_short(data, 59),
            "c21": extract_signed_short(data, 61),
            "c30": extract_signed_short(data, 63),
            "c00": extract_signed_int32(data, 65),
            "c10": extract_signed_int32(data, 69),
        }
        d.spl_raw = (extract_signed_int32(data, 73),
                     extract_signed_int32(data, 77))
        d.spl_cfg = (extract_byte(data, 81), extract_byte(data, 82),
                     extract_byte(data, 83))
        _spl_verify(d)
    return d


# SPL06 scale factor for oversampling x8 (both P and T channels).
SPL06_SCALE_X8 = 7864320.0


def _spl_verify(d: I2CErrorData) -> None:
    """Recompute the datasheet compensation from the streamed ground
    truth so log.txt carries Tcalc/Pcalc alongside whatever the FC
    reports — a mismatch localises the fault to FC math vs transport."""
    c = d.spl_coeffs
    if not c or len(d.spl_raw) != 2:
        return
    raw_p, raw_t = d.spl_raw
    if raw_p == 0 and raw_t == 0:
        return
    t_lin = raw_t / SPL06_SCALE_X8
    p_lin = raw_p / SPL06_SCALE_X8
    tr = t_lin - 0.5
    d.spl_tcalc = c["c0"] * 0.5 + c["c1"] * t_lin
    d.spl_pcalc = (
        c["c00"] + p_lin * (c["c10"] + p_lin * (c["c20"] + p_lin * c["c30"]))
        + tr * (c["c01"] + p_lin * (c["c11"] + p_lin * c["c21"])))


def parse_link_stats(data: bytes) -> LinkStatsData:
    """Parse UAVXLinkStatsPacket (Tag=69)
    Body: lq(1) + snr(1) + rssi(2) + signalLosses(2) + failsafes(2) = 8 bytes
    Extended: + rx_type(1) + usart1_sr(2) = 11 bytes
    """
    if len(data) < 8:
        return None

    f = LinkStatsData()
    f.uplink_lq = extract_byte(data, 0)
    f.uplink_snr = extract_byte(data, 1)
    f.uplink_rssi = extract_short(data, 2)
    f.rc_signal_losses = extract_short(data, 4)
    f.rc_failsafes = extract_short(data, 6)
    if len(data) >= 9:
        f.rx_type = extract_byte(data, 8)
    if len(data) >= 11:
        f.usart1_sr = extract_short(data, 9)

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
    Body: wpIdx(1) + lat(4) + lon(4) + alt(4, float32 m) + vel(4, float32 m/s) + loiter(4, float32 s)
          + orbitRadius(4, float32 m) + orbitAlt(4, float32 m) + orbitVel(4, float32 m/s) + pulseWidth(4)
          + pulsePeriod(4) + action(1) = 42 bytes
    """
    if len(data) < 42:
        return None
    
    result = {
        'wp_index': extract_byte(data, 0),
        'wp_lat': extract_signed_int32(data, 1) * SCALE_GPS_LATLON,
        'wp_lon': extract_signed_int32(data, 5) * SCALE_GPS_LATLON,
        'wp_alt': extract_float32(data, 9),        # m
        'wp_velocity': extract_float32(data, 13),  # m/s
        'wp_loiter': extract_float32(data, 17),    # s
        'wp_orbit_radius': extract_float32(data, 21),  # m
        'wp_orbit_alt': extract_float32(data, 25), # m
        'wp_orbit_velocity': extract_float32(data, 29),  # m/s
        'wp_pulse_width': extract_int32(data, 33), # mS
        'wp_pulse_period': extract_int32(data, 37),  # mS
        'wp_action': extract_byte(data, 41),
    }
    
    return result


def parse_airframe_name(data: bytes) -> Optional[dict]:
    """Parse UAVXConfigPacket (Tag=63) - from telem.c SendConfigPacket()
    Body: revision string (NUL-terminated) + AFType(1) + airframe name (NUL-terminated)
    """
    # Find first NUL terminator for the revision string
    null_idx = data.find(b'\x00')
    if null_idx < 0 or null_idx + 1 >= len(data):
        return None
    revision = data[:null_idx].decode('ascii', errors='replace')
    af_type = data[null_idx + 1]
    # Airframe name (if present): starts after the AFType byte
    name = ""
    rest = data[null_idx + 2:]
    name_end = rest.find(b'\x00')
    if name_end >= 0:
        name = rest[:name_end].decode('ascii', errors='replace')
    elif rest:
        name = rest.decode('ascii', errors='replace')
    return {
        'revision': revision,
        'af_type': af_type,
        'airframe_name': name,
    }


def parse_execution_time(data: bytes) -> Optional[ExecTimeData]:
    """Parse UAVXExecutionTimePacket (Tag=67) - from telem.c SendExecutionTimeStatus()
    Body: execUs u32(4) + execPeakUs u32(4)
          + revision(2) + imuRejectCount(2) + wdtTripMark(2)
          + imuFaultCode(2) + imuFaultRecoveries(2)
          + rateSlewMax rad/s f32x3(12) + accSlewMax m/s^2 f32x3(12)
          + reject-class counters i16x4(8): busWedge, freeFall, gyroSlew,
            accelSlew (cumulative, sum == imuRejectCount)
          = 50 bytes
    """
    if len(data) < 50:
        return None

    f = ExecTimeData()
    f.exec_us = extract_int32(data, 0)
    f.exec_peak_us = extract_int32(data, 4)
    f.revision = extract_short(data, 8)
    f.imu_rejects = extract_short(data, 10)
    f.wdt_mark = extract_short(data, 12)
    f.imu_fault_code = extract_short(data, 14)
    f.imu_fault_recoveries = extract_short(data, 16)
    f.rate_slew_max_rs = (extract_real32(data, 18),
                          extract_real32(data, 22),
                          extract_real32(data, 26))
    f.acc_slew_max_mps2 = (extract_real32(data, 30),
                           extract_real32(data, 34),
                           extract_real32(data, 38))
    f.imu_rej_bus_wedge = extract_short(data, 42)
    f.imu_rej_free_fall = extract_short(data, 44)
    f.imu_rej_gyro_slew = extract_short(data, 46)
    f.imu_rej_acc_slew = extract_short(data, 48)
    return f


def parse_tuning_packet(data: bytes) -> Optional[TuningData]:
    """Parse UAVXTuningPacket (Tag=57) - from telem.c SendTuningPacket()
    Body: flags(1) + thrByte(1) + 3 axes x (desired, actual, out) f32 = 38
          + ident block 3 axes x (a,b,sigA,sigB f32 + n u32) = 60  [v2]
    Rates in rad/s; axis output in native mixer units."""
    if len(data) < 38:
        return None
    t = TuningData()
    t.flags = data[0]
    t.throttle_pct = data[1] * 0.5
    off = 2
    t.rate_desired_roll = extract_real32(data, off);    off += 4
    t.rate_actual_roll = extract_real32(data, off);     off += 4
    t.out_roll = extract_real32(data, off);             off += 4
    t.rate_desired_pitch = extract_real32(data, off);   off += 4
    t.rate_actual_pitch = extract_real32(data, off);    off += 4
    t.out_pitch = extract_real32(data, off);            off += 4
    t.rate_desired_yaw = extract_real32(data, off);     off += 4
    t.rate_actual_yaw = extract_real32(data, off);      off += 4
    t.out_yaw = extract_real32(data, off);              off += 4

    # v2 identify block: Roll, Pitch, Yaw order (same as gain block).
    if len(data) >= off + 60:
        t.has_ident = True
        t.ident_a_roll = extract_real32(data, off);        off += 4
        t.ident_b_roll = extract_real32(data, off);        off += 4
        t.ident_sig_a_roll = extract_real32(data, off);    off += 4
        t.ident_sig_b_roll = extract_real32(data, off);    off += 4
        t.ident_n_roll = extract_int32(data, off);          off += 4
        t.ident_a_pitch = extract_real32(data, off);       off += 4
        t.ident_b_pitch = extract_real32(data, off);       off += 4
        t.ident_sig_a_pitch = extract_real32(data, off);   off += 4
        t.ident_sig_b_pitch = extract_real32(data, off);   off += 4
        t.ident_n_pitch = extract_int32(data, off);         off += 4
        t.ident_a_yaw = extract_real32(data, off);         off += 4
        t.ident_b_yaw = extract_real32(data, off);         off += 4
        t.ident_sig_a_yaw = extract_real32(data, off);     off += 4
        t.ident_sig_b_yaw = extract_real32(data, off);     off += 4
        t.ident_n_yaw = extract_int32(data, off);           off += 4

    return t

def parse_calibration_packet(data: bytes) -> Optional[dict]:
    """Parse UAVXCalibrationPacket (Tag=62) - from telem.c SendCalibrationPacket()
    Body: flags(12) + refTemp f32(4) + orientation(2)
          + 3 axes x (grad,rbias,accScale,accBias f32(16) + magLive i16(2)
          + magBias f32(4) = 22)
          + LPFs i16x5(10) + population i16x8(16) + mm i32(4) + ids(2)
          + mag config regs A/B/MODE u8(3) = 119 bytes

    Cal values arrive as raw float32 in FC-native units:
      rate grads/biases in raw gyro counts (apply RATE_GYRO_SCALE),
      acc scale unitless, acc bias in raw counts, mag bias in native units.
    Live mag reads stay int16 counts. Orientation = imuQuadrant/imuFlip/
    magQuadrant currently flashed on the FC.
    """
    if len(data) < 116:
        return None

    result = {
        'flags': [extract_byte(data, i) for i in range(12)],
        'cal_ref_temp': extract_real32(data, 12),
        'rate_temp_grad': [],
        'rate_bias': [],
        'acc_scale': [],
        'acc_bias': [],
        'mag_live': [],
        'mag_bias': [],
    }

    # Currently-flashed sensor orientation @16..17
    result['sensor_quadrant'] = data[16] & 0x03
    result['sensor_flip'] = bool(data[17] & 0x01)

    off = 18
    for _ in range(3):
        result['rate_temp_grad'].append(extract_real32(data, off))
        result['rate_bias'].append(extract_real32(data, off + 4))
        result['acc_scale'].append(extract_real32(data, off + 8))
        result['acc_bias'].append(extract_real32(data, off + 12))
        result['mag_live'].append(extract_signed_short(data, off + 16))
        result['mag_bias'].append(extract_real32(data, off + 18))
        off += 22

    # LPF bandwidths (5 x i16) - informational only
    result['lpf'] = [extract_short(data, 84 + 2 * i) for i in range(5)]

    # Mag calibration octant population counts and total
    result['octants'] = [extract_short(data, 94 + 2 * i) for i in range(8)]
    result['mm'] = extract_int32(data, 110)

    result['imu_id'] = data[114]
    result['mag_id'] = data[115]

    # Optional trailing mag config registers (FC snapshot at init)
    if len(data) >= 119:
        result['mag_cfg_a'] = data[116]
        result['mag_cfg_b'] = data[117]
        result['mag_mode'] = data[118]
    else:
        result['mag_cfg_a'] = None
        result['mag_cfg_b'] = None
        result['mag_mode'] = None

    return result


def parse_serial_ports_packet(data: bytes) -> Optional[dict]:
    """Parse UAVXSerialPortsPacket (Tag=66) - from telem.c SendSerialPortStatus()
    Body: numPorts(1) + bufSize(2)
          + telemetry_flags(1) + telemetry_tx(2) + telemetry_rx(2)
          + gps_flags(1) + gps_tx(2) + gps_rx(2)
          + softserial_flags(1) + softserial_tx(2) + softserial_rx(2)
          + uart4_sr(2)
          = 20 bytes
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

    d = {
        'num_ports': extract_byte(data, 0),
        'buffer_size': extract_short(data, 1),
        'telemetry': _port('Telemetry', 3),
        'gps': _port('GPS', 8),
        'softserial': _port('SoftSerial', 13),
    }

    if len(data) >= 20:
        d['uart4_sr'] = extract_short(data, 18)
    else:
        d['uart4_sr'] = None

    return d


def parse_origin_packet(data: bytes) -> Optional[OriginData]:
    """
    Parse UAVXOriginPacket (Tag=19)
    Based on telem.c SendOriginPacket()
    Body: numWP(1) + maxVel(1) + fenceRadius(4, float32) + homeAlt(4, float32)
          + homeLat(4) + homeLon(4) = 18 bytes
    """
    if len(data) < 18:
        return None
    
    o = OriginData()
    o.num_waypoints = extract_byte(data, 0)
    o.max_velocity = extract_byte(data, 1) * 0.1
    o.fence_radius = extract_float32(data, 2)
    o.home_altitude = extract_float32(data, 6)
    o.home_lat = extract_signed_int32(data, 10)
    o.home_lon = extract_signed_int32(data, 14)
    
    return o


def parse_control_packet(data: bytes) -> Optional[ControlData]:
    """
    Parse UAVXControlPacket (Tag=16)
    Based on telem.c SendControlPacket()
    Body: desThr f32 + ShowAttitude(quat 4xf32 + 3 axes x f32x5) + afType(1)
          + drives(count u8 + RawPW f32xN + bal f32xN) + mSClock(3)
    All fields are native floats: throttle fraction, angles/rates in
    radians & rad/s (converted to deg here), accel as m/s^2,
    RawPW normalized (1.0 == 1000uS).
    """
    MIN_CONTROL_BODY = 82  # desThr + quat + 3 axes + afType + drive count
    if len(data) < MIN_CONTROL_BODY:
        return None

    c = ControlData()

    # Desired throttle - fraction 0..1
    c.desired_throttle = extract_real32(data, 0)

    # Attitude data - quaternion at body[4], then Pitch/Roll/Yaw axis blocks
    AXIS_BLOCK = 20  # Desired, Angle, Acc, RateDesired, Rate (f32 each)
    quat_start = 4
    c.quaternion = [
        extract_real32(data, quat_start),
        extract_real32(data, quat_start + 4),
        extract_real32(data, quat_start + 8),
        extract_real32(data, quat_start + 12),
    ]

    # FC axis order is Pitch, Roll, Yaw; GCS attr names are fb/lr/du
    for a, (axis, acc_attr) in enumerate((('pitch', 'acc_fb'),
                                          ('roll', 'acc_lr'),
                                          ('yaw', 'acc_du'))):
        base = quat_start + 16 + a * AXIS_BLOCK
        setattr(c, f'desired_{axis}',
                extract_real32(data, base) * RAD_TO_DEG)
        setattr(c, f'angle_{axis}',
                extract_real32(data, base + 4) * RAD_TO_DEG)
        # Acc is already m/s^2 on the wire
        setattr(c, acc_attr, extract_real32(data, base + 8))
        setattr(c, f'desired_rate_{axis}',
                extract_real32(data, base + 12) * RAD_TO_DEG)
        setattr(c, f'rate_{axis}',
                extract_real32(data, base + 16) * RAD_TO_DEG)

    # Airframe (body[80])
    c.airframe = extract_byte(data, 80)

    # Drives: count byte then N normalized PWM floats + N balance fractions
    num_drives = extract_byte(data, 81)
    num_drives = min(num_drives, 12)
    pwm_start = 82
    diag_start = pwm_start + num_drives * 4
    for i in range(num_drives):
        offset = pwm_start + i * 4
        if offset + 3 < len(data):
            c.pwm.append(extract_real32(data, offset))
    for i in range(num_drives):
        offset = diag_start + i * 4
        if offset + 3 < len(data):
            c.pwm_diag.append(extract_real32(data, offset))

    # Mission time (last 3 bytes)
    if len(data) >= 3:
        time_start = len(data) - 3
        c.mission_time = extract_int24(data, time_start)

    return c


def parse_packet(data: bytes, voltage_trim: float = 1.0) -> Tuple[Optional[object], int]:
    """
    Parse a packet based on its tag.
    Input: full buffer [SOH, tag, len(u16 LE), body..., checksum, EOT]
    Strips framing and validates checksum (XOR of tag + len + body).
    """
    if len(data) < 6:
        return None, 0

    if data[0] != 0x01:  # SOH
        return None, 0

    tag = extract_byte(data, 1)
    body_length = extract_short(data, 2)  # u16 little-endian

    if len(data) < 4 + body_length + 1:
        return None, 0
    body = data[4:4 + body_length]

    match tag:
        case 13:
            return parse_flight_packet(body, voltage_trim), tag
        case 14:
            return parse_nav_packet(body), tag
        case 16:
            return parse_control_packet(body), tag
        case 17:
            return parse_param_packet(body), tag
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
        case 69:
            return parse_link_stats(body), tag
        case 70:
            if len(body) < 1:
                return None, tag
            from types import SimpleNamespace
            ns = SimpleNamespace()
            ns.flight_state = body[0]
            return ns, tag
        case 71:
            return parse_param_packet(body), tag
        case 74:
            return parse_test_response(body), tag
        case 76:
            return parse_i2c_errors(body), tag
        case _:
            return body, tag
