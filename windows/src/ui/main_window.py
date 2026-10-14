# ui/main_window.py - Fixed with continuous polling and all packet types
"""
UAVX Groundstation - Main Window

Calibration and flight-safety work 2026-09-27 by OpenCode, Big Pickle & Friends.
"""

import sys
import os
import struct
import time
import math
import zlib
from datetime import datetime
from typing import Optional

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol_constants import *
from protocol_enums import PacketTag, ParamIndex, FlightState, NavState, Config1Bits, Config2Bits, MiscCommand, AirframeType
from packet_parser import *
from widgets.attitude_indicator import AttitudeIndicator, ExecTimeBar
from widgets.anomaly_box import AnomalyBox

from core.data_manager import data_manager
from core.packet_logger import packet_logger
from core.ack_handler import ack_handler

from ui.parameter_window import ParameterWindow
from parameters import PARAM_DISPLAY_MULT, PARAM_TYPES
import workspace
from ui.nav_window import NavWindow
from ui.calibration_window import CalibrationWindow
from ui.misc_window import MiscWindow
from ui.dfu_flasher import DfuFlasherWindow
from core.speech import SpeechController, SpeechLevel, LEVEL_LABELS
from logger.logger import AnomalyLogger
from logger.rawlog import RawLogWriter
from core.frame_decoder import FrameDecoder
from ui.replay_window import ReplayWindow
from ui.trace_viewer import TraceViewer
from ui.identify_window import IdentifyWindow


FLAG_USING_GPS_ALT = 1
FLAG_USING_RANGEFINDER_ALT = 22

# Trace-type combo sync-state styling: the selector's BACKGROUND goes green when
# the FC has confirmed the value (download readback or upload ACK), orange while
# a locally-chosen value still awaits confirmation, default when unknown.
_TRACE_STATE_STYLE = {
    'neutral': (
        "QComboBox { background-color: #ffffff; color: #101418; }"
        "QComboBox::drop-down { border: none; background: transparent; }"
        "QComboBox QAbstractItemView { background: #ffffff; color: #101418; "
        "selection-background-color: #4a90d9; selection-color: #ffffff; }"),
    'synced': (
        "QComboBox { background-color: #27ae60; color: #0d1216; border: 1px solid #1f7a45; }"
        "QComboBox::drop-down { border: none; background: transparent; }"
        "QComboBox QAbstractItemView { background: #ffffff; color: #101418; "
        "selection-background-color: #4a90d9; selection-color: #ffffff; }"),
    'pending': (
        "QComboBox { background-color: #e67e22; color: #0d1216; border: 1px solid #b45f1b; }"
        "QComboBox::drop-down { border: none; background: transparent; }"
        "QComboBox QAbstractItemView { background: #ffffff; color: #101418; "
        "selection-background-color: #4a90d9; selection-color: #ffffff; }"),
}

# Speak a compass/distance callout only when the FC's home-relative offset puts
# the aircraft beyond this range, and only re-speak every N seconds while far.
# The 8-point compass is the coarse 45 deg resolution the pilot asked for.
_DIRECTION_FAR_M = 500
_DIRECTION_SPEAK_PERIOD_S = 10.0
_DIRECTION_POINTS = ("North", "North-East", "East", "South-East",
                     "South", "South-West", "West", "North-West")

# Per-param echo budget for the tag-17 ACK-driven transport.
#
# The 2026-09-29 first attempt set this to 400 ms, justified as "~20x the 20 ms
# telemetry cadence". That was the wrong reference: telemetry cadence says
# nothing about the FC's command turn-around, and it was sized far below the
# real one. It is a backstop against a genuinely dropped echo, not a normal-case
# deadline — correctness does NOT depend on it, because _on_param_write_echo
# reconciles an echo by INDEX rather than by strict lockstep (see there). So it
# is deliberately generous: too long costs a few seconds on a real fault, too
# short costs correctness. Re-size from a measured turn-around distribution once
# a clean bench run gives one; do not shrink it "because telemetry is 20 ms".
PARAM_WRITE_ECHO_TIMEOUT_MS = 6000.0

# After the last param is sent, hold the batch open this long for stragglers
# before declaring the remainder unconfirmed. This is what lets an echo that
# arrives after its own timeout still rescue its index.
PARAM_WRITE_DRAIN_MS = 5000.0
PARAM_WRITE_CHUNK_DELAY_MS = 15.0  # gap between chunks of param writes
PARAM_WRITE_CHUNK_SIZE = 25

# Single source of truth: one entry per FC Flags bit (main.h struct order).
# key==bit number matches flags order; (key, name, tooltip) drives both the
# compact curated view and the "Show all flags" grid so they never drift.
FLAG_BIT_DEFS = {
    0:  ('alt_hold', 'AltHold', 'Altitude Hold'),
    1:  ('unused_gps_alt', 'unusedGPSAlt', 'Reserved: F.UsingGPSAltitude never set by FC'),
    2:  ('vrs', 'VRS', 'Rapid Descent Hazard'),
    3:  ('land_sw', 'LandSw', 'Landing Switch'),
    4:  ('level', 'Level', 'Near Level'),
    5:  ('low_batt', 'LowBatt', 'Low Battery'),
    6:  ('gps_ok', 'GPS', 'GPS Valid'),
    7:  ('origin', 'Origin', 'Origin Valid'),
    8:  ('baro_fail', 'Baro', 'Barometer Failure'),
    9:  ('unused_imu_fail', 'unusedIMUFail', 'Reserved: F.IMUFailure never set by FC'),
    10: ('mag_fail', 'Mag', 'Magnetometer Failure'),
    11: ('unused_gps_fail', 'unusedGPSFail', 'Reserved: F.GPSFailure never set by FC'),
    12: ('att_hold', 'AttHold', 'Attitude Hold'),
    13: ('thr_move', 'ThrMove', 'Throttle Moving'),
    14: ('hold_alt', 'HoldAlt', 'Holding Altitude'),
    15: ('nav', 'Nav', 'Navigate'),
    16: ('rth', 'RTH', 'Return Home'),
    17: ('wp_ach', 'WPAch', 'Waypoint Achieved'),
    18: ('wp_cent', 'WPCent', 'Waypoint Centred'),
    19: ('orbit', 'Orbit', 'Orbiting WP'),
    20: ('auto_land', 'AutoLand', 'RTH Auto Descend'),
    21: ('baro', 'Baro', 'Baro Active'),
    22: ('rf', 'RF', 'Rangefinder Active'),
    23: ('use_rf', 'RF', 'Using Rangefinder'),
    24: ('poi', 'POI', 'Using POI'),
    25: ('pass_thru', 'PassThru', 'Bypass'),
    26: ('angle', 'Angle', 'Angle Control'),
    27: ('emul', 'Emul', 'Emulation'),
    28: ('offset', 'Offset', 'Offset Origin Valid'),
    29: ('armed', 'Armed', 'Drives Armed'),
    30: ('acc_z_bump', 'Bump', 'Acc Z Bump'),
    31: ('unused_dc_motors', 'unusedDCMotors', 'Reserved: F.DCMotorsDetected never set by FC'),
    32: ('unused_sat', 'unusedSat', 'Reserved: F.Saturation never set by FC'),
    33: ('dump_bb', 'DumpBB', 'Dumping Black Box'),
    34: ('cfg_pend', 'CfgPend', 'Config uncommitted on FC (ConfigChanged set, not yet flashed)'),
    35: ('signal', 'Signal', 'Signal'),
    36: ('wp_nav', 'WPNav', 'WP Navigation'),
    37: ('imu', 'IMU', 'IMU Active'),
    38: ('mag', 'Mag', 'Magnetometer Active'),
    39: ('armed2', 'Drives', 'Drives Active'),
    40: ('thr_open', 'ThrOpen', 'Throttle Open'),
    41: ('mag_cal', 'MagCal', 'Mag Calibrated'),
    42: ('rc_map_fail', 'RCMap', 'RC Map Fail'),
    43: ('new_alt', 'NewAlt', 'New Altitude Value'),
    # 44 is the ACCELEROMETER cal flag; ErectRateGyros() never sets it. Was mislabelled gyro_cal.
    44: ('acc_cal', 'IMUCal', 'Accel Calibrated'),
    45: ('fence_alarm', 'Fence', 'Fence Alarm'),
    46: ('excess_lift', 'ExLift', 'Excess Lift: throttle at floor but still climbing (lost alt control)'),
    47: ('imu_fault_lat', 'IMU', 'IMU Fault Latched'),
    48: ('new_baro', 'NewBaro', 'New Baro Value'),
    49: ('beeper', 'Beeper', 'Beeper In Use'),
    50: ('unused50', 'unused50', 'Reserved'),
    51: ('soaring', 'Soaring', 'Soaring'),
    52: ('gps_vel', 'GPSVel', 'Valid GPS Velocity'),
    53: ('lq', 'LQ', 'Link Quality (< 80%)'),
    54: ('unused_hover', 'unusedHover', 'Reserved: F.Hovering never set by FC'),
    55: ('as_active', 'AirSpd', 'Airspeed Sensor Active'),
    56: ('yaw_active', 'YawActive', 'Yaw Active'),
    57: ('have_gps', 'HaveGPS', 'Have GPS'),
    58: ('fs', 'FS', 'Failsafe (LQ < 50%)'),
    59: ('new_nav', 'NewNav', 'New Nav Update'),
    60: ('nv_mem', 'NVMem', 'Have NV Memory'),
    61: ('rapid_desc', 'Diving', 'Using Rapid Descent'),
    62: ('turn_wp', 'TurnWP', 'Using Turn To WP'),
    63: ('glide', 'Glide', 'Gliding'),
    64: ('new_mag', 'NewMag', 'New Mag Values'),
    65: ('alt_hold_alarm', 'AHAlarm', 'Using Alt Hold Alarm'),
    66: ('new_gps_pos', 'NewGPSPos', 'New GPS Position'),
    67: ('sio_fatal', 'SIOFatal', 'Serial I/O Fatal'),
    68: ('gps_pkt', 'GPSPkt', 'GPS Packet Received'),
    69: ('wind_est', 'WindEst', 'Wind Estimate Valid'),
    70: ('x_track', 'XTrack', 'Cross Track Active'),
    71: ('nav_enabled', 'NavEnabled', 'Navigation Enabled'),
    72: ('forced_landing', 'ForcedLan', 'Forced Landing'),
    73: ('unused73', 'unused73', 'Reserved'),
    74: ('drive_sym', 'DriveSym', 'Enforce Drive Symmetry'),
    75: ('rssi', 'RSSI', 'RSSI (< -90 dBm)'),
    76: ('ext_mag', 'ExtMag', 'External Mag Detected'),
    77: ('snr', 'SNR', 'SNR Low (< -10 dB)'),
    78: ('offset_home', 'OffsetHome', 'Using Offset Home'),
    79: ('gps_hdg', 'GPSHdg', 'Valid GPS Heading'),
    80: ('bad_bus', 'BadBus', 'Bad Bus Device Config'),
    81: ('dive_mode', 'Dive', 'Dive Mode'),
    82: ('test_active', 'TestActive', 'Test Active'),
    83: ('unused83', 'unused83', 'Reserved'),
    84: ('unused84', 'unused84', 'Reserved'),
    85: ('unused85', 'unused85', 'Reserved'),
    86: ('unused86', 'unused86', 'Reserved'),
    87: ('unused87', 'unused87', 'Reserved'),
    88: ('unused88', 'unused88', 'Reserved'),
    89: ('unused89', 'unused89', 'Reserved'),
    90: ('unused90', 'unused90', 'Reserved'),
    91: ('unused91', 'unused91', 'Reserved'),
    92: ('unused92', 'unused92', 'Reserved'),
    93: ('unused93', 'unused93', 'Reserved'),
    94: ('unused94', 'unused94', 'Reserved'),
    95: ('unused95', 'unused95', 'Reserved'),
}

# Logical groups (name -> FC bit numbers). Covers every bit 0..95 exactly once.
FLAG_GROUPS = [
    ('Attitude & Control', [12, 26, 4, 56]),
    ('Altitude', [0, 14, 23, 21, 61, 2, 46, 65]),
    ('Navigation', [15, 36, 16, 17, 18, 19, 24, 70]),
    ('RTH / Landing', [20, 72, 45, 3, 30]),
    ('Origin / Home', [7, 28, 78]),
    ('GPS', [6, 52, 79]),
    ('Sensors / Health', [47, 8, 10, 44, 41, 76, 55, 22]),
    ('RC & Input', [35, 53, 75, 58, 77, 81]),
    ('Soaring / Glide', [51, 63, 69]),
    ('Flight Mode / Throttle', [25, 40, 13]),
    ('Arming / Power', [29, 39, 5]),
    ('System / Debug', [33, 34, 67, 49, 80]),
]

# The neutral look of a flag label: name shown, no state asserted. Defined
# once and shared by the builder and the reset so "unknown" cannot drift
# between the two.
_FLAG_NEUTRAL_QSS = """
    background-color: #ddd;
    color: #666;
    font-weight: normal;
    border: 1px solid #999;
    border-radius: 3px;
    padding: 3px 6px;
"""

class TelemetryThread(QThread):
    """Background thread for serial telemetry"""
    data_received = pyqtSignal(bytes)
    raw_bytes = pyqtSignal(bytes)
    connected = pyqtSignal(bool)
    error = pyqtSignal(str)
    
    def __init__(self, port: str, baud: int):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False
        self.serial = None
        self._is_closing = False
        self.verbose_serial = os.environ.get("UAVXGS_VERBOSE_SERIAL", "0") == "1"

    # Auto-reconnect pacing after a dropped CDC link. A bench USB reset takes
    # the port away for ~30 s (host re-enumeration); retry until it returns.
    _RECONNECT_BACKOFF_START_S = 1.0
    _RECONNECT_BACKOFF_MAX_S = 5.0
    
    def run(self):
        import serial
        import serial.tools.list_ports
        
        self.running = True
        self._is_closing = False
        
        try:
            if self.port == "auto":
                ports = serial.tools.list_ports.comports()
                for p in ports:
                    if "USB" in p.description or "ttyUSB" in p.device:
                        self.port = p.device
                        break
                else:
                    self.error.emit("No USB serial port found")
                    return
            elif not os.path.exists(self.port):
                import serial.tools.list_ports
                available = [p.device for p in serial.tools.list_ports.comports()]
                self.error.emit(
                    "Port %s does not exist. Available: %s"
                    % (self.port, ", ".join(available) if available else "none"))
                return

            self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
            # First successful open: signal the GUI connected NOW, not only on
            # a later reopen. A reopen (line 274) exists for dropped CDC links;
            # a stable UART adapter never drops, so without this the GUI stays
            # stuck at "Connecting..." and never triggers the on_connected(True)
            # param readback / rawlog start / read-lock release.
            self.connected.emit(True)
            self._link_loop(serial)

        except Exception as e:
            if not self._is_closing:
                self.error.emit(str(e))
        finally:
            self._close_serial()
            self.connected.emit(False)

    def _link_loop(self, serial):
        """Read/decode loop with automatic reconnect.

        A vanished or stalled CDC device surfaces as OSError from read().
        That is retryable: a bench USB reset takes the port away for ~30 s
        while the host re-enumerates. We emit connected(False), close the
        port, then keep trying to reopen with a bounded backoff until the
        device returns (or stop()). A non-OSError exception is a decode or
        logic bug - log it and stand down instead of hot-reopening a broken
        loop forever. A dropped link never feeds bytes into the old rawlog:
        the GUI's on_connected(False) already stopped it, and a successful
        reopen starts a fresh rawlog session via on_connected(True).
        """
        buffer = bytearray()
        packet = bytearray()
        esc_flag = False
        backoff = self._RECONNECT_BACKOFF_START_S

        while self.running and not self._is_closing:
            try:
                if self.serial is None or not self.serial.is_open:
                    # Port was dropped (or stop() is closing us). Try to
                    # reopen the device; reopen failures are EXPECTED until
                    # the USB stack re-enumerates, so they just pace the
                    # backoff instead of killing the link.
                    try:
                        self.serial = serial.Serial(
                            self.port, self.baud, timeout=0.1)
                    except Exception:
                        if not self._reconnect_pause(backoff):
                            break
                        backoff = min(backoff * 2.0,
                                      self._RECONNECT_BACKOFF_MAX_S)
                        continue
                    self.connected.emit(True)
                    backoff = self._RECONNECT_BACKOFF_START_S
                    buffer.clear()
                    packet.clear()
                    esc_flag = False
                    continue

                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    if data and self.verbose_serial:
                        print(f"[SERIAL] Read {len(data)} bytes: {data[:20].hex()}{'...' if len(data) > 20 else ''}")
                    if data:
                        self.raw_bytes.emit(bytes(data))
                    buffer.extend(data)
                    
                    i = 0
                    while i < len(buffer):
                        ch = buffer[i]
                        
                        if esc_flag:
                            packet.append(ch)
                            esc_flag = False
                            i += 1
                            continue
                        
                        if ch == ESC:
                            esc_flag = True
                            i += 1
                            continue
                        
                        if ch == SOH:
                            if len(packet) > 3:
                                txt = bytes(packet).decode("ascii", "replace")
                                printable = all(32 <= b < 127 or b in (10, 13)
                                                for b in packet)
                                if printable:
                                    print(f"[BOOT] {txt.strip()}")
                                else:
                                    print(f"[SERIAL] Discarding "
                                          f"{len(packet)} bytes before SOH")
                            packet.clear()
                            packet.append(ch)
                            i += 1
                            continue
                        
                        if ch == EOT:
                            if len(packet) >= 3:
                                self.data_received.emit(bytes(packet))
                            packet.clear()
                            i += 1
                            continue
                        
                        packet.append(ch)
                        i += 1
                    
                    buffer.clear()
                
            except OSError as e:
                if self._is_closing:
                    break
                print(f"<telem> link error: {e!r} -> reconnecting", flush=True)
                self.connected.emit(False)
                self._close_port()
                # The loop top reopens (or paces the backoff if not yet back)
                
            except Exception as e:
                if not self._is_closing:
                    print(f"<telem> decode error (not reconnecting): {e!r}",
                          flush=True)
                    self.connected.emit(False)
                break
            
            QThread.msleep(10)

    def _reconnect_pause(self, seconds):
        """Sleep up to `seconds` in small slices so stop() can interrupt."""
        deadline = time.monotonic() + seconds
        while self.running and not self._is_closing \
                and time.monotonic() < deadline:
            QThread.msleep(50)
        return self.running and not self._is_closing

    def _close_port(self):
        if self.serial:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except (OSError, Exception):
                pass
            self.serial = None
    
    def _close_serial(self):
        self._is_closing = True
        self.running = False
        self._close_port()
    
    def stop(self):
        self._is_closing = True
        self.running = False
        self._close_serial()
        self.wait()


# ============================================================================
# Preflight term labels (bit order mirrors FC alarms.h PfTerms). Keyed by bit
# index to match the FC flag-bit table above. A set bit = term BLOCKING.
# ============================================================================
PF_TERM_DEFS = {
    0: ('signal', 'Signal', 'No RC signal'),
    1: ('rc_map', 'RCMap', 'RC map is invalid'),
    2: ('rc_start', 'RCStart', 'RC start stick is not centred'),
    3: ('armed', 'Armed', 'Armed on the first pass'),
    4: ('throttle', 'ThrOpen', 'Throttle is not at idle'),
    5: ('navsw', 'NavSw', 'Nav switch is not in LOW'),
    6: ('imu', 'IMU', 'IMU not active'),
    7: ('acc_cal', 'IMUCal', 'Accelerometer not calibrated'),
    8: ('baro', 'Baro', 'Baro not active'),
    9: ('mag', 'Mag', 'Mag in use but not calibrated/active'),
    10: ('batt', 'Batt/Nav', 'Low battery, serial fatal, RTH or navigation active'),
}


def pf_term_labels(mask):
    """Names of the preflight terms blocking, FC-side bit order."""
    return [PF_TERM_DEFS[b][2] for b in sorted(PF_TERM_DEFS)
            if mask & (1 << b)]


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.flight_data: FlightData = FlightData()
        self.nav_data: Optional[FlightData] = None
        self.link_stats: Optional[LinkStatsData] = None
        self.wind_data: Optional[WindData] = None
        self.serial_ports: Optional[dict] = None
        self.exec_time: Optional[ExecTimeData] = None
        self.i2c_errors: Optional[I2CErrorData] = None
        self._last_reported_reset_cause = -1
        self._last_reported_boot_diag = -1
        self._last_reported_imu_fault_code = -1
        self._last_reported_imu_fault_detections = -1
        self.origin_data: Optional[OriginData] = None
        self.control_data: Optional[ControlData] = None
        self.guidance_data: Optional[GuidanceData] = None
        self.telemetry: Optional[TelemetryThread] = None
        self.connected = False
        self.current_flag_bits = []
        self.flag_labels = {}
        self.config_labels = []
        self.config1_cache = 0
        self.config2_cache = 0
        
        self.param_window: Optional[ParameterWindow] = None
        self._pending_params = None
        self._pending_typed_params = None
        self._pending_param_verification = False
        self._armed_boot_verify = False
        self.nav_window: Optional[NavWindow] = None
        self.calib_window: Optional[CalibrationWindow] = None
        self.misc_window: Optional[MiscWindow] = None
        self.dfu_flasher_window: Optional[DfuFlasherWindow] = None
        self._trace_viewer = None
        self._identify_window: Optional[IdentifyWindow] = None
        
        self.speech = SpeechController()
        self.anomaly_logger = AnomalyLogger()
        self._anomaly_pending = False
        self.raw_logger = RawLogWriter()
        self.replaying = False
        self.replay_window: Optional[ReplayWindow] = None
        self._last_spoken_batt = -999
        self._last_spoken_gps_ok = False
        self._last_spoken_armed = False
        self._last_spoken_wp_ach = False
        self._last_spoken_flight_state = -1
        self._last_spoken_alarm_state = -1
        self._last_spoken_low_batt = False
        self._last_spoken_alt = -999999.0
        self._last_direction_speak = -999999.0
        self._pending_flight_state = -1
        self._state_change_time = 0.0
        self.log_dir = None
        self._bb_chunks = {}
        self._last_wp_data = None
        self._mag_live: Optional[list] = None
        self._param_write_list = []
        self._param_write_port = None
        self._param_write_on_complete = None
        self._param_write_progress = None
        self._param_write_total = 0
        self._param_write_index = 0
        # ACK-driven tag-17 transport (2026-09-29). The pump used to schedule the
        # next param on a fixed 5 ms timer and never waited for the FC, so
        # on_complete - and therefore the tag-72 flash commit that depends on it
        # - fired the moment the last byte hit the wire, while the FC was still
        # working through the queue. That let the commit pack a partially
        # applied RAM image. _param_write_await_idx is the single index in
        # flight; it clears only on its own echo, or on the watchdog recording it
        # as unconfirmed so the caller can refuse to commit.
        self._param_write_await_idx = None
        self._param_write_await_value = None
        self._param_write_await_sent_ms = 0.0
        # Every index SENT and not yet echoed, idx -> value we sent. The
        # authoritative "unconfirmed" set: it is derived from this, not
        # appended to as timeouts happen, so an echo arriving late can still
        # rescue its index instead of being discarded.
        self._param_write_sent_pending = {}
        self._param_write_drain_until_ms = 0.0
        self._param_write_ack_timer = QTimer(self)
        self._param_write_ack_timer.setInterval(20)
        self._param_write_ack_timer.timeout.connect(self._check_param_write_ack)
        self._revision_from_afname = False
        self._pending_flash_airframe_name = None
        self._pending_configuration_required = None
        self._last_tune = None
        self._last_rx_time = 0.0
        self._pf_term_mask = 0
        self._log_level = "All"
        self._flight_log_only = False  # terminal: show only FLIGHT packets
        self._trace_combo_loading = False
        self._trace_fc_value = None
        self._trace_write_pending = None

        self.setup_ui()
        self.setup_menu()
        self.setup_connections()
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(100)
        
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.check_connection)
        self.status_timer.start(1000)
        
        # BB dump watchdog: finalize when chunks stop arriving. The FC ends the
        # dump after a final chunk that is either short (<128 B, e.g. trailer
        # tail) OR exactly TRACE_HEADER_SIZE (128 B) when there is no capture —
        # the all-zero header fills one whole chunk. A strict "last chunk <128"
        # test never fires for that exact-128 case, so guard with idle time.
        self._bb_dump_timer = QTimer(self)
        self._bb_dump_timer.setSingleShot(True)
        self._bb_dump_timer.timeout.connect(self._finalize_bb_dump)

        # Periodic tag-57 poll (~1 Hz) to keep the Identify window fresh.
        self._identify_poll_timer = QTimer(self)
        self._identify_poll_timer.timeout.connect(self._poll_tuning)
        
        self.setWindowTitle("UAVX Groundstation")
        self.setMinimumSize(1320, 990)
        
        self.load_settings()

        # Spoken boot greeting — lets you confirm voice feedback works before
        # any telemetry events fire (level must be All or higher to hear it).
        QTimer.singleShot(1500, self._speak_boot_greeting)
    
    def setup_menu(self):
        menubar = self.menuBar()
        
        file_menu = menubar.addMenu("&File")
        log_folder_action = QAction("Set UAVX &Workspace Folder…", self)
        log_folder_action.triggered.connect(self.select_workspace_folder)
        file_menu.addAction(log_folder_action)
        replay_action = QAction("&Replay Log…", self)
        replay_action.setShortcut("Ctrl+R")
        replay_action.triggered.connect(self.open_replay)
        file_menu.addAction(replay_action)
        trace_action = QAction("Open &Trace Dump…", self)
        trace_action.triggered.connect(self.show_trace_viewer)
        file_menu.addAction(trace_action)
        file_menu.addSeparator()
        exit_action = QAction("E&xit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        view_menu = menubar.addMenu("&View")
        
        params_action = QAction("&Parameters", self)
        params_action.setShortcut("Ctrl+P")
        params_action.triggered.connect(self.show_parameter_window)
        view_menu.addAction(params_action)
        
        nav_action = QAction("&Navigation", self)
        nav_action.setShortcut("Ctrl+N")
        nav_action.triggered.connect(self.show_nav_window)
        view_menu.addAction(nav_action)
        
        calib_action = QAction("&Calibration", self)
        calib_action.setShortcut("Ctrl+C")
        calib_action.triggered.connect(self.show_calibration_window)
        view_menu.addAction(calib_action)

        misc_action = QAction("&Misc Packets", self)
        misc_action.setShortcut("Ctrl+M")
        misc_action.triggered.connect(self.show_misc_window)
        view_menu.addAction(misc_action)

        view_menu.addSeparator()
        
        
        tools_menu = menubar.addMenu("&Tools")
        flash_action = QAction("&Flash Firmware", self)
        flash_action.setShortcut("Ctrl+F")
        flash_action.triggered.connect(self.show_dfu_flasher)
        tools_menu.addAction(flash_action)

        help_menu = menubar.addMenu("&Help")
        voice_action = QAction("&Test Spoken Feedback", self)
        voice_action.setShortcut("Ctrl+T")
        voice_action.triggered.connect(self._speak_boot_greeting)
        help_menu.addAction(voice_action)
        about_action = QAction("&About", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
    
    def show_parameter_window(self):
        if self.param_window is None:
            self.param_window = ParameterWindow(self)
            print("🪟 Created new ParameterWindow")
            if self._pending_typed_params is not None:
                self.param_window.update_params_from_typed(self._pending_typed_params)
                self._pending_typed_params = None
            if self._pending_flash_airframe_name is not None:
                self.param_window.set_flash_airframe_name(self._pending_flash_airframe_name)
                self._pending_flash_airframe_name = None
            if self._pending_configuration_required is not None:
                self.param_window.set_configuration_required(
                    self._pending_configuration_required)
                self._pending_configuration_required = None

        self._pending_configuration_required = None

        self.param_window.show()
        self.param_window.raise_()
        self.param_window.activateWindow()
    
    def show_replay_window(self):
        if self.replay_window is None:
            self.replay_window = ReplayWindow(self)
        self.replay_window.show()
        self.replay_window.raise_()
        self.replay_window.activateWindow()

    def show_trace_viewer(self):
        """File > Open Trace Dump… — open a TRAC snapshot dump off disk (e.g.
        a synthetic sample from tests/trace_samples or a previously saved
        dump). Live dumps arrive through _finalize_bb_dump instead."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Trace Dump", self._log_dir(),
            "Trace Dumps (*.bin);;All Files (*)")
        if not path:
            return
        with open(path, 'rb') as f:
            data = f.read()
        if data[:4] != b'TRAC':
            QMessageBox.warning(
                self, "Trace Viewer",
                f"{path}\nis not a TRAC snapshot dump (magic != 'TRAC').")
            return
        try:
            viewer = TraceViewer(data, source=os.path.basename(path))
        except ValueError as e:
            QMessageBox.warning(self, "Trace Viewer", str(e))
            return
        self._trace_viewer = viewer
        viewer.show()
        viewer.raise_()
        viewer.activateWindow()

    def open_replay(self):
        """Enter replay mode: drop the live FC connection, then open the
        replay window. Connecting later terminates the replay immediately."""
        if self.telemetry and self.telemetry.isRunning():
            self.disconnect()
        self.raw_logger.forbidden = True
        self.replaying = True
        self.show_replay_window()

    def show_nav_window(self):
        if self.nav_window is None:
            self.nav_window = NavWindow(self)
        self.nav_window.show()
        self.nav_window.raise_()
        self.nav_window.activateWindow()
    
    def show_calibration_window(self):
        if self.calib_window is None:
            self.calib_window = CalibrationWindow(self)
        self.calib_window.show()
        self.calib_window.raise_()
        self.calib_window.activateWindow()
    
    def show_misc_window(self):
        if self.misc_window is None:
            self.misc_window = MiscWindow(self)
        self.misc_window.show()
        self.misc_window.raise_()
        self.misc_window.activateWindow()

    def show_dfu_flasher(self):
        if self.dfu_flasher_window is None:
            self.dfu_flasher_window = DfuFlasherWindow(self)
        self.dfu_flasher_window.show()
        self.dfu_flasher_window.raise_()
        self.dfu_flasher_window.activateWindow()

    def show_identify_window(self):
        if self._identify_window is None:
            self._identify_window = IdentifyWindow(self)
        self._identify_window.show()
        self._identify_window.raise_()
        self._identify_window.activateWindow()
        self._poll_tuning()

    def show_about(self):
        QMessageBox.about(
            self,
            "About UAVX Groundstation",
            """
            <h2>UAVX Groundstation</h2>
            <p>Python port of the original UAVX groundstation.</p>
            <p><b>Version:</b> 1.0.0</p>
            """
        )
    
    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        self.setStyleSheet("QLabel { font-weight: bold; }")
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(5)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # ---- Toolbar ----
        toolbar = QHBoxLayout()
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.connect_btn.setFixedWidth(100)
        toolbar.addWidget(self.connect_btn)
        
        toolbar.addWidget(QLabel("COM:"))
        self.port_combo = QComboBox()
        self.port_combo.addItems(["auto", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"])
        self.port_combo.setCurrentText("/dev/ttyUSB0")
        self.port_combo.setEditable(True)
        toolbar.addWidget(self.port_combo)
        
        toolbar.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "128000"])
        self.baud_combo.setCurrentText("115200")
        toolbar.addWidget(self.baud_combo)
        
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        toolbar.addWidget(sep)
        
        self.params_btn = QPushButton("📋 Params")
        self.params_btn.setToolTip("Open Parameter Window (Ctrl+P)")
        self.params_btn.setFixedWidth(80)
        toolbar.addWidget(self.params_btn)
        
        self.nav_btn = QPushButton("🗺️ Nav")
        self.nav_btn.setToolTip("Open Navigation Window (Ctrl+N)")
        self.nav_btn.setFixedWidth(80)
        toolbar.addWidget(self.nav_btn)
        
        self.calib_btn = QPushButton("🔧 Calib")
        self.calib_btn.setToolTip("Open Calibration Window (Ctrl+C)")
        self.calib_btn.setFixedWidth(80)
        toolbar.addWidget(self.calib_btn)

        self.misc_btn = QPushButton("📊 Misc")
        self.misc_btn.setToolTip("Open Misc Packets Window")
        self.misc_btn.setFixedWidth(80)
        toolbar.addWidget(self.misc_btn)

        self.flash_btn = QPushButton("⚡ Flash")
        self.flash_btn.setToolTip("Open Firmware Flasher (Ctrl+F)")
        self.flash_btn.setFixedWidth(80)
        self.flash_btn.setStyleSheet("font-weight: bold; color: #9b59b6;")
        toolbar.addWidget(self.flash_btn)

        self.ident_btn = QPushButton("Identify")
        self.ident_btn.setToolTip(
            "Open the Plant Identification window: passive per-axis plant "
            "identification streamed on tag 57 (~1 Hz while connected)")
        self.ident_btn.setFixedWidth(80)
        toolbar.addWidget(self.ident_btn)

        self.esc_btn = QPushButton("ESC")
        self.esc_btn.setToolTip(
            "Enter ESC programming mode: the GCS hands the serial port to the "
            "AM32/BLHeli App, which connects directly to the FC (10 s connect "
            "window, countdown beeps). Only when disarmed.")
        self.esc_btn.setFixedWidth(80)
        self.esc_btn.setStyleSheet("font-weight: bold; color: #e67e22;")
        toolbar.addWidget(self.esc_btn)

        # Trace/Replay buttons hidden for now (redundant)
        self.replay_btn = QPushButton("Replay")
        self.replay_btn.setToolTip("Replay a raw telemetry log (Ctrl+R)")
        self.replay_btn.setFixedWidth(80)
        self.replay_btn.setVisible(False)
        toolbar.addWidget(self.replay_btn)

        toolbar.addStretch()

        self.trace_label = QLabel("Trace:")
        self.trace_label.setVisible(False)
        toolbar.addWidget(self.trace_label)
        self.trace_type_combo = QComboBox()
        for tt, label in ((0, 'None'), (1, 'Rate'), (2, 'Attitude'),
                          (3, 'AltHold'), (4, 'Actuator'), (5, 'IMU')):
            self.trace_type_combo.addItem(label, tt)
        self.trace_type_combo.setCurrentIndex(0)  # None is the safe startup default; connect adopts the FC value
        self.trace_type_combo.setFixedWidth(96)
        self._trace_combo_tooltip = (
            "Trace capture type (param 125, FC ParamTable U8 enum). "
            "Green = value confirmed by the FC; orange = change awaiting "
            "confirmation. Writes are sent live; flash persists on commit / "
            "grounded config refresh.")
        self._set_trace_combo_state('neutral')
        self.trace_type_combo.setToolTip(self._trace_combo_tooltip)
        self.trace_type_combo.currentIndexChanged.connect(self._on_trace_type_changed)
        self.trace_type_combo.setVisible(False)
        toolbar.addWidget(self.trace_type_combo)

        self.dump_trace_btn = QPushButton("Dump")
        self.dump_trace_btn.setToolTip("Dump trace / capture ring from flight controller (momentary)")
        self.dump_trace_btn.setStyleSheet("font-weight: bold;")
        self.dump_trace_btn.setFixedWidth(56)
        self.dump_trace_btn.setVisible(False)
        toolbar.addWidget(self.dump_trace_btn)

        self.status_label = QLabel("● Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        toolbar.addWidget(self.status_label)
        
        main_layout.addLayout(toolbar)
        
        # ---- Status Bar (compact, replaces debug box) ----
        status_bar = QHBoxLayout()
        status_bar.setSpacing(6)

        self.status_msg = QLabel("Ready")
        self.status_msg.setStyleSheet("color: #888; font-size: 14px;")
        status_bar.addWidget(self.status_msg, 1)

        status_bar.addWidget(QLabel("Speech:"))
        self.speech_level_combo = QComboBox()
        for label in LEVEL_LABELS.values():
            self.speech_level_combo.addItem(label)
        self.speech_level_combo.setCurrentIndex(self.speech.level)
        self.speech_level_combo.setFixedWidth(80)
        status_bar.addWidget(self.speech_level_combo)

        main_layout.addLayout(status_bar)
        
        # ---- Main Content ----
        content = QHBoxLayout()
        content.setSpacing(10)
        
        # ---- Left Panel: Alarm Box + Attitude Indicator + Altitude ----
        left_panel = QVBoxLayout()
        left_panel.setSpacing(5)
        left_panel.setAlignment(Qt.AlignCenter)

        # Attitude Indicator (with compass integrated) — the artificial horizon
        # now fills the panel fully (the red flashing AlarmFlashBox was removed
        # 2026-09-10; alarms still surface via the alarm-state label and logs).
        self.attitude = AttitudeIndicator()
        self.attitude.setMinimumSize(188, 260)
        self.attitude.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left_panel.addWidget(self.attitude, 3)

        # Execution time bargraph (between AH and Altitude)
        self.exec_bar = ExecTimeBar()
        left_panel.addWidget(self.exec_bar)

        # Firmware revision label (below exec bargraph)
        self.revision_label = QLabel("UAVX")
        self.revision_label.setAlignment(Qt.AlignCenter)
        self.revision_label.setStyleSheet("""
            font-size: 14px;
            font-weight: bold;
            color: black;
            padding: 4px;
            border: 1px solid #444;
            border-radius: 4px;
            min-height: 28px;
        """)
        left_panel.addWidget(self.revision_label)

        # Altitude display (centered below AH, just below version box)
        alt_box = QGroupBox("")
        alt_box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 12px; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        alt_box.setAlignment(Qt.AlignCenter)
        alt_layout = QGridLayout()
        alt_layout.setAlignment(Qt.AlignCenter)
        alt_layout.setContentsMargins(10, 8, 10, 8)
        alt_layout.setHorizontalSpacing(12)
        alt_layout.setVerticalSpacing(4)
        
        self.roc_label = QLabel("0.0")
        self.roc_label.setAlignment(Qt.AlignCenter)
        self.roc_label.setStyleSheet("""
            font-size: 42px;
            font-weight: bold;
            color: #00ff00;
            background-color: black;
            padding: 12px;
            border: 2px solid #444;
            border-radius: 4px;
            min-height: 75px;
            min-width: 180px;
        """)
        alt_layout.addWidget(self.roc_label, 0, 1)

        self.altitude_label = QLabel("0.0")
        self.altitude_label.setAlignment(Qt.AlignCenter)
        self.altitude_label.setStyleSheet("""
            font-size: 48px;
            font-weight: bold;
            color: #00ff00;
            background-color: black;
            padding: 12px;
            border: 2px solid #444;
            border-radius: 4px;
            min-height: 75px;
            min-width: 210px;
        """)
        alt_layout.addWidget(self.altitude_label, 0, 0)

        self.roc_sub_label = QLabel("ROC")
        self.roc_sub_label.setAlignment(Qt.AlignCenter)
        self.roc_sub_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        alt_layout.addWidget(self.roc_sub_label, 1, 1)

        self.alt_source_label = QLabel("Altitude")
        self.alt_source_label.setAlignment(Qt.AlignCenter)
        self.alt_source_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        alt_layout.addWidget(self.alt_source_label, 1, 0)

        alt_box.setLayout(alt_layout)
        # Logo + Altitude side by side in left panel
        alt_row = QHBoxLayout()
        alt_row.setSpacing(5)
        
        # Logo
        self.logo_label = QLabel()
        here = os.path.dirname(os.path.abspath(__file__))
        logo_candidates = [
            os.path.join(here, "XGS.png"),
            os.path.join(here, "XGS.ico"),
        ]
        for logo_path in logo_candidates:
            if os.path.isfile(logo_path):
                pix = QPixmap(logo_path)
                if not pix.isNull():
                    self.logo_label.setPixmap(pix.scaledToHeight(96, Qt.SmoothTransformation))
                    self.logo_label.setToolTip("UAVX Ground Station")
                    break
        self.logo_label.setFixedSize(96, 96)
        alt_row.addWidget(self.logo_label, 0, Qt.AlignVCenter)
        
        # Altitude box
        alt_row.addWidget(alt_box, 1)
        left_panel.addLayout(alt_row, 1)
        
        content.addLayout(left_panel, 2)
        
        # ---- Right Panel: Info Boxes ----
        right_panel = QVBoxLayout()
        right_panel.setSpacing(5)
        right_panel.setAlignment(Qt.AlignTop)
        
        # State + Battery side by side (State first)
        state_batt_row = QHBoxLayout()
        state_batt_row.setSpacing(5)

        # State
        state_box = QGroupBox("State")
        state_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        state_layout = QGridLayout()
        self.flight_state_label = QLabel("Unknown")
        self.flight_state_label.setStyleSheet("font-weight: bold;")
        self.nav_state_label = QLabel("Unknown")
        self.nav_state_label.setStyleSheet("font-weight: bold;")
        self.alarm_state_label = QLabel("None")
        self.curr_wp_label = QLabel("0")
        
        state_layout.addWidget(QLabel("Flight:"), 0, 0)
        state_layout.addWidget(self.flight_state_label, 0, 1)
        state_layout.addWidget(QLabel("Nav:"), 0, 2)
        state_layout.addWidget(self.nav_state_label, 0, 3)
        state_layout.addWidget(QLabel("WP:"), 1, 0)
        state_layout.addWidget(self.curr_wp_label, 1, 1)
        state_layout.addWidget(QLabel("Alarm:"), 1, 2)
        state_layout.addWidget(self.alarm_state_label, 1, 3)
        self.pf_block_label = QLabel("")
        self.pf_block_label.setWordWrap(True)
        self.pf_block_label.setStyleSheet("color: #cc0000; font-weight: bold;")
        self.pf_block_label.setVisible(False)
        state_layout.addWidget(self.pf_block_label, 2, 0, 1, 4)
        state_box.setLayout(state_layout)
        state_batt_row.addWidget(state_box)
        
        # Battery
        batt_box = QGroupBox("Battery")
        batt_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        batt_layout = QHBoxLayout()
        self.batt_volts = QLabel("0.00 V")
        self.batt_volts.setStyleSheet("font-size: 16px; font-weight: bold;")
        self.batt_current = QLabel("0.0 A")
        self.batt_charge = QLabel("0 mAh")
        self.batt_remain = QLabel("--:--")
        batt_layout.addWidget(self.batt_volts)
        batt_layout.addWidget(self.batt_current)
        batt_layout.addWidget(self.batt_charge)
        batt_layout.addWidget(self.batt_remain)
        batt_box.setLayout(batt_layout)
        state_batt_row.addWidget(batt_box)

        right_panel.addLayout(state_batt_row)
        
        # Primary Controls (iconic bargraphs)
        controls_box = QGroupBox("Controls")
        controls_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(2)
        controls_layout.setContentsMargins(2, 0, 2, 0)
        self.control_bars = []
        control_names = ["THR", "ROL", "PIT", "YAW"]
        for i in range(4):
            vb = QVBoxLayout()
            vb.setSpacing(1)
            lbl = QLabel(control_names[i])
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("font-size: 8px; font-weight: bold;")
            vb.addWidget(lbl)
            bar = QProgressBar()
            bar.setStyleSheet("""
                QProgressBar { border: 1px solid #555; border-radius: 2px; text-align: center; height: 10px; }
                QProgressBar::chunk { background: #2ecc71; border-radius: 1px; }
            """)
            bar.setRange(0, 1000)
            bar.setTextVisible(False)
            vb.addWidget(bar)
            controls_layout.addLayout(vb)
            self.control_bars.append(bar)
        controls_box.setLayout(controls_layout)
        right_panel.addWidget(controls_box)
        
        # IMU
        imu_box = QGroupBox("IMU")
        self.imu_box = imu_box
        imu_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        imu_layout = QGridLayout()
        imu_layout.setHorizontalSpacing(8)
        imu_layout.setVerticalSpacing(1)
        
        imu_layout.addWidget(QLabel("Angles:"), 0, 0)
        self.angle_roll = QLabel("0.0")
        self.angle_pitch = QLabel("0.0")
        self.angle_yaw = QLabel("0.0")
        for lbl in (self.angle_roll, self.angle_pitch, self.angle_yaw):
            lbl.setStyleSheet("font-weight: bold;")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(QLabel("Roll:"), 0, 1)
        imu_layout.addWidget(self.angle_roll, 0, 2)
        imu_layout.addWidget(QLabel("Pitch:"), 0, 3)
        imu_layout.addWidget(self.angle_pitch, 0, 4)
        imu_layout.addWidget(QLabel("Yaw:"), 0, 5)
        imu_layout.addWidget(self.angle_yaw, 0, 6)

        imu_layout.addWidget(QLabel("Gyros:"), 1, 0)
        self.gyro_roll = QLabel("0.0")
        self.gyro_pitch = QLabel("0.0")
        self.gyro_yaw = QLabel("0.0")
        for lbl in (self.gyro_roll, self.gyro_pitch, self.gyro_yaw):
            lbl.setStyleSheet("font-weight: bold;")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(QLabel("Roll:"), 1, 1)
        imu_layout.addWidget(self.gyro_roll, 1, 2)
        imu_layout.addWidget(QLabel("Pitch:"), 1, 3)
        imu_layout.addWidget(self.gyro_pitch, 1, 4)
        imu_layout.addWidget(QLabel("Yaw:"), 1, 5)
        imu_layout.addWidget(self.gyro_yaw, 1, 6)
        
        imu_layout.addWidget(QLabel("Accels:"), 2, 0)
        self.acc_lr = QLabel("0.0")
        self.acc_fb = QLabel("0.0")
        self.acc_du = QLabel("0.0")
        for lbl in (self.acc_lr, self.acc_fb, self.acc_du):
            lbl.setStyleSheet("font-weight: bold;")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(QLabel("L/R:"), 2, 1)
        imu_layout.addWidget(self.acc_lr, 2, 2)
        imu_layout.addWidget(QLabel("F/B:"), 2, 3)
        imu_layout.addWidget(self.acc_fb, 2, 4)
        imu_layout.addWidget(QLabel("D/U:"), 2, 5)
        imu_layout.addWidget(self.acc_du, 2, 6)
        
        imu_layout.addWidget(QLabel("GyroΔ:"), 3, 0)
        self.slew_gyro_roll = QLabel("0.0")
        self.slew_gyro_pitch = QLabel("0.0")
        self.slew_gyro_yaw = QLabel("0.0")
        for lbl in (self.slew_gyro_roll, self.slew_gyro_pitch,
                    self.slew_gyro_yaw):
            lbl.setStyleSheet("font-weight: bold; color: #ff7f00;")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(QLabel("Roll:"), 3, 1)
        imu_layout.addWidget(self.slew_gyro_roll, 3, 2)
        imu_layout.addWidget(QLabel("Pitch:"), 3, 3)
        imu_layout.addWidget(self.slew_gyro_pitch, 3, 4)
        imu_layout.addWidget(QLabel("Yaw:"), 3, 5)
        imu_layout.addWidget(self.slew_gyro_yaw, 3, 6)

        imu_layout.addWidget(QLabel("AccΔ:"), 4, 0)
        self.slew_acc_roll = QLabel("0.0")
        self.slew_acc_pitch = QLabel("0.0")
        self.slew_acc_yaw = QLabel("0.0")
        for lbl in (self.slew_acc_roll, self.slew_acc_pitch, self.slew_acc_yaw):
            lbl.setStyleSheet("font-weight: bold; color: #ff7f00;")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(QLabel("L/R:"), 4, 1)
        imu_layout.addWidget(self.slew_acc_roll, 4, 2)
        imu_layout.addWidget(QLabel("F/B:"), 4, 3)
        imu_layout.addWidget(self.slew_acc_pitch, 4, 4)
        imu_layout.addWidget(QLabel("D/U:"), 4, 5)
        imu_layout.addWidget(self.slew_acc_yaw, 4, 6)

        imu_layout.addWidget(QLabel("Mag:"), 5, 0)
        self.mag_x = QLabel("0")
        self.mag_y = QLabel("0")
        self.mag_z = QLabel("0")
        for lbl in (self.mag_x, self.mag_y, self.mag_z):
            lbl.setStyleSheet("font-weight: bold;")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(QLabel("Mx:"), 5, 1)
        imu_layout.addWidget(self.mag_x, 5, 2)
        imu_layout.addWidget(QLabel("My:"), 5, 3)
        imu_layout.addWidget(self.mag_y, 5, 4)
        imu_layout.addWidget(QLabel("Mz:"), 5, 5)
        imu_layout.addWidget(self.mag_z, 5, 6)

        imu_layout.addWidget(QLabel("Mag Hdg:"), 6, 0)
        self.mag_heading = QLabel("0.0°")
        self.mag_heading.setStyleSheet("font-weight: bold;")
        self.mag_heading.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(self.mag_heading, 6, 1, 1, 2)

        imu_layout.addWidget(QLabel("Acc Conf:"), 6, 3)
        self.acc_confidence = QLabel("0")
        self.acc_confidence.setStyleSheet("font-weight: bold;")
        self.acc_confidence.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(self.acc_confidence, 6, 4)

        imu_box.setLayout(imu_layout)
        # Pin the IMU box width once at natural size +20% so numeric label
        # growth (e.g. "0.0" -> "-0.00") never reflows/resizes the box.
        imu_box.setFixedWidth(int(imu_box.sizeHint().width() * 1.2))

        # Anomalies box to the right of IMU: IMU health + per-device
        # I2C error counters (Tag=76). The pair shares one row so the IMU
        # box can be kept narrow and the health data is right beside it.
        imu_anom_row = QHBoxLayout()
        imu_anom_row.setSpacing(8)
        imu_anom_row.addWidget(imu_box)
        self.anomaly_box = AnomalyBox()
        imu_anom_row.addWidget(self.anomaly_box)
        right_panel.addLayout(imu_anom_row)
        
        # Navigation
        nav_gps_row = QHBoxLayout()
        nav_gps_row.setSpacing(5)

        nav_box = QGroupBox("Navigation")
        nav_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        nav_layout = QGridLayout()
        nav_layout.setVerticalSpacing(1)
        nav_layout.setHorizontalSpacing(4)
        self.wp_bearing = QLabel("0")
        self.distance_to_wp = QLabel("0")
        self.cross_track = QLabel("0")
        self.nav_curr_wp = QLabel("0")
        self.guidance_distance = QLabel("0")
        self.guidance_bearing = QLabel("0")
        self.guidance_elevation = QLabel("0")
        self.guidance_hint = QLabel("0")
        for lbl in (self.guidance_distance, self.guidance_bearing, self.guidance_elevation, self.guidance_hint):
            lbl.setStyleSheet("font-weight: bold;")
        
        nav_layout.addWidget(QLabel("Next WP:"), 0, 0)
        nav_layout.addWidget(self.nav_curr_wp, 0, 1)
        nav_layout.addWidget(QLabel("WP Bearing:"), 0, 2)
        nav_layout.addWidget(self.wp_bearing, 0, 3)
        nav_layout.addWidget(QLabel("Distance:"), 1, 0)
        nav_layout.addWidget(self.distance_to_wp, 1, 1)
        nav_layout.addWidget(QLabel("Cross Track:"), 1, 2)
        nav_layout.addWidget(self.cross_track, 1, 3)
        nav_layout.addWidget(QLabel("Bearing:"), 2, 0)
        nav_layout.addWidget(self.guidance_bearing, 2, 1)
        nav_layout.addWidget(QLabel("Elevation:"), 2, 2)
        nav_layout.addWidget(self.guidance_elevation, 2, 3)
        nav_layout.addWidget(QLabel("Home Dist:"), 3, 0)
        nav_layout.addWidget(self.guidance_distance, 3, 1)
        nav_layout.addWidget(QLabel("Hint:"), 3, 2)
        nav_layout.addWidget(self.guidance_hint, 3, 3)

        self.wind_speed = QLabel("---")
        self.wind_dir = QLabel("---")
        self.mag_var_wmm = QLabel("---")
        for lbl in (self.wind_speed, self.wind_dir, self.mag_var_wmm):
            lbl.setStyleSheet("font-weight: bold;")
        nav_layout.addWidget(QLabel("Wind:"), 4, 0)
        nav_layout.addWidget(self.wind_speed, 4, 1)
        nav_layout.addWidget(QLabel("Dir:"), 4, 2)
        nav_layout.addWidget(self.wind_dir, 4, 3)
        nav_layout.addWidget(QLabel("MagVar:"), 5, 0)
        nav_layout.addWidget(self.mag_var_wmm, 5, 1, 1, 3)

        nav_box.setLayout(nav_layout)
        nav_gps_row.addWidget(nav_box)
        
        # Altitude
        self.altitude_box = QGroupBox("Altitude")
        self.altitude_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        alt_layout = QGridLayout()
        alt_layout.setVerticalSpacing(1)
        alt_layout.setHorizontalSpacing(4)
        self.alt_kf = QLabel("0.0")
        self.alt_roc = QLabel("0.0")
        self.alt_baro_var = QLabel("0.01")

        self.alt_accu_var = QLabel("0.10")

        self.alt_accu_bias_var = QLabel("0")
        self.alt_desired = QLabel("0.0")
        self.alt_baro_temp = QLabel("0.0")
        self.alt_baro_press = QLabel("0.0")
        for lbl in (self.alt_kf, self.alt_roc, self.alt_baro_var, self.alt_accu_var, self.alt_accu_bias_var, self.alt_desired, self.alt_baro_temp, self.alt_baro_press):
            lbl.setStyleSheet("font-weight: bold;")
        alt_layout.addWidget(QLabel("KF Alt:"), 0, 0)
        alt_layout.addWidget(self.alt_kf, 0, 1)
        alt_layout.addWidget(QLabel("ROC:"), 0, 2)
        alt_layout.addWidget(self.alt_roc, 0, 3)
        alt_layout.addWidget(QLabel("Temp:"), 2, 2)
        alt_layout.addWidget(self.alt_baro_temp, 2, 3)
        alt_layout.addWidget(QLabel("Press:"), 3, 2)
        alt_layout.addWidget(self.alt_baro_press, 3, 3)
        alt_layout.addWidget(QLabel("Baro Var:"), 2, 0)
        alt_layout.addWidget(self.alt_baro_var, 2, 1)
        alt_layout.addWidget(QLabel("AccU Var:"), 3, 0)
        alt_layout.addWidget(self.alt_accu_var, 3, 1)
        alt_layout.addWidget(QLabel("Bias Var:"), 4, 0)
        alt_layout.addWidget(self.alt_accu_bias_var, 4, 1)
        alt_layout.addWidget(QLabel("Desired:"), 5, 0)
        alt_layout.addWidget(self.alt_desired, 5, 1)
        self.altitude_box.setLayout(alt_layout)
        nav_gps_row.addWidget(self.altitude_box, 0)
        
        # GPS
        gps_box = QGroupBox("GPS")
        gps_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        gps_layout = QGridLayout()
        gps_layout.setVerticalSpacing(1)
        
        self.gps_sats = QLabel("0")
        self.gps_fix = QLabel("0")
        self.gps_lat = QLabel("---")
        self.gps_lon = QLabel("---")
        self.gps_alt = QLabel("---")
        self.gps_vel = QLabel("---")
        self.gps_hacc = QLabel("---")
        self.gps_vacc = QLabel("---")
        self.gps_sacc = QLabel("---")
        self.gps_cacc = QLabel("---")
        self.gps_rate_label = QLabel("---")
        
        for lbl in (self.gps_lat, self.gps_lon, self.gps_alt, self.gps_vel, self.gps_hacc,
                     self.gps_vacc, self.gps_sacc, self.gps_cacc):
            lbl.setStyleSheet("color: #e74c3c;")
        
        gps_layout.addWidget(QLabel("Lat:"), 0, 0)
        gps_layout.addWidget(self.gps_lat, 0, 1)
        gps_layout.addWidget(QLabel("Lon:"), 0, 2)
        gps_layout.addWidget(self.gps_lon, 0, 3)

        gps_layout.addWidget(QLabel("Alt:"), 1, 0)
        gps_layout.addWidget(self.gps_alt, 1, 1)
        gps_layout.addWidget(QLabel("Vel:"), 1, 2)
        gps_layout.addWidget(self.gps_vel, 1, 3)

        gps_layout.addWidget(QLabel("Sats:"), 2, 0)
        gps_layout.addWidget(self.gps_sats, 2, 1)
        gps_layout.addWidget(QLabel("Fix:"), 2, 2)
        gps_layout.addWidget(self.gps_fix, 2, 3)

        gps_layout.addWidget(QLabel("hAcc:"), 3, 0)
        gps_layout.addWidget(self.gps_hacc, 3, 1)
        gps_layout.addWidget(QLabel("sAcc:"), 3, 2)
        gps_layout.addWidget(self.gps_sacc, 3, 3)

        gps_layout.addWidget(QLabel("vAcc:"), 4, 0)
        gps_layout.addWidget(self.gps_vacc, 4, 1)
        gps_layout.addWidget(QLabel("Rate:"), 4, 2)
        gps_layout.addWidget(self.gps_rate_label, 4, 3)

        gps_layout.addWidget(QLabel("cAcc:"), 5, 0)
        gps_layout.addWidget(self.gps_cacc, 5, 1)

        fm = self.gps_lat.fontMetrics()
        w_latlon = fm.horizontalAdvance("-123.456789") + 4
        w_num = fm.horizontalAdvance("-12345.6") + 4
        for lbl in (self.gps_lat, self.gps_lon):
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setMinimumWidth(w_latlon)
        for lbl in (self.gps_alt, self.gps_vel, self.gps_sats, self.gps_fix,
                    self.gps_hacc, self.gps_sacc, self.gps_vacc, self.gps_cacc,
                    self.gps_rate_label):
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setMinimumWidth(w_num)

        gps_box.setLayout(gps_layout)
        nav_gps_row.addWidget(gps_box)

        right_panel.addLayout(nav_gps_row)
        
        # Link Stats
        link_box = QGroupBox("Link Stats")
        link_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        link_layout = QGridLayout()
        link_layout.setVerticalSpacing(1)
        link_layout.setHorizontalSpacing(4)
        self.link_lq = QLabel("--%")
        self.link_snr = QLabel("--")
        self.link_rssi = QLabel("--")
        self.link_losses = QLabel("--")
        self.link_failsafes = QLabel("--")
        for lbl in (self.link_lq, self.link_snr, self.link_rssi, self.link_losses,
                    self.link_failsafes):
            lbl.setStyleSheet("font-weight: bold;")
        link_layout.addWidget(QLabel("LQ:"), 0, 0)
        link_layout.addWidget(self.link_lq, 0, 1)
        link_layout.addWidget(QLabel("SNR:"), 0, 2)
        link_layout.addWidget(self.link_snr, 0, 3)
        link_layout.addWidget(QLabel("RSSI:"), 1, 0)
        link_layout.addWidget(self.link_rssi, 1, 1)
        link_layout.addWidget(QLabel("Losses:"), 1, 2)
        link_layout.addWidget(self.link_losses, 1, 3)
        link_layout.addWidget(QLabel("Failsafes:"), 2, 0)
        link_layout.addWidget(self.link_failsafes, 2, 1)
        link_box.setLayout(link_layout)
        
        # Serial Ports (side-by-side with Link Stats)
        serial_box = QGroupBox("Serial Ports")
        serial_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        serial_layout = QGridLayout()
        serial_layout.setVerticalSpacing(1)
        serial_layout.setHorizontalSpacing(4)
        serial_layout.addWidget(QLabel(""), 0, 0)
        serial_layout.addWidget(QLabel("TX"), 0, 1)
        serial_layout.addWidget(QLabel("RX"), 0, 2)
        serial_layout.addWidget(QLabel("Ov"), 0, 3)
        bar_style = """
            QProgressBar { border: 1px solid #555; border-radius: 3px; text-align: center; height: 14px; font-size: 9px; }
            QProgressBar::chunk { background: #3498db; border-radius: 2px; }
        """
        self.ser_telem_tx = QProgressBar(); self.ser_telem_tx.setStyleSheet(bar_style); self.ser_telem_tx.setRange(0, 512); self.ser_telem_tx.setValue(0); self.ser_telem_tx.setTextVisible(True)
        self.ser_telem_rx = QProgressBar(); self.ser_telem_rx.setStyleSheet(bar_style); self.ser_telem_rx.setRange(0, 512); self.ser_telem_rx.setValue(0); self.ser_telem_rx.setTextVisible(True)
        self.ser_telem_ov = QLabel("--")
        self.ser_gps_tx = QProgressBar(); self.ser_gps_tx.setStyleSheet(bar_style); self.ser_gps_tx.setRange(0, 512); self.ser_gps_tx.setValue(0); self.ser_gps_tx.setTextVisible(True)
        self.ser_gps_rx = QProgressBar(); self.ser_gps_rx.setStyleSheet(bar_style); self.ser_gps_rx.setRange(0, 512); self.ser_gps_rx.setValue(0); self.ser_gps_rx.setTextVisible(True)
        self.ser_gps_ov = QLabel("--")
        for ov in (self.ser_telem_ov, self.ser_gps_ov):
            ov.setStyleSheet("font-weight: bold;")
        serial_layout.addWidget(QLabel("Telem:"), 1, 0)
        serial_layout.addWidget(self.ser_telem_tx, 1, 1)
        serial_layout.addWidget(self.ser_telem_rx, 1, 2)
        serial_layout.addWidget(self.ser_telem_ov, 1, 3)
        serial_layout.addWidget(QLabel("GPS:"), 2, 0)
        serial_layout.addWidget(self.ser_gps_tx, 2, 1)
        serial_layout.addWidget(self.ser_gps_rx, 2, 2)
        serial_layout.addWidget(self.ser_gps_ov, 2, 3)
        serial_box.setLayout(serial_layout)
        
        link_serial_row = QHBoxLayout()
        link_serial_row.setSpacing(5)
        link_serial_row.addWidget(link_box)
        link_serial_row.addWidget(serial_box)
        right_panel.addLayout(link_serial_row)
        
        # Motors / Servos bargraphs
        motors_box = QGroupBox("Motors / Servos")
        motors_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        motors_layout = QGridLayout()
        motors_layout.setVerticalSpacing(1)
        motors_layout.setHorizontalSpacing(4)
        self.motor_bars = []
        for i in range(10):
            row = (i // 5) * 2
            col = i % 5
            lbl = QLabel(f"M{i}")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("font-size: 9px; font-weight: bold;")
            motors_layout.addWidget(lbl, row, col)
            bar = QProgressBar()
            bar.setStyleSheet("""
                QProgressBar { border: 1px solid #555; border-radius: 3px; text-align: center; height: 14px; font-size: 9px; }
                QProgressBar::chunk { background: #2ecc71; border-radius: 2px; }
            """)
            bar.setRange(0, 1000)
            bar.setValue(0)
            bar.setTextVisible(True)
            bar.setFormat("%v")
            bar.setFixedHeight(14)
            motors_layout.addWidget(bar, row + 1, col)
            self.motor_bars.append(bar)
        motors_box.setLayout(motors_layout)
        right_panel.addWidget(motors_box)
        
        content.addLayout(right_panel, 2)
        main_layout.addLayout(content)
        
        # ---- Config Summary Box (one line, all bits) ----
        self.config_box = QGroupBox("Configuration Summary")
        self.config_box.setStyleSheet("""
            QGroupBox { 
                font-weight: bold; 
                border: 1px solid black;
                border-radius: 4px;
                margin-top: 6px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 6px;
                padding: 0 3px 0 3px;
            }
        """)
        self.config_box.setMinimumHeight(44)
        config_layout = QHBoxLayout()
        config_layout.setSpacing(6)
        config_layout.setContentsMargins(8, 6, 8, 6)
        
        self.config_labels = []
        self.config_flags = [
            (1, 0, "Ext Mag"), (1, 1, "Autoland"), (1, 2, "Use Mag"), (1, 3, "Emulation"),
            (1, 4, "AH Alarm"), (1, 5, "GPS Alt"), (1, 7, "Clamp"),
            (2, 0, "Batt Comp"), (2, 3, "Have GPS"), (2, 4, "Prop In"),
            (2, 5, "Turn WP"), (2, 6, "Beep WP"),
        ]
        for which, bit, name in self.config_flags:
            label = QLabel(name)
            label.setMinimumWidth(70)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("""
                background-color: #ddd;
                color: #666;
                font-weight: normal;
                border: 1px solid #999;
                border-radius: 3px;
                padding: 3px 6px;
            """)
            config_layout.addWidget(label)
            self.config_labels.append(label)
        
        emul_label = self.config_labels[self.config_flags.index((1, 3, "Emulation"))]
        emul_label.setToolTip("Unavailable — emulation disabled; commissioning is real flight only.")
        emul_label.setStyleSheet("""
            background-color: #f0f0f0;
            color: #c0c0c0;
            font-weight: normal;
            font-style: italic;
            border: 1px dashed #ccc;
            border-radius: 3px;
            padding: 3px 6px;
        """)
        config_layout.addStretch()
        
        self.config_box.setLayout(config_layout)
        main_layout.addWidget(self.config_box)
        
        # ---- Flags (full width at bottom) ----
        self.flags_box = QGroupBox("Flags")
        self.flags_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        # Fixed vertical policy: the box is exactly as tall as its content,
        # never stretching or squeezing the widgets above it.
        self.flags_box.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.flags_box.setVisible(True)
        
        self.flag_labels = {}
        
        flags_layout = QVBoxLayout()
        flags_layout.setContentsMargins(4, 8, 4, 4)
        
        self._build_flag_groups(flags_layout)
        self.flags_box.setLayout(flags_layout)
        main_layout.addWidget(self.flags_box)
        
        self.reset_flight_readouts()
        self.statusBar().showMessage("Ready")
    
    # Readouts that carry a value measured by the FC. They are shown as "---"
    # until a packet actually supplies the field, because a hard-coded 0.0
    # reads as "measured zero" and survives across sessions, so a stale value
    # from a previously connected board is indistinguishable from a live one.
    # Updates are unconditional once data arrives: this never masks a value the
    # FC reported, including NaN.
    _FLIGHT_READOUTS = (
        'roc_label', 'altitude_label', 'flight_state_label', 'nav_state_label',
        'curr_wp_label', 'alarm_state_label', 'batt_volts', 'batt_current',
        'batt_charge', 'batt_remain', 'angle_roll', 'angle_pitch',
        'angle_yaw', 'gyro_roll', 'gyro_pitch', 'gyro_yaw',
        'acc_lr', 'acc_fb', 'acc_du',
        'slew_gyro_roll', 'slew_gyro_pitch', 'slew_gyro_yaw',
        'slew_acc_roll', 'slew_acc_pitch', 'slew_acc_yaw',
        'mag_heading', 'mag_x', 'mag_y', 'mag_z', 'acc_confidence', 'wp_bearing', 'distance_to_wp',
        'cross_track', 'guidance_distance', 'guidance_bearing',
        'guidance_elevation', 'guidance_hint',
    )

    def reset_flight_readouts(self):
        for name in self._FLIGHT_READOUTS:
            label = getattr(self, name, None)
            if hasattr(label, 'setText'):
                label.setText("---")
        # A flag label's text is its NAME, set once at build; its state lives
        # in the stylesheet. Resetting to "---" would erase the name and
        # nothing puts it back (update_flag_colors only restyles), so the
        # unknown state is the neutral colour instead.
        self._reset_flag_colors()

    def _reset_flag_colors(self):
        """Put every flag label back to the neutral, no-state-reported look."""
        for flag_data in getattr(self, 'flag_labels', {}).values():
            flag_data['widget'].setStyleSheet(_FLAG_NEUTRAL_QSS)
    
    def _build_flag_groups(self, flags_layout):
        """Build the logical flag group boxes showing every assigned FC
        Flags bit. Reserved (unassigned) bits are never displayed."""
        self.flag_group_boxes = []
        grid = QGridLayout()
        grid.setSpacing(4)

        group_style = "QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }"

        def make_flag_label(name, tooltip):
            label = QLabel(name)
            label.setAlignment(Qt.AlignCenter)
            label.setToolTip(tooltip)
            # Fixed size: height just fits the text, width permits a
            # 7-character label. Width follows no label; it is uniform.
            fm = label.fontMetrics()
            h = fm.height() + 8               # 3px pad x2 + 1px border x2
            w = fm.horizontalAdvance('7') * 7 + 14
            label.setFixedSize(w, h)
            label.setStyleSheet(_FLAG_NEUTRAL_QSS)
            return label

        for gi, (group_name, bits) in enumerate(FLAG_GROUPS):
            group = QGroupBox(group_name)
            group.setStyleSheet(group_style)
            gl = QVBoxLayout()
            gl.setContentsMargins(4, 8, 4, 4)
            gl.setSpacing(1)
            group_keys = []
            for bi in range(0, len(bits), 6):
                row = QHBoxLayout()
                row.setSpacing(4)
                row.addStretch()
                for bit in bits[bi:bi + 6]:
                    key, name, tooltip = FLAG_BIT_DEFS[bit]
                    label = make_flag_label(name, tooltip)
                    row.addWidget(label)
                    self.flag_labels[key] = {
                        'widget': label, 'bit': bit, 'name': name, 'key': key
                    }
                    group_keys.append(key)
                row.addStretch()
                gl.addLayout(row)
            group.setLayout(gl)
            self.flag_group_boxes.append((group, group_keys))
            grid.addWidget(group, gi // 3, gi % 3, alignment=Qt.AlignTop)

        self.flags_grid_widget = QWidget()
        self.flags_grid_widget.setLayout(grid)
        flags_layout.addWidget(self.flags_grid_widget)

        # Identify window (plant-ID telemetry) — separate top-level window,
        # created lazily on first click (CalibrationWindow pattern).
    
    def setup_connections(self):
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.params_btn.clicked.connect(self.show_parameter_window)
        self.nav_btn.clicked.connect(self.show_nav_window)
        self.calib_btn.clicked.connect(self.show_calibration_window)
        self.misc_btn.clicked.connect(self.show_misc_window)
        self.flash_btn.clicked.connect(self.show_dfu_flasher)
        self.ident_btn.clicked.connect(self.show_identify_window)
        self.esc_btn.clicked.connect(self.enter_esc_programming)
        self.dump_trace_btn.clicked.connect(self.dump_black_box)
        self.replay_btn.clicked.connect(self.open_replay)
        self.speech_level_combo.currentIndexChanged.connect(self.speech_level_changed)
        
        for btn in [self.connect_btn]:
            btn.setProperty('original_text', btn.text())
        
    def dump_black_box(self):
        if self._trace_fc_value in (None, 0):
            self.log_debug(
                "Dump requested while the FC's trace type is None — the live "
                "ring has no snapshot. Only a previously committed flash capture "
                "(from a non-None type) would return data; otherwise expect the "
                "'no snapshot' result.", "Warnings")
        self._bb_chunks = {}
        self.send_request(PacketTag.MISC, MiscCommand.BB_DUMP, 0)
        self.log_debug("📤 Sent Dump Trace / capture ring command", "Info")

    def enter_esc_programming(self):
        if not self.connected:
            QMessageBox.information(self, "ESC Programming",
                                    "Connect to the FC first (Connect button).")
            return
        if self._in_flight:
            QMessageBox.warning(self, "ESC Programming",
                                "Only available while DISARMED — the FC refuses "
                                "ESC access in flight.")
            return
        ret = QMessageBox.question(
            self, "ESC Programming",
            "Enter ESC programming mode?\n\n"
            "The FC opens a 10 s connect window and the GCS releases the serial "
            "port so the AM32/BLHeli App can connect directly to it.\n\n"
            "Reconnect with the Connect button when finished.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ret != QMessageBox.Yes:
            return
        self.esc_btn.setEnabled(False)
        self.esc_btn.setStyleSheet("font-weight: bold; color: #c0392b; background-color: #b03a2e;")
        self.esc_btn.setText("ESC •")
        self.log_debug("📤 Requesting ESC programming mode (miscESCProg)", "Info")
        self.send_request(PacketTag.MISC, MiscCommand.ESC_PROG, 0)

    def _on_esc_prog_ack(self):
        self.esc_btn.setStyleSheet("font-weight: bold; color: #c0392b; background-color: #b03a2e;")
        self.esc_btn.setText("ESC •")
        self.esc_btn.setToolTip(
            "ESC programming active: the FC is listening for the AM32/BLHeli App "
            "(10 s window, countdown beeps). Reconnect when finished.")
        self.log_debug(
            "⚡ ESC programming mode confirmed — releasing the serial port in 2 s "
            "so the AM32/BLHeli App can connect directly. Reconnect when finished.",
            "Info")
        QTimer.singleShot(2000, self.disconnect)

    def _on_esc_prog_denied(self):
        self.esc_btn.setEnabled(True)
        self.esc_btn.setText("ESC")
        self.esc_btn.setStyleSheet("font-weight: bold; color: #e67e22;")
        self.log_debug("❌ ESC programming refused by the FC (armed?).",
                       "Errors")
        QMessageBox.warning(self, "ESC Programming",
                            "The FC refused ESC programming mode.\n\n"
                            "Disarm the aircraft and try again.")

    def _finalize_bb_dump(self):
        self._bb_dump_timer.stop()
        if not self._bb_chunks:
            return
        seqs = sorted(self._bb_chunks.keys())
        data = b''
        for s in seqs:
            data += self._bb_chunks[s]
        self._bb_chunks = {}
        # Trace dump with no capture: the FC streams an all-zero
        # TRACE_HEADER_SIZE (128 B v2 / 32 B v1) header with no records. Detect
        # any all-zero dump and report instead of offering a pointless save.
        if not any(data) and len(data) >= 32:
            msg = ("No trace capture on the FC.\n\n"
                   "The FC returned an all-zero header. Possible causes:\n"
                   "• Trace type is None (set the Trace combo to a probe type)\n"
                   "• ch8 was never held in flight (capture needs the go-ahead)\n"
                   "• no disarm commit since the capture\n\n"
                   "Nothing was saved.")
            self.log_debug(
                "Trace dump has no capture: the FC returned the all-zero "
                "header (trace type None on the FC, or ch8 never armed a "
                "capture in flight, or no disarm commit). Nothing to save — "
                "set the Trace type, fly, disarm, then dump again.", "Warnings")
            QMessageBox.warning(self, "Trace Dump — No Capture", msg)
            return
        if data[:4] == b'TRAC':
            try:
                viewer = TraceViewer(data, source="live dump")
            except ValueError as e:
                QMessageBox.warning(self, "Trace Viewer", str(e))
                return
            self._trace_viewer = viewer
            viewer.show()
            viewer.raise_()
            viewer.activateWindow()
            self.log_debug(f"✅ TRAC snapshot ({len(data)}B) → Trace Viewer",
                           "Info")
            return
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Black Box Dump",
            f"bb_dump_{timestamp}.bin",
            "Binary Files (*.bin);;All Files (*)")
        if path:
            with open(path, 'wb') as f:
                f.write(data)
            self.log_debug(f"✅ BB dump saved ({len(data)}B) → {path}", "Info")

    def _log_dir(self):
        """Absolute directory for logs/KML. Never relative, so files never
        land inside the project tree. Cross-platform via QStandardPaths, with
        a home-dir fallback if DocumentsLocation is unavailable."""
        if self.log_dir:
            return self.log_dir
        d = workspace.logs_dir()
        os.makedirs(d, exist_ok=True)
        return d

    # IMU integrity fault codes must match the enum in UAVXArmQ/src/inertial.h
    _IMU_FAULT_NAMES = {
        0: "none",
        1: "non-finite quaternion/Euler - recovered from last-good",
        2: "accel magnitude zero - accel correction skipped",
        3: "non-finite gyro rate - ignored",
        4: "non-finite accel - ignored",
    }

    # Reset causes must match the enum in UAVXArmQ/src/main.h (upper bits
    # persist as param 127, pPowerResetCause; only refreshed on non-POR boots).
    _RESET_CAUSE_NAMES = {
        0: "unknown", 1: "low-power", 2: "window-watchdog",
        3: "independent-watchdog", 4: "software", 5: "power-on/power-down",
        6: "external-pin", 7: "brownout",
    }

    # Config.BootDiag must match the BootDiag enum in UAVXArmQ/src/params.h.
    # Every non-restored outcome routes through UseDefaultParametersClearCal()
    # in the FC, so they all mean the same actionable thing: the config is
    # gone, the calibration with it, and the pilot must re-commit and
    # re-calibrate. There is no "calibration preserved" case to advertise.
    _BOOT_DIAG_NAMES = {
        0: "not reported (FC predates this field)",
        1: "config restored from flash",
        2: "config not written by this build (blank/DFU-padded/foreign magic)",
        3: "parameter table changed since the config was written",
        4: "ConfigStruct layout changed (firmware updated)",
        5: "config flash blank - nothing has ever been committed",
    }

    def _report_reset_cause(self, parsed):
        """Report the FC's persisted last-reset cause once per change. Param 127
        survives any meaningful reset (brownout, external pin, window-watchdog,
        independent watchdog, ...), so this is the single discriminator for why
        the FC last restarted."""
        entries = getattr(parsed, "entries", None)
        if not entries or ParamIndex.POWER_RESET_CAUSE not in entries:
            return
        cause = int(entries[ParamIndex.POWER_RESET_CAUSE][1])
        if cause == self._last_reported_reset_cause:
            return
        self._last_reported_reset_cause = cause
        name = self._RESET_CAUSE_NAMES.get(cause, "unknown")
        if cause:
            print(f"[RST] last reset cause: {cause} ({name})")
            self.log_debug(f"FC reset cause {cause} ({name}) on last boot", "Warnings")
        else:
            print("[RST] no meaningful reset recorded (unknown/power-on)")
            self.log_debug("No meaningful reset cause on last boot", "Info")

    def _report_boot_diag(self, parsed):
        """Report how the FC's last boot obtained its config block, once per
        change. Config.BootDiag arrives in the tag-67 offset-12 hole.

        This exists because the fallback is otherwise invisible. When the magic,
        the ParamTable CRC or the layout stamp does not match, IsFlashClean()
        routes the board to UseDefaultParametersClearCal() - compiled defaults,
        no airframe, and a wiped calibration - and the pilot's first clue is
        the arming gate refusing because Config.IMUCal.Calibrated is not 1.
        Nothing says why, and the stock remedy (calibrate) silently redoes the
        work every boot without fixing the cause.

        Deliberately log-only, NOT a banner. The flashing AlarmFlashBox was
        retired 2026-09-10 and must not be resurrected; alarm reporting lives
        in the non-flashing alarm_state_label. This reporter additionally has
        no banner path of its own to share, so a banner here could only ever be
        an inconsistency between the two reporters on the same tag-67 packet.
        """
        code = getattr(parsed, "boot_diag", 0)
        if code == self._last_reported_boot_diag:
            return
        self._last_reported_boot_diag = code
        name = self._BOOT_DIAG_NAMES.get(code, "unknown")
        if code in (0, 1):
            print(f"[BOOT] config load: {code} ({name})")
            self.log_debug(f"FC config load outcome {code}: {name}", "Info")
        else:
            msg = (f"FC reverted to compiled defaults on last boot - {name}. "
                   f"Config, airframe and calibration are gone; load an "
                   f"airframe, Apply & Reboot, then re-calibrate IMU and mag")
            print(f"[BOOT] {msg}")
            self.log_debug(msg, "Warnings")

    def _verify_post_commit_config(self, parsed):
        """Gate the deferred post-commit readback on the FC's own reboot report.

        The tag-77 ACK is the FC's verified truth: ApplyParamImage returns true
        only after erase + program + byte-for-byte read-back. The FC then waits
        for the GCS to send the tag-78 reboot command, and reboots only then.
        What the ACK cannot show is whether flash holds the values the GCS
        intended, so an independent tag-71 comparison against what was written
        is still required.

        That readback must be served by the FC that came back AFTER the reset.
        This used to gate on a tag-70 flight_state of eStepNav, which could
        never arrive twice over: TxState() emits unframed debug text the
        FrameDecoder cannot read as a packet, and InitNavigation() re-runs on
        every arming, so it is neither machine-readable nor reboot-specific.
        Tag 68 (SendRebootInfoPacket, one call at the end of InitHarness) is
        the replacement - an explicit boot edge carrying the reason.
        """
        if not self._pending_param_verification or not self.param_window:
            return

        self._pending_param_verification = False
        self._armed_boot_verify = False

        # The FC has reported, so the reconnect watchdog has served its purpose
        # either way - a late arrival still needs releasing.
        wd = getattr(self.param_window, '_reconnect_watchdog_timer', None)
        if wd is not None:
            wd.stop()
            self.param_window._reconnect_watchdog_timer = None

        cause = getattr(parsed, "reset_cause", 0)
        code = getattr(parsed, "boot_diag", 0)
        name = self._BOOT_DIAG_NAMES.get(code, "unknown")
        cause_name = self._RESET_CAUSE_NAMES.get(cause, "unknown")
        reason = f"reset cause {cause} ({cause_name}), config load {code} ({name})"

        # The cause is recorded but NOT used as a gate. Readback is the authority
        # whatever the cause: a brownout or power-off inside the commit window
        # leaves flash in an unknown state that only the comparison can settle,
        # and a watchdog reset may still have committed cleanly. Log it so an
        # unexpected cause is visible, and let the readback decide.
        self.log_debug(f"FC rebooted after commit: {reason}", "Info")

        # Code 1 is the only value that positively asserts the config block was
        # restored from flash. 2-5 mean the FC reverted to compiled defaults, so
        # the block the commit wrote is not the block now in flash and a readback
        # would compare against reverted values - fail loudly instead. 0 is an
        # FC predating the field: fall through to the readback, which is itself
        # sufficient proof (it compares flash against the snapshot), so refusing
        # here would only lose verification on older firmware.
        if code not in (0, 1):
            msg = (f"Commit verification FAILED - FC reports {reason}. The "
                   f"config block was not restored from flash, so the commit "
                   f"did not persist. Load an airframe, Apply & Reboot.")
            print(f"[COMMIT] {msg}")
            self.log_debug(msg, "Errors")
            return

        # Re-pull the config packet (tag-63) too. The airframe name rides the
        # config block and survives the reboot in flash, but the connect handler
        # only asks for it on a port reopen - which a UART link does not see
        # across the commit reboot.
        self.send_request(PacketTag.AFNAME, 0, 0, None)

        self.param_window._request_write_verify()

    def _report_imu_fault(self, parsed):
        """Report an attitude-integrity (NaN/Inf) DETECTION from the FC once per
        change. The FC latches F.IMUFaultLatched (flag idx 47) and streams the
        fault code + cumulative detection count via tag 67.

        The wording matters: the FC does NOT recover the attitude. It is a
        report-only tripwire (inertial.c), because reverting to a held attitude
        is "hold last setting" - and with the gyro still live the angle loop
        would fly the aircraft back to an attitude that is no longer true. Bad
        SENSOR samples are discarded upstream in PostFilterAccAndRate(); a
        non-finite ATTITUDE means an unguarded source escaped, which is a bug
        to investigate. So never say "recovered" here: that tells the pilot the
        FC handled something it deliberately does not attempt to handle.

        Report-only, with no banner of any kind. This branch used to call
        _raise_fault_banner()/_hide_fault_banner(), neither of which has ever
        existed in this tree (not even in the imported baseline) - the function
        was dead code with no call site, so the dead reference was harmless.
        Giving it a call site at the tag-67 case made the FC's FIRST ExecTime
        packet raise AttributeError in a Qt slot, which PyQt5 turns into
        qFatal and kills the whole GCS. Do not reintroduce a banner: the
        flashing AlarmFlashBox was retired 2026-09-10 and the tracing/console
        output is the intended reporting path."""
        code = getattr(parsed, "imu_fault_code", 0)
        rec = getattr(parsed, "imu_fault_detections", 0)
        if code == self._last_reported_imu_fault_code \
                and rec == self._last_reported_imu_fault_detections:
            return
        self._last_reported_imu_fault_code = code
        self._last_reported_imu_fault_detections = rec
        if code:
            name = self._IMU_FAULT_NAMES.get(code, "unknown")
            msg = (f"IMU integrity fault code={code} ({name}); detected {rec}x "
                   f"- NOT recovered (report-only), investigate the sensor path")
            print(f"[IMU] {msg}")
            self.log_debug(msg, "Errors")

    def _on_raw_bytes(self, data: bytes):
        """Tee raw serial bytes to the always-on raw log (all bytes, binary
        and printable alike)."""
        if self.raw_logger.active:
            self.raw_logger.write(data)

    def select_workspace_folder(self):
        start = self.log_dir or workspace.workspace_dir()
        folder = QFileDialog.getExistingDirectory(
            self, "Select UAVX workspace folder (saved airframes + logs)", start)
        if folder:
            workspace.set_workspace_dir(folder)
            _air, self.log_dir = workspace.ensure_dirs()
            self.raw_logger.log_dir = self.log_dir
            self.save_settings()
            self.log_debug(f"📂 Workspace folder set → {folder}", "Info")

    def load_settings(self):
        settings = QSettings("UAVX", "Groundstation")
        # only accept plausible device paths - the editable combo lets any
        # text persist (notes, typos) which would otherwise fail at connect
        saved_port = str(settings.value("port", "/dev/ttyUSB0"))
        if not saved_port.startswith("/dev/") and saved_port != "auto":
            saved_port = "auto"
        self.port_combo.setCurrentText(saved_port)
        self.baud_combo.setCurrentText(settings.value("baud", "115200"))
        # Speech level: default is OFF (2026-09-04, Greg). The operator opts in by
        # selecting a level from the status-bar combo; the persisted choice is
        # honoured, but a never-chosen GCS defaults to silenced.
        stored_level = settings.value("speech_level", None)
        if stored_level is None:
            speech_level = SpeechLevel.OFF
        else:
            try:
                speech_level = SpeechLevel(int(stored_level))
            except (TypeError, ValueError):
                speech_level = SpeechLevel.OFF
        self.speech.level = speech_level
        self.speech_level_combo.setCurrentIndex(self.speech.level)
        if not self.speech.available:
            self.log_debug(
                f"🔇 Speech unavailable: {self.speech.init_error}", "Info")

        # Workspace folder: prompt once on first run, else use the saved root.
        # Saved airframes and telemetry logs both live under it.
        if not workspace.has_workspace_dir():
            folder = QFileDialog.getExistingDirectory(
                self, "Select your UAVX workspace folder (airframes + logs)",
                workspace.default_workspace_dir())
            if folder:
                workspace.set_workspace_dir(folder)
        _air, self.log_dir = workspace.ensure_dirs()
        self.raw_logger.log_dir = self.log_dir
        # Verify + confirm the active workspace on every startup
        if not os.access(self.log_dir, os.W_OK):
            self.log_debug(f"⚠️ Workspace not writable: {self.log_dir}", "Errors")
        else:
            self.log_debug(f"📂 Workspace → {self.log_dir}", "Info")

    def save_settings(self):
        settings = QSettings("UAVX", "Groundstation")
        settings.setValue("port", self.port_combo.currentText())
        settings.setValue("baud", self.baud_combo.currentText())
        settings.setValue("speech_level", str(self.speech.level.value))
        settings.setValue("workspace_dir", workspace.workspace_dir())
    
    def toggle_connection(self):
        if self.telemetry and self.telemetry.isRunning():
            self.disconnect()
        else:
            self.connect_telemetry()
    
    def connect_telemetry(self):
        self.reset_flight_readouts()
        # Connecting while replaying a log terminates the replay immediately
        # and returns the GCS to the live FC link.
        if self.replaying:
            self.replaying = False
            self.raw_logger.forbidden = False
            if self.replay_window is not None:
                self.replay_window.close()
            self.log_debug("⏹ Replay terminated by Connect", "Info")

        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        
        # A previous thread may still be auto-reconnecting after a drop; stop
        # it first so two threads never pump the same link.
        if self.telemetry and self.telemetry.isRunning():
            self.telemetry.stop()
            self.telemetry = None
        
        self.telemetry = TelemetryThread(port, baud)
        self.telemetry.data_received.connect(self.process_packet)
        self.telemetry.raw_bytes.connect(self._on_raw_bytes)
        self.telemetry.connected.connect(self.on_connected)
        self.telemetry.error.connect(self.on_error)
        self.telemetry.start()
        
        self.connect_btn.setText("Connecting...")
        self.connect_btn.setEnabled(False)
        self.statusBar().showMessage(f"Connecting to {port} @ {baud}...")
    
    def disconnect(self):
        self.connect_btn.setEnabled(False)
        self.connect_btn.setText("Disconnecting...")
        self._pending_param_verification = False
        self._armed_boot_verify = False
        if self.param_window is not None:
            # Clear the write interlock so a mid-commit disconnect/reconnect
            # doesn't leave the commit path locked forever.
            self.param_window._flash_write_pending = False
            self.param_window._write_in_progress = False
            self.param_window._apply_retry_pending = False
            if getattr(self.param_window, '_reconnect_watchdog_timer', None) is not None:
                self.param_window._reconnect_watchdog_timer.stop()
                self.param_window._reconnect_watchdog_timer = None
        
        if self.telemetry:
            self.telemetry.stop()
            self.telemetry = None
        
        self.connected = False
        self._in_flight = False
        self.raw_logger.stop()
        self._trace_fc_value = None
        self._trace_write_pending = None
        self._set_trace_combo_state('neutral')
        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.connect_btn.setEnabled(True)
        self.reset_flight_readouts()
        self.status_label.setText("● Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        self.statusBar().showMessage("Disconnected")
        self.log_debug("🔌 Disconnected", "Info")
    
    def on_connected(self, connected: bool):
        self.connected = connected
        self.connect_btn.setEnabled(True)
        
        if connected:
            self._in_flight = False
            self.esc_btn.setEnabled(True)
            self.esc_btn.setText("ESC")
            self.esc_btn.setStyleSheet("font-weight: bold; color: #e67e22;")
            self.raw_logger.log_dir = self.log_dir or workspace.logs_dir()
            self.raw_logger.start()
            self.log_debug(f"🖊️ Raw log → {os.path.abspath(self.raw_logger._path)}", "Info")
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet("background-color: green; color: white; font-weight: bold;")
            self.status_label.setText("● Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.statusBar().showMessage("Connected to UAVX")
            self.log_debug("✅ Connected to UAVX", "Info")
            # Skip the connect-time tag-71 when a commit's verification is
            # outstanding - _verify_post_commit_config() issues that single
            # readback once the first post-reconnect tag-67 reports the config
            # block is valid. Avoids a racing double-read.
            if self._pending_param_verification:
                # Arm the tag-67 boot-edge trigger HERE, on the genuine
                # post-reboot reconnection. The FC waits 500 ms before it
                # resets, and the outgoing instance keeps streaming tag-67
                # with the pre-commit boot_diag during that window; arming
                # only on reconnect guarantees the readback is served by the
                # FC that came back, not the one on its way down.
                self._armed_boot_verify = True
                # The FC sends tag 68 once at the end of InitHarness(), which is
                # buffered and drained on either link - but on USB the host may
                # still be mid-reopen backoff, so ask for it again. Free when
                # nothing is pending verification.
                self.send_request(PacketTag.REQUEST, PacketTag.REBOOT_INFO, 0)
            else:
                self.send_request(PacketTag.PARAM_TAGGED_READ, 255, 0, None)
            self.send_request(PacketTag.MIN, 0, 0, None)
            self.send_request(PacketTag.AFNAME, 0, 0, None)
            self.send_request(PacketTag.TUNING, 0, 0, None)
            self._identify_poll_timer.start(1000)
        else:
            self._in_flight = False
            self.raw_logger.stop()
            self._identify_poll_timer.stop()
            self.connect_btn.setText("Connect")
            self.connect_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
            self.status_label.setText("● Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
    
    def on_error(self, error: str):
        self.connect_btn.setEnabled(True)
        self.statusBar().showMessage(f"Error: {error}")
        self.log_debug(f"❌ Error: {error}", "Errors")
        QMessageBox.warning(self, "Telemetry Error", f"Failed to connect: {error}")
    
    def clear_debug(self):
        self.status_msg.setText("Ready")
        self.status_msg.setStyleSheet("color: #888; font-size: 14px;")
    
    def speech_level_changed(self, index):
        level = SpeechLevel(index)
        self.speech.level = level
        label = LEVEL_LABELS.get(level, "Off")
        self.log_debug(f"🔊 Speech level: {label}", "Info")

    def _speak_boot_greeting(self):
        if self.speech.available:
            self.speech.speak("Ready", SpeechLevel.ALL, volume=0.6)
            self.log_debug(
                f"🔊 Boot greeting spoken via {self.speech.backend}", "Info")
        else:
            self.log_debug(
                f"🔇 Boot greeting skipped — speech unavailable: "
                f"{self.speech.init_error}", "Info")
    
    def log_debug(self, message, level="Info"):
        current_level = self._log_level
        
        if current_level == "Info" and level != "Info":
            return
        elif current_level == "Warnings" and level not in ["Warnings", "Errors"]:
            return
        elif current_level == "Errors" and level != "Errors":
            return
        
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss.zzz")
        
        colors = {
            "Info": "#4CAF50",
            "Warnings": "#f39c12",
            "Errors": "#e74c3c"
        }
        color = colors.get(level, "#d4d4d4")
        
        html = f'<span style="color: #666;">[{timestamp}]</span> <span style="color: {color};">{level}:</span> {message}'
        self.status_msg.setText(f"[{timestamp}] {level}: {message}")
        self.status_msg.setStyleSheet(f"color: {color}; font-size: 14px;")

    def _trace(self, message, level="Info"):
        """Commit-path diagnostic: the status bar is transient, so commit
        send/verify outcomes are also printed to the console (stdout) where
        they survive for post-mortem alongside the rawlog."""
        self.log_debug(message, level)
        print(message)

    def _build_packet_with_checksum(self, data: bytes) -> bytes:
        packet = bytearray()
        packet.append(0xFF)
        packet.append(SOH)
        
        checksum = 0
        
        for b in data:
            if b in [SOH, EOT, ESC]:
                packet.append(ESC)
            packet.append(b)
            checksum ^= b
        
        if checksum in [SOH, EOT, ESC]:
            packet.append(ESC)
        packet.append(checksum)
        
        packet.append(EOT)
        packet.append(CR)
        packet.append(LF)
        
        return bytes(packet)
    
    def send_request(self, tag: int, a1: int, a2: int, request_id=None):
        if not self.connected or not self.telemetry:
            self.log_debug("❌ Cannot send request - not connected", "Errors")
            return

        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            self.log_debug("❌ Serial port not open", "Errors")
            return

        raw_data = bytearray()
        raw_data.append(PacketTag.REQUEST)
        raw_data.extend(struct.pack('<H', 3))
        raw_data.append(tag)
        raw_data.append(a1)
        raw_data.append(a2)

        packet = self._build_packet_with_checksum(raw_data)

        if tag == PacketTag.MISC:
            self._pending_misc_command = a1

        try:
            serial_port.write(packet)
            req_data = {'request_tag': tag, 'a1': a1, 'a2': a2}
            packet_logger.log_sent(PacketTag.REQUEST, "REQUEST", req_data)
            self.log_debug(f"📤 Sent request: tag={tag}, a1={a1}, a2={a2}", "Info")
            if request_id is not None:
                ack_handler.request_sent(request_id)
            else:
                found_id = None
                for req_id, req in ack_handler.pending_requests.items():
                    if req['tag'] == tag:
                        found_id = req_id
                        break
                if found_id is not None:
                    ack_handler.request_sent(found_id)
        except Exception as e:
            self.log_debug(f"❌ Failed to send request: {e}", "Errors")
    
    def send_raw_packet(self, tag: int, body: bytes):
        """Send an arbitrary packet with given tag and body"""
        if not self.connected or not self.telemetry:
            self.log_debug("❌ Cannot send - not connected", "Errors")
            return
        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            return
        raw_data = bytearray()
        raw_data.append(tag)
        raw_data.extend(struct.pack('<H', len(body)))
        raw_data.extend(body)
        packet = self._build_packet_with_checksum(raw_data)
        try:
            serial_port.write(packet)
            self.log_debug(f"📤 Sent raw packet: tag={tag}, len={len(body)}", "Info")
        except Exception as e:
            self.log_debug(f"❌ Failed to send raw packet: {e}", "Errors")
    
    def send_test(self, test_id: int, action: int = 0):
        """Send a diagnostic test request (Tag=73). Body: testId, action."""
        self.send_raw_packet(PacketTag.TEST_REQUEST, bytes([test_id, action]))

    def handle_anomaly(self, parsed):
        """FC reported an IMU I2C fault during live flight (tag 74, id 0xff).

        Latch a sticky orange highlight on the IMU group box (it stays
        until cleared) and append the record to the anomaly log file.
        """
        d = parsed.data or [0] * 16
        self.anomaly_logger.log_anomaly(
            d[1], d[2], d[3], d[4], d[5], d[6], d[7],
            d[8], d[9], d[10], d[11], d[12], d[13], d[14], d[15])
        if not self._anomaly_pending:
            self._anomaly_pending = True
            self.log_debug("🔺 I2C ANOMALY — IMU group latched orange", "Errors")
        self._set_imu_alert(True)

    def _set_imu_alert(self, on: bool):
        """Orange (sticky) or normal border on the IMU group box."""
        box = getattr(self, "imu_box", None)
        if box is None:
            return
        if on:
            box.setStyleSheet(
                "QGroupBox { font-weight: bold; border: 3px solid #ff7f00; "
                "border-radius: 4px; margin-top: 6px; } QGroupBox::title { "
                "subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; "
                "color: #ff7f00; }")
        else:
            box.setStyleSheet(
                "QGroupBox { font-weight: bold; border: 1px solid black; "
                "border-radius: 4px; margin-top: 6px; } QGroupBox::title { "
                "subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")

    def send_afname(self, name: str):
        """Push the airframe name to the FC config flash (Tag=75). Called right
        before the param commit so the name is persisted in the SAME config
        flash write as the parameters — the single source of truth for the
        airframe the GCS displays after restart.
        """
        if not self.connected or not self.telemetry:
            self._trace("❌ [commit] Cannot send AFName - not connected", "Errors")
            return
        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            self._trace("❌ [commit] Serial port not open - cannot send AFName", "Errors")
            return
        if not name:
            self._trace("  ℹ️ [commit] No airframe name to persist — skipping AFName write", "Info")
            return
        body = name.encode('ascii', errors='replace')
        if len(body) > 31:
            body = body[:31]
        raw_data = bytearray()
        raw_data.append(PacketTag.AFNAME_SET)
        raw_data.extend(struct.pack('<H', len(body)))
        raw_data.extend(body)
        packet = self._build_packet_with_checksum(raw_data)
        try:
            n = serial_port.write(packet)
            self._trace(f"📤 [commit] Sent AFName: {body.decode('ascii')!r} ({len(packet)} B wired, write()={n})", "Info")
        except Exception as e:
            self._trace(f"❌ [commit] Failed to send AFName: {e}", "Errors")

    def send_param_commit(self):
        """Tell FC to save current params to flash."""
        raw_data = bytearray([PacketTag.PARAM_COMMIT])
        raw_data.extend(struct.pack('<H', 0))
        packet = self._build_packet_with_checksum(raw_data)
        serial_port = getattr(self.telemetry, 'serial', None)
        if serial_port and serial_port.is_open:
            try:
                serial_port.write(packet)
                self.log_debug("📤 Sent param commit")
            except Exception as e:
                self.log_debug(f"❌ Failed to send commit: {e}", "Errors")

    def send_param_image(self):
        """Send the complete 128-value parameter image as tag 77 (512 B + CRC32).

        The FC verifies the CRC, stages the image, clamps and copies it into the
        active Config mirror, writes the whole config to flash in one blocking
        erase+program, conditions it via LoadParameters(), ACKs, then waits for
        the GCS reboot command (tag 78), which is sent once the ACK is seen.
        This is the bulk replacement for the per-parameter tag-17 batch + tag-72
        commit. The image is built by the parameter window, which fills params it
        has no widget for from the last full tag-71 readback so a block write can
        never zero a parameter the GCS does not manage.
        """
        if not self.param_window:
            self._trace("❌ [commit] Cannot send param image - no parameter window", "Errors")
            return
        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            self._trace("❌ [commit] Serial port not open - cannot send param image", "Errors")
            return
        try:
            image = self.param_window.build_raw_image()
        except Exception as e:
            self._trace(f"❌ [commit] Failed to build param image: {e}", "Errors")
            return
        body = struct.pack('<%df' % len(image), *image)
        crc = zlib.crc32(body) & 0xFFFFFFFF
        raw_data = bytearray([PacketTag.PARAM_IMAGE])
        raw_data.extend(struct.pack('<H', len(body) + 4))
        raw_data.extend(body)
        raw_data.extend(struct.pack('<I', crc))
        packet = self._build_packet_with_checksum(raw_data)
        try:
            n = serial_port.write(packet)
            self._trace(
                f"📤 [commit] Sent param image ({len(body)} B + CRC 0x{crc:08X}, "
                f"{len(packet)} B wired, write()={n})")
        except Exception as e:
            self._trace(f"❌ [commit] Failed to send param image: {e}", "Errors")

    def send_reboot(self):
        """Authorise the FC to reset after a verified flash commit (tag 78).

        The FC writes and verifies the whole config to flash, ACKs tag 77, and
        then waits — it no longer resets itself. This command is the GCS's
        explicit go-ahead, so the reboot edge happens on the GCS's terms and the
        post-reboot readback is served by the FC that came back. There is no
        ACK: the link drop and reconnect are the confirmation, and a reboot ACK
        would just re-enter the same post-reset race this handshake removes.
        """
        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            self._trace("❌ [commit] Cannot send reboot command - serial port not open", "Errors")
            return
        raw_data = bytearray([PacketTag.REBOOT])
        raw_data.extend(struct.pack('<H', 0))
        packet = self._build_packet_with_checksum(raw_data)
        try:
            n = serial_port.write(packet)
            self._trace(f"📤 [commit] Sent reboot command ({len(packet)} B wired, write()={n})")
        except Exception as e:
            self._trace(f"❌ [commit] Failed to send reboot command: {e}", "Errors")

    def abort_param_writes(self):
        """Stop an in-flight tag-17 batch without reporting completion.

        Used before a tag-77 block send: the block carries the full widget image,
        so any queued/awaited per-parameter write is superseded and must not be
        left racing the 516-byte block onto the wire.
        """
        self._param_write_ack_timer.stop()
        self._param_write_list = []
        self._param_write_sent_pending = {}
        self._param_write_await_idx = None
        self._param_write_await_value = None
        self._param_write_drain_until_ms = 0.0
        self._param_write_on_complete = None

    def _handle_param_packet(self, parsed):
        """Shared handler for tag 17 and tag 71 param packets"""
        if hasattr(parsed, 'entries'):
            if ParamIndex.CONFIG1_BITS in parsed.entries:
                self.config1_cache = int(parsed.entries[ParamIndex.CONFIG1_BITS][1])
            if ParamIndex.CONFIG2_BITS in parsed.entries:
                self.config2_cache = int(parsed.entries[ParamIndex.CONFIG2_BITS][1])
            if ParamIndex.TRACE_TYPE in parsed.entries:
                self._sync_trace_combo(parsed.entries[ParamIndex.TRACE_TYPE][1])
            if self.param_window is not None:
                self.param_window.update_params_from_typed(parsed)
            else:
                self._pending_typed_params = parsed
        self._report_reset_cause(parsed)
        version = getattr(parsed, 'version_name', '')
        if version and not self._revision_from_afname:
            self.revision_label.setText(f"UAVX {version}")

    def can_read_parameters(self):
        """GCS-side gate: allowed to request a param download from the FC."""
        if not self.connected or not self.telemetry:
            return False, "Not connected"
        if self.param_window is not None and self.param_window._flash_write_pending:
            return False, "Flash write/commit outstanding"
        return True, ""

    def can_write_parameters(self):
        """GCS-side gate: allowed to commit parameters to FC flash.

        The FC hard-blocks flash erase/write in flight (RefreshConfig gates on
        State != eInFlight), and heads towards a verify after a commit, so this
        is defense-in-depth plus the user-facing explanation for the block.
        """
        if not self.connected or not self.telemetry:
            return False, "Not connected"
        if self._flash_write_in_progress():
            return False, "A flash write/commit is already in progress"
        fd = getattr(self, 'flight_data', None)
        if fd and fd.flight_state == FlightState.eInFlight:
            return False, "The aircraft is in flight"
        return True, ""

    def _flash_write_in_progress(self):
        if self.param_window is not None:
            return (self.param_window._flash_write_pending
                    or self.param_window._write_in_progress)
        return False

    def send_params_typed(self, on_complete=None, indices=None, progress_cb=None):
        """Send widget values as individual param packets, ACK-driven.

        If indices is provided, only those param indices are sent.
        progress_cb(sent, total) is called after each packet is written.

        on_complete(unconfirmed) is called once the last param has been
        WRITTEN *and* confirmed by a tag-17 echo (or recorded as unconfirmed by
        the echo watchdog). unconfirmed is the list of indices the FC never
        acknowledged, so a caller about to commit a flash block can refuse
        rather than pack a partially applied image.
        """
        if not self.connected or not self.telemetry:
            on_complete and on_complete([])
            return
        if not self.param_window:
            on_complete and on_complete([])
            return
        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            on_complete and on_complete([])
            return

        param_range = indices if indices is not None else range(128)
        params = []
        skipped = []
        for i in param_range:
            # param_widget() falls back to the Setup group, so a setup-only
            # parameter (AF_TYPE et al) is sent with its real value instead of
            # being written to the FC as a silent 0.0.
            widget = self.param_window.param_widget(i)
            if widget is None:
                # No widget means no value to assert. Writing 0.0 here (the
                # previous behaviour) silently CLOBBERED the FC's stored value
                # for every parameter the GCS has no widget for - a live way to
                # zero a parameter the user never touched. Skip instead: the FC
                # keeps whatever it already holds, and the omission is reported
                # rather than written.
                skipped.append(i)
                continue
            if isinstance(widget, QDoubleSpinBox):
                display = widget.value()
                mult = self.param_window._display_mult(i)
                fval = display / mult
            elif isinstance(widget, QSpinBox):
                fval = float(widget.value())
            elif isinstance(widget, QComboBox):
                data = widget.currentData()
                if data is None:
                    raise ValueError(
                        f"Combo param {i} selection has no data "
                        f"(index={widget.currentIndex()}) — refusing to write "
                        f"a positional index as the value")
                fval = float(data)
            else:
                skipped.append(i)
                continue
            params.append((i, fval))

        if skipped:
            self.log_debug(
                f"⚠️ Skipped {len(skipped)} param(s) with no widget — FC values "
                f"left untouched: {skipped}", "Warnings")
        if not params:
            if skipped:
                self.log_debug(
                    f"❌ Nothing to send: all {len(skipped)} param(s) had no "
                    f"widget", "Errors")
            on_complete and on_complete(skipped)
            return

        self._param_write_list = params
        self._param_write_port = serial_port
        self._param_write_on_complete = on_complete
        self._param_write_progress = progress_cb
        self._param_write_total = len(params)
        self._param_write_index = 0
        self._param_write_sent_pending = {}
        self._param_write_drain_until_ms = 0.0
        self._param_write_chunk_sent = 0
        # Clear any dangling await from a previous cycle. _finish_param_write
        # already does this on the normal exit; doing it here too means a
        # re-entrant call (e.g. the commit path's flush) can never inherit a
        # stale await and stall the new batch before it sends a single byte.
        self._param_write_await_idx = None
        self._param_write_await_value = None
        self._param_write_ack_timer.start()
        self._send_next_param()

    def _finish_param_write(self, unconfirmed):
        """Close out the sender and hand the unconfirmed list to the caller.

        on_complete is called WITH the list of indices the FC never echoed, so
        a caller that is about to commit a flash block can refuse rather than
        pack a partially applied image.
        """
        self._param_write_ack_timer.stop()
        self._param_write_await_idx = None
        self._param_write_await_value = None
        self._param_write_drain_until_ms = 0.0
        self._param_write_sent_pending = {}
        cb = self._param_write_on_complete
        self._param_write_on_complete = None
        self._param_write_progress = None
        self._param_write_port = None
        cb and cb(unconfirmed)

    def _send_next_param(self):
        # One index in flight at a time: the next param is not sent until the
        # FC has echoed the current one (or the watchdog gives up on it).
        if self._param_write_await_idx is not None:
            return
        if not self._param_write_list:
            if not self._param_write_sent_pending:
                self.log_debug("✅ All param packets confirmed by FC")
                self._finish_param_write([])
                return
            # List is empty but echoes are still outstanding: hold the batch
            # open for stragglers instead of declaring them lost immediately.
            if self._param_write_drain_until_ms <= 0.0:
                self._param_write_drain_until_ms = (time.monotonic() * 1000.0
                                                    + PARAM_WRITE_DRAIN_MS)
                self.log_debug(
                    f"⏳ Draining {len(self._param_write_sent_pending)} "
                    f"outstanding echo(s)…")
            return
        idx, fval = self._param_write_list.pop(0)
        serial_port = self._param_write_port
        if not serial_port or not serial_port.is_open:
            self.log_debug("❌ Serial port closed during param send", "Errors")
            self._param_write_list = []
            self._finish_param_write(list(range(idx, 128)))
            return
        try:
            raw_data = bytearray([PacketTag.PARAM])
            raw_data.extend(struct.pack('<H', 5))
            raw_data.append(idx)
            raw_data.extend(struct.pack('<f', fval))
            packet = self._build_packet_with_checksum(raw_data)
            serial_port.write(packet)
        except Exception as e:
            self.log_debug(f"❌ Failed to send param {idx}: {e}", "Errors")
            self._param_write_list = []
            self._finish_param_write([idx])
            return
        self._param_write_index += 1
        self._param_write_sent_pending[idx] = fval
        # Any send makes the batch active again, so drop a drain deadline left
        # over from a previous quiet period — it must be re-armed fresh when the
        # batch next goes quiet, not inherited and expire under the new param.
        self._param_write_drain_until_ms = 0.0
        self._param_write_await_idx = idx
        self._param_write_await_value = fval
        self._param_write_await_sent_ms = time.monotonic() * 1000.0
        self.log_debug(f"📤 Sent param {idx} = {fval} (awaiting echo)")
        pcb = self._param_write_progress
        if pcb:
            pcb(self._param_write_index, self._param_write_total)

    def _on_param_write_echo(self, parsed):
        """Tag-17 echo hook: reconcile an echo BY INDEX, then release the sender.

        Called only from the tag-17 dispatch, so a tag-71 bulk readback (same
        handler, different tag) can never be mistaken for a single-param ACK.

        WHY index-reconciliation and not strict lockstep (2026-09-29): the first
        version required `base_idx == _param_write_await_idx` and discarded every
        other echo. Combined with an echo timeout shorter than the FC's real
        turn-around, that desynchronised the sender by one index and then kept
        it one ahead permanently — one late echo made EVERY subsequent param
        report unconfirmed (15 confirmed, 113 unconfirmed), even though the FC
        had applied all 128 writes correctly. The data was fine; the
        confirmations were being thrown away.

        Now any echo for an index in _param_write_sent_pending confirms THAT
        index, whether or not it is the one currently awaited. A late echo
        therefore rescues its own index instead of poisoning the rest of the
        batch, and the timeout degrades to a reporting speed rather than a
        correctness dependency.
        """
        idx = getattr(parsed, 'base_idx', None)
        if idx is None or idx not in self._param_write_sent_pending:
            return          # not ours, or already confirmed — a stale echo
        sent_value = self._param_write_sent_pending.pop(idx)
        awaited = self._param_write_await_idx
        entries = getattr(parsed, 'entries', None) or {}
        echoed = entries[idx][1] if idx in entries else None
        # Float compare with a small tolerance: the value round-trips through
        # float32 on the wire, so an exact == would miss legitimate echoes.
        if echoed is not None and abs(float(echoed) - sent_value) > 1e-6:
            self.log_debug(
                f"❌ Echo mismatch for param {idx}: sent "
                f"{sent_value}, FC returned {echoed}", "Errors")
        elif idx == awaited:
            self.log_debug(f"✅ Param {idx} confirmed by FC ({echoed})")
        else:
            # Late echo for an index the watchdog already gave up on. It
            # arrived, so it is confirmed — this is the case the old code threw
            # away, and it is the common case whenever the timeout is tight.
            self.log_debug(
                f"✅ Param {idx} confirmed by late echo ({echoed}) "
                f"— was timed out, now reconciled")
        if idx == awaited:
            self._param_write_await_idx = None
            self._param_write_await_value = None
        # A no-op if something is still awaited; otherwise pumps the next param
        # or opens the drain window.
        QTimer.singleShot(0, self._send_next_param)

    def _check_param_write_ack(self):
        """Watchdog: stop waiting on a param the FC has not echoed yet, and
        close the batch out once the straggler drain window expires.

        A timeout here no longer records the index as failed. It only stops
        WAITING: the index stays in _param_write_sent_pending, so its echo can
        still arrive and confirm it (see _on_param_write_echo). The
        authoritative unconfirmed set is whatever is still pending when the
        drain window closes, and it is reported through on_complete so the
        flash commit can be REFUSED rather than packing a partially applied RAM
        image.
        """
        now = time.monotonic() * 1000.0
        idx = self._param_write_await_idx
        if idx is not None:
            elapsed = now - self._param_write_await_sent_ms
            if elapsed < PARAM_WRITE_ECHO_TIMEOUT_MS:
                return
            self.log_debug(
                f"⏱ No echo for param {idx} after {elapsed:.0f} ms — "
                f"moving on (echo may still arrive and confirm it)", "Warnings")
            self._param_write_await_idx = None
            self._param_write_await_value = None
            QTimer.singleShot(0, self._send_next_param)
            return
        if self._param_write_drain_until_ms <= 0.0:
            return
        if now < self._param_write_drain_until_ms:
            return
        self._param_write_drain_until_ms = 0.0
        missing = sorted(self._param_write_sent_pending)
        if missing:
            self.log_debug(
                f"⚠️ Param write finished with {len(missing)} unconfirmed: "
                f"{missing}", "Warnings")
        self._finish_param_write(missing)

    def _write_param_direct(self, idx, fval):
        """Queue one raw (idx, fc-float) param write through the tag-17 sender."""
        self._write_params_live([(idx, float(fval))])

    def _write_params_live(self, items):
        """Queue raw (idx, fc-float) param writes through the tag-17 sender.

        Live-write path (debounced spinbox/combo changes and the trace combo):
        writes reach the FC's RAM image immediately; flash is a separate,
        explicit commit. If a batch is already in flight, append to its tail —
        the pump drains the list until empty, so nothing is dropped (the GCS
        commit path depends on every pending value landing in RAM before the
        flash commit packs the block).
        """
        if not items:
            return
        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            return
        if (self._param_write_list or self._param_write_await_idx is not None
                or self._param_write_sent_pending):
            # A param is "in flight" while its list entry is still queued, while
            # the last one is waiting on its echo, OR while any sent param is
            # still unconfirmed (which includes the straggler-drain window) —
            # the list empties before the echo arrives, so testing the list
            # alone would restart the sender and orphan the in-flight awaits
            # (losing their echoes).
            self.log_debug(f"⏳ Param write in flight — appending {len(items)} live writes")
            self._param_write_list.extend(items)
            # Re-pump in case the in-flight state is the drain window (await is
            # None, list was empty): without this the appended live write would
            # sit unsent until the drain closed out and then be dropped by
            # _finish_param_write. A no-op when something is already awaited.
            QTimer.singleShot(0, self._send_next_param)
            return
        self._param_write_list = list(items)
        self._param_write_port = serial_port
        self._param_write_on_complete = None
        self._param_write_progress = None
        self._param_write_index = 0
        self._param_write_total = len(items)
        self._param_write_sent_pending = {}
        self._param_write_drain_until_ms = 0.0
        self._param_write_chunk_sent = 0
        self._param_write_await_idx = None
        self._param_write_await_value = None
        self._param_write_ack_timer.start()
        self._send_next_param()

    def _set_trace_combo_state(self, state):
        style = _TRACE_STATE_STYLE.get(state)
        self.trace_type_combo.setStyleSheet(style or "")
        suffix = {
            'pending': ' — orange: awaiting FC confirmation',
            'synced': ' — green: value confirmed by the FC',
        }.get(state, '')
        self.trace_type_combo.setToolTip(self._trace_combo_tooltip + suffix)

    def _set_trace_combo_value(self, idx):
        if not 0 <= idx < self.trace_type_combo.count():
            idx = 1
        self._trace_combo_loading = True
        self.trace_type_combo.setCurrentIndex(idx)
        self._trace_combo_loading = False

    def _on_trace_type_changed(self, index):
        if self._trace_combo_loading or not self.connected:
            return
        tt = self.trace_type_combo.itemData(index)
        if tt is None:
            return
        self._trace_write_pending = int(tt)
        self._set_trace_combo_state('pending')
        self.log_debug(f"🎚 Trace type set to {tt}", "Info")
        self._write_param_direct(int(ParamIndex.TRACE_TYPE), float(tt))

    def _sync_trace_combo(self, fc_val):
        f = int(round(fc_val))
        self._trace_fc_value = f
        cur = self.trace_type_combo.currentData()
        if cur == f:
            self._trace_write_pending = None
            self._set_trace_combo_state('synced')
        elif self._trace_write_pending is not None:
            self._set_trace_combo_state('pending')
        else:
            self._set_trace_combo_value(f)
            self._set_trace_combo_state('synced')

    def _check_speech_events(self, f):
        batt = f.battery_volts
        if batt > 0 and abs(batt - self._last_spoken_batt) >= 0.5:
            self.speech.speak_battery(batt)
            self._last_spoken_batt = batt

        alt = getattr(f, 'altitude', 0.0)
        if alt >= 0:
            # Announce on each 5 m step up (or down) so altitude is spoken
            # without constant chatter. 5.0 is a runtime constant, not a divisor.
            bucket = int(alt * 0.2)   # 5 m bucket from metres
            if bucket != self._last_spoken_alt and bucket > 0:
                self.speech.speak_altitude(bucket * 5.0)
                self._last_spoken_alt = bucket

        # Direction + distance callout. The FC computes the aircraft's home-
        # relative range/bearing itself and ships it in the guidance packet
        # (tag 59, SendGuidancePacket, gated on in-flight + OriginValid), so
        # use its authoritative numbers — distance (m) and bearing (deg).
        g = self.guidance_data
        if g is not None and hasattr(g, 'distance') and hasattr(g, 'bearing'):
            now = time.time()
            if g.distance > _DIRECTION_FAR_M and now - self._last_direction_speak >= _DIRECTION_SPEAK_PERIOD_S:
                # Round to 10 m: use the inverse (0.1) not a division.
                rounded = int(round(g.distance * 0.1)) * 10
                # 45 deg sectors, 8 compass points. *0.0222.. = /45.
                idx = int(round((g.bearing % 360.0) * (1.0/45.0))) % 8
                point = _DIRECTION_POINTS[idx]
                self.speech.speak_direction(rounded, point)
                self._last_direction_speak = now

        if hasattr(f, 'flag_bits') and f.flag_bits:
            fb = f.flag_bits
            gps_ok = len(fb) > 6 and fb[6]
            armed = len(fb) > 29 and fb[29]
            wp_ach = len(fb) > 17 and fb[17]
            low_batt = len(fb) > 5 and fb[5]

            if gps_ok != self._last_spoken_gps_ok:
                if gps_ok:
                    sats = getattr(f, 'gps_sats', 0)
                    self.speech.speak_gps_acquired(sats)
                else:
                    self.speech.speak_gps_lost()
                self._last_spoken_gps_ok = gps_ok

            if armed != self._last_spoken_armed:
                if armed:
                    self.speech.speak_armed()
                else:
                    self.speech.speak_disarmed()
                self._last_spoken_armed = armed

            if wp_ach and not self._last_spoken_wp_ach:
                self.speech.speak_waypoint_reached(getattr(f, 'curr_wp', 0))
            self._last_spoken_wp_ach = wp_ach

            if low_batt and batt > 0 and not self._last_spoken_low_batt:
                self.speech.speak_battery_warning(batt)
            self._last_spoken_low_batt = low_batt

        fs = getattr(f, 'flight_state', -1)
        if fs >= 0:
            if fs != self._last_spoken_flight_state:
                if fs != self._pending_flight_state:
                    self._pending_flight_state = fs
                    self._state_change_time = time.monotonic()

            now = time.monotonic()
            if (self._pending_flight_state == fs
                    and now - self._state_change_time >= 0.3):
                name = FlightState.get_name(fs)
                self.speech.speak_mode(name)
                self._last_spoken_flight_state = fs
                self._pending_flight_state = -1

        alarm = getattr(f, 'alarm_state', -1)
        if alarm != self._last_spoken_alarm_state and alarm > 0:
            alarm_name = {
                2: "Low battery", 3: "Signal lost", 4: "Hit fence",
                5: "Upside down", 6: "Forced landing",
            }.get(alarm, f"Alarm {alarm}")
            self.speech.speak_alarm(alarm_name)
            self._last_spoken_alarm_state = alarm

    def process_packet(self, data: bytes):
        self._last_rx_time = time.monotonic()
        parsed, tag = parse_packet(data, 1.0)

        TAG_NAMES = {
            13: "Flight", 14: "Nav", 15: "Stats", 16: "Control", 17: "Param",
            18: "Min", 19: "Origin", 20: "WP", 21: "Mission", 22: "RC",
            50: "Request", 51: "ACK", 52: "Misc", 53: "Unused", 54: "BB",
            55: "Unused", 56: "Unused", 57: "Tuning", 58: "Unused", 59: "Guidance",
            60: "Unused", 61: "Unused", 62: "Calibration", 63: "AFName",
            64: "Wind", 65: "Unused", 66: "SerialPorts", 67: "ExecTime",
            68: "Unused", 69: "LinkStats", 70: "InitState", 71: "ParamRead",
            76: "I2CErrors", 77: "ParamImage",
        }
        tag_name = TAG_NAMES.get(tag, f"Unknown({tag})")
        tag_ok = "OK" if parsed is not None else "FAIL"
        # Rate-limit the per-packet terminal noise: coalesce repeated packets
        # (high-rate telemetry, RC bursts, test responses) to ~1 Hz while
        # single-shot packets (param/config/etc.) still print every time.
        now_t = time.time()
        last = getattr(self, '_rx_print_t', {})
        interval = last.get(tag, 0.0)
        if tag_ok == "FAIL" or tag > 68 or now_t - interval >= 0.8:
            detail = f" raw={data.hex()}" if tag_ok == "FAIL" else ""
            print(f"[RX] tag={tag} ({tag_name}) {tag_ok} len={len(data)}B{detail}")
        last[tag] = now_t
        self._rx_print_t = last
        if self.param_window and hasattr(self.param_window, 'retry_read_if_pending'):
            self.param_window.retry_read_if_pending()

        # Terminal dump of decoded packet contents (same ~1 Hz rate gate as
        # the [RX] header above so high-rate telemetry stops flooding the
        # console; FAIL frames always show). Uses the pre-update interval.
        dump_ok = (tag_ok == "FAIL" or tag > 68 or now_t - interval >= 0.8)
        if tag == 13 and hasattr(parsed, 'flag_bits') and dump_ok:
            fb = parsed.flag_bits
            def fbok(i): return fb[i] if len(fb) > i else False
            r2d = 57.2958
            pwm_str = ""
            if hasattr(parsed, 'pwm') and parsed.pwm:
                # RawPW idle-zero-referenced (0=1000uS, 0.5=1500uS); show uS.
                # Guard the round(): a non-finite FC value would raise
                # ValueError ("cannot convert float NaN to integer").
                pwm_vals = [f"M{i}={round((v + 1.0) * 1000):+5d}"
                            if math.isfinite(v) else f"M{i}=  nan"
                            for i, v in enumerate(parsed.pwm)]
                pwm_str = f" pwm={' '.join(pwm_vals)}"
            fs_name = FlightState.get_name(parsed.flight_state)
            print(f"[FLIGHT] state={fs_name} "
                  f"{'A' if fbok(29) else '_'}{'E' if fbok(27) else '_'}"
                  f"{'I' if fbok(37) else '_'}{'M' if fbok(38) else '_'}{'G' if fbok(6) else '_'}"
                  f" ang=({parsed.angle_roll*r2d:+6.1f},{parsed.angle_pitch*r2d:+6.1f},{parsed.angle_yaw*r2d:+6.1f})"
                  f" dAng=({parsed.desired_roll*r2d:+5.1f},{parsed.desired_pitch*r2d:+5.1f},{parsed.desired_yaw*r2d:+5.1f})"
                  f" rate=({parsed.rate_roll*r2d:+6.2f},{parsed.rate_pitch*r2d:+6.2f},{parsed.rate_yaw*r2d:+6.2f})"
                  f" dRate=({parsed.desired_rate_roll*r2d:+5.1f},{parsed.desired_rate_pitch*r2d:+5.1f},{parsed.desired_rate_yaw*r2d:+5.1f})"
                  f" acc=({parsed.acc_lr:+5.2f},{parsed.acc_fb:+5.2f},{parsed.acc_du:+5.2f})"
                  f" thr={parsed.desired_throttle:.3f} alt={parsed.altitude:.1f}{pwm_str}")
        elif tag == 14 and hasattr(parsed, 'gps_lat') and dump_ok:
            print(f"[NAV] lat={parsed.gps_lat:.6f} lon={parsed.gps_lon:.6f}"
                  f" spd={parsed.gps_vel:.1f} hdg={parsed.gps_heading * 57.2958:.1f}"
                  f" sats={parsed.gps_sats} fix={parsed.gps_fix}")
        elif tag == 16 and hasattr(parsed, 'angle_roll') and dump_ok:
            r2d = 57.2958
            print(f"[CTRL] ang=({parsed.angle_roll*r2d:+6.1f},{parsed.angle_pitch*r2d:+6.1f},{parsed.angle_yaw*r2d:+6.1f})"
                  f" rate=({parsed.rate_roll*r2d:+6.2f},{parsed.rate_pitch*r2d:+6.2f},{parsed.rate_yaw*r2d:+6.2f})"
                  f" thr={parsed.desired_throttle:.3f}")
        elif tag == 22 and hasattr(parsed, 'rc_channels') and dump_ok:
            rc = parsed.rc_channels
            if rc:
                def rc_name(i):
                    return ["Thr","Rol","Pit","Yaw","Nav","Att","NQ","Cam","Trace","Trn","PT","Dive"][i]
                parts = [f"{rc_name(i)}={round(rc[i]) if math.isfinite(rc[i]) else 'nan'}"
                         for i in range(min(len(rc), 12))]
                print("[RC] " + " ".join(parts))
        elif tag == 62 and isinstance(parsed, dict) and dump_ok:
            flags = parsed.get('flags', [])
            if len(flags) > 5:
                imu   = bool(flags[4] & 32)  # bit 5 byte 4 = IMUActive
                mag   = bool(flags[4] & 64)  # bit 6 byte 4 = MagnetometerActive
                imuc  = bool(flags[5] & 16)  # bit 4 byte 5 = IMUCal
                magc  = bool(flags[5] & 2)   # bit 1 byte 5 = MagnetometerCalibrated
                rb    = parsed.get('rate_bias', [])
                gs    = (f"gyr=({rb[0]*RATE_GYRO_SCALE*57.2958:.1f},"
                         f"{rb[1]*RATE_GYRO_SCALE*57.2958:.1f},"
                         f"{rb[2]*RATE_GYRO_SCALE*57.2958:.1f})") if len(rb) > 2 else ""
                ids   = f"imu_id=0x{parsed.get('imu_id',0):02X} mag_id=0x{parsed.get('mag_id',0):02X}"
                hx    = lambda v: f"0x{v:02X}" if isinstance(v, int) else "--"
                cfg   = (f"mag=[A={hx(parsed.get('mag_cfg_a'))} "
                         f"B={hx(parsed.get('mag_cfg_b'))} "
                         f"MODE={hx(parsed.get('mag_mode'))}]")
                print(f"[CALIB] IMU={'Y' if imu else 'N'}{'C' if imuc else '_'}"
                      f" Mag={'Y' if mag else 'N'}{'C' if magc else '_'} {gs} {ids} {cfg}")

        if tag == 13:
            self.log_debug(f"📦 Flight ({len(data)}B)", "Info")
        elif tag == 14:
            pass  # suppress NAV clutter during emulation

        match tag:
            case 13:
                if self.flight_data:
                    old_pwm = self.flight_data.pwm if hasattr(self.flight_data, 'pwm') and self.flight_data.pwm else None
                    old_rc = self.flight_data.rc_channels if hasattr(self.flight_data, 'rc_channels') and self.flight_data.rc_channels else None
                    old_discovered = self.flight_data.discovered_channels if hasattr(self.flight_data, 'discovered_channels') and self.flight_data.discovered_channels else None
                    old_rc_physical = getattr(self.flight_data, 'rc_physical', None)
                    old_rc_flags = getattr(self.flight_data, 'rc_flags', None)
                    old_gps = {attr: getattr(self.flight_data, attr, 0) for attr in
                               ("gps_lat", "gps_lon", "gps_sats", "gps_fix",
                                "gps_hacc", "gps_vacc", "gps_sacc", "gps_cacc",
                                "gps_vel", "gps_heading", "gps_altitude",
                                "gps_type", "gps_update_rate",
                                "curr_wp", "wp_bearing",
                                "distance_to_wp", "cross_track_error",
                                "mag_var_wmm")}
                else:
                    old_pwm = None
                    old_rc = None
                    old_discovered = None
                    old_gps = {}
                self.flight_data = parsed
                if old_pwm is not None and not self.flight_data.pwm:
                    self.flight_data.pwm = old_pwm
                if old_rc is not None and not self.flight_data.rc_channels:
                    self.flight_data.rc_channels = old_rc
                if old_discovered is not None and not self.flight_data.discovered_channels:
                    self.flight_data.discovered_channels = old_discovered
                if old_rc_physical:
                    self.flight_data.rc_physical = old_rc_physical
                if old_rc_flags is not None:
                    self.flight_data.rc_flags = old_rc_flags
                for attr, val in old_gps.items():
                    if getattr(self.flight_data, attr, None) in (None, 0, 0.0):
                        setattr(self.flight_data, attr, val)
                data_manager.update_flight_data(self.flight_data)
                packet_logger.log_received(13, "FLIGHT", parsed)
                if hasattr(parsed, 'flag_bits'):
                    self.current_flag_bits = parsed.flag_bits

                # Track flight lifecycle
                if hasattr(parsed, 'flag_bits') and len(parsed.flag_bits) > 29:
                    armed = parsed.flag_bits[29]
                    self._in_flight = armed

                self._check_speech_events(parsed)

            case 14:
                self.nav_data = parsed
                data_manager.update_nav_data(parsed)
                packet_logger.log_received(14, "NAV", parsed)
                if self.flight_data:
                    for attr in ("gps_lat", "gps_lon", "gps_sats", "gps_fix",
                                 "gps_hacc", "gps_vacc", "gps_sacc", "gps_cacc",
                                 "gps_vel", "gps_heading", "gps_altitude",
                                 "gps_type", "gps_update_rate",
                                 "curr_wp", "wp_bearing", "cross_track_error",
                                 "nav_state", "alarm_state",
                                 "mag_var_wmm"):
                        setattr(self.flight_data, attr, getattr(parsed, attr, getattr(self.flight_data, attr)))
                    data_manager.update_flight_data(self.flight_data)

            case 16:
                self.control_data = parsed
                packet_logger.log_received(16, "CONTROL", parsed)
                if self.flight_data:
                    for attr in ("angle_roll", "angle_pitch", "angle_yaw",
                                 "rate_roll", "rate_pitch", "rate_yaw",
                                 "desired_throttle"):
                        setattr(self.flight_data, attr, getattr(parsed, attr, getattr(self.flight_data, attr)))
                    if parsed.pwm:
                        self.flight_data.pwm = parsed.pwm
                    data_manager.update_flight_data(self.flight_data)

            case 19:
                self.origin_data = parsed
                packet_logger.log_received(19, "ORIGIN", parsed)
                if isinstance(parsed, OriginData):
                    self.log_debug(f"  🏠 Origin: {parsed.num_waypoints} WPs", "Info")

            case 20:
                packet_logger.log_received(20, "WP", parsed)
                self._last_wp_data = parsed if isinstance(parsed, dict) else None
                if isinstance(parsed, dict):
                    self.log_debug(f"  📍 WP {parsed.get('wp_index')}: "
                                  f"lat={parsed.get('wp_lat', 0):.6f}", "Info")

            case 22:
                packet_logger.log_received(22, "RC", parsed)
                if hasattr(parsed, 'rc_channels'):
                    self.flight_data.rc_channels = parsed.rc_channels
                    self.flight_data.rc_raw = getattr(parsed, 'rc_raw', [])
                    self.flight_data.rc_physical = getattr(parsed, 'rc_physical', [])
                    self.flight_data.rc_flags = getattr(parsed, 'rc_flags', 0)
                    self.flight_data.discovered_channels = getattr(parsed, 'discovered_channels', 0)
                    data_manager.update_flight_data(self.flight_data)

            case 51:
                if isinstance(parsed, dict) and 'ack_tag' in parsed:
                    at = parsed['ack_tag']
                    ok = parsed['ack_success']
                    packet_logger.log_received(51, "ACK", parsed)
                    if at in (241, 223):
                        return
                    # PARAM_COMMIT and PARAM_IMAGE are never registered with
                    # ack_handler (the Read/Write buttons they serve are gone),
                    # so handle_ack can only report "No pending request" for
                    # them. That is noise, and it printed a red ❌ immediately
                    # before a correct ✅. Both ACKs are handled on the dedicated
                    # path below.
                    if at not in (PacketTag.PARAM_COMMIT, PacketTag.PARAM_IMAGE):
                        ack_handler.handle_ack(at, ok)

                    cmd = getattr(self, '_pending_misc_command', None)
                    if at == PacketTag.MISC and cmd is not None:
                        self._pending_misc_command = None
                        if cmd == MiscCommand.ESC_PROG:
                            if ok:
                                self._on_esc_prog_ack()
                            else:
                                self._on_esc_prog_denied()
                        elif self.calib_window is not None:
                            self.calib_window.on_misc_ack(cmd, ok)

                    if at in (PacketTag.PARAM_COMMIT, PacketTag.PARAM_IMAGE):
                        # The ACK reflects whether the config (+ name) is
                        # CONFIRMED in flash (erase + program + read-back
                        # verify). ok=False means the FC did NOT save and will
                        # NOT reboot: surface it loudly and un-arm the deferred
                        # verification. tag-72 is the self-rebooting legacy path;
                        # tag-77 leaves the FC waiting, so the reset is only
                        # authorised here, once the flash commit is ACKed.
                        if self.param_window is not None:
                            self.param_window.on_param_commit_ack(ok)
                        if ok:
                            self.log_debug("✅ Flash commit CONFIRMED in FC",
                                           "Info")
                            if at == PacketTag.PARAM_IMAGE:
                                self.send_reboot()
                        else:
                            self.log_debug("❌ Flash commit FAILED — params NOT saved",
                                           "Errors")

            case 59:
                self.guidance_data = parsed
                packet_logger.log_received(59, "GUIDANCE", parsed)
                if self.flight_data and hasattr(parsed, 'distance'):
                    self.flight_data.distance_to_wp = parsed.distance

            case 62:
                packet_logger.log_received(62, "CALIB", parsed)
                if parsed is None:
                    self.log_debug("  CALIB: packet too short (<80) - firmware without chip-ID telemetry?", "Info")
                elif isinstance(parsed, dict):
                    data_manager.update_calibration_data(parsed)
                    mag_live = parsed.get('mag_live')
                    if mag_live and len(mag_live) == 3:
                        self._mag_live = mag_live
                    imu_id = parsed.get('imu_id')
                    mag_id = parsed.get('mag_id')
                    if imu_id is not None or mag_id is not None:
                        self.log_debug(f"  CALIB IDs: IMU=0x{imu_id:02X} Mag=0x{mag_id:02X}", "Info")

            case 63:
                if isinstance(parsed, dict) and 'revision' in parsed:
                    rev = parsed['revision']
                    packet_logger.log_received(63, "CONFIG", parsed)
                    af_type = parsed.get('af_type')
                    self.log_debug(f"  FW: {rev}, AFType: {af_type if af_type is not None else '?'}, "
                                   f"AFName: {parsed.get('airframe_name', '')}", "Info")
                    self.revision_label.setText(rev)
                    self._revision_from_afname = True
                    # Persisted airframe name from FC config flash is the single
                    # source of truth shown by the GCS (no local QSettings name).
                    af_name = parsed.get('airframe_name') or ''
                    if self.raw_logger.active:
                        self.raw_logger.rename(af_name)
                    # The tag-63 AFType byte is the single authoritative signal
                    # for "an airframe is configured". While the FC runs on the
                    # compiled ParamTable defaults it holds Config.Empty, and
                    # SendConfigPacket() reports that as eAFUnknown in this byte;
                    # once a real airframe commit clears Config.Empty the byte
                    # carries the true pAFType. Config.Empty is the FC-owned,
                    # persisted record ("this config is the ParamTable default,
                    # no airframe loaded"); the GCS only reads it.
                    #
                    # It must NOT key on the airframe NAME: a cal-cleared or
                    # never-configured board has AirframeName[0]==0, but the name
                    # is a display string and the round-trip through tag-75/tag-77
                    # is not what marks the FC's state. It must NOT key on the raw
                    # pAFType value either — the ParamTable default is eQuadXAF,
                    # so a blank FC would otherwise look configured. Nor on
                    # ConfigChanged, which the FC raises in normal flight.
                    # (Greg 2026-09-25; re-keyed on the Empty sentinel 2026-10-11.)
                    #
                    # Only gate once the FC has actually answered, so working
                    # offline with .af files is unaffected.
                    config_required = None
                    if af_type is not None:
                        config_required = (af_type == AirframeType.eAFUnknown)
                        if config_required:
                            self.log_debug(
                                "  ⚠️ FC has no airframe configured "
                                "(eAFUnknown / Config.Empty) — params page gated "
                                "until a generic .af is loaded.", "Warning")
                    if self.param_window is not None:
                        self.param_window.set_flash_airframe_name(af_name)
                        if config_required is not None:
                            self.param_window.set_configuration_required(
                                config_required)
                    else:
                        self._pending_flash_airframe_name = af_name
                        self._pending_configuration_required = config_required

            case 66:
                packet_logger.log_received(66, "SERIAL", parsed)
                if isinstance(parsed, dict) and 'telemetry' in parsed:
                    self.serial_ports = parsed
                    self.update_serial_ports_display()

            case 70:
                # One function only: the uint16 preflight term mask. The packet
                # arrives only on change, so this is a latched fault set. It is
                # latched unconditionally - gating it on a flight_state the
                # packet no longer carries silently dropped every term. Boot-edge
                # detection (post-commit verification) is tag 68's job.
                if parsed is not None and hasattr(parsed, 'pf_terms'):
                    self._pf_term_mask = parsed.pf_terms or 0

            case 67:
                packet_logger.log_received(67, "EXEC", parsed)
                self.exec_time = parsed
                # Both reporters are edge-triggered (once per change), so
                # calling them on every tag-67 frame is cheap. _report_imu_fault
                # used to have no call site at all: it was written, documented
                # and initialised, but nothing invoked it, so an IMU integrity
                # fault was parsed and then silently dropped.
                self._report_imu_fault(parsed)
                self._report_boot_diag(parsed)
                # tag-67 carries the same Config.BootDiag as the one-shot
                # tag-68, but streams continuously, so its edge is the reliable
                # post-commit boot trigger when tag-68 is lost across a USB
                # re-enumeration. Armed only by on_connected(True), i.e. by the
                # reconnection after the reboot; see _verify_post_commit_config.
                if self._armed_boot_verify:
                    self._verify_post_commit_config(parsed)

            case 64:
                packet_logger.log_received(64, "WIND", parsed)
                if isinstance(parsed, WindData):
                    self.wind_data = parsed

            case 68:
                packet_logger.log_received(68, "REBOOT", parsed)
                # Only meaningful as the boot edge after our own commit; see
                # _verify_post_commit_config().
                self._verify_post_commit_config(parsed)

            case 69:
                packet_logger.log_received(69, "LINK", parsed)
                if isinstance(parsed, LinkStatsData):
                    self.link_stats = parsed
                    self.update_link_stats_display()

            case 76:
                packet_logger.log_received(76, "I2CERR", parsed)
                if isinstance(parsed, I2CErrorData):
                    self.i2c_errors = parsed
                    if self.anomaly_box is not None:
                        self.anomaly_box.update_sio_counts(parsed)

            case 74:
                packet_logger.log_received(74, "TESTRSP", parsed)
                if hasattr(parsed, 'phase'):
                    if getattr(parsed, 'test_id', 0) == 0xff:
                        self.handle_anomaly(parsed)
                    elif parsed.phase == 255:
                        # Test finished (done marker). Auto-restart regardless
                        # of the Misc window's visibility so unattended loops
                        # (e.g. the I2C fault hammer) keep iteration even when
                        # that panel is closed/hidden. The Misc window's own
                        # re-arm was removed to avoid a double send.
                        self.log_debug(
                            f"🧪 test {parsed.test_id} phase {parsed.phase} "
                            f"d0={parsed.data[0] if parsed.data else '?'}",
                            "Info")
                        if (self.misc_window is not None
                                and self.misc_window.free_run_cb.isChecked()):
                            self.send_test(parsed.test_id, 0)
                            self.log_debug(
                                f"♻ auto-restarted test {parsed.test_id}",
                                "Info")
                    else:
                        self.log_debug(
                            f"🧪 test {parsed.test_id} phase {parsed.phase} "
                            f"d0={parsed.data[0] if parsed.data else '?'}",
                            "Info")

            case 57:
                packet_logger.log_received(57, "TUNE", parsed)
                self._last_tune = parsed
                if self._identify_window is not None:
                    self._identify_window._refresh(parsed)

            case 54:
                packet_logger.log_received(54, "BB", parsed)
                if isinstance(parsed, (bytes, bytearray)) and len(parsed) >= 4:
                    seq_no = struct.unpack('<i', parsed[:4])[0]
                    chunk = parsed[4:]
                    self._bb_chunks[seq_no] = chunk
                    total = len(self._bb_chunks) * 128
                    self.log_debug(f"📦 BB chunk {seq_no} ({len(chunk)}B, ~{total}B total)", "Info")
                    self._bb_dump_timer.start(500)
                    if len(chunk) < 128:
                        self._finalize_bb_dump()

            case 17:
                packet_logger.log_received(17, "PARAM", parsed)
                self._handle_param_packet(parsed)
                # Same tag as the bulk readback's sibling path but a 5-byte
                # single-param echo - this is the only place the sender can
                # safely learn that the FC received a value.
                self._on_param_write_echo(parsed)

            case 71:
                packet_logger.log_received(71, "PARAM_TAGGED", parsed)
                self._handle_param_packet(parsed)

            case _:
                packet_logger.log_received(tag, f"TAG{tag}", parsed)

        MISC_TAGS = {15, 21, 50, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 64, 65, 67, 68, 74, 76}
        if tag in MISC_TAGS and self.misc_window and self.misc_window.isVisible():
            body = data[3:3+data[2]] if len(data) > 3 and data[0] == 0x01 else data
            self.misc_window.add_packet(tag, parsed if parsed is not None else body, "R")
    
    def update_config_colors(self):
        """Update config bit display based on current parameters"""
        config1 = self.config1_cache
        config2 = self.config2_cache
        
        if self.param_window:
            if hasattr(self.param_window, '_config_values'):
                cv = self.param_window._config_values
                if int(ParamIndex.CONFIG1_BITS) in cv:
                    self.config1_cache = cv.get(int(ParamIndex.CONFIG1_BITS), self.config1_cache)
                if int(ParamIndex.CONFIG2_BITS) in cv:
                    self.config2_cache = cv.get(int(ParamIndex.CONFIG2_BITS), self.config2_cache)
            elif hasattr(self.param_window, 'params'):
                if ParamIndex.CONFIG1_BITS in self.param_window.params:
                    widget = self.param_window.params[ParamIndex.CONFIG1_BITS]
                    if isinstance(widget, QDoubleSpinBox):
                        self.config1_cache = int(widget.value())
                if ParamIndex.CONFIG2_BITS in self.param_window.params:
                    widget = self.param_window.params[ParamIndex.CONFIG2_BITS]
                    if isinstance(widget, QDoubleSpinBox):
                        self.config2_cache = int(widget.value())
        
        config1 = self.config1_cache
        config2 = self.config2_cache
        
        for i, (which, bit, name) in enumerate(self.config_flags):
            if name == "Emulation":
                continue
            val = config1 if which == 1 else config2
            is_set = (val & (1 << bit)) != 0
            if is_set:
                self.config_labels[i].setStyleSheet("""
                    background-color: #27ae60;
                    color: white;
                    font-weight: bold;
                    border: 1px solid #1e8449;
                    border-radius: 3px;
                    padding: 3px 6px;
                """)
            else:
                self.config_labels[i].setStyleSheet("""
                    background-color: #ddd;
                    color: #666;
                    font-weight: normal;
                    border: 1px solid #999;
                    border-radius: 3px;
                    padding: 3px 6px;
                """)
    
    def update_flag_colors(self, flag_bits):
        if not flag_bits:
            return

        crsf_active = self.link_stats is not None
        crsf_keys = {'lq', 'rssi', 'fs', 'snr'}

        if self.param_window is not None and len(flag_bits) > 34:
            # bit 34 = cfg_pend: the FC's own ConfigChanged, set by a captured
            # calibration (acc/mag/gyro) that no local widget edit reflects.
            self.param_window.on_fc_config_uncommitted(bool(flag_bits[34]))

        if crsf_active:
            lq = self.link_stats.uplink_lq
            snr = self.link_stats.uplink_snr
            rssi = self.link_stats.uplink_rssi
            flag_bits[53] = lq < 80       # LQ (warning)
            flag_bits[58] = lq < 50       # FS (failsafe territory)
            flag_bits[75] = True          # RSSI (always shown, colored by value)
            flag_bits[77] = True          # SNR (always shown, colored by value)

        for flag_key, flag_data in self.flag_labels.items():
            bit = flag_data['bit']
            widget = flag_data['widget']

            if flag_key in crsf_keys and not crsf_active:
                widget.setStyleSheet("""
                    background-color: #ddd;
                    color: #999;
                    font-weight: normal;
                    border: 1px solid #bbb;
                    border-radius: 2px;
                    padding: 2px;
                """)
                continue

            is_active = bit < len(flag_bits) and flag_bits[bit]
            
            # 3-state CRSF flags: always show when CRSF active, color by value
            if flag_key in ('lq', 'rssi', 'snr') and crsf_active:
                if flag_key == 'lq':
                    val = self.link_stats.uplink_lq
                    if val >= 80:
                        color, text_color, font_weight = "#27ae60", "white", "bold"
                    elif val >= 50:
                        color, text_color, font_weight = "#f39c12", "white", "bold"
                    else:
                        color, text_color, font_weight = "#e74c3c", "white", "bold"
                elif flag_key == 'rssi':
                    val = self.link_stats.uplink_rssi
                    if val >= -90:
                        color, text_color, font_weight = "#27ae60", "white", "bold"
                    elif val >= -100:
                        color, text_color, font_weight = "#f39c12", "white", "bold"
                    else:
                        color, text_color, font_weight = "#e74c3c", "white", "bold"
                else:  # snr
                    val = self.link_stats.uplink_snr
                    if val >= 6:
                        color, text_color, font_weight = "#27ae60", "white", "bold"
                    elif val >= 0:
                        color, text_color, font_weight = "#f39c12", "white", "bold"
                    else:
                        color, text_color, font_weight = "#e74c3c", "white", "bold"
            elif is_active:
                if flag_key in ['level', 'att_hold']:
                    color = "#3cb371"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['low_batt', 'land_sw', 'mag_fail', 'baro_fail', 'loss', 'fs']:
                    color = "#e74c3c"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['vrs', 'rc_map_fail', 'fence_alarm', 'excess_lift', 'snr', 'ext_mag', 'imu_fault_lat', 'cfg_pend']:
                    color = "#f39c12"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['armed', 'armed2']:
                    color = "#e74c3c"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['gps_ok', 'origin', 'gps_alt', 'alt_hold', 'hold_alt', 'signal']:
                    color = "#27ae60"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['rth', 'nav', 'wp_nav']:
                    color = "#27ae60"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['baro', 'imu', 'mag']:
                    color = "#27ae60"
                    text_color = "white"
                    font_weight = "bold"
                else:
                    color = "#27ae60"
                    text_color = "white"
                    font_weight = "bold"
            else:
                if flag_key in ['gps_ok', 'baro', 'imu', 'mag']:
                    color = "#e74c3c"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['acc_cal', 'mag_cal']:
                    color = "#f39c12"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['baro_fail', 'mag_fail', 'imu_fault_lat']:
                    color = "#27ae60"
                    text_color = "white"
                    font_weight = "bold"
                else:
                    color = "#ddd"
                    text_color = "#666"
                    font_weight = "normal"
            
            widget.setStyleSheet(f"""
                background-color: {color};
                color: {text_color};
                font-weight: {font_weight};
                border: 1px solid #999;
                border-radius: 2px;
                padding: 2px;
            """)
    
    def _poll_tuning(self):
        """Periodic tag-57 request (~1 Hz) so the Identify window stays fresh."""
        if not self._identify_window or not self._identify_window.isVisible():
            return
        if not self.telemetry or not getattr(self.telemetry, 'serial', None):
            return
        self.send_request(PacketTag.TUNING, 0, 0, None)

    def update_ui(self):
        if not self.flight_data:
            return
        
        f = self.flight_data
        
        pitch = f.angle_pitch * 57.2958
        roll = f.angle_roll * 57.2958
        heading = f.heading * 57.2958
        self.attitude.set_attitude(pitch, roll, heading)
        
        if self.exec_time:
            self.exec_bar.set_exec_time(
                self.exec_time.exec_us,
                self.exec_time.exec_peak_us
            )
            if self.anomaly_box is not None:
                self.anomaly_box.update_exec(
                    self.exec_time.imu_rejects,
                    self.exec_time.exec_us,
                    self.exec_time.exec_peak_us
                )
        
        self.angle_roll.setText(f"{roll:.1f}")
        self.angle_pitch.setText(f"{pitch:.1f}")
        self.angle_yaw.setText(f"{heading:.1f}")
        
        self.altitude_label.setText(f"{f.altitude:.1f}")
        self.alt_source_label.setText("Altitude")
        
        # Update ROC label under artificial horizon, turn orange when descending > 3 m/s
        roc_val = f.roc
        self.roc_label.setText(f"{roc_val:.1f}")
        if roc_val < -3.0:
            self.roc_label.setStyleSheet("""
                font-size: 28px;
                font-weight: bold;
                color: #ff8800;
                background-color: black;
                padding: 8px;
                border: 2px solid #ff8800;
                border-radius: 4px;
                min-height: 50px;
                min-width: 100px;
            """)
        else:
            self.roc_label.setStyleSheet("""
                font-size: 28px;
                font-weight: bold;
                color: #00ff00;
                background-color: black;
                padding: 8px;
                border: 2px solid #444;
                border-radius: 4px;
                min-height: 50px;
                min-width: 100px;
            """)
        
        if hasattr(f, 'flag_bits') and f.flag_bits:
            self.current_flag_bits = f.flag_bits
            self.update_flag_colors(f.flag_bits)
        elif self.current_flag_bits:
            self.update_flag_colors(self.current_flag_bits)
        
        self.batt_volts.setText(f"{f.battery_volts:.2f} V")
        self.batt_current.setText(f"{f.battery_current:.1f} A")
        self.batt_charge.setText(f"{f.battery_charge:.0f} mAh")
        # Remaining flight time from FC telemetry (seconds)
        rem_sec = getattr(f, 'battery_time_remaining_sec', 0)
        if rem_sec > 0:
            self.batt_remain.setText(f"{rem_sec//60}:{rem_sec%60:02d}")
        else:
            self.batt_remain.setText("--:--")
        
        self.gyro_roll.setText(f"{f.rate_roll * 57.2958:.1f}")
        self.gyro_pitch.setText(f"{f.rate_pitch * 57.2958:.1f}")
        self.gyro_yaw.setText(f"{f.rate_yaw * 57.2958:.1f}")
        
        self.acc_lr.setText(f"{f.acc_lr:.2f}")
        self.acc_fb.setText(f"{f.acc_fb:.2f}")
        self.acc_du.setText(f"{f.acc_du:.2f}")
        
        self.mag_heading.setText(f"{f.mag_heading * 57.2958:.1f}°")
        self.acc_confidence.setText(f"{f.acc_confidence * 100:.0f}%")
        if self._mag_live and len(self._mag_live) == 3:
            mx, my, mz = self._mag_live
            norm = math.sqrt(mx * mx + my * my + mz * mz)
            if norm > 0:
                self.mag_x.setText(f"{mx / norm:.2f}")
                self.mag_y.setText(f"{my / norm:.2f}")
                self.mag_z.setText(f"{mz / norm:.2f}")

        # Newtonian slew-window peaks (tag 67): gyro rad/s -> deg/s,
        # accel arrives as m/s^2 - displayed as-is.
        if self.exec_time:
            r, p, y = self.exec_time.rate_slew_max_rs
            self.slew_gyro_roll.setText(f"{r * 57.2958:.1f}")
            self.slew_gyro_pitch.setText(f"{p * 57.2958:.1f}")
            self.slew_gyro_yaw.setText(f"{y * 57.2958:.1f}")
            r, p, y = self.exec_time.acc_slew_max_mps2
            self.slew_acc_roll.setText(f"{r:.2f}")
            self.slew_acc_pitch.setText(f"{p:.2f}")
            self.slew_acc_yaw.setText(f"{y:.2f}")

        self.alt_kf.setText(f"{f.baro_altitude:.2f}")
        self.alt_roc.setText(f"{f.roc:.1f}")
        self.alt_baro_var.setText(f"{f.baro_variance:.3f}")
        self.alt_baro_temp.setText(f"{f.baro_temp:.1f}°C")
        self.alt_baro_press.setText(f"{f.baro_pressure:.1f} hPa")

        self.alt_accu_var.setText(f"{f.accu_variance:.3f}")
        self.alt_accu_bias_var.setText(f"{f.accu_bias_variance:.6f}")
        if hasattr(f, 'flight_state'):
            state_name = FlightState.get_name(f.flight_state)
            pf_mask = self._pf_term_mask
            pf_blocked = (pf_mask
                          and f.flight_state in (FlightState.ePreflight,
                                                 FlightState.eLivePreflight))
            if pf_blocked:
                self.pf_block_label.setText("Blocking preflight: %s"
                                            % ", ".join(pf_term_labels(pf_mask)))
                self.pf_block_label.setVisible(True)
                state_name = "%s - BLOCKED" % state_name
            else:
                self.pf_block_label.setVisible(False)
            self.flight_state_label.setText(state_name)
            state_colors = {
                FlightState.eStarting: "#add8e6",
                FlightState.eWarmup: "#00ff00",
                FlightState.eLanding: "#008000",
                FlightState.eLanded: "#bc8f8f",
                FlightState.eShutdown: "#ffa500",
                FlightState.eInFlight: "#c0c0c0",
                FlightState.eIREmulateUNUSED: "#bc8f8f",
                FlightState.ePreflight: "#ff0000",
                FlightState.eReady: "#ffd700",
                FlightState.eThrottleOpenCheck: "#ff0000",
                FlightState.eErectingGyros: "#ff0000",
                FlightState.eMonitorInstruments: "#c0c0c0",
                FlightState.eInitialisingGPS: "#87ceeb",
                FlightState.eStepSerial: "#87ceeb",
                FlightState.eStepTarget: "#87ceeb",
                FlightState.eStepDrives: "#87ceeb",
                FlightState.eStepRC: "#87ceeb",
                FlightState.eStepBuses: "#87ceeb",
                FlightState.eStepAnalog: "#87ceeb",
                FlightState.eStepOLED: "#87ceeb",
                FlightState.eStepBattery: "#87ceeb",
                FlightState.eStepLEDs: "#87ceeb",
                FlightState.eStepIMU: "#87ceeb",
                FlightState.eStepMag: "#87ceeb",
                FlightState.eStepMadgwick: "#87ceeb",
                FlightState.eStepAlt: "#87ceeb",
                FlightState.eStepNVMem: "#87ceeb",
                FlightState.eStepTemp: "#87ceeb",
                FlightState.eStepCtrl: "#87ceeb",
                FlightState.eStepNav: "#87ceeb",
                FlightState.eStepEmu: "#87ceeb",
                FlightState.eStepSPI: "#87ceeb",
                FlightState.eStepInertFlt: "#87ceeb",
                FlightState.eStepIMUDrv: "#87ceeb",
                FlightState.eStepBaro: "#87ceeb",
                FlightState.eStepBaroDrv: "#87ceeb",
                FlightState.eStepGPS: "#87ceeb",
                FlightState.eStepIdentify: "#87ceeb",
                FlightState.eLivePreflight: "#2e8b57",
                FlightState.eLiveReady: "#2e8b57",
                FlightState.eLiveStarting: "#2e8b57",
                FlightState.eLiveErectingGyros: "#2e8b57",
                FlightState.eLiveWarmup: "#2e8b57",
                FlightState.eLiveMonitorInstruments: "#2e8b57",
                FlightState.eLiveInitialisingGPS: "#2e8b57",
                FlightState.eLiveThrottleOpenCheck: "#2e8b57",
                FlightState.eLiveLanded: "#2e8b57",
                FlightState.eLiveLanding: "#2e8b57",
                FlightState.eLiveShutdown: "#2e8b57",
            }
            color = state_colors.get(f.flight_state, "#ffffff")
            self.flight_state_label.setStyleSheet(f"font-weight: bold; background-color: {color}; padding: 2px 4px; border-radius: 2px;")
        
        if hasattr(f, 'gps_fix') and hasattr(f, 'gps_sats'):
            self.gps_sats.setText(str(f.gps_sats))
            self.gps_fix.setText(str(f.gps_fix))
            self.gps_lat.setText(f"{f.gps_lat:.6f}")
            self.gps_lon.setText(f"{f.gps_lon:.6f}")
            if hasattr(f, 'flag_bits') and len(f.flag_bits) > 6:
                gps_valid = f.flag_bits[6]
            else:
                gps_valid = (f.gps_fix >= 3 and f.gps_sats >= 6)
            gps_alt_display = (f.gps_fix >= 3)
            self.gps_alt.setText(f"{f.gps_altitude:.1f}" if gps_alt_display else "---")
            self.gps_vel.setText(f"{f.gps_vel:.1f}")
            self.gps_hacc.setText(f"{f.gps_hacc:.1f}")
            self.gps_vacc.setText(f"{f.gps_vacc:.1f}")
            self.gps_sacc.setText(f"{f.gps_sacc:.1f}")
            self.gps_cacc.setText(f"{f.gps_cacc:.1f}")
            color = "#27ae60" if gps_valid else "#e74c3c"
            self.gps_lat.setStyleSheet(f"color: {color}; font-weight: bold;")
            self.gps_lon.setStyleSheet(f"color: {color}; font-weight: bold;")
            self.gps_alt.setStyleSheet(f"color: {color};")
            self.gps_vel.setStyleSheet(f"color: {color};")
            self.gps_hacc.setStyleSheet(f"color: {color};")
            self.gps_vacc.setStyleSheet(f"color: {color};")
            self.gps_sacc.setStyleSheet(f"color: {color};")
            self.gps_cacc.setStyleSheet(f"color: {color};")
        
        if hasattr(f, 'gps_update_rate') and f.gps_update_rate > 0:
            self.gps_rate_label.setText(f"{f.gps_update_rate:.0f} Hz")
        elif hasattr(f, 'gps_update_rate'):
            self.gps_rate_label.setText("---")
        
        if hasattr(f, 'nav_state'):
            nav_state_name = NavState.get_name(f.nav_state)
            self.nav_state_label.setText(nav_state_name)
            nav_colors = {
                0: "#add8e6",
                1: "#00ff00",
                2: "#008000",
                3: "#ffa500",
                4: "#bc8f8f",
                5: "#c0c0c0",
                6: "#ffd700",
                7: "#ffd700",
                8: "#bc8f8f",
                9: "#ff0000",
                10: "#ffa500",
                11: "#ffa500",
                12: "#ffd700",
                13: "#ffd700",
                14: "#ffd700",
                15: "#ffd700",
                16: "#ffd700",
                17: "#ffd700",
                18: "#ff0000",
                19: "#ff0000",
            }
            color = nav_colors.get(f.nav_state, "#ffffff")
            self.nav_state_label.setStyleSheet(f"font-weight: bold; background-color: {color}; padding: 2px 4px; border-radius: 2px;")
        
        alarm = getattr(f, 'alarm_state', 0)
        alarm_names = {0: "None", 1: "Monitor", 2: "Low Bat", 3: "Lost Sig", 4: "Hit Fence", 5: "Upside Down", 6: "Forced Land", 7: "GPS Passthru", 8: "Arm Timeout", 9: "Close Thr"}
        alarm_name = alarm_names.get(alarm, f"Alarm {alarm}")
        self.alarm_state_label.setText(alarm_name)
        if alarm >= 2:
            self.alarm_state_label.setStyleSheet("font-weight: bold; color: #ffffff; background-color: #e74c3c; padding: 2px 4px; border-radius: 2px;")
        else:
            self.alarm_state_label.setStyleSheet("font-weight: bold;")
        self.curr_wp_label.setText(str(f.curr_wp))
        self.nav_curr_wp.setText(str(f.curr_wp))
        
        if self.guidance_data:
            g = self.guidance_data
            self.guidance_distance.setText(f"{g.distance:.0f}")
            self.guidance_bearing.setText(f"{g.bearing:.0f}")
            self.guidance_elevation.setText(f"{g.elevation:.0f}")
            self.guidance_hint.setText(f"{g.hint:.0f}")
        
        has_wp = getattr(f, 'curr_wp', 0) > 0
        if self.nav_data:
            n = self.nav_data
            self.wp_bearing.setText(f"{n.wp_bearing * 57.2958:.0f}")
            # Distance to waypoint from guidance packet, fallback to north/east pos
            dist = getattr(f, 'distance_to_wp', 0.0)
            if dist <= 0 and has_wp and hasattr(n, 'north_pos_e'):
                dist = (n.north_pos_e**2 + n.east_pos_e**2)**0.5
            if has_wp and dist > 0:
                self.distance_to_wp.setText(f"{dist:.1f}")
            else:
                self.distance_to_wp.setText("---")
            # Cross-track error
            cte = n.cross_track_error
            if has_wp and 0 <= cte <= 10000:
                self.cross_track.setText(f"{cte:.1f}")
            else:
                self.cross_track.setText("---")
        
        if self.wind_data:
            w = self.wind_data
            self.wind_speed.setText(f"{w.speed:.1f}")
            dir_deg = w.direction * 57.2958
            if dir_deg < 0:
                dir_deg += 360
            self.wind_dir.setText(f"{dir_deg:.0f}°")
            # Match FC WindEstValid gate (wind.c WIND_CONFIDENCE_VALID = 0.5):
            # green when usable, orange when not yet confident.
            valid = w.confidence >= 0.5 and w.speed > 0.3
            color = "#22cc22" if valid else "#ff8800"
            for lbl in (self.wind_speed, self.wind_dir):
                lbl.setStyleSheet(f"font-weight: bold; color: {color};")
        else:
            for lbl in (self.wind_speed, self.wind_dir):
                lbl.setStyleSheet("font-weight: bold;")

        if hasattr(f, 'mag_var_wmm'):
            self.mag_var_wmm.setText(f"{f.mag_var_wmm * 57.2958:.1f}°")

        self.update_config_colors()
        
        # Always show altitude box (emulation provides fake but usable data)
        self.update_link_stats_display()
        self.update_motors_display(f)
        self.update_controls_display(f)
    
    def update_link_stats_display(self):
        if not self.link_stats:
            return
        s = self.link_stats

        bold = "font-weight: bold; color:"
        self.link_lq.setText(f"{s.uplink_lq}%")
        lq_c = "#22cc22" if s.uplink_lq >= 80 else "#ff8800" if s.uplink_lq >= 50 else "#e74c3c"
        self.link_lq.setStyleSheet(f"{bold} {lq_c};")
        self.link_rssi.setText(f"{s.uplink_rssi}")
        rssi_c = "#22cc22" if s.uplink_rssi >= -90 else "#ff8800" if s.uplink_rssi >= -100 else "#e74c3c"
        self.link_rssi.setStyleSheet(f"{bold} {rssi_c};")
        self.link_snr.setText(f"{s.uplink_snr}")
        snr_c = "#22cc22" if s.uplink_snr >= 6 else "#ff8800" if s.uplink_snr >= 0 else "#e74c3c"
        self.link_snr.setStyleSheet(f"{bold} {snr_c};")
        self.link_losses.setText(f"{s.rc_signal_losses}")
        self.link_failsafes.setText(f"{s.rc_failsafes}")
        fs_c = "#22cc22" if s.rc_failsafes == 0 else "#e74c3c"
        self.link_failsafes.setStyleSheet(f"{bold} {fs_c};")

    def update_motors_display(self, f):
        throttle = getattr(f, 'desired_throttle', 0.0)
        pwm = getattr(f, 'pwm', None)
        if pwm is not None:
            for i in range(min(len(self.motor_bars), len(pwm))):
                # RawPW idle-zero-referenced: 0=1000uS, 0.5=1500uS,
                # 1.0=2000uS -> bar 0..1000 = percent above idle
                us = (pwm[i] + 1.0) * 1000.0
                # Guard NaN (graceful degradation) - never crash the whole GCS
                # on a broken telemetry float.
                raw = int(us - 1000) if math.isfinite(us) else 0
                raw = max(0, min(1000, raw))
                self.motor_bars[i].setValue(raw)
        elif throttle > 0.01:
            val = int(throttle * 1000) if math.isfinite(throttle) else 0
            val = max(0, min(1000, val))
            for bar in self.motor_bars:
                bar.setValue(val)
        else:
            for bar in self.motor_bars:
                bar.setValue(0)

    def update_controls_display(self, f):
        if hasattr(f, 'rc_channels') and len(f.rc_channels) >= 4:
            for i in range(4):
                # rc_channels are integer uS (idle=1000, neutral=1500)
                us = f.rc_channels[i]
                self.control_bars[i].setValue(max(0, min(1000, int(us - 1000))))
        else:
            for bar in self.control_bars:
                bar.setValue(0)

    def update_serial_ports_display(self):
        if not self.serial_ports:
            return
        buf_size = self.serial_ports.get('buffer_size', 512)
        def _set(port_key, tx_bar, rx_bar, ov_lbl):
            p = self.serial_ports.get(port_key)
            if not p:
                return
            tx_bar.setRange(0, buf_size)
            tx_bar.setValue(p['tx_q'])
            tx_bar.setFormat(f"{p['tx_q']}")
            rx_bar.setRange(0, buf_size)
            rx_bar.setValue(p['rx_q'])
            rx_bar.setFormat(f"{p['rx_q']}")
            ov = []
            if p['tx_overflow']: ov.append("TX")
            if p['rx_overflow']: ov.append("RX")
            if ov:
                ov_lbl.setText("/".join(ov))
                ov_lbl.setStyleSheet("font-weight: bold; color: #e74c3c;")
            else:
                ov_lbl.setText("--")
                ov_lbl.setStyleSheet("font-weight: bold;")
        _set('telemetry', self.ser_telem_tx, self.ser_telem_rx, self.ser_telem_ov)
        _set('gps', self.ser_gps_tx, self.ser_gps_rx, self.ser_gps_ov)

    def check_connection(self):
        now = time.monotonic()
        if hasattr(self, '_last_rx_time') and self._last_rx_time > 0:
            elapsed = now - self._last_rx_time
            if elapsed > 3.0 and self.connected:
                self.status_msg.setText(f"⏳ No FC data for {elapsed:.0f}s")
                self.status_msg.setStyleSheet("color: #e74c3c; font-weight: bold;")
        if self.telemetry and not self.telemetry.isRunning() and self.connected:
            self.disconnect()
    
    def closeEvent(self, event):
        self.save_settings()
        self.anomaly_logger.close()
        self.raw_logger.stop()
        
        if self.param_window:
            self.param_window.close()
        if self.nav_window:
            self.nav_window.close()
        if self.calib_window:
            self.calib_window.close()
        if self.misc_window:
            self.misc_window.close()
        if self.replay_window:
            self.replay_window.close()
        
        if self.telemetry:
            self.telemetry.stop()
            self.telemetry.wait()
        event.accept()
