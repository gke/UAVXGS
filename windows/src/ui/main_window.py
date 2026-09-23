# ui/main_window.py - Fixed with continuous polling and all packet types
"""
UAVX Groundstation - Main Window
"""

import sys
import os
import struct
import time
import math
from datetime import datetime
from typing import Optional

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol_constants import *
from protocol_enums import PacketTag, ParamIndex, FlightState, NavState, Config1Bits, Config2Bits, MiscCommand
from packet_parser import *
from widgets.attitude_indicator import AttitudeIndicator, ExecTimeBar
from widgets.anomaly_box import AnomalyBox

from core.data_manager import data_manager
from core.packet_logger import packet_logger
from core.ack_handler import ack_handler

from ui.parameter_window import ParameterWindow
from parameters import PARAM_DISPLAY_MULT, PARAM_TYPES
from ui.nav_window import NavWindow
from ui.calibration_window import CalibrationWindow
from ui.misc_window import MiscWindow
from ui.dfu_flasher import DfuFlasherWindow
from core.speech import SpeechController, SpeechLevel, LEVEL_LABELS
from logger.logger import AnomalyLogger, ImuStatsLogger
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
    34: ('unused_param', 'unusedParam', 'Reserved: F.ParametersValid never set by FC'),
    35: ('signal', 'Signal', 'Signal'),
    36: ('wp_nav', 'WPNav', 'WP Navigation'),
    37: ('imu', 'IMU', 'IMU Active'),
    38: ('mag', 'Mag', 'Magnetometer Active'),
    39: ('armed2', 'Drives', 'Drives Active'),
    40: ('thr_open', 'ThrOpen', 'Throttle Open'),
    41: ('mag_cal', 'MagCal', 'Mag Calibrated'),
    42: ('rc_map_fail', 'RCMap', 'RC Map Fail'),
    43: ('new_alt', 'NewAlt', 'New Altitude Value'),
    44: ('gyro_cal', 'IMUCal', 'IMU Calibrated'),
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
    ('System / Debug', [33, 67, 49, 80]),
]

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
        self._last_reported_wdt_mark = -1
        self._last_reported_reset_cause = -1
        self._last_reported_imu_fault_code = -1
        self._last_reported_imu_fault_recoveries = -1
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
        self.nav_window: Optional[NavWindow] = None
        self.calib_window: Optional[CalibrationWindow] = None
        self.misc_window: Optional[MiscWindow] = None
        self.dfu_flasher_window: Optional[DfuFlasherWindow] = None
        self._trace_viewer = None
        self._identify_window: Optional[IdentifyWindow] = None
        
        self.speech = SpeechController()
        self.anomaly_logger = AnomalyLogger()
        self.imu_stats_logger = ImuStatsLogger()
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
        self._param_write_list = []
        self._param_write_port = None
        self._param_write_on_complete = None
        self._param_write_progress = None
        self._param_write_total = 0
        self._param_write_index = 0
        self._revision_from_afname = False
        self._pending_flash_airframe_name = None
        self._last_tune = None
        self._last_rx_time = 0.0
        self._last_init_state = -1
        self._last_init_state_change = 0.0
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
        log_folder_action = QAction("Set Log &KML Folder…", self)
        log_folder_action.triggered.connect(self.select_log_folder)
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

        imu_layout.addWidget(QLabel("IMU Temp:"), 5, 0)
        self.mpu_temp = QLabel("0.0°C")
        self.mpu_temp.setStyleSheet("font-weight: bold;")
        self.mpu_temp.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(self.mpu_temp, 5, 1, 1, 2)

        imu_layout.addWidget(QLabel("Acc Conf:"), 5, 3)
        self.acc_confidence = QLabel("0")
        self.acc_confidence.setStyleSheet("font-weight: bold;")
        self.acc_confidence.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        imu_layout.addWidget(self.acc_confidence, 5, 4)

        imu_box.setLayout(imu_layout)
        # Pin the IMU box width once at natural size +20% so numeric label
        # growth (e.g. "0.0" -> "-0.00") never reflows/resizes the box.
        imu_box.setFixedWidth(int(imu_box.sizeHint().width() * 1.2))

        # Anomalies box to the right of IMU: IMU/WDT health + per-device
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
        self.ser_soft_tx = QProgressBar(); self.ser_soft_tx.setStyleSheet(bar_style); self.ser_soft_tx.setRange(0, 512); self.ser_soft_tx.setValue(0); self.ser_soft_tx.setTextVisible(True)
        self.ser_soft_rx = QProgressBar(); self.ser_soft_rx.setStyleSheet(bar_style); self.ser_soft_rx.setRange(0, 512); self.ser_soft_rx.setValue(0); self.ser_soft_rx.setTextVisible(True)
        self.ser_soft_ov = QLabel("--")
        for ov in (self.ser_telem_ov, self.ser_gps_ov, self.ser_soft_ov):
            ov.setStyleSheet("font-weight: bold;")
        serial_layout.addWidget(QLabel("Telem:"), 1, 0)
        serial_layout.addWidget(self.ser_telem_tx, 1, 1)
        serial_layout.addWidget(self.ser_telem_rx, 1, 2)
        serial_layout.addWidget(self.ser_telem_ov, 1, 3)
        serial_layout.addWidget(QLabel("GPS:"), 2, 0)
        serial_layout.addWidget(self.ser_gps_tx, 2, 1)
        serial_layout.addWidget(self.ser_gps_rx, 2, 2)
        serial_layout.addWidget(self.ser_gps_ov, 2, 3)
        serial_layout.addWidget(QLabel("Soft:"), 3, 0)
        serial_layout.addWidget(self.ser_soft_tx, 3, 1)
        serial_layout.addWidget(self.ser_soft_rx, 3, 2)
        serial_layout.addWidget(self.ser_soft_ov, 3, 3)
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
            (2, 0, "Batt Comp"), (2, 1, "Fast Start"), (2, 3, "Have GPS"), (2, 4, "Prop In"),
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
        
        self.statusBar().showMessage("Ready")
    
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
            label.setStyleSheet("""
                background-color: #ddd;
                color: #666;
                font-weight: normal;
                border: 1px solid #999;
                border-radius: 3px;
                padding: 3px 6px;
            """)
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
        d = os.path.expanduser("~/UAVX")
        os.makedirs(d, exist_ok=True)
        return d

    # WDT trip codes must match the emum in UAVXArmQ/src/wdt.h
    _WDT_TRIP_NAMES = {
        1: "boot", 2: "init", 10: "loop-top", 11: "poll-diag",
        12: "inertial", 13: "control", 14: "housekeeping", 15: "nav-state",
        16: "state", 30: "i2c-er-start-wait", 31: "i2c-er-stop-wait",
        32: "i2c-ev-start-wait", 33: "i2c-rd-stop-wait", 34: "i2c-wr-stop-wait",
        40: "serial-tx-wait", 41: "flash-wait",
    }

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

    def _report_reset_cause(self, parsed):
        """Report the FC's persisted last-reset cause once per change. This is
        the real discriminator when a self-reboot produced no WDT marker: the
        WDT path only records an IWDG trip, while param 127 survives any
        meaningful reset (brownout, external pin, window-watchdog, ...)."""
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

    def _report_wdt_trip(self, parsed):
        """Report a watchdog trip once per change: the exec bar shows only
        'WDT <num>' but the trip must survive to the terminal log for later
        correlation against the flight trace."""
        mark = getattr(parsed, "wdt_mark", 0)
        if mark == self._last_reported_wdt_mark:
            return
        self._last_reported_wdt_mark = mark
        if mark:
            name = self._WDT_TRIP_NAMES.get(mark, "unknown")
            print(f"[WDT] watchdog trip MARK={mark} ({name}) on last reset")
            self.log_debug(f"Watchdog trip marker {mark} ({name}) on last reset", "Warnings")
        else:
            print("[WDT] last boot was NOT a watchdog trip")
            self.log_debug("No watchdog trip on last boot", "Info")

    def _report_imu_fault(self, parsed):
        """Report an attitude-integrity (NaN/Inf) recovery from the FC once per
        change. The FC latches F.IMUFaultLatched (flag idx 47) and streams the
        fault code + cumulative recovery count via tag 67. A recovered value
        means the FC discarded the bad sample and keeps flying, but the
        incident must be called out loudly on the ground station."""
        code = getattr(parsed, "imu_fault_code", 0)
        rec = getattr(parsed, "imu_fault_recoveries", 0)
        if code == self._last_reported_imu_fault_code \
                and rec == self._last_reported_imu_fault_recoveries:
            return
        self._last_reported_imu_fault_code = code
        self._last_reported_imu_fault_recoveries = rec
        if code:
            name = self._IMU_FAULT_NAMES.get(code, "unknown")
            msg = f"IMU integrity fault code={code} ({name}); recovered {rec}x"
            print(f"[IMU] {msg}")
            self.log_debug(msg, "Errors")
            self._raise_fault_banner(msg)
        else:
            self._hide_fault_banner()

    def _on_raw_bytes(self, data: bytes):
        """Tee raw serial bytes to the always-on raw log (all bytes, binary
        and printable alike)."""
        if self.raw_logger.active:
            self.raw_logger.write(data)

    def select_log_folder(self):
        start = self.log_dir or os.path.expanduser("~/UAVX")
        folder = QFileDialog.getExistingDirectory(self, "Select Log Folder", start)
        if folder:
            self.log_dir = folder
            os.makedirs(self.log_dir, exist_ok=True)
            self.raw_logger.log_dir = folder
            self.save_settings()
            self.log_debug(f"📂 Log folder set → {folder}", "Info")

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

        # Log folder: prompt once on first run, otherwise use saved location
        self.log_dir = settings.value("log_dir", None)
        if not self.log_dir:
            default_dir = os.path.expanduser("~/UAVX")
            folder = QFileDialog.getExistingDirectory(
                self, "Select folder for raw telemetry logs", default_dir)
            if folder:
                self.log_dir = folder
            else:
                self.log_dir = default_dir
            settings.setValue("log_dir", self.log_dir)
        os.makedirs(self.log_dir, exist_ok=True)
        self.raw_logger.log_dir = self.log_dir
        # Verify + confirm the active log location on every startup
        if not os.access(self.log_dir, os.W_OK):
            self.log_debug(f"⚠️ Log folder not writable: {self.log_dir}", "Errors")
        else:
            self.log_debug(f"📂 Log folder → {self.log_dir}", "Info")

    def save_settings(self):
        settings = QSettings("UAVX", "Groundstation")
        settings.setValue("port", self.port_combo.currentText())
        settings.setValue("baud", self.baud_combo.currentText())
        settings.setValue("speech_level", str(self.speech.level.value))
        settings.setValue("log_dir", self.log_dir)
    
    def toggle_connection(self):
        if self.telemetry and self.telemetry.isRunning():
            self.disconnect()
        else:
            self.connect_telemetry()
    
    def connect_telemetry(self):
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
        if self.param_window is not None:
            # Clear the write interlock so a mid-commit disconnect/reconnect
            # doesn't leave the commit path locked forever.
            self.param_window._flash_write_pending = False
            self.param_window._write_in_progress = False
        
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
            self.raw_logger.log_dir = self.log_dir or os.path.expanduser("~/UAVX")
            self.raw_logger.start()
            self.log_debug(f"🖊️ Raw log → {os.path.abspath(self.raw_logger._path)}", "Info")
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet("background-color: green; color: white; font-weight: bold;")
            self.status_label.setText("● Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.statusBar().showMessage("Connected to UAVX")
            self.log_debug("✅ Connected to UAVX", "Info")
            # Skip the connect-time tag-71 when a commit's deferred verification is
            # outstanding — _request_write_verify() (first flight packet) does that
            # single readback once the FC is fully back. Avoids a racing double-read.
            if not self._pending_param_verification:
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
            self.log_debug("❌ Cannot send AFName - not connected", "Errors")
            return
        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            self.log_debug("❌ Serial port not open - cannot send AFName", "Errors")
            return
        if not name:
            self.log_debug("  ℹ️ No airframe name to persist — skipping AFName write", "Info")
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
            serial_port.write(packet)
            self.log_debug(f"📤 Sent AFName: {body.decode('ascii')}", "Info")
        except Exception as e:
            self.log_debug(f"❌ Failed to send AFName: {e}", "Errors")

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
        """Send widget values as individual param packets.
        If indices is provided, only those param indices are sent.
        progress_cb(sent, total) is called after each packet is written.
        """
        if not self.connected or not self.telemetry:
            on_complete and on_complete()
            return
        if not self.param_window:
            on_complete and on_complete()
            return
        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            on_complete and on_complete()
            return

        param_range = indices if indices is not None else range(128)
        params = []
        for i in param_range:
            if i in self.param_window.params:
                widget = self.param_window.params[i]
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
                            f"(index {widget.currentIndex()}) — refusing to write "
                            f"a positional index as the value")
                    fval = float(data)
                else:
                    fval = 0.0
            else:
                fval = 0.0
            params.append((i, fval))

        self._param_write_list = params
        self._param_write_port = serial_port
        self._param_write_on_complete = on_complete
        self._param_write_progress = progress_cb
        self._param_write_total = len(params)
        self._param_write_index = 0
        self._send_next_param()

    def _send_next_param(self):
        if not self._param_write_list:
            self.log_debug("✅ All param packets sent")
            cb = self._param_write_on_complete
            self._param_write_on_complete = None
            self._param_write_progress = None
            self._param_write_port = None
            cb and cb()
            return
        idx, fval = self._param_write_list.pop(0)
        serial_port = self._param_write_port
        if not serial_port or not serial_port.is_open:
            self.log_debug("❌ Serial port closed during param send", "Errors")
            self._param_write_list = []
            cb = self._param_write_on_complete
            self._param_write_on_complete = None
            self._param_write_progress = None
            self._param_write_port = None
            cb and cb()
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
            cb = self._param_write_on_complete
            self._param_write_on_complete = None
            self._param_write_progress = None
            self._param_write_port = None
            cb and cb()
            return
        self._param_write_index += 1
        self.log_debug(f"📤 Sent param {idx} = {fval}")
        pcb = self._param_write_progress
        if pcb:
            pcb(self._param_write_index, self._param_write_total)
        QTimer.singleShot(5, self._send_next_param)

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
        if self._param_write_list:
            self.log_debug(f"⏳ Param write in flight — appending {len(items)} live writes")
            self._param_write_list.extend(items)
            return
        self._param_write_list = list(items)
        self._param_write_port = serial_port
        self._param_write_on_complete = None
        self._param_write_progress = None
        self._param_write_index = 0
        self._param_write_total = len(items)
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
            76: "I2CErrors",
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

                # Trigger deferred param verification after FC reboot from commit
                if self._pending_param_verification and self.param_window:
                    self._pending_param_verification = False
                    self.param_window._request_write_verify()

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

                    if at == PacketTag.PARAM_COMMIT:
                        # The commit ACK reflects whether the config (+ name) is
                        # CONFIRMED in flash (erase + program + read-back verify).
                        # ok=False means the FC did NOT save and will NOT reboot:
                        # surface it loudly and un-arm the deferred verification.
                        if self.param_window is not None:
                            self.param_window.on_param_commit_ack(ok)
                        if ok:
                            self.log_debug("✅ Flash commit CONFIRMED in FC",
                                           "Info")
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
                    imu_id = parsed.get('imu_id')
                    mag_id = parsed.get('mag_id')
                    if imu_id is not None or mag_id is not None:
                        self.log_debug(f"  CALIB IDs: IMU=0x{imu_id:02X} Mag=0x{mag_id:02X}", "Info")

            case 63:
                if isinstance(parsed, dict) and 'revision' in parsed:
                    rev = parsed['revision']
                    packet_logger.log_received(63, "CONFIG", parsed)
                    self.log_debug(f"  FW: {rev}, AFType: {parsed.get('af_type', '?')}, AFName: {parsed.get('airframe_name', '')}", "Info")
                    self.revision_label.setText(rev)
                    self._revision_from_afname = True
                    # Persisted airframe name from FC config flash is the single
                    # source of truth shown by the GCS (no local QSettings name).
                    af_name = parsed.get('airframe_name') or ''
                    if self.raw_logger.active:
                        self.raw_logger.rename(af_name)
                    if self.param_window is not None:
                        self.param_window.set_flash_airframe_name(af_name)
                    else:
                        self._pending_flash_airframe_name = af_name

            case 66:
                packet_logger.log_received(66, "SERIAL", parsed)
                if isinstance(parsed, dict) and 'telemetry' in parsed:
                    self.serial_ports = parsed
                    self.update_serial_ports_display()

            case 67:
                packet_logger.log_received(67, "EXEC", parsed)
                self.exec_time = parsed
                self._report_wdt_trip(parsed)

            case 64:
                packet_logger.log_received(64, "WIND", parsed)
                if isinstance(parsed, WindData):
                    self.wind_data = parsed

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
                    elif getattr(parsed, 'test_id', 0) == 2:
                        # IMU Stats benchmark: log every burst packet live.
                        self.imu_stats_logger.log(
                            parsed.phase, parsed.data or [0] * 16)
                        self.log_debug(
                            f"🧪 IMU stats t{parsed.test_id} "
                            f"p{parsed.phase} run="
                            f"{(parsed.data or [0]*16)[1]}",
                            "Info")
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
                elif flag_key in ['vrs', 'rc_map_fail', 'fence_alarm', 'excess_lift', 'snr', 'ext_mag', 'imu_fault_lat']:
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
                elif flag_key in ['param']:
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
                elif flag_key in ['gyro_cal', 'acc_cal', 'mag_cal']:
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
                    self.exec_time.wdt_mark,
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
        
        self.mpu_temp.setText(f"{f.mpu_temp:.1f}°C")
        self.acc_confidence.setText(f"{f.acc_confidence * 100:.0f}%")

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
                FlightState.eStepClocks: "#87ceeb",
                FlightState.eStepMisc: "#87ceeb",
                FlightState.eStepParams: "#87ceeb",
                FlightState.eStepHarness: "#87ceeb",
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
                FlightState.eStepESCProg: "#87ceeb",
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
            self.gps_alt.setText(f"{f.gps_altitude:.1f}" if gps_valid else "---")
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
        _set('softserial', self.ser_soft_tx, self.ser_soft_rx, self.ser_soft_ov)

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
        self.imu_stats_logger.close()
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
