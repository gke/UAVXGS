# ui/main_window.py - Fixed with continuous polling and all packet types
"""
UAVX Groundstation - Main Window
"""

import sys
import os
import struct
import time
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
from logger.logger import Logger, GpsKmlLogger


FLAG_USING_GPS_ALT = 1
FLAG_USING_RANGEFINDER_ALT = 22

# Tee stdout to a file so the assistant can read debug logs directly
_DEBUG_LOG = None
class _Tee:
    def write(self, msg):
        sys.__stdout__.write(msg)
        sys.__stdout__.flush()
        global _DEBUG_LOG
        if _DEBUG_LOG is None:
            _DEBUG_LOG = open("/tmp/uavxgs_debug.log", "w", buffering=1)
        _DEBUG_LOG.write(msg)
    def flush(self):
        sys.__stdout__.flush()
        global _DEBUG_LOG
        if _DEBUG_LOG:
            _DEBUG_LOG.flush()

class TelemetryThread(QThread):
    """Background thread for serial telemetry"""
    data_received = pyqtSignal(bytes)
    connected = pyqtSignal(bool)
    error = pyqtSignal(str)
    
    def __init__(self, port: str, baud: int):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False
        self.serial = None
        self._is_closing = False
        self.verbose_serial = False
    
    def run(self):
        import serial
        import serial.tools.list_ports
        import time
        
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
            
            self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
            self.connected.emit(True)
            
            buffer = bytearray()
            packet = bytearray()
            esc_flag = False
            
            while self.running and not self._is_closing:
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    try:
                        data = self.serial.read(self.serial.in_waiting)
                        if data and self.verbose_serial:
                            print(f"[SERIAL] Read {len(data)} bytes: {data[:20].hex()}{'...' if len(data) > 20 else ''}")
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
                                if len(packet) > 0:
                                    if self.verbose_serial:
                                        print(f"[SERIAL] Discarding partial packet ({len(packet)} bytes)")
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
                        
                    except (OSError, Exception):
                        break
                
                QThread.msleep(10)
                
        except Exception as e:
            if not self._is_closing:
                self.error.emit(str(e))
        finally:
            self._close_serial()
            self.connected.emit(False)
    
    def _close_serial(self):
        self._is_closing = True
        self.running = False
        if self.serial:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except (OSError, Exception):
                pass
            self.serial = None
    
    def stop(self):
        self._is_closing = True
        self.running = False
        self._close_serial()
        self.wait()


class MainWindow(QMainWindow):
    def __init__(self):
        sys.stdout = _Tee()
        super().__init__()
        self.flight_data: FlightData = FlightData()
        self.nav_data: Optional[FlightData] = None
        self.link_stats: Optional[LinkStatsData] = None
        self.wind_data: Optional[WindData] = None
        self.serial_ports: Optional[dict] = None
        self.exec_time: Optional[ExecTimeData] = None
        self.min_data: Optional[FlightData] = None
        self.origin_data: Optional[OriginData] = None
        self.control_data: Optional[ControlData] = None
        self.guidance_data: Optional[GuidanceData] = None
        self.altitude_control_data: Optional[AltitudeControlData] = None
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
        
        self.speech = SpeechController()
        self.logger = Logger()
        self.gps_kml_logger = GpsKmlLogger()
        self._last_spoken_alt = -999
        self._last_spoken_batt = -999
        self._last_spoken_gps_ok = False
        self._last_spoken_armed = False
        self._last_spoken_wp_ach = False
        self._last_spoken_flight_state = -1
        self._last_spoken_alarm_state = -1
        self._last_spoken_low_batt = False
        self._pending_flight_state = -1
        self._state_change_time = 0.0
        self._logging_active = False
        self._kml_auto = False
        self.kml_path = None
        self.csv_path = None
        self.log_dir = None
        self._bb_chunks = {}
        self._last_wp_data = None
        self._param_write_list = []
        self._param_write_port = None
        self._param_write_on_complete = None
        self._param_write_index = 0
        self._revision_from_afname = False
        self._last_tune = None

        self.setup_ui()
        self.setup_menu()
        self.setup_connections()
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(100)
        
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.check_connection)
        self.status_timer.start(1000)
        
        self.setWindowTitle("UAVX Groundstation")
        self.setMinimumSize(1200, 900)
        
        self.load_settings()
    
    def setup_menu(self):
        menubar = self.menuBar()
        
        file_menu = menubar.addMenu("&File")
        log_folder_action = QAction("Set Log & KML Folder…", self)
        log_folder_action.triggered.connect(self.select_log_folder)
        file_menu.addAction(log_folder_action)
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
        self.param_window.show()
        self.param_window.raise_()
        self.param_window.activateWindow()
    
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

        self.kml_check = QCheckBox("KML")
        self.kml_check.setToolTip("Generate KML track file from incoming GPS")
        toolbar.addWidget(self.kml_check)

        self.kml_file_btn = QPushButton("📁")
        self.kml_file_btn.setToolTip("Select KML output file")
        self.kml_file_btn.setFixedWidth(32)
        toolbar.addWidget(self.kml_file_btn)

        self.csv_log_check = QCheckBox("Log")
        self.csv_log_check.setToolTip("Log flight data to CSV file")
        toolbar.addWidget(self.csv_log_check)

        self.csv_file_btn = QPushButton("📁")
        self.csv_file_btn.setToolTip("Select flight log output file")
        self.csv_file_btn.setFixedWidth(32)
        toolbar.addWidget(self.csv_file_btn)

        self.dump_bb_check = QCheckBox("BB")
        self.dump_bb_check.setToolTip("Dump Black Box from flight controller")
        self.dump_bb_check.setStyleSheet("font-weight: bold;")
        toolbar.addWidget(self.dump_bb_check)

        toolbar.addStretch()
        
        self.status_label = QLabel("● Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        toolbar.addWidget(self.status_label)
        
        main_layout.addLayout(toolbar)
        
        # ---- Status Bar (compact, replaces debug box) ----
        status_bar = QHBoxLayout()
        status_bar.setSpacing(6)

        self.status_msg = QLabel("Ready")
        self.status_msg.setStyleSheet("color: #888; font-size: 10px;")
        status_bar.addWidget(self.status_msg, 1)

        self.packet_log_check = QCheckBox("Debug")
        self.packet_log_check.setChecked(True)
        self.packet_log_check.setToolTip("Show debug messages in status bar")
        self.packet_log_check.setStyleSheet("font-size: 10px;")
        status_bar.addWidget(self.packet_log_check)

        status_bar.addWidget(QLabel("Level:"))
        self.debug_level_combo = QComboBox()
        self.debug_level_combo.addItems(["Info", "Warnings", "Errors", "All"])
        self.debug_level_combo.setCurrentIndex(3)
        self.debug_level_combo.setFixedWidth(80)
        status_bar.addWidget(self.debug_level_combo)

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
        
        # ---- Left Panel: Attitude Indicator + Altitude ----
        left_panel = QVBoxLayout()
        left_panel.setSpacing(5)
        left_panel.setAlignment(Qt.AlignCenter)
        
        # Attitude Indicator (with compass integrated) - 25% larger
        self.attitude = AttitudeIndicator()
        self.attitude.setMinimumSize(375, 375)
        self.attitude.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left_panel.addWidget(self.attitude, 2)

        # Execution time bargraph (between AH and Altitude)
        self.exec_bar = ExecTimeBar()
        left_panel.addWidget(self.exec_bar)

        # Firmware revision label (below exec bargraph)
        self.revision_label = QLabel("UAVX")
        self.revision_label.setAlignment(Qt.AlignCenter)
        self.revision_label.setStyleSheet("""
            font-size: 14px;
            font-weight: bold;
            color: #00ff00;
            background-color: black;
            padding: 4px;
            border: 1px solid #444;
            border-radius: 4px;
            min-height: 28px;
        """)
        left_panel.addWidget(self.revision_label)

        # Altitude display (centered below AH)
        alt_box = QGroupBox("")
        alt_box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 12px; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        alt_box.setAlignment(Qt.AlignCenter)
        alt_layout = QGridLayout()
        alt_layout.setAlignment(Qt.AlignCenter)
        alt_layout.setContentsMargins(10, 5, 10, 5)
        alt_layout.setHorizontalSpacing(8)
        
        self.roc_label = QLabel("0.0")
        self.roc_label.setAlignment(Qt.AlignCenter)
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
        alt_layout.addWidget(self.roc_label, 0, 1)

        self.altitude_label = QLabel("0.0")
        self.altitude_label.setAlignment(Qt.AlignCenter)
        self.altitude_label.setStyleSheet("""
            font-size: 32px;
            font-weight: bold;
            color: #00ff00;
            background-color: black;
            padding: 8px;
            border: 2px solid #444;
            border-radius: 4px;
            min-height: 50px;
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
        left_panel.addWidget(alt_box, 1)
        
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
        batt_layout.addWidget(self.batt_volts)
        batt_layout.addWidget(self.batt_current)
        batt_layout.addWidget(self.batt_charge)
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
        imu_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        imu_layout = QGridLayout()
        imu_layout.setHorizontalSpacing(8)
        imu_layout.setVerticalSpacing(4)
        
        imu_layout.addWidget(QLabel("Angles:"), 0, 0)
        self.angle_roll = QLabel("0.0")
        self.angle_pitch = QLabel("0.0")
        self.angle_yaw = QLabel("0.0")
        for lbl in (self.angle_roll, self.angle_pitch, self.angle_yaw):
            lbl.setStyleSheet("font-weight: bold;")
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
        imu_layout.addWidget(QLabel("L/R:"), 2, 1)
        imu_layout.addWidget(self.acc_lr, 2, 2)
        imu_layout.addWidget(QLabel("F/B:"), 2, 3)
        imu_layout.addWidget(self.acc_fb, 2, 4)
        imu_layout.addWidget(QLabel("D/U:"), 2, 5)
        imu_layout.addWidget(self.acc_du, 2, 6)
        
        imu_layout.addWidget(QLabel("MPU Temp:"), 3, 0)
        self.mpu_temp = QLabel("0.0°C")
        self.mpu_temp.setStyleSheet("font-weight: bold;")
        imu_layout.addWidget(self.mpu_temp, 3, 1, 1, 2)

        imu_layout.addWidget(QLabel("Acc Conf:"), 4, 0)
        self.acc_confidence = QLabel("0")
        self.acc_confidence.setStyleSheet("font-weight: bold;")
        imu_layout.addWidget(self.acc_confidence, 4, 1, 1, 2)

        imu_box.setLayout(imu_layout)
        right_panel.addWidget(imu_box)
        
        # Navigation
        nav_gps_row = QHBoxLayout()
        nav_gps_row.setSpacing(5)

        nav_box = QGroupBox("Navigation")
        nav_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        nav_layout = QGridLayout()
        nav_layout.setVerticalSpacing(2)
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
        alt_layout.setVerticalSpacing(2)
        alt_layout.setHorizontalSpacing(4)
        self.alt_kf = QLabel("0.0")
        self.alt_roc = QLabel("0.0")
        self.alt_baro_var = QLabel("0.01")

        self.alt_accu_var = QLabel("0.10")

        self.alt_accu_bias_var = QLabel("0")
        self.alt_desired = QLabel("0.0")
        for lbl in (self.alt_kf, self.alt_roc, self.alt_baro_var, self.alt_accu_var, self.alt_accu_bias_var, self.alt_desired):
            lbl.setStyleSheet("font-weight: bold;")
        alt_layout.addWidget(QLabel("KF Alt:"), 0, 0)
        alt_layout.addWidget(self.alt_kf, 0, 1)
        alt_layout.addWidget(QLabel("ROC:"), 0, 2)
        alt_layout.addWidget(self.alt_roc, 0, 3)
        alt_layout.addWidget(QLabel("Baro Var:"), 2, 0)
        alt_layout.addWidget(self.alt_baro_var, 2, 1)
        alt_layout.addWidget(QLabel("AccU Var:"), 3, 0)
        alt_layout.addWidget(self.alt_accu_var, 3, 1)
        alt_layout.addWidget(QLabel("Bias Var:"), 4, 0)
        alt_layout.addWidget(self.alt_accu_bias_var, 4, 1)
        alt_layout.addWidget(QLabel("Desired:"), 5, 0)
        alt_layout.addWidget(self.alt_desired, 5, 1)
        self.altitude_box.setLayout(alt_layout)
        nav_gps_row.addWidget(self.altitude_box)
        
        # GPS
        gps_box = QGroupBox("GPS")
        gps_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        gps_layout = QGridLayout()
        gps_layout.setVerticalSpacing(4)
        
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
        gps_layout.addWidget(self.gps_lat, 0, 1, 1, 2)
        gps_layout.addWidget(QLabel("Lon:"), 0, 3)
        gps_layout.addWidget(self.gps_lon, 0, 4, 1, 2)
        
        gps_layout.addWidget(QLabel("Alt:"), 1, 0)
        gps_layout.addWidget(self.gps_alt, 1, 1)
        gps_layout.addWidget(QLabel("Vel:"), 1, 2)
        gps_layout.addWidget(self.gps_vel, 1, 3)
        
        gps_layout.addWidget(QLabel("Sats:"), 2, 0)
        gps_layout.addWidget(self.gps_sats, 2, 1)
        gps_layout.addWidget(QLabel("Fix:"), 2, 2)
        gps_layout.addWidget(self.gps_fix, 2, 3)
        gps_layout.addWidget(QLabel("hAcc:"), 2, 4)
        gps_layout.addWidget(self.gps_hacc, 2, 5)
        
        gps_layout.addWidget(QLabel("vAcc:"), 3, 0)
        gps_layout.addWidget(self.gps_vacc, 3, 1)
        gps_layout.addWidget(QLabel("sAcc:"), 3, 2)
        gps_layout.addWidget(self.gps_sacc, 3, 3)
        gps_layout.addWidget(QLabel("cAcc:"), 3, 4)
        gps_layout.addWidget(self.gps_cacc, 3, 5)

        gps_layout.addWidget(QLabel("Rate:"), 4, 0)
        gps_layout.addWidget(self.gps_rate_label, 4, 1)
        
        self.gps_init_btn = QPushButton("Force GPS Init")
        self.gps_init_btn.setStyleSheet("color: #e67e22; font-weight: bold;")
        gps_layout.addWidget(self.gps_init_btn, 5, 0, 1, 6)
        
        gps_box.setLayout(gps_layout)
        nav_gps_row.addWidget(gps_box)

        right_panel.addLayout(nav_gps_row)
        
        # Link Stats
        link_box = QGroupBox("Link Stats")
        link_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        link_layout = QGridLayout()
        link_layout.setVerticalSpacing(2)
        link_layout.setHorizontalSpacing(4)
        self.link_lq = QLabel("--%")
        self.link_snr = QLabel("--")
        self.link_rssi = QLabel("--")
        self.link_losses = QLabel("--")
        self.link_failsafes = QLabel("--")
        for lbl in (self.link_lq, self.link_snr, self.link_rssi, self.link_losses, self.link_failsafes):
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
        link_layout.addWidget(self.link_failsafes, 2, 1, 1, 3)
        link_box.setLayout(link_layout)
        
        # Serial Ports (side-by-side with Link Stats)
        serial_box = QGroupBox("Serial Ports")
        serial_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        serial_layout = QGridLayout()
        serial_layout.setVerticalSpacing(2)
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
        self.motor_values = []
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
            motors_layout.addWidget(bar, row + 1, col)
            val_lbl = QLabel("1000")
            val_lbl.setAlignment(Qt.AlignCenter)
            val_lbl.setStyleSheet("font-size: 8px; color: #666;")
            motors_layout.addWidget(val_lbl, row + 2, col)
            self.motor_bars.append(bar)
            self.motor_values.append(val_lbl)
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
        config_names = ["Ext Mag", "Autoland", "No LEDs", "Emulation", "AH Alarm", "GPS Alt", "Disable VRS",
                        "Batt Comp", "Fast Start", "BLHeli", "Glider", "Rev Props", "Turn WP", "Beep WP"]
        for name in config_names:
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
        self.flags_box.setMinimumHeight(200)
        self.flags_box.setVisible(True)
        
        self.flag_labels = {}
        
        flags_layout = QVBoxLayout()
        flags_layout.setContentsMargins(4, 8, 4, 4)
        
        flag_defs = [
            # Core State
            ("Armed", 29, "armed", "Drives Armed"),
            ("Drives", 39, "armed2", "Drives Active"),
            ("DCMotors", 31, "dc_motors", "DC Motors Detected"),
            ("Signal", 35, "signal", "Signal"),
            ("RCMapFail", 43, "rc_map_fail", "RC Map Fail"),
            ("Emul", 27, "emul", "Emulation"),
            # Core Flags
            ("Nav", 15, "nav", "Navigate"),
            ("RTH", 16, "rth", "Return Home"),
            ("Origin", 7, "origin", "Origin Valid"),
            ("WPCent", 18, "wp_cent", "Waypoint Centred"),
            ("WPAch", 17, "wp_ach", "Waypoint Achieved"),
            ("Orbit", 19, "orbit", "Orbiting WP"),
            ("POI", 24, "poi", "Using POI"),
            ("VRS", 2, "vrs", "Rapid Descent Hazard"),
            ("Fence", 46, "fence_alarm", "Fence Alarm"),
            ("AutoLand", 20, "auto_land", "RTH Auto Descend"),
            ("LandSw", 3, "land_sw", "Landing Switch"),
            # Flight Modes
            ("Level", 4, "level", "Near Level"),
            ("Angle", 26, "angle", "Angle Control"),
            ("AttHold", 12, "att_hold", "Attitude Hold"),
            ("PassThru", 25, "pass_thru", "Bypass"),
            ("FixedWing", 40, "fixed_wing", "Is Fixed Wing"),
            # Altitude & RTH
            ("AltHold", 0, "alt_hold", "Altitude Hold"),
            ("HoldAlt", 14, "hold_alt", "Holding Altitude"),
            ("NewAlt", 44, "new_alt", "New Altitude Value"),
            ("GPS Alt", 1, "gps_alt", "Using GPS Altitude"),
            ("Baro", 21, "baro", "Baro Active"),
            ("RF", 22, "rf", "Rangefinder Active"),
            ("UseRF", 23, "use_rf", "Using Rangefinder"),
            # Navigation
            ("WPNav", 36, "wp_nav", "WP Navigation"),
            ("GPS", 6, "gps_ok", "GPS Valid"),
            ("Sat", 32, "sat", "Saturation"),
            ("Offset", 28, "offset", "Offset Origin Valid"),
            # Throttle
            ("ThrMove", 13, "thr_move", "Throttle Moving"),
            ("ThrOpen", 41, "thr_open", "Throttle Open"),
            # Sensors & Calibration
            ("IMU", 37, "imu", "IMU Active"),
            ("Mag", 38, "mag", "Magnetometer Active"),
            ("IMUCal", 45, "gyro_cal", "IMU Calibrated"),
            ("MagCal", 42, "mag_cal", "Mag Calibrated"),
            ("AccZBump", 30, "acc_z_bump", "Acc Z Bump"),
            # System
            ("Param", 34, "param", "Parameters Valid"),
            ("DumpBB", 33, "dump_bb", "Dumping Black Box"),
            ("LowBatt", 5, "low_batt", "Low Battery"),
        ]
        
        def make_flag_label(name, tooltip):
            label = QLabel(name)
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumWidth(70)
            label.setToolTip(tooltip)
            label.setStyleSheet("""
                background-color: #ddd;
                color: #666;
                font-weight: normal;
                border: 1px solid #999;
                border-radius: 3px;
                padding: 3px 6px;
            """)
            return label
        
        core_keys = {'armed', 'signal', 'rc_map_fail', 'thr_move', 'param', 'hold_alt', 'emul', 'low_batt'}
        sensor_keys = {'imu', 'gyro_cal', 'mag', 'mag_cal', 'rf', 'baro', 'gps_ok'}
        nav_keys = {'nav', 'rth', 'origin', 'wp_cent', 'wp_ach', 'auto_land', 'land_sw', 'orbit', 'poi', 'vrs', 'fence_alarm'}
        alt_keys = {'level', 'angle', 'use_rf', 'gps_alt', 'new_alt', 'alt_hold', 'acc_z_bump', 'att_hold'}
        modes_keys = {'armed2', 'fixed_wing', 'pass_thru', 'dc_motors', 'offset', 'thr_open', 'dump_bb', 'wp_nav', 'sat'}

        top_row = QHBoxLayout()
        top_row.setSpacing(6)
        for group_name, keys in [("Core", core_keys), ("Sensors", sensor_keys)]:
            group = QGroupBox(group_name)
            group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
            gl = QHBoxLayout()
            gl.setContentsMargins(4, 8, 4, 4)
            for name, bit, key, tooltip in flag_defs:
                if key in keys:
                    label = make_flag_label(name, tooltip)
                    gl.addWidget(label)
                    self.flag_labels[key] = {
                        'widget': label, 'bit': bit, 'name': name, 'key': key
                    }
            group.setLayout(gl)
            top_row.addWidget(group)
        flags_layout.addLayout(top_row)

        for group_name, keys in [("Nav", nav_keys)]:
            group = QGroupBox(group_name)
            group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
            gl = QHBoxLayout()
            gl.setContentsMargins(4, 8, 4, 4)
            for name, bit, key, tooltip in flag_defs:
                if key in keys:
                    label = make_flag_label(name, tooltip)
                    gl.addWidget(label)
                    self.flag_labels[key] = {
                        'widget': label, 'bit': bit, 'name': name, 'key': key
                    }
            group.setLayout(gl)
            flags_layout.addWidget(group)

        alt_modes_row = QHBoxLayout()
        alt_modes_row.setSpacing(6)
        for group_name, keys in [("Altitude", alt_keys), ("Modes", modes_keys)]:
            group = QGroupBox(group_name)
            group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
            gl = QHBoxLayout()
            gl.setContentsMargins(4, 8, 4, 4)
            for name, bit, key, tooltip in flag_defs:
                if key in keys:
                    label = make_flag_label(name, tooltip)
                    gl.addWidget(label)
                    self.flag_labels[key] = {
                        'widget': label, 'bit': bit, 'name': name, 'key': key
                    }
            group.setLayout(gl)
            alt_modes_row.addWidget(group)
        flags_layout.addLayout(alt_modes_row)


        self.flags_box.setLayout(flags_layout)
        main_layout.addWidget(self.flags_box)
        
        self.statusBar().showMessage("Ready")
    
    def setup_connections(self):
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.params_btn.clicked.connect(self.show_parameter_window)
        self.nav_btn.clicked.connect(self.show_nav_window)
        self.calib_btn.clicked.connect(self.show_calibration_window)
        self.misc_btn.clicked.connect(self.show_misc_window)
        self.flash_btn.clicked.connect(self.show_dfu_flasher)
        self.kml_check.toggled.connect(self._on_kml_toggled)
        self.csv_log_check.toggled.connect(self._on_log_toggled)
        self.kml_file_btn.clicked.connect(self.select_kml_file)
        self.csv_file_btn.clicked.connect(self.select_csv_file)
        self.dump_bb_check.toggled.connect(self.on_dump_bb_toggled)
        self.speech_level_combo.currentIndexChanged.connect(self.speech_level_changed)
        
        for btn in [self.connect_btn]:
            btn.setProperty('original_text', btn.text())
        
        self.gps_init_btn.clicked.connect(self.force_gps_init)
    
    def force_gps_init(self):
        self.send_request(PacketTag.MISC, MiscCommand.GPS_INIT, 0)
        self.log_debug("📤 Sent Force GPS Init command", "Info")

    def dump_black_box(self):
        self._bb_chunks = {}
        self.send_request(PacketTag.MISC, MiscCommand.BB_DUMP, 0)
        self.log_debug("📤 Sent Dump Black Box command", "Info")

    def on_dump_bb_toggled(self, checked):
        if checked:
            self.dump_black_box()
            self.dump_bb_check.setChecked(False)

    def _finalize_bb_dump(self):
        if not self._bb_chunks:
            return
        seqs = sorted(self._bb_chunks.keys())
        data = b''
        for s in seqs:
            data += self._bb_chunks[s]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Black Box Dump",
            f"bb_dump_{timestamp}.bin",
            "Binary Files (*.bin);;All Files (*)")
        if path:
            with open(path, 'wb') as f:
                f.write(data)
            self.log_debug(f"✅ BB dump saved ({len(data)}B) → {path}", "Info")
        self._bb_chunks = {}

    def _log_dir(self):
        """Absolute directory for logs/KML. Never relative, so files never
        land inside the project tree. Cross-platform via QStandardPaths, with
        a home-dir fallback if DocumentsLocation is unavailable."""
        if self.log_dir:
            return self.log_dir
        d = os.path.expanduser("~/UAVX")
        os.makedirs(d, exist_ok=True)
        return d

    def _new_kml_path(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self._log_dir(), f"{timestamp}_gps_track.kml")

    def _sync_loggers(self):
        # Logs only run while connected AND in flight. The checkbox state records
        # intent; files are started when we take off (armed) and flushed+closed
        # when we land (disarmed) so the actual landing is captured. A fresh
        # file is begun on the next flight if the box is still ticked.
        if not self.connected:
            return

        # KML runs if either the KML box or the flight-log (CSV) box is checked
        kml_wanted = self.kml_check.isChecked() or self.csv_log_check.isChecked()
        if self._in_flight and kml_wanted and not self.gps_kml_logger.active:
            path = os.path.abspath(self.kml_path or self._new_kml_path())
            self.gps_kml_logger.start(path)
            self.log_debug(f"📍 KML logging → {path}", "Info")
        elif not kml_wanted and self.gps_kml_logger.active:
            path = self.gps_kml_logger.stop()
            if path:
                self.log_debug(f"✅ KML saved → {path}", "Info")
            else:
                self.log_debug("ℹ️ No GPS points recorded", "Info")

        # CSV flight log only when the Log box is checked and we are in flight
        if self._in_flight and self.csv_log_check.isChecked() and self.logger.csv_writer is None:
            path = self.logger.start_log("flight", path=self.csv_path)
            self._logging_active = True
            self.log_debug(f"📝 Flight log → {os.path.abspath(path)}", "Info")
        elif not self.csv_log_check.isChecked() and self.logger.csv_writer is not None:
            self.logger.close()
            self._logging_active = False
            self.log_debug("✅ Flight log closed", "Info")

    def _finalize_loggers(self):
        """Flush and close the current log files (called when we land)."""
        if self.gps_kml_logger.active:
            path = self.gps_kml_logger.stop()
            if path:
                self.log_debug(f"✅ KML saved → {path}", "Info")
            else:
                self.log_debug("ℹ️ No GPS points recorded", "Info")
        if self.logger.csv_writer is not None:
            self.logger.close()
            self._logging_active = False
            self.log_debug("✅ Flight log closed", "Info")

    def _on_kml_toggled(self, checked):
        if not self.kml_check.isEnabled():
            return
        self._kml_auto = False  # explicit user choice
        self._sync_loggers()

    def _on_log_toggled(self, checked):
        if checked:
            # Enabling flight log also enables KML generation
            self.kml_check.setEnabled(False)
            if not self.kml_check.isChecked():
                self.kml_check.setChecked(True)
                self._kml_auto = True
            self._sync_loggers()
        else:
            self.kml_check.setEnabled(True)
            if self._kml_auto:
                self._kml_auto = False
                self.kml_check.setChecked(False)
            self._sync_loggers()

    def select_kml_file(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Select KML Track File",
            self.kml_path or os.path.join(self._log_dir(), "gps_track.kml"),
            "KML Files (*.kml);;All Files (*)")
        if path:
            self.kml_path = path
            self.log_debug(f"📁 KML file set → {path}", "Info")

    def select_csv_file(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Select Flight Log File",
            self.csv_path or os.path.join(self._log_dir(), "flight.csv"),
            "CSV Files (*.csv);;All Files (*)")
        if path:
            self.csv_path = path
            self.log_debug(f"📁 Flight log file set → {path}", "Info")

    def select_log_folder(self):
        start = self.log_dir or os.path.expanduser("~/UAVX")
        folder = QFileDialog.getExistingDirectory(self, "Select Log & KML Folder", start)
        if folder:
            self.log_dir = folder
            os.makedirs(self.log_dir, exist_ok=True)
            self.save_settings()
            self.log_debug(f"📂 Log folder set → {folder}", "Info")
        self._update_log_location_indicator()

    def _update_log_location_indicator(self):
        """Colour the file buttons: orange when the save location is unknown,
        green once a folder has been selected."""
        known = bool(self.log_dir) and os.path.isdir(self.log_dir)
        color = "green" if known else "orange"
        style = f"background-color: {color}; color: white; font-weight: bold;"
        self.kml_file_btn.setStyleSheet(style)
        self.csv_file_btn.setStyleSheet(style)

    def load_settings(self):
        settings = QSettings("UAVX", "Groundstation")
        self.port_combo.setCurrentText(settings.value("port", "/dev/ttyUSB0"))
        self.baud_combo.setCurrentText(settings.value("baud", "115200"))
        speech_level = int(settings.value("speech_level", SpeechLevel.ALL.value))
        self.speech.level = SpeechLevel(speech_level)
        self.speech_level_combo.setCurrentIndex(speech_level)

        # Log folder: prompt once on first run, otherwise use saved location
        self.log_dir = settings.value("log_dir", None)
        if not self.log_dir:
            default_dir = os.path.expanduser("~/UAVX")
            folder = QFileDialog.getExistingDirectory(
                self, "Select folder for KML & flight logs", default_dir)
            if folder:
                self.log_dir = folder
            else:
                self.log_dir = default_dir
            settings.setValue("log_dir", self.log_dir)
        os.makedirs(self.log_dir, exist_ok=True)
        # Route the CSV logger's default (no explicit file chosen) to this folder
        self.logger.base_dir = self.log_dir
        # Verify + confirm the active log location on every startup
        if not os.access(self.log_dir, os.W_OK):
            self.log_debug(f"⚠️ Log folder not writable: {self.log_dir}", "Errors")
        else:
            self.log_debug(f"📂 Log folder → {self.log_dir}", "Info")
        self._update_log_location_indicator()

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
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        
        self.telemetry = TelemetryThread(port, baud)
        self.telemetry.data_received.connect(self.process_packet)
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
        
        if self.telemetry:
            self.telemetry.stop()
            self.telemetry = None
        
        self.connected = False
        self._in_flight = False
        self._finalize_loggers()
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
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet("background-color: green; color: white; font-weight: bold;")
            self.status_label.setText("● Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.statusBar().showMessage("Connected to UAVX")
            self.log_debug("✅ Connected to UAVX", "Info")
            self.send_request(PacketTag.PARAM_TAGGED_READ, 255, 0, None)
            self.send_request(PacketTag.MIN, 0, 0, None)
            self.send_request(PacketTag.AFNAME, 0, 0, None)
            self.send_request(PacketTag.TUNING, 0, 0, None)
        else:
            self._in_flight = False
            self._finalize_loggers()
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
        self.status_msg.setStyleSheet("color: #888; font-size: 10px;")
        levels = ["Info", "Warnings", "Errors", "All"]
        self.log_debug(f"🔍 Debug level: {levels[index]}", "Info")
    
    def speech_level_changed(self, index):
        level = SpeechLevel(index)
        self.speech.level = level
        label = LEVEL_LABELS.get(level, "Off")
        self.log_debug(f"🔊 Speech level: {label}", "Info")
    
    def log_debug(self, message, level="Info"):
        if not self.packet_log_check.isChecked() and "packet" in message.lower():
            return
        
        level_idx = self.debug_level_combo.currentIndex()
        levels = ["Info", "Warnings", "Errors", "All"]
        current_level = levels[level_idx]
        
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
        self.status_msg.setStyleSheet(f"color: {color}; font-size: 10px;")
    
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
        raw_data.append(3)
        raw_data.append(tag)
        raw_data.append(a1)
        raw_data.append(a2)

        packet = self._build_packet_with_checksum(raw_data)

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
        raw_data.append(len(body))
        raw_data.extend(body)
        packet = self._build_packet_with_checksum(raw_data)
        try:
            serial_port.write(packet)
            self.log_debug(f"📤 Sent raw packet: tag={tag}, len={len(body)}", "Info")
        except Exception as e:
            self.log_debug(f"❌ Failed to send raw packet: {e}", "Errors")
    
    def send_param_commit(self):
        """Tell FC to save current params to flash."""
        raw_data = bytearray([PacketTag.PARAM_COMMIT, 0])
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
            if self.param_window is not None:
                self.param_window.update_params_from_typed(parsed)
            else:
                self._pending_typed_params = parsed
        version = getattr(parsed, 'version_name', '')
        if version and not self._revision_from_afname:
            self.revision_label.setText(f"UAVX {version}")

    def send_params(self, request_id=None):
        if not self.connected or not self.telemetry:
            self.log_debug("❌ Cannot send params - not connected", "Errors")
            return

        if not self.param_window:
            self.log_debug("❌ Parameter window not open", "Errors")
            return

        serial_port = getattr(self.telemetry, 'serial', None)
        if not serial_port or not serial_port.is_open:
            self.log_debug("❌ Serial port not open", "Errors")
            return

        raw_data = bytearray()
        raw_data.append(PacketTag.PARAM)
        
        version_len = 0
        raw_data.append(1 + 128 + version_len + 1)
        raw_data.append(0)

        param_count = 0
        param_values = []
        
        for i in range(128):
            if i in self.param_window.params:
                widget = self.param_window.params[i]
                if isinstance(widget, QDoubleSpinBox):
                    val = int(widget.value())
                elif isinstance(widget, QComboBox):
                    data = widget.itemData(widget.currentIndex())
                    val = data if data is not None else widget.currentIndex()
                else:
                    val = 0
                raw_data.append(val & 0xFF)
                param_count += 1
                param_values.append((i, val))
            else:
                raw_data.append(0)
                param_values.append((i, 0))
        
        raw_data.append(0)

        log_str = ", ".join([f"P{p}={v}" for p, v in param_values[:10]])
        self.log_debug(f"📤 Sending {param_count} params (first 10: {log_str})", "Info")

        packet = self._build_packet_with_checksum(raw_data)

        try:
            serial_port.write(packet)
            self.log_debug(f"📤 Sent {param_count} parameters", "Info")
            if request_id is not None:
                ack_handler.request_sent(request_id)
            else:
                found_id = None
                for req_id, req in ack_handler.pending_requests.items():
                    if req['tag'] == PacketTag.PARAM:
                        found_id = req_id
                        break
                if found_id is not None:
                    ack_handler.request_sent(found_id)
        except Exception as e:
            self.log_debug(f"❌ Failed to send params: {e}", "Errors")

    def send_params_typed(self, on_complete=None, indices=None):
        """Send widget values as individual param packets.
        If indices is provided, only those param indices are sent.
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
                    mult = PARAM_DISPLAY_MULT.get(i, 1.0)
                    fval = display / mult
                elif isinstance(widget, QSpinBox):
                    fval = float(widget.value())
                elif isinstance(widget, QComboBox):
                    fval = float(widget.currentIndex())
                else:
                    fval = 0.0
            else:
                fval = 0.0
            params.append((i, fval))

        self._param_write_list = params
        self._param_write_port = serial_port
        self._param_write_on_complete = on_complete
        self._param_write_index = 0
        self._send_next_param()

    def _send_next_param(self):
        if not self._param_write_list:
            self.log_debug("✅ All param packets sent")
            cb = self._param_write_on_complete
            self._param_write_on_complete = None
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
            self._param_write_port = None
            cb and cb()
            return
        try:
            raw_data = bytearray([PacketTag.PARAM, 5, idx])
            raw_data.extend(struct.pack('<f', fval))
            packet = self._build_packet_with_checksum(raw_data)
            serial_port.write(packet)
            self._param_write_index += 1
            self.log_debug(f"📤 Sent param {idx} = {fval}")
        except Exception as e:
            self.log_debug(f"❌ Failed to send param {idx}: {e}", "Errors")
            self._param_write_list = []
            cb = self._param_write_on_complete
            self._param_write_on_complete = None
            self._param_write_port = None
            cb and cb()
            return
        QTimer.singleShot(5, self._send_next_param)
    
    def _check_speech_events(self, f):
        alt = int(f.altitude)
        if abs(alt - self._last_spoken_alt) >= 10:
            self.speech.speak_altitude(alt)
            self._last_spoken_alt = alt

        batt = f.battery_volts
        if batt > 0 and abs(batt - self._last_spoken_batt) >= 0.5:
            self.speech.speak_battery(batt)
            self._last_spoken_batt = batt

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
        parsed, tag = parse_packet(data, 1.0)

        TAG_NAMES = {
            13: "Flight", 14: "Nav", 15: "Stats", 16: "Control", 17: "Param",
            18: "Min", 19: "Origin", 20: "WP", 21: "Mission", 22: "RC",
            50: "Request", 51: "ACK", 52: "Misc", 53: "Noise", 54: "BB",
            55: "Inertial", 56: "MinimOSD", 57: "Tuning", 58: "UKF", 59: "Guidance",
            60: "AltCtrl", 61: "Soaring", 62: "Calibration", 63: "AFName",
            64: "Wind", 65: "Track", 66: "SerialPorts", 67: "ExecTime",
             68: "AttCtrl", 69: "LinkStats",
        }
        tag_name = TAG_NAMES.get(tag, f"Unknown({tag})")

        # Terminal dump of decoded packet contents
        if tag == 13 and hasattr(parsed, 'flag_bits'):
            fb = parsed.flag_bits
            def fbok(i): return fb[i] if len(fb) > i else False
            r2d = 57.2958
            pwm_str = ""
            if hasattr(parsed, 'pwm') and parsed.pwm:
                pwm_vals = [f"M{i}={v:+5d}" for i, v in enumerate(parsed.pwm)]
                pwm_str = f" pwm={' '.join(pwm_vals)}"
            print(f"[FLIGHT] {'A' if fbok(29) else '_'}{'E' if fbok(27) else '_'}"
                  f"{'I' if fbok(37) else '_'}{'M' if fbok(38) else '_'}{'G' if fbok(6) else '_'}"
                  f" ang=({parsed.angle_roll*r2d:+6.1f},{parsed.angle_pitch*r2d:+6.1f},{parsed.angle_yaw*r2d:+6.1f})"
                  f" dAng=({parsed.desired_roll*r2d:+5.1f},{parsed.desired_pitch*r2d:+5.1f},{parsed.desired_yaw*r2d:+5.1f})"
                  f" rate=({parsed.rate_roll*r2d:+6.2f},{parsed.rate_pitch*r2d:+6.2f},{parsed.rate_yaw*r2d:+6.2f})"
                  f" dRate=({parsed.desired_rate_roll*r2d:+5.1f},{parsed.desired_rate_pitch*r2d:+5.1f},{parsed.desired_rate_yaw*r2d:+5.1f})"
                  f" acc=({parsed.acc_lr:+5.2f},{parsed.acc_fb:+5.2f},{parsed.acc_du:+5.2f})"
                  f" thr={parsed.desired_throttle:.3f} alt={parsed.altitude:.1f}{pwm_str}")
        elif tag == 14 and hasattr(parsed, 'gps_lat'):
            print(f"[NAV] lat={parsed.gps_lat:.6f} lon={parsed.gps_lon:.6f}"
                  f" spd={parsed.gps_vel:.1f} hdg={parsed.gps_heading:.1f}"
                  f" sats={parsed.gps_sats} fix={parsed.gps_fix}")
        elif tag == 16 and hasattr(parsed, 'angle_roll'):
            r2d = 57.2958
            print(f"[CTRL] ang=({parsed.angle_roll*r2d:+6.1f},{parsed.angle_pitch*r2d:+6.1f},{parsed.angle_yaw*r2d:+6.1f})"
                  f" rate=({parsed.rate_roll*r2d:+6.2f},{parsed.rate_pitch*r2d:+6.2f},{parsed.rate_yaw*r2d:+6.2f})"
                  f" thr={parsed.desired_throttle:.3f}")
        elif tag == 22 and hasattr(parsed, 'rc_channels'):
            rc = parsed.rc_channels
            if rc:
                def rc_name(i):
                    return ["Thr","Rol","Pit","Yaw","Nav","Att","NQ","Cam","Aux2","Trn","PT","Dive"][i]
                parts = [f"{rc_name(i)}={rc[i]}" for i in range(min(len(rc), 12))]
                print("[RC] " + " ".join(parts))
        elif tag == 62 and isinstance(parsed, dict):
            flags = parsed.get('flags', [])
            if len(flags) > 5:
                imu   = bool(flags[5] & 32)
                mag   = bool(flags[4] & 64)
                imuc  = bool(flags[5] & 128)
                magc  = bool(flags[5] & 4)
                cal   = parsed.get('cal_data', [])
                gs    = f"gyr=({cal[1]/1000:.1f},{cal[7]/1000:.1f},{cal[13]/1000:.1f})" if len(cal)>13 else ""
                print(f"[CALIB] IMU={'Y' if imu else 'N'}{'C' if imuc else '_'}"
                      f" Mag={'Y' if mag else 'N'}{'C' if magc else '_'} {gs}")
        elif tag == 68 and hasattr(parsed, 'rate_roll'):
            r2d = 57.2958
            print(f"[ATTCTRL] rate=({parsed.rate_roll*r2d:+6.2f},{parsed.rate_pitch*r2d:+6.2f},{parsed.rate_yaw*r2d:+6.2f})"
                  f" q=({getattr(parsed,'q0',0):+.3f},{getattr(parsed,'q1',0):+.3f},{getattr(parsed,'q2',0):+.3f},{getattr(parsed,'q3',0):+.3f})")

        if tag == 13:
            self.log_debug(f"📦 Flight ({len(data)}B)", "Info")
        elif tag == 14:
            pass  # suppress NAV clutter during emulation

        match tag:
            case 13:
                if self.flight_data:
                    old_pwm = self.flight_data.pwm if hasattr(self.flight_data, 'pwm') and self.flight_data.pwm else None
                    old_rc = self.flight_data.rc_channels if hasattr(self.flight_data, 'rc_channels') and self.flight_data.rc_channels else None
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
                    old_gps = {}
                self.flight_data = parsed
                if old_pwm is not None and not self.flight_data.pwm:
                    self.flight_data.pwm = old_pwm
                if old_rc is not None and not self.flight_data.rc_channels:
                    self.flight_data.rc_channels = old_rc
                for attr, val in old_gps.items():
                    if getattr(self.flight_data, attr, None) in (None, 0, 0.0):
                        setattr(self.flight_data, attr, val)
                data_manager.update_flight_data(self.flight_data)
                packet_logger.log_received(13, "FLIGHT", parsed)
                if hasattr(parsed, 'flag_bits'):
                    self.current_flag_bits = parsed.flag_bits
                if self._logging_active:
                    self.logger.log_flight_data(parsed)

                if self.gps_kml_logger.active and hasattr(parsed, 'gps_lat'):
                    self.gps_kml_logger.add_point(
                        parsed.gps_lat, parsed.gps_lon, parsed.gps_altitude,
                        parsed.gps_heading if hasattr(parsed, 'gps_heading') else 0,
                        parsed.gps_vel if hasattr(parsed, 'gps_vel') else 0,
                        parsed.mission_time if hasattr(parsed, 'mission_time') else 0
                    )

                # Flight lifecycle: start logs on take-off (armed), flush+close
                # on landing (disarmed) so the actual touchdown is captured.
                if hasattr(parsed, 'flag_bits') and len(parsed.flag_bits) > 29:
                    armed = parsed.flag_bits[29]
                    if armed and not self._in_flight:
                        self._in_flight = True
                        self._sync_loggers()
                    elif not armed and self._in_flight:
                        self._in_flight = False
                        self._finalize_loggers()

                self._check_speech_events(parsed)

                # Trigger deferred param verification after FC reboot from commit
                if self._pending_param_verification and self.param_window:
                    self._pending_param_verification = False
                    self.param_window._request_write_verify()

            case 14:
                self.nav_data = parsed
                data_manager.update_nav_data(parsed)
                packet_logger.log_received(14, "NAV", parsed)
                if self.gps_kml_logger.active and hasattr(parsed, 'gps_lat'):
                    self.gps_kml_logger.add_point(
                        parsed.gps_lat, parsed.gps_lon, parsed.gps_altitude,
                        parsed.gps_heading if hasattr(parsed, 'gps_heading') else 0,
                        parsed.gps_vel if hasattr(parsed, 'gps_vel') else 0,
                        getattr(parsed, 'mission_time', 0)
                    )
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

            case 18:
                self.min_data = parsed
                packet_logger.log_received(18, "MIN", parsed)
                if self.flight_data:
                    for attr in ("flight_state", "nav_state", "alarm_state",
                                 "battery_volts", "battery_current", "battery_charge",
                                 "angle_roll", "angle_pitch", "altitude",
                                 "roc", "heading", "gps_lat", "gps_lon", "airframe_type"):
                        setattr(self.flight_data, attr, getattr(parsed, attr, getattr(self.flight_data, attr)))
                    if hasattr(parsed, 'flag_bits'):
                        self.current_flag_bits = self.flight_data.flag_bits = parsed.flag_bits
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
                    data_manager.update_flight_data(self.flight_data)

            case 51:
                if isinstance(parsed, dict) and 'ack_tag' in parsed:
                    at = parsed['ack_tag']
                    ok = parsed['ack_success']
                    packet_logger.log_received(51, "ACK", parsed)
                    if at in (241, 223):
                        return
                    ack_handler.handle_ack(at, ok)

            case 59:
                self.guidance_data = parsed
                packet_logger.log_received(59, "GUIDANCE", parsed)
                if self.flight_data and hasattr(parsed, 'distance'):
                    self.flight_data.distance_to_wp = parsed.distance

            case 60:
                self.altitude_control_data = parsed
                packet_logger.log_received(60, "ALTCTRL", parsed)

            case 62:
                packet_logger.log_received(62, "CALIB", parsed)
                if isinstance(parsed, dict):
                    data_manager.update_calibration_data(parsed)

            case 63:
                if isinstance(parsed, dict) and 'revision' in parsed:
                    rev = parsed['revision']
                    packet_logger.log_received(63, "CONFIG", parsed)
                    self.log_debug(f"  FW: {rev}, AFType: {parsed.get('af_type', '?')}", "Info")
                    self.revision_label.setText(rev)
                    self._revision_from_afname = True

            case 66:
                packet_logger.log_received(66, "SERIAL", parsed)
                if isinstance(parsed, dict) and 'telemetry' in parsed:
                    self.serial_ports = parsed
                    self.update_serial_ports_display()

            case 67:
                packet_logger.log_received(67, "EXEC", parsed)
                self.exec_time = parsed

            case 68:
                packet_logger.log_received(68, "ATTCTRL", parsed)
                if parsed is not None and getattr(parsed, 'is_quaternion', False):
                    print(f"[Q] rate=({parsed.rate_roll:+.2f},{parsed.rate_pitch:+.2f},{parsed.rate_yaw:+.2f}) "
                          f"out=({parsed.out_roll:+.2f},{parsed.out_pitch:+.2f},{parsed.out_yaw:+.2f}) "
                          f"q=({parsed.q0:+.3f},{parsed.q1:+.3f},{parsed.q2:+.3f},{parsed.q3:+.3f}) "
                          f"ang=({parsed.angle_roll:+.2f},{parsed.angle_pitch:+.2f})")

            case 64:
                packet_logger.log_received(64, "WIND", parsed)
                if isinstance(parsed, WindData):
                    self.wind_data = parsed

            case 69:
                packet_logger.log_received(69, "LINK", parsed)
                if isinstance(parsed, LinkStatsData):
                    self.link_stats = parsed
                    self.update_link_stats_display()

            case 57:
                packet_logger.log_received(57, "TUNE", parsed)
                self._last_tune = parsed

            case 54:
                packet_logger.log_received(54, "BB", parsed)
                if isinstance(parsed, (bytes, bytearray)) and len(parsed) >= 4:
                    seq_no = struct.unpack('<i', parsed[:4])[0]
                    chunk = parsed[4:]
                    self._bb_chunks[seq_no] = chunk
                    total = len(self._bb_chunks) * 128
                    self.log_debug(f"📦 BB chunk {seq_no} ({len(chunk)}B, ~{total}B total)", "Info")
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

        MISC_TAGS = {15, 21, 50, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 64, 65, 67, 68}
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
        
        bits = [0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6]
        vals = [config1] * 7 + [config2] * 7
        for i, (bit, val) in enumerate(zip(bits, vals)):
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
        
        for flag_key, flag_data in self.flag_labels.items():
            bit = flag_data['bit']
            widget = flag_data['widget']
            is_active = bit < len(flag_bits) and flag_bits[bit]
            
            if is_active:
                if flag_key in ['level', 'att_hold']:
                    color = "#3cb371"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['low_batt', 'land_sw']:
                    color = "#e74c3c"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['vrs', 'rc_map_fail', 'fence_alarm']:
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
                if flag_key in ['gps_ok', 'baro', 'rf', 'imu', 'mag']:
                    color = "#e74c3c"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['gyro_cal', 'acc_cal', 'mag_cal']:
                    color = "#f39c12"
                    text_color = "white"
                    font_weight = "bold"
                elif flag_key in ['fixed_wing']:
                    color = "#95a5a6"
                    text_color = "white"
                    font_weight = "normal"
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
    
    def update_ui(self):
        if not self.flight_data:
            return
        
        f = self.flight_data
        
        pitch = -f.angle_pitch * 57.2958
        roll = f.angle_roll * 57.2958
        heading = f.heading * 57.2958
        self.attitude.set_attitude(pitch, roll, heading)
        
        if self.exec_time:
            self.exec_bar.set_exec_time(
                self.exec_time.exec_percent,
                self.exec_time.exec_peak_percent
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
        
        self.gyro_roll.setText(f"{f.rate_roll * 57.2958:.1f}")
        self.gyro_pitch.setText(f"{f.rate_pitch * 57.2958:.1f}")
        self.gyro_yaw.setText(f"{f.rate_yaw * 57.2958:.1f}")
        
        self.acc_lr.setText(f"{f.acc_lr:.2f}")
        self.acc_fb.setText(f"{f.acc_fb:.2f}")
        self.acc_du.setText(f"{f.acc_du:.2f}")
        
        self.mpu_temp.setText(f"{f.mpu_temp:.1f}°C")
        self.acc_confidence.setText(f"{f.acc_confidence:.0f}")

        self.alt_kf.setText(f"{f.baro_altitude:.2f}")
        self.alt_roc.setText(f"{f.roc:.1f}")
        self.alt_baro_var.setText(f"{f.baro_variance:.3f}")

        self.alt_accu_var.setText(f"{f.accu_variance:.3f}")
        self.alt_accu_bias_var.setText(f"{f.accu_bias_variance:.6f}")
        if hasattr(f, 'flight_state'):
            state_name = FlightState.get_name(f.flight_state)
            self.flight_state_label.setText(state_name)
            state_colors = {
                FlightState.STARTING: "#add8e6",
                FlightState.WARMUP: "#00ff00",
                FlightState.LANDING: "#008000",
                FlightState.LANDED: "#bc8f8f",
                FlightState.SHUTDOWN: "#ffa500",
                FlightState.FLYING: "#c0c0c0",
                FlightState.IREMULATE: "#bc8f8f",
                FlightState.PREFLIGHT: "#ff0000",
                FlightState.READY: "#ffd700",
                FlightState.THROTTLE_OPEN_CHECK: "#ff0000",
                FlightState.ERECTING_GYROS: "#ff0000",
                FlightState.MONITOR_INSTRUMENTS: "#c0c0c0",
                FlightState.INITIALISING_GPS: "#add8e6",
            }
            color = state_colors.get(f.flight_state, "#ffffff")
            self.flight_state_label.setStyleSheet(f"font-weight: bold; background-color: {color}; padding: 2px 4px; border-radius: 2px;")
        
        if hasattr(f, 'gps_fix') and hasattr(f, 'gps_sats'):
            self.gps_sats.setText(str(f.gps_sats))
            self.gps_fix.setText(str(f.gps_fix))
            self.gps_lat.setText(f"{f.gps_lat:.6f}")
            self.gps_lon.setText(f"{f.gps_lon:.6f}")
            self.gps_alt.setText(f"{f.gps_altitude:.1f}")
            self.gps_vel.setText(f"{f.gps_vel:.1f}")
            self.gps_hacc.setText(f"{f.gps_hacc:.1f}")
            self.gps_vacc.setText(f"{f.gps_vacc:.1f}")
            self.gps_sacc.setText(f"{f.gps_sacc:.1f}")
            self.gps_cacc.setText(f"{f.gps_cacc:.1f}")

            if hasattr(f, 'flag_bits') and len(f.flag_bits) > 6:
                gps_valid = f.flag_bits[6]
            else:
                gps_valid = (f.gps_fix >= 3 and f.gps_sats >= 6)
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
            self.wp_bearing.setText(f"{n.wp_bearing:.0f}")
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

        if hasattr(f, 'mag_var_wmm'):
            self.mag_var_wmm.setText(f"{f.mag_var_wmm:.1f}°")

        self.update_config_colors()
        
        # Always show altitude box (emulation provides fake but usable data)
        self.update_link_stats_display()
        self.update_motors_display(f)
        self.update_controls_display(f)
    
    def update_link_stats_display(self):
        if not self.link_stats:
            return
        s = self.link_stats
        self.link_lq.setText(f"{s.uplink_lq}%")
        self.link_snr.setText(f"{s.uplink_snr}")
        self.link_rssi.setText(f"{s.uplink_rssi}")
        self.link_losses.setText(f"{s.rc_signal_losses}")
        self.link_failsafes.setText(f"{s.rc_failsafes}")

    def update_motors_display(self, f):
        throttle = getattr(f, 'desired_throttle', 0.0)
        pwm = getattr(f, 'pwm', None)
        if pwm is not None:
            for i in range(min(len(self.motor_bars), len(pwm))):
                raw = pwm[i]  # 0..+1000 from FC (RawPW * 1000)
                self.motor_bars[i].setValue(max(0, min(1000, raw)))
                self.motor_values[i].setText(f"{raw:+5d}")
        elif throttle > 0.01:
            val = max(0, min(1000, int(throttle * 1000)))
            for bar in self.motor_bars:
                bar.setValue(val)
            for lbl in self.motor_values:
                lbl.setText(f"{val:+5d}")
        else:
            for bar in self.motor_bars:
                bar.setValue(0)
            for lbl in self.motor_values:
                lbl.setText(f"{0:+5d}")

    def update_controls_display(self, f):
        if hasattr(f, 'rc_channels') and len(f.rc_channels) >= 4:
            for i in range(4):
                self.control_bars[i].setValue(max(0, min(1000, f.rc_channels[i] - 1000)))
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
        if self.telemetry and not self.telemetry.isRunning() and self.connected:
            self.disconnect()
    
    def closeEvent(self, event):
        self.save_settings()
        
        if self.param_window:
            self.param_window.close()
        if self.nav_window:
            self.nav_window.close()
        if self.calib_window:
            self.calib_window.close()
        if self.misc_window:
            self.misc_window.close()
        
        if self.telemetry:
            self.telemetry.stop()
            self.telemetry.wait()
        event.accept()
