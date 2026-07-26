# ui/calibration_window.py
"""
UAVX Calibration Window
IMU (Gyro + Accel), Magnetometer, and RC calibration
"""

import sys
import os
from typing import Optional

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol_constants import *
from protocol_enums import PacketTag, MiscCommand
from packet_parser import *
from core.data_manager import data_manager


class CalibrationWindow(QMainWindow):
    """Calibration window for IMU, mag, and RC"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.setWindowTitle("UAVX Calibration")
        self.setMinimumSize(600, 700)
        
        # IMU calibration state
        self.imu_calibrating = False
        self.imu_calibration_requested = False
        self.imu_calibration_start_time = None
        self.imu_calibration_timeout = 60000
        self.current_imu_calibration_type = MiscCommand.CAL_IMU
        
        # Mag calibration state
        self.mag_points = []
        self.mag_calibration_requested = False
        self.mag_calibration_start_time = None
        self.mag_calibration_timeout = 120000  # 2 minute safety timeout
        
        # RC calibration state
        self.rc_calibrating = False
        self.rc_samples = {}
        
        self.setup_ui()
        self.setup_connections()
        
        # Start telemetry update timer
        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self.update_from_telemetry)
        self.telemetry_timer.start(100)  # 10Hz
        
        # Initial update
        self.update_from_telemetry()
    
    def _log(self, msg):
        """Send debug message to parent window's log if available"""
        if self.parent_window and hasattr(self.parent_window, 'log_debug'):
            self.parent_window.log_debug(msg, "Info")
        print(msg)
    
    def setup_ui(self):
        """Create the calibration UI"""
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # ---- Title ----
        title = QLabel("UAVX Calibration")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # ---- Tab Widget ----
        self.tab_widget = QTabWidget()
        
        # Tab 1: IMU Calibration (Gyro + Accel)
        self.imu_tab = self._create_imu_tab()
        self.tab_widget.addTab(self.imu_tab, "IMU")
        
        # Tab 2: Magnetometer Calibration
        self.mag_tab = self._create_mag_tab()
        self.tab_widget.addTab(self.mag_tab, "Magnetometer")
        
        # Tab 3: RC Calibration
        self.rc_tab = self._create_rc_tab()
        self.tab_widget.addTab(self.rc_tab, "RC")
        
        main_layout.addWidget(self.tab_widget)
        
        # ---- Status Bar ----
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("padding: 5px; background-color: #f0f0f0;")
        main_layout.addWidget(self.status_label)
    
    def _create_imu_tab(self):
        """Create IMU calibration tab (Gyro + Accel)"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Info
        info = QLabel(
            "IMU Calibration\n"
            "Place the vehicle on a level, stationary surface.\n"
            "Select the calibration type and click 'Start Calibration'.\n"
            "Do not move the vehicle during calibration."
        )
        info.setStyleSheet("color: #555; padding: 5px; border: 1px solid #ddd; border-radius: 4px;")
        info.setWordWrap(True)
        layout.addWidget(info)
        
        self.imu_cal_status = QLabel("IMU Calibration: Unknown")
        self.imu_cal_status.setStyleSheet("padding: 5px; background-color: #f39c12; color: white; font-weight: bold; border-radius: 4px;")
        layout.addWidget(self.imu_cal_status)
        
        # Calibration type selection
        type_group = QGroupBox("Calibration Type")
        type_layout = QHBoxLayout()
        
        type_layout.addWidget(QLabel("Type:"))
        self.imu_cal_type_combo = QComboBox()
        self.imu_cal_type_combo.addItem("IMU (Gyro + Accel)", MiscCommand.CAL_IMU)
        self.imu_cal_type_combo.addItem("Accelerometer Only", MiscCommand.CAL_ACC)
        self.imu_cal_type_combo.setCurrentIndex(1)
        self.imu_cal_type_combo.currentIndexChanged.connect(self._on_imu_type_changed)
        type_layout.addWidget(self.imu_cal_type_combo)
        
        type_layout.addStretch()
        type_group.setLayout(type_layout)
        layout.addWidget(type_group)
        
        # Heat-cycling warning for full IMU cal
        self.imu_warning = QLabel(
            "⚠️ CAUTION: Full IMU (Gyro + Accel) calibration requires heat cycling.\n"
            "The FC must be powered for several minutes to stabilize temperature\n"
            "before accurate offsets can be determined."
        )
        self.imu_warning.setStyleSheet(
            "color: #c0392b; background-color: #fde8e8; padding: 10px; "
            "border: 2px solid #c0392b; border-radius: 4px; font-weight: bold;"
        )
        self.imu_warning.setWordWrap(True)
        self.imu_warning.hide()
        layout.addWidget(self.imu_warning)
        
        self.imu_confirm_cb = QCheckBox("I understand the heat cycling requirement — proceed with full IMU calibration")
        self.imu_confirm_cb.setStyleSheet("color: #c0392b; font-weight: bold;")
        self.imu_confirm_cb.hide()
        layout.addWidget(self.imu_confirm_cb)
        
        # Mild warning for Accelerometer Only (no thermal compensation)
        self.acc_only_warning = QLabel(
            "Note: Accelerometer Only calibration captures gyro biases at room temperature\n"
            "but does NOT perform thermal compensation. Gyro drift may increase with\n"
            "temperature changes. Madgwick filter will correct for residual drift."
        )
        self.acc_only_warning.setStyleSheet(
            "color: #e67e22; background-color: #fef5e7; padding: 8px; "
            "border: 1px solid #e67e22; border-radius: 4px;"
        )
        self.acc_only_warning.setWordWrap(True)
        self.acc_only_warning.hide()
        layout.addWidget(self.acc_only_warning)
        
        # Current values
        values_group = QGroupBox("Current IMU Values")
        values_layout = QGridLayout()
        values_layout.setHorizontalSpacing(15)
        
        # Gyro header
        values_layout.addWidget(QLabel("<b>Gyros (deg/s)</b>"), 0, 0, 1, 3)
        values_layout.addWidget(QLabel("<b>Accels (G)</b>"), 0, 3, 1, 3)
        
        # Row 1: Roll / X
        values_layout.addWidget(QLabel("Roll:"), 1, 0)
        self.gyro_roll = QLabel("0.0")
        self.gyro_roll.setStyleSheet("font-weight: bold; color: #3498db; font-size: 14px;")
        values_layout.addWidget(self.gyro_roll, 1, 1)
        
        self.gyro_roll_offset = QLabel("0.0")
        self.gyro_roll_offset.setStyleSheet("color: #666;")
        values_layout.addWidget(self.gyro_roll_offset, 1, 2)
        
        values_layout.addWidget(QLabel("X (L/R):"), 1, 3)
        self.acc_x = QLabel("0.00")
        self.acc_x.setStyleSheet("font-weight: bold; color: #3498db; font-size: 14px;")
        values_layout.addWidget(self.acc_x, 1, 4)
        
        self.acc_x_offset = QLabel("0.00")
        self.acc_x_offset.setStyleSheet("color: #666;")
        values_layout.addWidget(self.acc_x_offset, 1, 5)
        
        # Row 2: Pitch / Y
        values_layout.addWidget(QLabel("Pitch:"), 2, 0)
        self.gyro_pitch = QLabel("0.0")
        self.gyro_pitch.setStyleSheet("font-weight: bold; color: #2ecc71; font-size: 14px;")
        values_layout.addWidget(self.gyro_pitch, 2, 1)
        
        self.gyro_pitch_offset = QLabel("0.0")
        self.gyro_pitch_offset.setStyleSheet("color: #666;")
        values_layout.addWidget(self.gyro_pitch_offset, 2, 2)
        
        values_layout.addWidget(QLabel("Y (F/B):"), 2, 3)
        self.acc_y = QLabel("0.00")
        self.acc_y.setStyleSheet("font-weight: bold; color: #2ecc71; font-size: 14px;")
        values_layout.addWidget(self.acc_y, 2, 4)
        
        self.acc_y_offset = QLabel("0.00")
        self.acc_y_offset.setStyleSheet("color: #666;")
        values_layout.addWidget(self.acc_y_offset, 2, 5)
        
        # Row 3: Yaw / Z
        values_layout.addWidget(QLabel("Yaw:"), 3, 0)
        self.gyro_yaw = QLabel("0.0")
        self.gyro_yaw.setStyleSheet("font-weight: bold; color: #e74c3c; font-size: 14px;")
        values_layout.addWidget(self.gyro_yaw, 3, 1)
        
        self.gyro_yaw_offset = QLabel("0.0")
        self.gyro_yaw_offset.setStyleSheet("color: #666;")
        values_layout.addWidget(self.gyro_yaw_offset, 3, 2)
        
        values_layout.addWidget(QLabel("Z (D/U):"), 3, 3)
        self.acc_z = QLabel("0.00")
        self.acc_z.setStyleSheet("font-weight: bold; color: #e74c3c; font-size: 14px;")
        values_layout.addWidget(self.acc_z, 3, 4)
        
        self.acc_z_offset = QLabel("0.00")
        self.acc_z_offset.setStyleSheet("color: #666;")
        values_layout.addWidget(self.acc_z_offset, 3, 5)
        
        values_group.setLayout(values_layout)
        layout.addWidget(values_group)
        
        # Calibration progress
        progress_group = QGroupBox("Calibration Progress")
        progress_layout = QVBoxLayout()
        
        self.imu_status = QLabel("Waiting to start...")
        self.imu_status.setStyleSheet("padding: 5px; background-color: #f8f8f8; border-radius: 4px;")
        progress_layout.addWidget(self.imu_status)
        
        self.imu_progress = QProgressBar()
        self.imu_progress.setRange(0, 100)
        self.imu_progress.setValue(0)
        self.imu_progress.setFormat("%v%")
        progress_layout.addWidget(self.imu_progress)
        
        progress_group.setLayout(progress_layout)
        layout.addWidget(progress_group)
        
        # Buttons
        btn_layout = QHBoxLayout()
        
        self.imu_start_btn = QPushButton("Start Calibration")
        self.imu_start_btn.setStyleSheet("font-weight: bold; padding: 8px; background-color: #3498db; color: white;")
        btn_layout.addWidget(self.imu_start_btn)
        
        self.imu_reset_btn = QPushButton("Reset Offsets")
        self.imu_reset_btn.setStyleSheet("padding: 8px;")
        btn_layout.addWidget(self.imu_reset_btn)
        
        btn_layout.addStretch()
        layout.addLayout(btn_layout)
        layout.addStretch()
        
        return tab
    
    def _create_mag_tab(self):
        """Create magnetometer calibration tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Info
        info = QLabel(
            "Magnetometer Calibration\n"
            "Rotate the vehicle slowly through all axes.\n"
            "Click 'Start Calibration' to begin.\n"
            "The FC will collect data automatically."
        )
        info.setStyleSheet("color: #555; padding: 5px; border: 1px solid #ddd; border-radius: 4px;")
        info.setWordWrap(True)
        layout.addWidget(info)
        
        self.mag_cal_status = QLabel("Mag Calibration: Unknown")
        self.mag_cal_status.setStyleSheet("padding: 5px; background-color: #f39c12; color: white; font-weight: bold; border-radius: 4px;")
        layout.addWidget(self.mag_cal_status)
        
        # Current values
        values_group = QGroupBox("Current Magnetometer Values")
        values_layout = QGridLayout()
        
        values_layout.addWidget(QLabel("X:"), 0, 0)
        self.mag_x = QLabel("0")
        self.mag_x.setStyleSheet("font-weight: bold; color: #3498db; font-size: 14px;")
        values_layout.addWidget(self.mag_x, 0, 1)
        
        values_layout.addWidget(QLabel("Y:"), 0, 2)
        self.mag_y = QLabel("0")
        self.mag_y.setStyleSheet("font-weight: bold; color: #2ecc71; font-size: 14px;")
        values_layout.addWidget(self.mag_y, 0, 3)
        
        values_layout.addWidget(QLabel("Z:"), 0, 4)
        self.mag_z = QLabel("0")
        self.mag_z.setStyleSheet("font-weight: bold; color: #e74c3c; font-size: 14px;")
        values_layout.addWidget(self.mag_z, 0, 5)
        
        # Stored bias row
        values_layout.addWidget(QLabel("Bias:"), 1, 0)
        self.mag_bias_x = QLabel("0")
        self.mag_bias_x.setStyleSheet("color: #666;")
        values_layout.addWidget(self.mag_bias_x, 1, 1)
        self.mag_bias_y = QLabel("0")
        self.mag_bias_y.setStyleSheet("color: #666;")
        values_layout.addWidget(self.mag_bias_y, 1, 3)
        self.mag_bias_z = QLabel("0")
        self.mag_bias_z.setStyleSheet("color: #666;")
        values_layout.addWidget(self.mag_bias_z, 1, 5)
        
        values_group.setLayout(values_layout)
        layout.addWidget(values_group)
        
        # Octant sample bargraphs (mag calibration progress)
        octant_group = QGroupBox("Mag Calibration Octant Samples")
        octant_layout = QGridLayout()
        octant_layout.setHorizontalSpacing(10)
        octant_layout.setVerticalSpacing(4)
        
        # Column headers
        octant_layout.addWidget(QLabel("<b>Octant</b>"), 0, 0)
        octant_layout.addWidget(QLabel("<b>Samples</b>"), 0, 1)
        octant_layout.addWidget(QLabel("<b>Bar</b>"), 0, 2)
        octant_layout.addWidget(QLabel("<b>Octant</b>"), 0, 3)
        octant_layout.addWidget(QLabel("<b>Samples</b>"), 0, 4)
        octant_layout.addWidget(QLabel("<b>Bar</b>"), 0, 5)
        
        self.octant_bars = {}
        self.octant_labels = {}
        
        # 8 octants in 2 columns x 4 rows
        # Octants 1-4 in left column, 5-8 in right column
        for i in range(8):
            row = i % 4 + 1  # rows 1-4
            col_offset = 3 if i >= 4 else 0  # left or right column
            
            oct_num = i + 1
            name_label = QLabel(f"Q{oct_num}")
            name_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            octant_layout.addWidget(name_label, row, col_offset)
            
            value_label = QLabel("0")
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_label.setStyleSheet("font-weight: bold; font-size: 11px;")
            octant_layout.addWidget(value_label, row, col_offset + 1)
            self.octant_labels[i] = value_label
            
            bar = QProgressBar()
            bar.setRange(0, 55)  # Max 55 samples per octant (matching original GS)
            bar.setValue(0)
            bar.setFixedHeight(14)
            bar.setFixedWidth(120)
            bar.setTextVisible(False)
            octant_layout.addWidget(bar, row, col_offset + 2)
            self.octant_bars[i] = bar
        
        octant_group.setLayout(octant_layout)
        layout.addWidget(octant_group)
        
        # Calibration status
        status_group = QGroupBox("Calibration Status")
        status_layout = QVBoxLayout()
        
        self.mag_status = QLabel("Ready to calibrate")
        self.mag_status.setStyleSheet("padding: 5px; background-color: #f8f8f8; border-radius: 4px;")
        status_layout.addWidget(self.mag_status)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # Buttons
        btn_layout = QHBoxLayout()
        
        self.mag_start_btn = QPushButton("Start Calibration")
        self.mag_start_btn.setStyleSheet("font-weight: bold; padding: 8px; background-color: #3498db; color: white;")
        btn_layout.addWidget(self.mag_start_btn)
        
        self.mag_clear_btn = QPushButton("Clear Calibration")
        self.mag_clear_btn.setStyleSheet("padding: 8px;")
        btn_layout.addWidget(self.mag_clear_btn)
        
        layout.addLayout(btn_layout)
        layout.addStretch()
        
        return tab
    
    def _create_rc_tab(self):
        """Create RC calibration tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Info
        info = QLabel(
            "RC Calibration\n"
            "Move each RC stick to its extremes and center.\n"
            "Click 'Auto Calibrate' to find min/max/center automatically.\n"
            "Watch the values update as you move the sticks."
        )
        info.setStyleSheet("color: #555; padding: 5px; border: 1px solid #ddd; border-radius: 4px;")
        info.setWordWrap(True)
        layout.addWidget(info)
        
        self.rc_cal_status = QLabel("RC Calibration: Unknown")
        self.rc_cal_status.setStyleSheet("padding: 5px; background-color: #f39c12; color: white; font-weight: bold; border-radius: 4px;")
        layout.addWidget(self.rc_cal_status)
        
        # RC values
        rc_group = QGroupBox("RC Channel Values")
        rc_layout = QGridLayout()
        
        # Headers
        rc_layout.addWidget(QLabel("<b>Ch</b>"), 0, 0)
        rc_layout.addWidget(QLabel("<b>Function</b>"), 0, 1)
        rc_layout.addWidget(QLabel("<b>Raw</b>"), 0, 2)
        rc_layout.addWidget(QLabel("<b>Min</b>"), 0, 3)
        rc_layout.addWidget(QLabel("<b>Center</b>"), 0, 4)
        rc_layout.addWidget(QLabel("<b>Max</b>"), 0, 5)
        rc_layout.addWidget(QLabel("<b>µs</b>"), 0, 6)
        
        self.rc_raw_labels = {}
        self.rc_min_labels = {}
        self.rc_center_labels = {}
        self.rc_max_labels = {}
        self.rc_us_labels = {}
        
        channel_names = ["Throttle", "Roll", "Pitch", "Yaw", "Aux1", "Aux2", "Aux3", "Aux4"]
        
        for i in range(8):
            row = i + 1
            rc_layout.addWidget(QLabel(str(i+1)), row, 0)
            rc_layout.addWidget(QLabel(channel_names[i]), row, 1)
            
            raw_label = QLabel("0")
            raw_label.setStyleSheet("font-weight: bold;")
            rc_layout.addWidget(raw_label, row, 2)
            self.rc_raw_labels[i] = raw_label
            
            min_label = QLabel("0")
            rc_layout.addWidget(min_label, row, 3)
            self.rc_min_labels[i] = min_label
            
            center_label = QLabel("0")
            rc_layout.addWidget(center_label, row, 4)
            self.rc_center_labels[i] = center_label
            
            max_label = QLabel("0")
            rc_layout.addWidget(max_label, row, 5)
            self.rc_max_labels[i] = max_label
            
            us_label = QLabel("0")
            us_label.setStyleSheet("font-weight: bold; color: #3498db;")
            rc_layout.addWidget(us_label, row, 6)
            self.rc_us_labels[i] = us_label
        
        rc_group.setLayout(rc_layout)
        layout.addWidget(rc_group)
        
        # RC calibration status
        self.rc_status = QLabel("Ready")
        self.rc_status.setStyleSheet("padding: 5px; background-color: #f8f8f8; border-radius: 4px;")
        layout.addWidget(self.rc_status)
        
        # Buttons
        btn_layout = QHBoxLayout()
        self.rc_auto_btn = QPushButton("Auto Calibrate")
        self.rc_auto_btn.setStyleSheet("font-weight: bold; padding: 8px; background-color: #3498db; color: white;")
        btn_layout.addWidget(self.rc_auto_btn)
        
        self.rc_reset_btn = QPushButton("Reset to Defaults")
        self.rc_reset_btn.setStyleSheet("padding: 8px;")
        btn_layout.addWidget(self.rc_reset_btn)
        
        layout.addLayout(btn_layout)
        layout.addStretch()
        
        return tab
    
    def setup_connections(self):
        """Connect signals and slots"""
        from core.ack_handler import ack_handler
        
        # IMU
        self.imu_start_btn.clicked.connect(self.start_imu_calibration)
        self.imu_reset_btn.clicked.connect(self.reset_imu_offsets)
        self.imu_confirm_cb.stateChanged.connect(
            lambda: self.imu_start_btn.setEnabled(
                self.imu_confirm_cb.isChecked()
                if self.imu_cal_type_combo.currentData() == MiscCommand.CAL_IMU
                else True
            )
        )
        
        # Magnetometer
        self.mag_start_btn.clicked.connect(self.start_mag_calibration)
        self.mag_clear_btn.clicked.connect(self.clear_mag_calibration)
        
        # RC
        self.rc_auto_btn.clicked.connect(self.auto_rc_calibration)
        self.rc_reset_btn.clicked.connect(self.reset_rc_defaults)
        
        # Register buttons with ACK handler
        for btn in [self.imu_start_btn,
                   self.mag_start_btn,
                   self.rc_auto_btn]:
            btn.setProperty('original_text', btn.text())
    
    def _send_misc_command(self, command: MiscCommand, param: int = 0):
        """Send a MISC command to the FC"""
        if not self.parent_window or not hasattr(self.parent_window, 'send_request'):
            self._log("❌ Cannot send command - no connection")
            return
        
        # Misc packet structure: tag=52, a1=command, a2=param
        self.parent_window.send_request(PacketTag.MISC, command, param)
        command_names = {
            MiscCommand.CAL_IMU: "CAL_IMU",
            MiscCommand.CAL_MAG: "CAL_MAG",
            MiscCommand.CAL_ACC: "CAL_ACC",
            MiscCommand.CAL_GYRO: "CAL_GYRO",
            MiscCommand.LB: "LB",
            MiscCommand.BB_DUMP: "BB_DUMP",
            MiscCommand.GPS_PASS_THRU: "GPS_PASS_THRU",
            MiscCommand.BOOTLOADER: "BOOTLOADER",
        }
        self._log(f"📤 Sent MISC command: {command_names.get(command, str(command))} (param={param})")
    
    def _update_cal_status(self, label, calibrated):
        if calibrated:
            label.setText("✅ Calibrated")
            label.setStyleSheet("padding: 5px; background-color: #27ae60; color: white; font-weight: bold; border-radius: 4px;")
        else:
            label.setText("❌ Not Calibrated")
            label.setStyleSheet("padding: 5px; background-color: #f39c12; color: white; font-weight: bold; border-radius: 4px;")
    
    def update_from_telemetry(self):
        """Update values from telemetry"""
        cal_data = data_manager.get_calibration_data()
        cd_available = cal_data and 'cal_data' in cal_data and len(cal_data['cal_data']) >= 31
        
        # Get flags from cal_data or flight_data
        flags = cal_data.get('flags', []) if cal_data else []
        flight_data = data_manager.get_flight_data()
        if not flags and flight_data and hasattr(flight_data, 'flag_bits') and flight_data.flag_bits:
            flags = []
            fb = flight_data.flag_bits
            for byte_idx in range(6):
                byte_val = 0
                for bit_idx in range(8):
                    bit_pos = byte_idx * 8 + bit_idx
                    if bit_pos < len(fb) and fb[bit_pos]:
                        byte_val |= (1 << bit_idx)
                flags.append(byte_val)
        
        # Update calibration status labels from flag bits
        if len(flags) >= 6:
            imu_cal = bool(flags[5] & 32)   # bit 45
            acc_cal = bool(flags[5] & 128)  # bit 47
            mag_cal = bool(flags[5] & 4)    # bit 42
            mag_active = bool(flags[4] & 64) if len(flags) >= 5 else False  # bit 38
            self._update_cal_status(self.imu_cal_status, imu_cal)
            self._update_cal_status(self.mag_cal_status, mag_cal)
            if self.mag_calibration_requested and not mag_active:
                self._log("⚠️ Mag sensor not active - calibration cannot proceed")
            self.rc_cal_status.setText("Ready")
            self.rc_cal_status.setStyleSheet("padding: 5px; background-color: #27ae60; color: white; font-weight: bold; border-radius: 4px;")
        else:
            self._update_cal_status(self.imu_cal_status, False)
            self._update_cal_status(self.mag_cal_status, False)
            self.rc_cal_status.setText("Unknown")
            self.rc_cal_status.setStyleSheet("padding: 5px; background-color: #95a5a6; color: white; font-weight: bold; border-radius: 4px;")
        
        # Populate stored calibration values from cal_data
        if cd_available:
            cd = cal_data['cal_data']
            # Gyro biases: cal_data[1]=X, [7]=Y, [13]=Z → deg/s = value/1000
            self.gyro_roll_offset.setText(f"{cd[1] / 1000.0:.2f}")
            self.gyro_pitch_offset.setText(f"{cd[7] / 1000.0:.2f}")
            self.gyro_yaw_offset.setText(f"{cd[13] / 1000.0:.2f}")
            # Accel biases: cal_data[3]=X, [9]=Y, [15]=Z → Gs = value/1000
            self.acc_x_offset.setText(f"{cd[3] / 1000.0:.3f}")
            self.acc_y_offset.setText(f"{cd[9] / 1000.0:.3f}")
            self.acc_z_offset.setText(f"{cd[15] / 1000.0:.3f}")
            # Mag biases: cal_data[5]=X, [11]=Y, [17]=Z → raw * 1000
            self.mag_bias_x.setText(str(cd[5]))
            self.mag_bias_y.setText(str(cd[11]))
            self.mag_bias_z.setText(str(cd[17]))
        
        # Update octant bargraphs from calibration packet data
        if cd_available:
            for i in range(8):
                idx = 23 + i
                val = cal_data['cal_data'][idx]
                if val < 0:
                    val = 0
                if val > 55:
                    val = 55
                self.octant_bars[i].setValue(val)
                self.octant_labels[i].setText(str(val))
            
            if self.mag_calibration_requested:
                oct_vals = [cal_data['cal_data'][23 + i] for i in range(8)]
                mm_val = cal_data['cal_data'][31] if len(cal_data['cal_data']) > 31 else 0
                self._log(f"Octants: {oct_vals}  total(mm)={mm_val}")
            
            if self.mag_calibration_requested:
                cal_flags = cal_data.get('flags', [])
                if len(cal_flags) >= 6 and (cal_flags[5] & 4):
                    self._finish_mag_calibration()
                elif self.mag_calibration_start_time and self.mag_calibration_start_time.hasExpired(self.mag_calibration_timeout):
                    self.mag_status.setText("⚠️ Mag calibration timed out (2 min)")
                    self.mag_status.setStyleSheet("padding: 5px; background-color: #f8d7da; color: #721c24; border-radius: 4px;")
                    self.mag_calibration_requested = False
                    self.mag_start_btn.setEnabled(True)
        
        # Check IMU calibration completion via GyroCal flag (bit 5 of byte 5)
        if self.imu_calibration_requested:
            if len(flags) >= 6 and (flags[5] & 32):  # GyroCal
                self._finish_imu_calibration()
            elif self.imu_calibration_start_time and self.imu_calibration_start_time.hasExpired(self.imu_calibration_timeout):
                self.imu_status.setText("⚠️ IMU calibration timed out")
                self.imu_status.setStyleSheet("padding: 5px; background-color: #f8d7da; color: #721c24; border-radius: 4px;")
                self.imu_calibration_requested = False
                self.imu_calibrating = False
                self.imu_start_btn.setEnabled(True)
                self.imu_progress.setRange(0, 100)
                self.imu_progress.setValue(0)
        
        if not flight_data:
            return
        
        # Update gyro values (convert rad/s to deg/s)
        gyro_roll = flight_data.rate_roll * 57.2958
        gyro_pitch = flight_data.rate_pitch * 57.2958
        gyro_yaw = flight_data.rate_yaw * 57.2958
        
        self.gyro_roll.setText(f"{gyro_roll:.1f}")
        self.gyro_pitch.setText(f"{gyro_pitch:.1f}")
        self.gyro_yaw.setText(f"{gyro_yaw:.1f}")
        
        # Update accelerometer values
        self.acc_x.setText(f"{flight_data.acc_lr:.2f}")
        self.acc_y.setText(f"{flight_data.acc_fb:.2f}")
        self.acc_z.setText(f"{flight_data.acc_du:.2f}")
        
        # Update mag display from calibration data if available
        if hasattr(flight_data, 'pwm_diag') and len(flight_data.pwm_diag) >= 3:
            self.mag_x.setText(str(flight_data.pwm_diag[0]))
            self.mag_y.setText(str(flight_data.pwm_diag[1]))
            self.mag_z.setText(str(flight_data.pwm_diag[2]))
        
        # Update RC values from telemetry
        rc_raw = getattr(flight_data, 'rc_raw', [])
        if rc_raw:
            for i in range(min(len(rc_raw), 8)):
                us = rc_raw[i] if rc_raw[i] > 0 else 0
                self.rc_raw_labels[i].setText(str(us))
                self.rc_us_labels[i].setText(str(us))
    
    def _on_imu_type_changed(self, idx):
        """Show/hide heat-cycle warning for full IMU cal"""
        cal_type = self.imu_cal_type_combo.itemData(idx)
        is_full_imu = (cal_type == MiscCommand.CAL_IMU)
        self.imu_warning.setVisible(is_full_imu)
        self.imu_confirm_cb.setVisible(is_full_imu)
        self.imu_confirm_cb.setChecked(False)
        is_acc_only = (cal_type == MiscCommand.CAL_ACC)
        self.acc_only_warning.setVisible(is_acc_only)
        self.imu_start_btn.setEnabled(self.imu_confirm_cb.isChecked() if is_full_imu else True)

    def start_imu_calibration(self):
        """Start IMU calibration based on selected type"""
        if self.imu_calibrating:
            return
        
        # Get selected calibration type
        self.current_imu_calibration_type = self.imu_cal_type_combo.currentData()
        if self.current_imu_calibration_type == MiscCommand.CAL_IMU and not self.imu_confirm_cb.isChecked():
            self.status_label.setText("❌ Please confirm heat cycling requirement to proceed with full IMU calibration")
            return
        
        # Map to display name
        type_names = {
            MiscCommand.CAL_IMU: "IMU (Gyro + Accel)",
            MiscCommand.CAL_ACC: "Accelerometer Only",
            MiscCommand.CAL_GYRO: "Gyro Only",
        }
        type_name = type_names.get(self.current_imu_calibration_type, "Unknown")
        
        self.imu_calibrating = True
        self.imu_progress.setRange(0, 0)  # indeterminate
        self.imu_progress.setValue(0)
        self.imu_status.setText(f"⏳ {type_name} calibration in progress... (FC is calibrating)")
        self.imu_status.setStyleSheet("padding: 5px; background-color: #fff3cd; color: #856404; border-radius: 4px;")
        self.imu_start_btn.setEnabled(False)
        self.status_label.setText(f"{type_name} calibration in progress...")
        
        # Send MISC command to FC to start calibration
        self._send_misc_command(self.current_imu_calibration_type)
        self.imu_calibration_requested = True
        
        # Start timeout tracking (CAL_IMU with heat soak can take ~40s)
        self.imu_calibration_start_time = QElapsedTimer()
        self.imu_calibration_start_time.start()
        self.imu_calibration_timeout = 60000  # 60 second safety timeout
    
    def _finish_imu_calibration(self):
        """Finish IMU calibration — mark complete, enable button"""
        self.imu_calibrating = False
        self.imu_calibration_requested = False
        self.imu_start_btn.setEnabled(True)
        self.imu_progress.setRange(0, 100)
        self.imu_progress.setValue(100)
        
        type_names = {
            MiscCommand.CAL_IMU: "IMU (Gyro + Accel)",
            MiscCommand.CAL_ACC: "Accelerometer",
            MiscCommand.CAL_GYRO: "Gyro",
        }
        type_name = type_names.get(self.current_imu_calibration_type, "Unknown")
        self.imu_status.setText(f"✅ {type_name} calibration complete!")
        self.imu_status.setStyleSheet("padding: 5px; background-color: #d4edda; color: #155724; border-radius: 4px;")
        self.status_label.setText(f"✅ {type_name} calibration complete!")
    
    def reset_imu_offsets(self):
        """Reset IMU offsets to zero"""
        self.gyro_roll_offset.setText("0.00")
        self.gyro_pitch_offset.setText("0.00")
        self.gyro_yaw_offset.setText("0.00")
        self.acc_x_offset.setText("0.00")
        self.acc_y_offset.setText("0.00")
        self.acc_z_offset.setText("0.00")
        self.imu_status.setText("IMU offsets reset to zero")
        self.imu_status.setStyleSheet("padding: 5px; background-color: #f8f8f8; border-radius: 4px;")
        self.imu_progress.setValue(0)
        self.status_label.setText("IMU offsets reset")
    
    def start_mag_calibration(self):
        """Start magnetometer calibration"""
        if self.mag_calibration_requested:
            return
        
        self.mag_status.setText("Magnetometer calibration in progress... Rotate vehicle slowly through all axes.")
        self.mag_status.setStyleSheet("padding: 5px; background-color: #fff3cd; color: #856404; border-radius: 4px;")
        self.mag_start_btn.setEnabled(False)
        self.status_label.setText("Magnetometer calibration in progress...")
        
        # Reset octant bars
        for bar in self.octant_bars.values():
            bar.setValue(0)
        for label in self.octant_labels.values():
            label.setText("0")
        
        # Send MISC command to FC to start magnetometer calibration
        self._send_misc_command(MiscCommand.CAL_MAG)
        self.mag_calibration_requested = True
        self.mag_calibration_start_time = QElapsedTimer()
        self.mag_calibration_start_time.start()
        self.mag_calibration_timeout = 120000  # 2 minute safety timeout
    
    def _finish_mag_calibration(self):
        """Finish magnetometer calibration"""
        self.mag_calibration_requested = False
        self.mag_start_btn.setEnabled(True)
        self.mag_status.setText("✅ Magnetometer calibration complete!")
        self.mag_status.setStyleSheet("padding: 5px; background-color: #d4edda; color: #155724; border-radius: 4px;")
        self.status_label.setText("✅ Magnetometer calibration complete!")
    
    def clear_mag_calibration(self):
        """Clear magnetometer calibration"""
        self.mag_status.setText("Calibration cleared")
        self.mag_status.setStyleSheet("padding: 5px; background-color: #f8f8f8; border-radius: 4px;")
        self.status_label.setText("Magnetometer calibration cleared")
    
    def auto_rc_calibration(self):
        """Auto-calibrate RC channels from live telemetry"""
        flight_data = data_manager.get_flight_data()
        rc_raw = getattr(flight_data, 'rc_raw', []) if flight_data else []
        
        if not rc_raw:
            self.rc_status.setText("No RC data available - connect receiver and move sticks")
            self.rc_status.setStyleSheet("padding: 5px; background-color: #fff3cd; color: #856404; border-radius: 4px;")
            return
        
        self.rc_status.setText("RC data captured from telemetry...")
        self.rc_status.setStyleSheet("padding: 5px; background-color: #fff3cd; color: #856404; border-radius: 4px;")
        
        for i in range(min(len(rc_raw), 8)):
            raw = rc_raw[i]
            min_val = int(raw * 0.9)
            max_val = int(raw * 1.1)
            center = raw
            
            self.rc_raw_labels[i].setText(str(raw))
            self.rc_min_labels[i].setText(str(min_val))
            self.rc_center_labels[i].setText(str(center))
            self.rc_max_labels[i].setText(str(max_val))
            
            us = raw if raw > 0 else 1500
            self.rc_us_labels[i].setText(str(us))
        
        self.rc_status.setText("✅ RC data captured - adjust min/max/center as needed")
        self.rc_status.setStyleSheet("padding: 5px; background-color: #d4edda; color: #155724; border-radius: 4px;")
        self.status_label.setText("RC data captured from telemetry")
    
    def reset_rc_defaults(self):
        """Reset RC to default values"""
        for i in range(8):
            self.rc_raw_labels[i].setText("0")
            self.rc_min_labels[i].setText("0")
            self.rc_center_labels[i].setText("1500")
            self.rc_max_labels[i].setText("0")
            self.rc_us_labels[i].setText("0")
        self.rc_status.setText("RC defaults reset")
        self.rc_status.setStyleSheet("padding: 5px; background-color: #f8f8f8; border-radius: 4px;")
        self.status_label.setText("RC defaults reset")
    
    def closeEvent(self, event):
        """Clean up on close"""
        if self.telemetry_timer:
            self.telemetry_timer.stop()
        event.accept()
