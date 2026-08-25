# ui/calibration_window.py
"""
UAVX Calibration Window
IMU (Gyro + Accel), Magnetometer, and RC calibration
"""

import sys
import os
import math
from typing import Optional

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol_constants import *
from protocol_enums import PacketTag, MiscCommand
from packet_parser import *
from core.data_manager import data_manager
from orientation_solver import classify as ori_classify, next_knobs as ori_next_knobs

ORI_PROBE_TIMEOUT_TICKS = 150  # 15 s to complete the maneuver
RAD_TO_DEG = 57.29578          # display edge only: FC angles/rates are radians
DEG_TO_RAD = math.pi / 180.0
ORI_TRIP_ANGLE_RAD = 30.0 * DEG_TO_RAD  # probe trip threshold, FC units
GRAVITY_MPS_S = 9.80665        # FC publishes accel as m/s^2 (misctypes.h)


class CalibrationWindow(QMainWindow):
    """Calibration window for IMU, mag, and RC"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.setWindowTitle("UAVX Calibration")
        self.setMinimumSize(660, 770)
        
        # IMU calibration state
        self.imu_calibrating = False
        self.imu_calibration_requested = False
        self.imu_calibration_start_time = None
        self.imu_calibration_timeout = 60000
        self.current_imu_calibration_type = MiscCommand.CAL_IMU
        self._mag_active = False
        self._imu_active = False
        
        # Mag calibration state
        self.mag_points = []
        self.mag_calibration_requested = False
        self.mag_calibration_start_time = None
        self.mag_calibration_timeout = 120000  # 2 minute safety timeout
        
        # RC calibration state
        self.rc_calibrating = False
        self.rc_samples = {}

        # Sensor orientation probe state
        self._ori_observations = {}   # probe key -> (channel/axis, sign)
        self._ori_pending = None      # probe key being captured
        self._ori_ticks = 0           # ticks elapsed in current capture
        self._ori_heading0 = None     # heading when yaw probe started
        
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

        # Tab 3: Sensor Orientation (porting aid)
        self.ori_tab = self._create_orientation_tab()
        self.tab_widget.addTab(self.ori_tab, "Orientation")

        # Tab 4: RC Calibration
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

        self.imu_chip = QLabel("IMU Chip: waiting for calibration data...")
        self.imu_chip.setStyleSheet("color: #888; font-weight: bold;")
        layout.addWidget(self.imu_chip)
        
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
        values_layout.addWidget(QLabel("<b>Accels (m/s²)</b>"), 0, 3, 1, 3)
        
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
        
        # Temp row - useful during temperature-cycled calibration
        values_layout.addWidget(QLabel("IMU Temp:"), 4, 0)
        self.cal_imu_temp = QLabel("0.0°C")
        self.cal_imu_temp.setStyleSheet("font-weight: bold; font-size: 14px;")
        values_layout.addWidget(self.cal_imu_temp, 4, 1, 1, 2)
        
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

        self.mag_chip = QLabel("Mag Chip: waiting for calibration data...")
        self.mag_chip.setStyleSheet("color: #888; font-weight: bold;")
        layout.addWidget(self.mag_chip)
        
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
    
    def _create_orientation_tab(self):
        """Create sensor orientation porting tab (see wiki report section 8)"""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        info = QLabel(
            "Sensor Orientation Porting\n"
            "Determines IMUQuadrant / IMUSensorFlip for a new target board.\n"
            "With the board level and still, run each probe while the FC is "
            "disarmed.\nEach probe trips when its angle (heading for yaw) "
            "moves past 30 deg; hold until it trips."
        )
        info.setStyleSheet("color: #555; padding: 5px; border: 1px solid #ddd; border-radius: 4px;")
        info.setWordWrap(True)
        layout.addWidget(info)

        # Currently-flashed orientation (from calibration packet tag 62)
        cur_group = QGroupBox("Currently Flashed (from FC)")
        cur_layout = QGridLayout()
        self.ori_cur_imu_q = QLabel("--")
        self.ori_cur_flip = QLabel("--")
        self.ori_cur_mag_q = QLabel("--")
        for row, (name, lbl) in enumerate([
                ("IMU Quadrant", self.ori_cur_imu_q),
                ("IMU Sensor Flip", self.ori_cur_flip),
                ("Mag Quadrant", self.ori_cur_mag_q)]):
            cur_layout.addWidget(QLabel(name), row, 0)
            lbl.setStyleSheet("font-weight: bold;")
            cur_layout.addWidget(lbl, row, 1)
        cur_group.setLayout(cur_layout)
        layout.addWidget(cur_group)

        self.ori_live = QLabel("angles: p --  r --  h --")
        self.ori_live.setStyleSheet(
            "color: #888; font-family: Monospace; padding: 2px;")
        layout.addWidget(self.ori_live)

        # Probe buttons + observations
        probe_group = QGroupBox("Probes (board level, disarmed)")
        probe_layout = QGridLayout()

        self.ori_probe_btns = {}
        self.ori_obs_lbls = {}
        for row, (key, label) in enumerate([
                ("yaw", "Yaw Right"), ("pitch", "Pitch Up"),
                ("roll", "Roll Right")]):
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, k=key: self._start_ori_probe(k))
            probe_layout.addWidget(btn, row, 0)
            self.ori_probe_btns[key] = btn

            obs = QLabel("not run")
            obs.setStyleSheet("color: #777;")
            probe_layout.addWidget(obs, row, 1)
            self.ori_obs_lbls[key] = obs

        self.ori_reset_btn = QPushButton("Clear Probes")
        self.ori_reset_btn.clicked.connect(self._reset_ori_probes)
        probe_layout.addWidget(self.ori_reset_btn, 3, 0)

        probe_group.setLayout(probe_layout)
        layout.addWidget(probe_group)

        # Verdict
        verdict_group = QGroupBox("Result")
        verdict_layout = QVBoxLayout()
        self.ori_verdict = QLabel("Run all three probes.")
        self.ori_verdict.setWordWrap(True)
        self.ori_verdict.setStyleSheet("font-weight: bold; padding: 4px;")
        verdict_layout.addWidget(self.ori_verdict)

        self.ori_inc_lines = QLabel("")
        self.ori_inc_lines.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.ori_inc_lines.setFont(QFont("Monospace"))
        verdict_layout.addWidget(self.ori_inc_lines)
        verdict_group.setLayout(verdict_layout)
        layout.addWidget(verdict_group)

        layout.addStretch()
        return tab

    def _start_ori_probe(self, key):
        """Begin capture for one probe maneuver"""
        if self._ori_pending is not None:
            return
        self._ori_pending = key
        self._ori_ticks = 0
        self.ori_probe_btns[key].setText("capturing...")

    def _end_ori_probe(self, key):
        self.ori_probe_btns[key].setText(
            dict(yaw="Yaw Right", pitch="Pitch Up",
                 roll="Roll Right")[key])
        self._ori_pending = None
        self._ori_ticks = 0

    def _record_ori_obs(self, key, channel, sign):
        """Store one observation, refresh its label, evaluate when complete"""
        # channel follows the solver encoding: 'rate_yaw' for spin probes,
        # bare 'pitch'/'roll' for tilt probes
        names = {'pitch': 'pitch', 'roll': 'roll', 'yaw': 'yaw',
                 'rate_pitch': 'pitch', 'rate_roll': 'roll',
                 'rate_yaw': 'yaw'}
        self._ori_observations[key] = (channel, sign)
        lbl = self.ori_obs_lbls[key]
        lbl.setText("%s %s" % (names[channel], '+' if sign > 0 else '-'))
        lbl.setStyleSheet("color: #27ae60; font-weight: bold;")
        if set(self._ori_observations) == {'yaw', 'pitch', 'roll'}:
            self._evaluate_ori_probes()

    def _reset_ori_probes(self):
        self._ori_observations = {}
        self._ori_pending = None
        self._ori_ticks = 0
        for key in self.ori_probe_btns:
            self.ori_probe_btns[key].setText(
                dict(yaw="Yaw Right", pitch="Pitch Up",
                     roll="Roll Right")[key])
            self.ori_obs_lbls[key].setText("not run")
            self.ori_obs_lbls[key].setStyleSheet("color: #777;")
        self.ori_verdict.setText("Run all three probes.")
        self.ori_inc_lines.setText("")

    def _evaluate_ori_probes(self):
        """Classify observations against currently-flashed orientation"""
        cal_data = data_manager.get_calibration_data() or {}
        q_cur = cal_data.get('imu_quadrant')
        flip_cur = cal_data.get('imu_flip')
        if q_cur is None or flip_cur is None:
            self.ori_verdict.setText(
                "FC firmware does not stream orientation "
                "(legacy calibration packet).")
            return

        result = ori_classify(self._ori_observations['yaw'],
                              self._ori_observations['pitch'],
                              self._ori_observations['roll'])
        q_new, flip_new, note = ori_next_knobs(result, q_cur, flip_cur)

        if result['status'] == 'pass':
            verdict = ("Orientation verified - matches flashed "
                       "Q=%d flip=%s." % (q_cur, flip_cur))
        elif q_new == q_cur and flip_new == flip_cur:
            verdict = note
        else:
            verdict = ("%s\nSuggested change from flashed values: "
                       "IMUQuadrant %d -> %d, IMUSensorFlip %s -> %s"
                       % (note, q_cur, q_new, flip_cur, flip_new))
        self.ori_verdict.setText(verdict)

        quad_names = {0: 'CW0_DEG', 1: 'CW90_DEG', 2: 'CW180_DEG',
                      3: 'CW270_DEG'}
        self.ori_inc_lines.setText(
            "const uint8 IMUQuadrant = %d; // %s\n"
            "const boolean IMUSensorFlip = %s;" % (
                q_new, quad_names[q_new],
                'true' if flip_new else 'false'))

    def _orientation_tick(self, cal_data, flight_data):
        """10 Hz tick: refresh current-values labels and capture probes"""
        if cal_data and 'imu_quadrant' in cal_data:
            self.ori_cur_imu_q.setText(str(cal_data['imu_quadrant']))
            self.ori_cur_flip.setText(
                'true' if cal_data['imu_flip'] else 'false')
            self.ori_cur_mag_q.setText(str(cal_data.get('mag_quadrant')))
        elif cal_data:
            self.ori_cur_imu_q.setText("legacy FW")

        if self._ori_pending is None or not flight_data:
            return

        # FC-native radians end to end; degrees appear only in the readout
        ap = flight_data.angle_pitch
        ar = flight_data.angle_roll
        hd = flight_data.heading

        # live readout: shows exactly what the GCS is receiving
        self.ori_live.setText("angles: p %+.1f  r %+.1f  h %.1f" % (
            ap * RAD_TO_DEG, ar * RAD_TO_DEG, hd * RAD_TO_DEG))

        key = self._ori_pending
        self._ori_ticks += 1

        if key == 'yaw':
            # baseline on the first tick so a stale start value can't bite;
            # then heading change past the threshold, shortest way round
            if self._ori_ticks == 1:
                self._ori_heading0 = hd
                return
            d = ((hd - self._ori_heading0 + math.pi)
                 % (2.0 * math.pi)) - math.pi
            if abs(d) >= ORI_TRIP_ANGLE_RAD:
                sign = 1 if d > 0 else -1
                self._record_ori_obs(key, 'rate_yaw', sign)
                self._end_ori_probe(key)
                return
        else:
            # tilt trips on whichever displayed angle passes the threshold
            if max(abs(ap), abs(ar)) >= ORI_TRIP_ANGLE_RAD:
                axis = 'pitch' if abs(ap) >= abs(ar) else 'roll'
                ang = ap if axis == 'pitch' else ar
                self._record_ori_obs(key, axis, 1 if ang > 0 else -1)
                self._end_ori_probe(key)
                return

        if self._ori_ticks >= ORI_PROBE_TIMEOUT_TICKS:
            self._end_ori_probe(key)
            self.ori_obs_lbls[key].setText(
                "no trip - hold maneuver past %d deg"
                % round(ORI_TRIP_ANGLE_RAD * RAD_TO_DEG))

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
            MiscCommand.GPS_PASS_THRU_UNUSED: "GPS_PASS_THRU",
            MiscCommand.BOOTLOADER: "BOOTLOADER",
        }
        self._log(f"📤 Sent MISC command: {command_names.get(command, str(command))} (param={param})")
    
    @staticmethod
    def _imu_chip_name(v):
        if v is None or not v:
            return "IMU Chip: --"
        names = {0x47: "ICM-42688-P", 0x42: "ICM-42605", 0x24: "BMI270"}
        label = names.get(int(v), f"unknown (0x{int(v):02x})")
        return f"IMU Chip: {label}"

    @staticmethod
    def _mag_chip_name(v):
        if v is None or not v:
            return "Mag Chip: --"
        label = "HMC5883/HMC5983" if int(v) == 0x48 else f"unknown (0x{int(v):02x})"
        return f"Mag Chip: {label}"

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
        cd_available = cal_data and 'rate_bias' in cal_data
        
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
        # FC main.h Flags struct byte 5: ThrottleOpen:0, MagnetometerCalibrated:1,
        #   RCMapFail:2, NewAltitudeValue:3, IMUCal:4, FenceAlarm:5
        if len(flags) >= 6:
            imu_cal = bool(flags[5] & 16)   # bit 4 = IMUCal
            mag_cal = bool(flags[5] & 2)    # bit 1 = MagnetometerCalibrated
            mag_active = bool(flags[4] & 64) if len(flags) >= 5 else False  # bit 38 = MagnetometerActive
            self._mag_active = mag_active
            self._imu_active = bool(flags[4] & 32) if len(flags) >= 5 else False  # bit 37 = IMUActive
            self._update_cal_status(self.imu_cal_status, imu_cal)
            self._update_cal_status(self.mag_cal_status, mag_cal)
            if self.mag_calibration_requested and not mag_active:
                self._log("⚠️ Mag sensor not active - calibration cannot proceed")
            self.rc_cal_status.setText("Ready")
            self.rc_cal_status.setStyleSheet("padding: 5px; background-color: #27ae60; color: white; font-weight: bold; border-radius: 4px;")
        else:
            self._mag_active = False
            self._imu_active = False
            self._update_cal_status(self.imu_cal_status, False)
            self._update_cal_status(self.mag_cal_status, False)
            self.rc_cal_status.setText("Unknown")
            self.rc_cal_status.setStyleSheet("padding: 5px; background-color: #95a5a6; color: white; font-weight: bold; border-radius: 4px;")

        if cal_data:
            self.imu_chip.setText(self._imu_chip_name(cal_data.get('imu_id')))
            self.imu_chip.setStyleSheet("color: #555; font-weight: bold;")
            self.mag_chip.setText(self._mag_chip_name(cal_data.get('mag_id')))
            self.mag_chip.setStyleSheet("color: #555; font-weight: bold;")

        # Orientation tab: current values + probe capture
        self._orientation_tick(cal_data, flight_data)
        
        # Populate stored calibration values from cal_data
        if cd_available:
            # FC native units: gyro/acc biases in raw counts, acc scale
            # dimensionless, mag bias native sensor units (f32 on wire)
            rb = cal_data['rate_bias']
            ab = cal_data['acc_bias']
            asc = cal_data['acc_scale']
            mb = cal_data['mag_bias']
            # mirrors the FC's INCLUDE_ICM426XX gate (imu.h): ICM 2048 LSB/g,
            # every other family 8192
            imu_1g = 2048.0 if cal_data.get('imu_id') in (0x47, 0x42) else 8192.0
            gyro_to_dps = RATE_GYRO_SCALE * RAD_TO_DEG

            self.gyro_roll_offset.setText(f"{rb[0] * gyro_to_dps:.2f}")
            self.gyro_pitch_offset.setText(f"{rb[1] * gyro_to_dps:.2f}")
            self.gyro_yaw_offset.setText(f"{rb[2] * gyro_to_dps:.2f}")
            self.acc_x_offset.setText(f"{ab[0] * asc[0] / imu_1g * GRAVITY_MPS_S:.3f}")
            self.acc_y_offset.setText(f"{ab[1] * asc[1] / imu_1g * GRAVITY_MPS_S:.3f}")
            self.acc_z_offset.setText(f"{ab[2] * asc[2] / imu_1g * GRAVITY_MPS_S:.3f}")
            self.mag_bias_x.setText(str(int(round(mb[0]))))
            self.mag_bias_y.setText(str(int(round(mb[1]))))
            self.mag_bias_z.setText(str(int(round(mb[2]))))

            # Update octant bargraphs from calibration packet data
            octants = cal_data['octants']
            for i in range(8):
                val = min(55, max(0, octants[i]))
                self.octant_bars[i].setValue(val)
                self.octant_labels[i].setText(str(val))

            if self.mag_calibration_requested:
                mm_val = cal_data.get('mm', 0)
                self._log(f"Octants: {octants}  total(mm)={mm_val}")
            
            if self.mag_calibration_requested:
                cal_flags = cal_data.get('flags', [])
                if len(cal_flags) >= 6 and (cal_flags[5] & 2):  # bit 1 = MagnetometerCalibrated
                    self._finish_mag_calibration()
                elif self.mag_calibration_start_time and self.mag_calibration_start_time.hasExpired(self.mag_calibration_timeout):
                    self.mag_status.setText("⚠️ Mag calibration timed out (2 min)")
                    self.mag_status.setStyleSheet("padding: 5px; background-color: #f8d7da; color: #721c24; border-radius: 4px;")
                    self.mag_calibration_requested = False
                    self.mag_start_btn.setEnabled(True)
        
        # Check IMU calibration completion via IMUCal flag (bit 4 of byte 5)
        if self.imu_calibration_requested:
            if len(flags) >= 6 and (flags[5] & 16):  # bit 4 = IMUCal
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
        
        # Update gyro values - FC rad/s, display deg/s
        self.gyro_roll.setText(f"{flight_data.rate_roll * RAD_TO_DEG:.1f}")
        self.gyro_pitch.setText(f"{flight_data.rate_pitch * RAD_TO_DEG:.1f}")
        self.gyro_yaw.setText(f"{flight_data.rate_yaw * RAD_TO_DEG:.1f}")

        # Update accelerometer values - FC-native m/s^2, no conversion
        self.acc_x.setText(f"{flight_data.acc_lr:.2f}")
        self.acc_y.setText(f"{flight_data.acc_fb:.2f}")
        self.acc_z.setText(f"{flight_data.acc_du:.2f}")

        # Live IMU temperature for temperature-cycled calibration
        if hasattr(self, 'cal_imu_temp'):
            self.cal_imu_temp.setText(f"{getattr(flight_data, 'mpu_temp', 0.0):.1f}°C")

        # Live magnetometer counts from the calibration packet
        if cal_data and 'mag_live' in cal_data:
            ml = cal_data['mag_live']
            self.mag_x.setText(str(ml[0]))
            self.mag_y.setText(str(ml[1]))
            self.mag_z.setText(str(ml[2]))
        
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
        
        cal_data = data_manager.get_calibration_data()
        imu_id = cal_data.get('imu_id') if cal_data else None
        if not self._imu_active or (imu_id is not None and imu_id == 0):
            self.imu_status.setText("This board has no working IMU - calibration is not available.")
            self.imu_status.setStyleSheet("padding: 5px; background-color: #f8d7da; color: #721c24; border-radius: 4px;")
            self._log("⚠️ No IMU detected - refusing to calibrate")
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
        
        cal_data = data_manager.get_calibration_data()
        mag_id = cal_data.get('mag_id') if cal_data else None
        no_mag = not self._mag_active or (mag_id is not None and mag_id == 0)
        if no_mag:
            self.mag_status.setText("This board has no magnetometer fitted - calibration is not available.")
            self.mag_status.setStyleSheet("padding: 5px; background-color: #f8d7da; color: #721c24; border-radius: 4px;")
            self.status_label.setText("No magnetometer fitted - calibration not available")
            self._log("⚠️ No magnetometer found - refusing to calibrate")
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
