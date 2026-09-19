# ui/parameter_window.py - Refactored with enums, no magic numbers
"""
UAVX Parameter Editor Window
"""

import sys
import os
from typing import Optional, Dict, List, Tuple
import time
import math
import re


from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol_enums import (
    ParamIndex,
    PacketTag,
    MiscCommand,
    RC_MAPPING_NAMES,
    RCControl,
    AirframeType,
    ESCType,
    RxType,
    ArmingMode,

    RX_TYPE_NAMES,
    RF_TYPE_NAMES,
    AS_SENSOR_TYPE_NAMES,
    IMU_FILTER_NAMES,
    GYRO_LPF_NAMES,
    ACC_LPF_NAMES,
    BB_LOG_NAMES,
    MOTOR_STOP_NAMES,
    FAILSAFE_ACTION_NAMES,
    ALL_AIRFRAMES,
    AIRFRAME_NAMES,
)
from protocol_constants import FlightState, FLIGHT_STATE_NAMES
from packet_parser import *
from core.data_manager import data_manager
from parameters import PARAMETER_DEFS, PARAM_DEFAULTS, PARAM_LIMITS, PARAM_TYPES, PARAM_DISPLAY_MULT, PARAM_SCALES, LEGACY_TAGS, PID_GAIN_TAGS, PARAM_BOOT_REQUIRED
from airframes import airframes as af_module

# Nav m/s limits (40 Nav.MaxVelocity, 62 Nav.PosIntLim): not gains — they get a
# clean 0.1 m/s step / 1 decimal, never the 4-decimal gain treatment.
ONE_DECIMAL_LIMIT_TAGS = frozenset({40, 62})


class TickBar(QWidget):
    """Progress bar with configurable tick marks"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self._value = 1500
        self._minimum = 900
        self._maximum = 2100
        self._ticks = [1000, 1500, 2000]
        self.setMinimumHeight(10)

    def setValue(self, value):
        self._value = value
        self.update()

    def setRange(self, minimum, maximum, ticks=None):
        self._minimum = minimum
        self._maximum = maximum
        if ticks is not None:
            self._ticks = ticks
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        w = self.width()
        h = self.height()

        p.fillRect(0, 0, w, h, QColor("#e0e0e0"))

        frac = (self._value - self._minimum) / (self._maximum - self._minimum)
        fill_w = max(0, min(w, int(frac * w)))
        if fill_w > 0:
            p.fillRect(0, 0, fill_w, h, QColor("#4a90d9"))

        p.setPen(QPen(QColor("#444"), 1))
        for tick in self._ticks:
            fx = (tick - self._minimum) / (self._maximum - self._minimum)
            if 0.0 <= fx <= 1.0:
                x = int(fx * w)
                p.drawLine(x, 0, x, h - 1)

        p.setPen(QPen(QColor("#aaa"), 1))
        p.drawRect(0, 0, w - 1, h - 1)


# No hardcoded aircraft templates - only safe default in FC flash and user-exported files


class AfSaveDialog(QFileDialog):
    """Custom file dialog with checkbox to include physics summary in filename"""
    
    def __init__(self, parent=None, suggested_name="Params.af", physics_summary="", start_dir=""):
        super().__init__(parent, "Save Airframe Parameters", suggested_name, "Airframe Files (*.af);;All Files (*)")
        self.setAcceptMode(QFileDialog.AcceptSave)
        self.setDefaultSuffix("af")
        self.setOption(QFileDialog.DontUseNativeDialog, True)
        if start_dir:
            self.setDirectory(start_dir)
        
        self._physics_summary = physics_summary
        self._extended_checkbox = QCheckBox("Include physics summary in filename")
        self._extended_checkbox.setToolTip("Add AUW, prop size, motor count etc. to filename")
        self._extended_checkbox.stateChanged.connect(self._update_filename)
        
        # Add checkbox to dialog layout
        layout = self.layout()
        if layout:
            layout.addWidget(self._extended_checkbox, layout.rowCount(), 0, 1, layout.columnCount())
        
        # Initial filename
        self._base_name = os.path.splitext(suggested_name)[0]
        self._ext = os.path.splitext(suggested_name)[1] or ".af"
        self._update_filename()
    
    def _update_filename(self):
        if self._extended_checkbox.isChecked() and self._physics_summary:
            new_name = f"{self._base_name}_{self._physics_summary}{self._ext}"
        else:
            new_name = f"{self._base_name}{self._ext}"
        self.selectFile(new_name)
    
    def get_selected_name(self):
        return self.selectedFiles()[0] if self.selectedFiles() else ""


class AfLoadDialog(QDialog):
    """Custom file dialog to load .af files with optional physics preview"""
    
    def __init__(self, parent=None, start_dir="", classify_fn=None):
        super().__init__(parent)
        self.setWindowTitle("Load Airframe Parameters")
        self.resize(770, 500)
        
        self._start_dir = start_dir or os.path.expanduser("~/UAVX")
        self._user_dir = os.path.join(os.path.dirname(__file__), '..', 'airframes', 'user')
        self._original_dir = os.path.join(os.path.dirname(__file__), '..', 'airframes', 'original')
        self._proposed_dir = os.path.join(os.path.dirname(__file__), '..', 'airframes', 'proposed')
        self._generic_dir = os.path.join(os.path.dirname(__file__), '..', 'airframes', 'generic')
        self._classify_fn = classify_fn  # path -> class label ('MR'/'FW'/...)
        self._show_physics = True
        
        layout = QVBoxLayout(self)
        
        # Checkbox
        self._physics_checkbox = QCheckBox("Show physics summary")
        self._physics_checkbox.setChecked(True)
        self._physics_checkbox.setToolTip("Show AUW, prop, motor count etc. for each file")
        self._physics_checkbox.toggled.connect(lambda checked: self._on_physics_toggled(checked))
        layout.addWidget(self._physics_checkbox)
        
        # Category filter
        self._filter_combo = QComboBox()
        self._filter_combo.addItems(["All", "User Airframes", "Proposed (Read-Only)", "Generic (Read-Only)"])
        self._filter_combo.currentTextChanged.connect(self._on_filter_changed)
        layout.addWidget(self._filter_combo)
        
        # File list
        self._list = QListWidget()
        self._list.setAlternatingRowColors(True)
        self._list.itemDoubleClicked.connect(self.accept)
        layout.addWidget(self._list)
        
        # Buttons: delete (user files only) + OK/Cancel
        bottom_row = QHBoxLayout()
        self._delete_btn = QPushButton("Delete Selected")
        self._delete_btn.setToolTip("Delete the selected user airframe file")
        self._delete_btn.setStyleSheet("color: #e74c3c; padding: 4px 12px;")
        self._delete_btn.clicked.connect(self._delete_selected)
        bottom_row.addWidget(self._delete_btn)
        bottom_row.addStretch(1)
        btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btn_box.accepted.connect(self.accept)
        btn_box.rejected.connect(self.reject)
        bottom_row.addWidget(btn_box)
        layout.addLayout(bottom_row)
        
        self._populate_list()
    
    def _populate_list(self):
        self._list.clear()
        filter_text = self._filter_combo.currentText()
        class_order = {'MR': 0, 'FW': 1, 'VTOL': 2, 'Land': 3, 'Sensor': 4}

        entries = []  # (class_order, -mtime, name_lower, readonly, fname, fpath)
        def collect(abs_dir, readonly):
            if not os.path.isdir(abs_dir):
                return
            for fname in os.listdir(abs_dir):
                if not fname.endswith(".af"):
                    continue
                fpath = os.path.join(abs_dir, fname)
                try:
                    mtime = os.path.getmtime(fpath)
                except OSError:
                    mtime = 0.0
                cls = self._classify(fpath)
                entries.append((class_order.get(cls, 9), -mtime, fname.lower(),
                                readonly, fname, fpath))

        if filter_text in ["All", "User Airframes"]:
            collect(self._user_dir, False)
        if filter_text in ["All", "Proposed (Read-Only)"]:
            collect(self._proposed_dir, True)
        if filter_text in ["All", "Generic (Read-Only)"]:
            collect(self._generic_dir, True)

        entries.sort(key=lambda r: (r[0], r[1], r[2]))

        last_cls = None
        for cls_ord, _nm, _low, readonly, fname, fpath in entries:
            cls = next((c for c, o in class_order.items() if o == cls_ord), 'MR')
            if cls != last_cls:
                header_item = QListWidgetItem(f"=== {cls} Airframes ===")
                header_item.setFlags(header_item.flags() & ~Qt.ItemIsSelectable)
                header_item.setBackground(QColor("#e0e0e0"))
                header_item.setForeground(QColor("#333333"))
                font = header_item.font()
                font.setBold(True)
                header_item.setFont(font)
                self._list.addItem(header_item)
                last_cls = cls

            prefix = "  [R/O] " if readonly else "  "
            if self._show_physics:
                meta = self._parse_physics(fpath)
                if meta:
                    parts = []
                    if meta.get('PHYS_AUW_G'):
                        parts.append(f"{meta['PHYS_AUW_G']}g")
                    if meta.get('PHYS_PROP_INCH'):
                        parts.append(f"{meta['PHYS_PROP_INCH']}in")
                    if meta.get('PHYS_MOTOR_COUNT'):
                        parts.append(f"{meta['PHYS_MOTOR_COUNT']}M")
                    if meta.get('PHYS_WINGSPAN_MM'):
                        parts.append(f"{meta['PHYS_WINGSPAN_MM']}mm")
                    info = "  [" + ", ".join(parts) + "]"
                    self._list.addItem(prefix + fname + info)
                else:
                    self._list.addItem(prefix + fname)
            else:
                self._list.addItem(prefix + fname)
            item = self._list.item(self._list.count() - 1)
            item.setData(Qt.UserRole, fpath)
            if readonly:
                item.setForeground(QColor("#888888"))
                item.setToolTip("Original untuned file (read-only)")

    def _classify(self, path):
        """Return the airframe class label ('MR'/'FW'/...) for an .af file."""
        if self._classify_fn is not None:
            try:
                return self._classify_fn(path)
            except Exception:
                pass
        return 'MR'

    def _delete_selected(self):
        """Delete the selected user airframe file after confirmation."""
        item = self._list.currentItem()
        if item is None:
            return
        path = item.data(Qt.UserRole)
        if not path:
            return
        user_dir_abs = os.path.abspath(self._user_dir)
        if not os.path.dirname(os.path.abspath(path)).startswith(user_dir_abs):
            QMessageBox.information(self, "Delete", "Only user airframes can be deleted.")
            return
        base = os.path.basename(path)
        reply = QMessageBox.question(
            self, "Delete Airframe",
            f"Delete \"{base}\"?\n\nThis cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return
        try:
            os.remove(path)
        except OSError as e:
            QMessageBox.critical(self, "Delete Failed", str(e))
            return
        self._populate_list()
        parent = self.parent()
        if parent is not None:
            if getattr(parent, '_current_airframe_path', None) == path:
                parent._current_airframe_path = None
            if hasattr(parent, 'status_label'):
                parent.status_label.setText(f"🗑 Deleted {base}")
                parent.status_label.setStyleSheet("color: #e74c3c;")
    
    def _parse_physics(self, path):
        try:
            with open(path) as f:
                content = f.read()
            meta = {}
            for line in content.split('\n'):
                if line.startswith('PHYS_'):
                    parts = line.split('=', 1)
                    if len(parts) == 2:
                        meta[parts[0].strip()] = parts[1].strip()
            return meta
        except:
            return {}
    
    def _refresh_list(self):
        self._populate_list()

    def _on_physics_toggled(self, checked):
        self._show_physics = checked
        self._populate_list()

    def _on_filter_changed(self, text):
        self._populate_list()
    
    def get_selected_path(self):
        item = self._list.currentItem()
        return item.data(Qt.UserRole) if item else ""


class ParameterWindow(QMainWindow):
    """Parameter editor window - refactored with enums"""

    # Emitted from the sim worker thread; auto-queued to the UI thread.
    sim_finished = pyqtSignal(bool, list, str, bool)

    # Safety-critical params — require confirmation to change
    # Physics-computed FW FF params + MC params where wrong values cause crashes
    _PROTECTED_PARAMS = {
        # FW FF: physics-computed defaults
        int(ParamIndex.FW_ROLL_PITCH_FF),
        int(ParamIndex.FW_PITCH_THROTTLE_FF),
        int(ParamIndex.FW_AILERON_DIFFERENTIAL),
        int(ParamIndex.FW_AILERON_RUDDER_MIX),
        # MC catastrophic: wrong values = crash or damage
        int(ParamIndex.AF_TYPE),
        int(ParamIndex.ESC_TYPE),
        int(ParamIndex.PERCENT_IDLE_THR),
        int(ParamIndex.CONFIG1_BITS),
        int(ParamIndex.VOLT_SCALE),
        int(ParamIndex.SERVO_SENSE),
    }
    _PROTECTED_NAMES = {
        int(ParamIndex.FW_ROLL_PITCH_FF): "FW Roll→Pitch FF",
        int(ParamIndex.FW_PITCH_THROTTLE_FF): "FW Pitch→Throttle FF",
        int(ParamIndex.FW_AILERON_DIFFERENTIAL): "FW Aileron Differential",
        int(ParamIndex.FW_AILERON_RUDDER_MIX): "FW Aileron→Rudder Mix",
        int(ParamIndex.AF_TYPE): "Airframe Type",
        int(ParamIndex.ESC_TYPE): "ESC Protocol",
        int(ParamIndex.PERCENT_IDLE_THR): "Idle Throttle %",
        int(ParamIndex.CONFIG1_BITS): "Config1 Bits",
        int(ParamIndex.VOLT_SCALE): "Voltage Scale",
        int(ParamIndex.SERVO_SENSE): "Servo Sense",
    }
    _PROTECTED_WARNINGS = {
        int(ParamIndex.FW_ROLL_PITCH_FF): "This is a physics-computed default based on your airframe's lift curve.\nWrong values cause poor roll-to-pitch coupling.",
        int(ParamIndex.FW_PITCH_THROTTLE_FF): "This is a physics-computed default based on your airframe's pitch-trim.\nWrong values cause throttle oscillation in climb/dive.",
        int(ParamIndex.FW_AILERON_DIFFERENTIAL): "This prevents adverse yaw from aileron deflection.\nWrong values cause uncoordinated turns and yaw oscillation.",
        int(ParamIndex.FW_AILERON_RUDDER_MIX): "This coordinates rudder with aileron input.\nWrong values cause slipping/skidding turns.",
        int(ParamIndex.AF_TYPE): "Wrong airframe type selects incorrect mixing.\nThis WILL cause an immediate crash on motor spin-up.",
        int(ParamIndex.ESC_TYPE): "Wrong ESC protocol means no motor response or random spin.\nMotors may not start or may spin erratically.",
        int(ParamIndex.PERCENT_IDLE_THR): "Idle throttle too low = motors stop in flight (descent).\nToo high = quad walks on ground and wastes power.",
        int(ParamIndex.CONFIG1_BITS): "Config bits enable/disable safety features.\nWrong bits can silently disable critical protections.",
        int(ParamIndex.VOLT_SCALE): "Wrong voltage scale defeats low-voltage battery protection.\nRisk of battery damage or fire from over-discharge.",
        int(ParamIndex.SERVO_SENSE): "Wrong servo sense reverses control surface direction.\nReversed ailerons = spiral dive. Reversed elevator = loss of pitch control.",
    }

    # Params the user must configure per-aircraft — highlighted until set to non-default
    _REQUIRED_SETUP_PARAMS = {
        int(ParamIndex.AF_TYPE): "Airframe type must match your actual airframe",
        int(ParamIndex.ESC_TYPE): "ESC protocol must match your ESCs",
        int(ParamIndex.RF_SENSOR_TYPE): "RF module type must match your radio",
        int(ParamIndex.AS_SENSOR_TYPE): "Airspeed sensor type (0 = none)",
        int(ParamIndex.BATTERY_CAPACITY): "Battery capacity in mAh (param stores 0.1 Ah units)",
        int(ParamIndex.NAV_MAG_VAR): "Magnetic declination in degrees (set to 0 if unsure)",
        int(ParamIndex.VOLT_SCALE): "Voltage sensor scale — calibrate against multimeter reading",
        int(ParamIndex.CURRENT_SCALE): "Current sensor scale — calibrate against known load",
    }

    # Character slider param curves: {param_idx: (conservative_display, aggressive_display)}
    # Slider 0.0 = conservative (stable, forgiving), 1.0 = aggressive (responsive, tight)
    # All values in display units — compute_defaults() converts to raw via _display_mult
    _PARAM_CURVES = {
        # Character-slider tuning ranges, in DISPLAY units (= raw * PARAM_DISPLAY_MULT).
        # Authoritative: FC ParamTable defaults (params.c) + test_pid_sim.py curve set.
        # Endpoints re-anchored Aug 29 to the critic-validated original/ fleet spans
        # (see wiki/Session_Report_CritiqueRetune_Aug29.md): MR reference = ~800 g /
        # 220 mm class (Ecks..Rok cluster), whoop-class frames covered by scale factors.
        # Angle quaternion gains (display = raw, mult 1.0)
        int(ParamIndex.ROLL_ANGLE_Q_KP):    (5.0, 9.0),      # fleet 5.75–7
        int(ParamIndex.PITCH_ANGLE_Q_KP):   (5.0, 9.0),      # fleet 5.75–7
        int(ParamIndex.YAW_ANGLE_Q_KP):     (2.0, 4.5),      # MR fleet 2.25–3.0 (FW overrides — Phoenix 7.68)
        # Angle integral gains (mult 1.0)
        int(ParamIndex.ROLL_ANGLE_Q_KI):    (0.05, 0.5),     # fleet 0.1–0.25
        int(ParamIndex.PITCH_ANGLE_Q_KI):   (0.05, 0.5),
        int(ParamIndex.YAW_ANGLE_Q_KI):     (0.05, 0.5),     # fleet 0.25 (FW overrides — Phoenix 0.64)
        # Angle integral limits (rad/s, mult 1.0)
        int(ParamIndex.ROLL_ANGLE_Q_INT_LIMIT): (0.005, 0.03),
        int(ParamIndex.PITCH_ANGLE_Q_INT_LIMIT): (0.005, 0.03),
        int(ParamIndex.YAW_ANGLE_Q_INT_LIMIT): (0.01, 0.06),
        # Rate proportional gains (mult 1.0)
        int(ParamIndex.ROLL_RATE_KP):     (0.125, 0.5),      # fleet 0.2–0.45 (flat across 30–1344 g)
        int(ParamIndex.PITCH_RATE_KP):    (0.125, 0.5),
        int(ParamIndex.YAW_RATE_KP):      (0.08, 0.15),      # ~800 g cluster 0.09–0.10; whoop sf≈4 → ~0.6
        # Rate derivative gains (mult 1.0)
        int(ParamIndex.ROLL_RATE_KD):     (0.002, 0.012),    # fleet 0.002–0.010 (was 0.005–0.02 — stale)
        int(ParamIndex.PITCH_RATE_KD):    (0.002, 0.012),
        int(ParamIndex.YAW_RATE_KD):      (0.0002, 0.001),   # fleet 0.0003375–0.00066 (was 0.005–0.02 — 15–60× stale)
        # Rate limits (rad->deg/s, mult 57.3)
        int(ParamIndex.MAX_ROLL_RATE):    (80.0, 450.0),     # fleet 210–420 (S500) — was 360
        int(ParamIndex.MAX_PITCH_RATE):   (60.0, 420.0),
        int(ParamIndex.MAX_HEADING_RATE): (15.0, 120.0),     # fleet 60–90
        # Altitude (alt params are NOT mass-scaled — fleet runs 1.83/0.005 flat)
        int(ParamIndex.ALT_POS_KP):       (1.0, 2.5),        # fleet 1.83 (was 0.2–0.5 — refused own fleet)
        int(ParamIndex.ALT_POS_KI):       (0.001, 0.005),
        int(ParamIndex.ALT_THROTTLE_COMP_LIMIT): (10.0, 25.0),   # mult 100 — mirrors FC pMaxAltHoldThrComp default 0.25 (aggressive end); min 10% = usable alt authority
        int(ParamIndex.ALT_ROC_KP):       (0.02, 1.5),       # fleet 0.05–1.0
        int(ParamIndex.UNUSED_ALT_VEL_KI): (0.0003, 0.003),
        # Navigation
        int(ParamIndex.NAV_POS_KP):       (0.075, 0.3),
        int(ParamIndex.NAV_POS_KI):       (0.006, 0.025),
        int(ParamIndex.NAV_POS_INT_LIM):  (0.2, 2.0),
        int(ParamIndex.NAV_VEL_KP):       (0.1, 0.4),
        int(ParamIndex.HORIZON):          (20.0, 50.0),
        # Angle limits (rad->deg, mult 57.3) — fleet runs 60° = the FC eClassAngle ceiling
        int(ParamIndex.MAX_PITCH_ANGLE):  (30.0, 60.0),
        int(ParamIndex.MAX_ROLL_ANGLE):   (30.0, 60.0),
    }

    # FW category overrides on top of _PARAM_CURVES (MR reference above).
    # Evidence: Phoenix — YawAngleQKp 7.68, YawAngleQKi 0.64, YawRateKp 0.285.
    # FW bank = ROLL angle (coordinated turn); pitch max stays a climb-attitude
    # limit, so the two differ for FW (user decision 2026-08-29: bank ≤ 45°).
    _PARAM_CURVES_FW = {
        int(ParamIndex.YAW_ANGLE_Q_KP):   (5.0, 10.0),
        int(ParamIndex.YAW_ANGLE_Q_KI):   (0.1, 0.8),
        int(ParamIndex.YAW_RATE_KP):      (0.2, 0.4),
        int(ParamIndex.MAX_ROLL_ANGLE):   (20.0, 45.0),
        int(ParamIndex.MAX_PITCH_ANGLE):  (7.0, 15.0),   # ~1/3 of max bank (FW ratio, 2026-08-29 decision)
    }

    # Physical descriptor definitions for Setup row (dynamic per AF category)
    # Each entry: (metadata_key, label, decimals, default, tooltip)
    _MR_PHYS_DESCRIPTORS = [
        ("PHYS_AUW_G",      "AUW (g)",         0, 800,    "All-up weight in grams — weigh the aircraft on a scale"),
        ("PHYS_ARM_MM",     "Arm (mm)",        0, 220,    "Motor shaft to center distance"),
        ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 300,    "Max thrust per motor in grams"),
        ("PHYS_PROP",       "Prop",            None, "11x4.5", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
        ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 20.0,   "Max current per motor in amps"),
        ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 14.8,   "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
        ("PHYS_MOTOR_COUNT","Motors",          0, 4,      "Number of motors"),
    ]
    _FW_PHYS_DESCRIPTORS = [
        ("PHYS_AUW_G",       "AUW (g)",        0, 1000,  "All-up weight in grams — weigh the aircraft on a scale"),
        ("PHYS_WINGSPAN_MM", "Wing (mm)",      0, 1800,  "Wingtip to wingtip"),
        ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 800,   "Max thrust per motor in grams"),
        ("PHYS_CHORD_MM",    "Chord (mm)",     0, 250,   "Average root chord"),
        ("PHYS_PROP",        "Prop",           None, "10x5", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
        ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 30.0,  "Max current per motor in amps"),
        ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 14.8,  "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
        ("PHYS_BATT_V",      "Batt V",         1, 50.0,  "Battery voltage under load (e.g. 3S=11.1, 4S=14.8)"),
    ]
    _LAND_PHYS_DESCRIPTORS = [
        ("PHYS_AUW_G",       "AUW (g)",        0, 5000,  "All-up weight in grams — weigh the vehicle on a scale"),
        ("PHYS_TRACK_MM",    "Track/WB (mm)",  0, 300,   "Track width or wheelbase in mm"),
        ("PHYS_WHEEL_DIA",   "Wheel dia (mm)", 0, 100,   "Drive wheel diameter in mm"),
    ]

    # Hover throttle sanity bounds
    # Hover throttle: 45-55% is ideal (equal climb/descent margin)
    # Warning outside 25-75% catches genuinely wrong inputs
    _HOVER_THR_MIN = 0.25
    _HOVER_THR_MAX = 0.75

    # MR shape factors for inertia from arm length
    _MR_SHAPE_KF = {4: 0.55, 6: 0.50, 8: 0.45}

    # Motor count lookup from AF type (MR only)
    _MR_MOTOR_COUNT = {
        3: 4, 4: 4, 5: 4, 6: 4,   # QUAD, QUAD_X, QUAD_COAX, QUAD_COAX_X
        7: 6, 8: 6,                 # HEX, HEX_X
        9: 8, 10: 8,                # OCT, OCT_X
    }

    # Airframe-specific physics defaults (matching generic .af files)
    _MR_PHYS_DEFAULTS = {
        4: [  # Quad (standard 5-6")
            ("PHYS_AUW_G",      "AUW (g)",         0, 750,   "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_ARM_MM",     "Arm (mm)",        0, 180,   "Motor shaft to center distance"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 300,   "Max thrust per motor in grams"),
            ("PHYS_PROP",       "Prop",            None, "6x4.5", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 25.0,  "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 14.8,  "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_MOTOR_COUNT","Motors",          0, 4,     "Number of motors"),
        ],
        5: [  # Quad_Medium (7" long-range)
            ("PHYS_AUW_G",      "AUW (g)",         0, 1000,  "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_ARM_MM",     "Arm (mm)",        0, 175,   "Motor shaft to center distance"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 500,   "Max thrust per motor in grams"),
            ("PHYS_PROP",       "Prop",            None, "7x4.5", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 20.0,  "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 14.8,  "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_MOTOR_COUNT","Motors",          0, 4,     "Number of motors"),
        ],
        6: [  # Quad_Racer (5" racer)
            ("PHYS_AUW_G",      "AUW (g)",         0, 450,   "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_ARM_MM",     "Arm (mm)",        0, 115,   "Motor shaft to center distance"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 500,   "Max thrust per motor in grams"),
            ("PHYS_PROP",       "Prop",            None, "5x5", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 35.0,  "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 14.8,  "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_MOTOR_COUNT","Motors",          0, 4,     "Number of motors"),
        ],
        7: [  # Hex
            ("PHYS_AUW_G",      "AUW (g)",         0, 1200,  "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_ARM_MM",     "Arm (mm)",        0, 275,   "Motor shaft to center distance"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 700,   "Max thrust per motor in grams"),
            ("PHYS_PROP",       "Prop",            None, "11x5", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 15.0,  "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 14.8,  "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_MOTOR_COUNT","Motors",          0, 6,     "Number of motors"),
        ],
        8: [  # Oct
            ("PHYS_AUW_G",      "AUW (g)",         0, 1600,  "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_ARM_MM",     "Arm (mm)",        0, 300,   "Motor shaft to center distance"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 800,   "Max thrust per motor in grams"),
            ("PHYS_PROP",       "Prop",            None, "12x6", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 20.0,  "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 22.2,  "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_MOTOR_COUNT","Motors",          0, 8,     "Number of motors"),
        ],
    }

    _FW_PHYS_DEFAULTS = {
        13: [  # ELEVON
            ("PHYS_AUW_G",       "AUW (g)",        0, 650,  "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_WINGSPAN_MM", "Wing (mm)",      0, 850,  "Wingtip to wingtip"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 400,  "Max thrust per motor in grams"),
            ("PHYS_CHORD_MM",    "Chord (mm)",     0, 180,  "Average root chord"),
            ("PHYS_PROP",        "Prop",           None, "8x5", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 20.0, "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 11.1, "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_BATT_V",      "Batt V",         1, 11.1, "Battery voltage under load (e.g. 3S=11.1, 4S=14.8)"),
        ],
        14: [  # DELTA
            ("PHYS_AUW_G",       "AUW (g)",        0, 550,  "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_WINGSPAN_MM", "Wing (mm)",      0, 850,  "Wingtip to wingtip"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 300,  "Max thrust per motor in grams"),
            ("PHYS_CHORD_MM",    "Chord (mm)",     0, 280,  "Average root chord"),
            ("PHYS_PROP",        "Prop",           None, "6.5x4", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 20.0, "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 11.1, "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_BATT_V",      "Batt V",         1, 11.1, "Battery voltage under load (e.g. 3S=11.1, 4S=14.8)"),
        ],
        15: [  # AILERON
            ("PHYS_AUW_G",       "AUW (g)",        0, 1200, "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_WINGSPAN_MM", "Wing (mm)",      0, 1200, "Wingtip to wingtip"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 800,  "Max thrust per motor in grams"),
            ("PHYS_CHORD_MM",    "Chord (mm)",     0, 220,  "Average root chord"),
            ("PHYS_PROP",        "Prop",           None, "10x6", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 30.0, "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 14.8, "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_BATT_V",      "Batt V",         1, 14.8, "Battery voltage under load (e.g. 3S=11.1, 4S=14.8)"),
        ],
        18: [  # RUDDER_ELEVATOR
            ("PHYS_AUW_G",       "AUW (g)",        0, 900,  "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_WINGSPAN_MM", "Wing (mm)",      0, 1380, "Wingtip to wingtip"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 600,  "Max thrust per motor in grams"),
            ("PHYS_CHORD_MM",    "Chord (mm)",     0, 220,  "Average root chord"),
            ("PHYS_PROP",        "Prop",           None, "10x6", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 25.0, "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 14.8, "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_BATT_V",      "Batt V",         1, 11.1, "Battery voltage under load (e.g. 3S=11.1, 4S=14.8)"),
        ],
        16: [  # AILERON_SPOILER_FLAPS
            ("PHYS_AUW_G",       "AUW (g)",        0, 1000, "All-up weight in grams — weigh the aircraft on a scale"),
            ("PHYS_WINGSPAN_MM", "Wing (mm)",      0, 1100, "Wingtip to wingtip"),
            ("PHYS_MOTOR_THRUST_G","Thrust (g)",   0, 700,  "Max thrust per motor in grams"),
            ("PHYS_CHORD_MM",    "Chord (mm)",     0, 200,  "Average root chord"),
            ("PHYS_PROP",        "Prop",           None, "9x5", "Prop diameter × pitch (× blades), e.g. 10x6 or 5x4.3x3"),
            ("PHYS_MOTOR_THRUST_A","Curr (A)",     1, 25.0, "Max current per motor in amps"),
            ("PHYS_MOTOR_THRUST_V","Volt (V)",     1, 14.8, "Motor voltage (e.g. 3S=11.1, 4S=14.8, 6S=22.2)"),
            ("PHYS_BATT_V",      "Batt V",         1, 14.8, "Battery voltage under load (e.g. 3S=11.1, 4S=14.8)"),
        ],
    }

    _LAND_PHYS_DEFAULTS = {
        22: [  # TRACKED
            ("PHYS_AUW_G",       "AUW (g)",        0, 5000, "All-up weight in grams — weigh the vehicle on a scale"),
            ("PHYS_TRACK_MM",    "Track/WB (mm)",  0, 300,  "Track width or wheelbase in mm"),
            ("PHYS_WHEEL_DIA",   "Wheel dia (mm)", 0, 100,  "Drive wheel diameter in mm"),
        ],
    }

    # Default base curves (unscaled) for computing scale factors
    # These represent a "reference" MR at 800g AUW, 220mm arm, 1000kv, 11"
    _REF_MR_AUW_G = 800
    _REF_MR_ARM_MM = 220
    _REF_MR_PROP_INCH = 11.0
    _REF_FW_AUW_G = 1000
    _REF_FW_WINGSPAN_MM = 1800

    # Parameter groups defined with enums
    # Yaw angle Ki/Kp is a separate tight fleet constant (0.083 across all
    # cohorts and both tuning generations) — yaw's Qa gain-to-rate authority
    # differs from roll/pitch (0.026). Unlike roll/pitch there is NO yaw
    # angle-max input (heading is unbounded through 360°), so yaw QAngleKp
    # is not derivable from Max Heading Rate; only yaw Ki is derived.
    _YAW_ANGLE_KI_KP_RATIO = 0.083

    PID_PARAMS = [
        ("Q Angle", ParamIndex.ROLL_ANGLE_Q_KP, ParamIndex.PITCH_ANGLE_Q_KP, ParamIndex.YAW_ANGLE_Q_KP, 7.0, 7.0, 3.0),
        ("Ki Angle", ParamIndex.ROLL_ANGLE_Q_KI, ParamIndex.PITCH_ANGLE_Q_KI, ParamIndex.YAW_ANGLE_Q_KI, 0.25, 0.25, 0.25),
        ("I-Limit", ParamIndex.ROLL_ANGLE_Q_INT_LIMIT, ParamIndex.PITCH_ANGLE_Q_INT_LIMIT, ParamIndex.YAW_ANGLE_Q_INT_LIMIT, 0.002618, 0.002618, 0.008727),
        ("Kp Rate", ParamIndex.ROLL_RATE_KP, ParamIndex.PITCH_RATE_KP, ParamIndex.YAW_RATE_KP, 0.1, 0.1, 0.1),
        ("Kd Rate", ParamIndex.ROLL_RATE_KD, ParamIndex.PITCH_RATE_KD, ParamIndex.YAW_RATE_KD, 0.0045, 0.0045, 0.001125),
        ("Max Rate (Deg/Sec)", ParamIndex.MAX_ROLL_RATE, ParamIndex.MAX_PITCH_RATE, ParamIndex.MAX_HEADING_RATE, 600, 600, 60),
    ]
    
    RC_MAP_PARAMS = [
        (ParamIndex.RX_THROTTLE_CH, "Throttle", 0),
        (ParamIndex.RX_ROLL_CH, "Roll", 1),
        (ParamIndex.RX_PITCH_CH, "Pitch", 2),
        (ParamIndex.RX_YAW_CH, "Yaw", 3),
        (ParamIndex.RX_AUX2_CH, "Arming", 4),
        (ParamIndex.RX_AUX1_CH, "AttMode", 5),
        (ParamIndex.RX_GEAR_CH, "NavMode", 6),
        (ParamIndex.RX_AUX6_CH, "PassThru", 7),
        (ParamIndex.RX_AUX7_CH, "Dive", 8),
        (ParamIndex.RX_AUX4_CH, "Trace", 9),
        (ParamIndex.RX_AUX5_CH, "RateGain", 10),
        (ParamIndex.RX_AUX3_CH, "CamPitch", 11),
    ]

    # RC-config channel-slot spin style (0-based slot in the discovered frame)
    RC_CH_STYLE = """
        QSpinBox { padding: 1px 2px; }
        QSpinBox::up-button, QSpinBox::down-button { width: 12px; }
    """
    RC_CH_BAD_STYLE = """
        QSpinBox { padding: 1px 2px; background-color: #e74c3c; color: white; }
        QSpinBox::up-button, QSpinBox::down-button { width: 12px; }
    """
    
    ALTITUDE_PARAMS = [
        (ParamIndex.ALT_POS_KP, "Kp Alt", 0.25),
        (ParamIndex.ALT_POS_KI, "Ki Alt", 0.0046),
        (ParamIndex.ALT_POS_INT_LIMIT, "I-Limit", 0.5),
        (ParamIndex.ALT_ROC_KP, "Kp ROC", 0.05),
        (ParamIndex.UNUSED_ALT_VEL_KI, "Ki ROC", 0.001),
        (ParamIndex.ALT_LPF, "Alt LPF (Hz)", 10),
        (ParamIndex.ALT_HOLD_THR_COMP_DECAY_PERCENT_PS, "Thr Decay (%/s)", 25),
        (ParamIndex.ALT_THROTTLE_COMP_LIMIT, "Thr Comp Limit (%)", 25),
        (ParamIndex.AH_THROTTLE_MOVING_TRIGGER, "Thr Move Trig (%)", 2),
        (ParamIndex.VRS_ROC, "VRS ROC (m/s)", 3),
        (ParamIndex.AH_ROC_WINDOW_MPS, "AH ROC Win (m/s)", 1),
    ]
    
    NAVIGATION_PARAMS = [
        (ParamIndex.NAV_POS_KP, "Kp Pos", 0.22),
        (ParamIndex.NAV_POS_KI, "Ki Pos", 0.012),
        (ParamIndex.NAV_POS_INT_LIM, "I-Limit (m/s)", 1.0),
        (ParamIndex.NAV_VEL_KP, "Kp Vel", 0.25),
        (ParamIndex.NAV_POS_INT_LIMIT, "Max Vel (m/s)", 5),
        (ParamIndex.NAV_MAX_ANGLE, "Max Angle (Deg)", 25),
        (ParamIndex.NAV_RTH_ALT, "RTH Alt (m)", 10),
        (ParamIndex.NAV_MAG_VAR, "Mag Var (Deg)", 12.8),
        (ParamIndex.NAV_CROSS_TRACK_KP, "XTrack Kp", 0.04),
        (ParamIndex.NAV_HEADING_TURNOUT, "Heading Turnout", 20),
        (ParamIndex.MAX_HEADING_RATE, "Max Heading Rate (°/s)", 30),
        (ParamIndex.NAV_PROX_ALT_M, "Prox Alt (m)", 4),
        (ParamIndex.NAV_PROX_RADIUS_M, "Prox Radius (m)", 3),
        (ParamIndex.NAV_FENCE_RADIUS_M, "Fence Rad (m)", 200),
        (ParamIndex.DESCENT_DELAY_S, "Land Delay (s)", 15),
        (ParamIndex.MAX_DESCENT_RATE_MP_S, "Descent Shape (m/s)", 3),
        (ParamIndex.MAX_CLIMB_RATE_MP_S, "Climb Shape (m/s)", 3),
        (ParamIndex.SPIRAL_DESCENT_BAND_M, "Spiral Band (m)", 3),
        (ParamIndex.MOTOR_STOP_SEL, "Motor Stop", 0, list(MOTOR_STOP_NAMES.values())),
        (ParamIndex.BB_LOG_TYPE, "BB Log", 0, list(BB_LOG_NAMES.values())),
    ]
    
    GENERAL_PARAMS = [
        (ParamIndex.AF_TYPE, "Airframe", 4, [AIRFRAME_NAMES[af] for af in ALL_AIRFRAMES], [af.value for af in ALL_AIRFRAMES]),
        (ParamIndex.ESC_TYPE, "ESC Type", 2, list(ESC_TYPE_NAMES.values())),
        (ParamIndex.RX_TYPE, "Rx Type", 0, list(RX_TYPE_NAMES.values())),
        (ParamIndex.ARMING_MODE, "Arming Mode", 1, list(ARMING_MODE_NAMES.values())),
        (ParamIndex.RF_SENSOR_TYPE, "RF Type", 0, list(RF_TYPE_NAMES.values())),
        (ParamIndex.AS_SENSOR_TYPE, "Airspeed", 4, list(AS_SENSOR_TYPE_NAMES.values())),
        (ParamIndex.BALANCE, "Balance (%)", 0),
        (ParamIndex.HORIZON, "Horizon (%)", 0.3),
    ]

    GENERAL_CAMERA_PARAMS = [
        (ParamIndex.ROLL_CAM_KP, "Roll Cam Kp", 1.0),
        (ParamIndex.PITCH_CAM_KP, "Pitch Cam Kp", 1.0),
        (ParamIndex.ROLL_CAM_TRIM, "Roll Cam Trim", 0),
        (ParamIndex.PERCENT_IDLE_THR, "Idle Thr (%)", 5.0),
        (ParamIndex.STICK_HYSTERESIS, "Hysteresis (%)", 2),
    ]
    
    FILTER_PARAMS = [
        (ParamIndex.GYRO_LPF_SEL, "Gyro LPF (Hz)", 2, list(GYRO_LPF_NAMES.values())),
        (ParamIndex.ACC_LPF_SEL, "Acc LPF (Hz)", 3, list(ACC_LPF_NAMES.values())),
        (ParamIndex.YAW_LPF_HZ, "Yaw LPF (Hz)", 50),
        (ParamIndex.SERVO_LPF_HZ, "Servo LPF (Hz)", 30),
        (ParamIndex.IMU_FILT_TYPE, "IMU Filter", 0, list(IMU_FILTER_NAMES.values())),
        (ParamIndex.UNUSED_92, "Gyro Slew Rate", 5),
    ]
    
    ESTIMATOR_PARAMS = [
        (ParamIndex.MADGWICK_KP_MAG, "Kp Mag", 0.5),
        (ParamIndex.MADGWICK_KP_ACC, "Kp Acc", 0.4),
        (ParamIndex.ACC_CONF_SD, "Acc Conf (SD)", 16.67),
    ]
    
    MISC1_PARAMS = []

    MISC2_PARAMS = [
        (ParamIndex.LOW_VOLT_THRES, "Low Volt (V)", 16),
        (ParamIndex.VOLT_SCALE, "Volt Scale", 19),
        (ParamIndex.CURRENT_SCALE, "Curr Scale", 0),
        (ParamIndex.BATTERY_CAPACITY, "Batt mAH", 1500),
        (ParamIndex.THROTTLE_GAIN_RATE, "Thr Gain (%)", 0),
    ]
    
    FW_PARAMS = [
        (ParamIndex.FW_MAX_CLIMB_ANGLE, "Climb Angle (Deg)", 60),
        (ParamIndex.FW_CLIMB_THROTTLE, "Climb Thr (%)", 0),
        (ParamIndex.FW_ROLL_PITCH_FF, "Roll/Pitch FF (%)", 0),
        (ParamIndex.FW_PITCH_THROTTLE_FF, "Pitch Thr FF (%)", 0),
        (ParamIndex.FW_AILERON_RUDDER_MIX, "Ail/Rud Mix (%)", 0),
        (ParamIndex.FW_ALT_SPOILER_FF, "Alt Spoiler FF (%)", 0),
        (ParamIndex.RUDDER_MOTOR_FF, "Rudder/Motor FF (%)", 0),
        (ParamIndex.FW_SPOILER_DECAY_PERCENT_PS, "Spoiler Decay (%/s)", 10),
        (ParamIndex.FW_AILERON_DIFFERENTIAL, "Aileron Diff (%)", 0),
        (ParamIndex.FW_STICK_SCALE, "Stick Scale(%)", 40),
        (ParamIndex.FW_BOARD_PITCH_ANGLE, "Board Pitch (Deg)", 0),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.params = {}
        self.dirty_params = set()
        self.rc_channels = [0] * 16
        self._param_grid = None
        self._config_group = None
        self._fw_left = None
        self._fw_right = None
        self.motor_bars = []
        self.motor_pct_labels = []
        self.motor_name_labels = []
        self._last_motor_fd = None
        self._motor_nan_warned = False
        self.parent_window = parent
        self.voltage_trim = 1.0
        self._params_received = False
        self._read_request_id = None
        self._committed_values = {}  # last FC-loaded values for protected param revert
        self._warned_protected = set()  # track which protected params have been warned this session
        self._baseline_snapshot = {}  # param values at last load — for dirty detection
        self._baseline_source_path = None  # .af path that was loaded (for default save path)
        self._read_timeout_timer = None
        self._write_timeout_timer = None
        self._verify_mode = False
        self._written_params = {}
        self._written_float = {}
        self._image_dirty = False  # a bulk widget set (load) is pending push to FC RAM
        self._write_in_progress = False
        self._flash_write_pending = False  # commit interlock: True from commit start until verify resolves
        self._verification_timer = None
        self._expecting_param_packet = False
        self._write_timestamp = 0
        self.MAX_PARAMS = 128
        self._config_values = {}
        self.servo_sense_checks = []
        self._current_airframe_path = None
        self._airframe_limits: Dict[int, Tuple[float, float]] = {}  # .af [LIMITS] raw ranges
        self._legacy_mode = False  # default OFF: unified raw float display (see legacy_check init)
        self._flash_airframe_name = None  # single source of truth: name from FC config flash
        self._generic_dir = os.path.join(os.path.dirname(__file__), '..', 'airframes', 'generic')
        self._user_dir = os.path.join(os.path.dirname(__file__), '..', 'airframes', 'user')
        self._proposed_dir = os.path.join(os.path.dirname(__file__), '..', 'airframes', 'proposed')
        self._warned_protected = set()  # track protected params that have been warned this session
        self._clamped_widgets = set()   # widgets currently showing the FC-clamp red highlight

        # Auto robustness safety check (debounced; run after Compute or tuning edits)
        self._suppress_sim_auto = True  # True during init/bulk loads; armed after __init__
        self._sim_busy = False
        self._sim_debounce = QTimer(self)
        self._sim_debounce.setSingleShot(True)
        self._sim_debounce.setInterval(800)
        self._sim_debounce.timeout.connect(self._run_auto_sim)
        self.sim_finished.connect(self._on_sim_finished)
        # Cascade-integrity caveats (20–30% I band, P-path vs rate cap)
        self._cascade_warnings = []
        self._last_caveat_msg = ""
        self._warn_cascade_dialog = True


        self.setup_ui()
        self._restore_airframe_selection()
        self.setup_connections()
        self.load_default_params()
        
        data_manager.register_observer(self.on_data_updated)
        
        self.rc_timer = QTimer()
        self.rc_timer.timeout.connect(self.update_rc_display)
        self.rc_timer.start(50)
        
        self.update_rc_display()
        self.reset_read_button()
        self.reset_write_button()

        self._suppress_sim_auto = False

        # Auto-read params from FC after a short delay to let the UI settle
        QTimer.singleShot(100, self.read_params)
    
    def _log(self, msg):
        """Send debug message to parent window's log if available"""
        if self.parent_window and hasattr(self.parent_window, 'log_debug'):
            self.parent_window.log_debug(msg, "Info")
        print(msg)
    

    
    def on_data_updated(self, flight_data):
        self.update_rc_display()
        self._check_mag_var_wmm(flight_data)

    def _check_mag_var_wmm(self, flight_data):
        MAG_VAR_THRESHOLD_DEG = 3.0
        NAV_MAG_VAR_IDX = ParamIndex.NAV_MAG_VAR

        wmm = getattr(flight_data, 'mag_var_wmm', 0.0)
        if wmm <= 0:
            return

        if NAV_MAG_VAR_IDX not in self.params:
            return
        widget = self.params[NAV_MAG_VAR_IDX]
        if not isinstance(widget, QDoubleSpinBox):
            return

        param_display = widget.value()
        if abs(param_display - wmm) > MAG_VAR_THRESHOLD_DEG:
            widget.setStyleSheet("""
                QDoubleSpinBox {
                    padding: 1px 2px;
                    background-color: #ffe0b0;
                }
                QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                    width: 12px;
                }
            """)
        else:
            widget.setStyleSheet("""
                QDoubleSpinBox {
                    padding: 1px 2px;
                }
                QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                    width: 12px;
                }
            """)
    
    @staticmethod
    def _is_float_param(idx: int) -> bool:
        return PARAM_TYPES.get(idx, 'U8') == 'FLOAT'

    def _display_mult(self, idx: int) -> float:
        """Effective display multiplier for a param index.

        Normal mode:  raw * PARAM_DISPLAY_MULT[idx]
        Legacy mode (for indices in LEGACY_TAGS):  raw / PARAM_SCALES[idx]
        The write path divides by this same factor, so the raw float32
        value sent to / read from the FC is identical in both modes.
        """
        if self._legacy_mode and idx in LEGACY_TAGS:
            return 1.0 / PARAM_SCALES.get(idx, 1.0)
        return PARAM_DISPLAY_MULT.get(idx, 1.0)

    def _param_limits(self, idx: int) -> Tuple[float, float]:
        """Effective raw (lo, hi) for a param — .af [LIMITS] range if present,
        else the derived class ceiling from PARAM_LIMITS."""
        return self._airframe_limits.get(idx,
                                         PARAM_LIMITS.get(idx, (0.0, 255.0)))

    def _min_decimals(self, idx: int, dec: int) -> int:
        """PID/gain terms (the 18 values across Roll/Pitch/Yaw/Alt/Nav) always
        need 4 decimal places so fine rad/s values round-trip cleanly; never
        let a coarse step-derived precision round them in the spinbox."""
        if idx in PID_GAIN_TAGS and dec < 4:
            return 4
        return dec

    def _resolve_combo_index(self, widget, value) -> int:
        """Return the combo index whose itemData == value, appending a marked
        "? Unknown (N)" item carrying the true raw value if absent.

        NEVER falls back to treating the raw value as a positional index, and
        NEVER leaves a stale selection in place. The AF_TYPE combo is sorted by
        display NAME, not by enum value (data domain = the full enum, incl.
        REDACTED types like eAileronAF), so value-as-index silently selected
        the WRONG airframe — e.g. a missed eAileronAF (15) clamps to index 9 =
        X Quadcopter and a re-save wrote eQuadXAF into the file (the FW-as-XQuad
        / Elevon-as-Delta corruption). A miss (corrupt file / future enum this
        GCS doesn't know) therefore APPENDS the value so it stays visible AND
        round-trips true: itemData == the actual value, so a save can never
        bake a stale substitution. findData after append always succeeds.
        """
        int_value = int(value)
        cb_idx = widget.findData(int_value)
        if cb_idx >= 0:
            return cb_idx
        widget.addItem(f"? Unknown ({int_value})", int_value)
        cb_idx = widget.findData(int_value)
        self._log(f"  ⚠️ Param value {int_value} not in combo data domain; "
                  f"appended '? Unknown ({int_value})' carrying the true value")
        return cb_idx

    def _on_legacy_toggled(self, checked: bool):
        self._legacy_mode = checked
        self._refresh_legacy_display()

    def _refresh_legacy_display(self):
        """Re-apply display multiplier to all QDoubleSpinBox params in-place.

        For each spinbox: read current value, divide by the OLD mult to
        recover the raw float32, then set the new range and value using
        the NEW (possibly legacy) mult.  The raw value is preserved.
        """
        if self._legacy_mode:
            prev_mult_fn = lambda i: PARAM_DISPLAY_MULT.get(i, 1.0)
        else:
            prev_mult_fn = lambda i: (1.0 / PARAM_SCALES.get(i, 1.0)
                                      if i in LEGACY_TAGS
                                      else PARAM_DISPLAY_MULT.get(i, 1.0))
        for idx, widget in self.params.items():
            if not isinstance(widget, QDoubleSpinBox):
                continue
            try:
                raw = widget.value() / prev_mult_fn(idx)
            except (ZeroDivisionError, ValueError):
                continue
            new_mult = self._display_mult(idx)
            limits = self._param_limits(idx)
            lo = limits[0] * new_mult
            hi = limits[1] * new_mult
            span = hi - lo
            step = max(0.001, span / 50.0)
            if new_mult == 100:
                step = 1.0
            if idx in (68,) and step < 1.0:
                step = 1.0
            if idx in ONE_DECIMAL_LIMIT_TAGS:
                step = 0.1
            dec = max(0, min(6, -int(math.floor(math.log10(step)))))
            # PID/gain terms: keep at least 4 decimals in every display mode
            if idx in PID_GAIN_TAGS:
                dec = max(dec, 4)
            if new_mult not in (1.0, 100.0) and dec == 0 and step > 0.1:
                step = 0.1
                dec = 1
            widget.blockSignals(True)
            widget.setRange(lo, hi)
            widget.setDecimals(dec)
            widget.setSingleStep(step)
            widget.setValue(max(lo, min(hi, raw * new_mult)))
            widget.blockSignals(False)

    def update_params_from_typed(self, typed_data):
        """Update parameters from tag 17 packet — handles verify + normal update"""
        if typed_data is None or not hasattr(typed_data, 'entries'):
            return

        # Cancel read timeout on successful typed response
        if self._read_timeout_timer:
            self._read_timeout_timer.stop()
            self._read_timeout_timer = None

        # Build both uint8 and float lookups — all entries are float32 on wire
        received_u8 = {}
        received_float = {}
        for idx, (typ, val) in typed_data.entries.items():
            if math.isnan(val) or math.isinf(val):
                val = 0.0
            received_u8[idx] = max(0, min(255, int(val)))
            received_float[idx] = val
        
        # Verification path — only on bulk readback (echo ACKs skip this)
        if (self._verify_mode and self._expecting_param_packet and self._written_params
                and typed_data.count > 100):
            self._log(f"  📥 Received typed params — verifying {len(received_u8)} params")
            if self._verification_timer:
                self._verification_timer.stop()
                self._verification_timer = None
            if self._write_timeout_timer:
                self._write_timeout_timer.stop()
                self._write_timeout_timer = None
            self._verify_parameters(received_u8, received_float)
            return
        
        # Normal update — block signals: a readback/echo is NOT a user edit.
        # Without this, every tag-71 readback (connect auto-populate) echoed
        # the FC's values back as phantom live-writes, popped the
        # protected-param confirm dialog for the first changed selector (e.g.
        # P90 ASSensorType), and re-armed the Apply & Reboot offer.
        for idx in received_u8:
            if idx not in self.params:
                continue
            widget = self.params[idx]
            try:
                widget.blockSignals(True)
                try:
                    if isinstance(widget, QDoubleSpinBox):
                        mult = self._display_mult(idx)
                        fc_float = received_float.get(idx, float(received_u8[idx]))
                        widget.setValue(fc_float * mult)
                        if idx in self._PROTECTED_PARAMS:
                            self._committed_values[idx] = widget.value()
                    elif isinstance(widget, QComboBox):
                        cb_idx = self._resolve_combo_index(widget, int(received_u8[idx]))
                        widget.setCurrentIndex(cb_idx)
                        if idx in self._PROTECTED_PARAMS:
                            self._committed_values[idx] = float(int(received_u8[idx]))
                finally:
                    widget.blockSignals(False)
            except Exception:
                pass

        # Update config values from FC typed data
        for cfg_idx in (ParamIndex.CONFIG1_BITS, ParamIndex.CONFIG2_BITS):
            ci = int(cfg_idx)
            if ci in received_float:
                self._config_values[ci] = int(received_float[ci])

        # Don't touch UI state during write — echo ACKs update widgets in-place
        if self._verify_mode or self._write_in_progress:
            return

        self.update_config_display()
        self.dirty_params.clear()
        self._params_received = True
        self.sync_setup_from_advanced()
        self.status_label.setText("✅ Parameters loaded from FC (typed)")
        self.status_label.setStyleSheet("color: #27ae60;")
        self.reset_read_button()
        self._read_request_id = None

    def update_params_from_packet(self, param_data):
        """Update parameters from a received packet - handles both verify and normal modes"""
        if param_data is None:
            self._log("⚠️ update_params_from_packet: param_data is None")
            return
        
        if not hasattr(param_data, 'params') or not hasattr(param_data, 'param_set'):
            self._log("⚠️ Invalid param_data: missing params or param_set")
            return
        
        params_list = param_data.params
        param_set = param_data.param_set
        version = param_data.version_name if hasattr(param_data, 'version_name') else ''
        
        self._log(f"📥 Updating params: Set={param_set}, Version='{version}', len={len(params_list)}")
        self._log(f"  🔍 _verify_mode={self._verify_mode}, _expecting_param_packet={self._expecting_param_packet}")
        
        time_since_write = time.time() - self._write_timestamp
        self._log(f"  ⏱️ Time since write: {time_since_write:.3f}s")
        
        received_params = {}
        for i in range(self.MAX_PARAMS):
            if i in self.params:
                received_params[i] = params_list[i] if i < len(params_list) else 0
        
        should_verify = (self._verify_mode and self._expecting_param_packet) or (self._write_timestamp > 0 and time_since_write < 2.0)
        
        if self._verification_timer:
            self._verification_timer.stop()
            self._verification_timer = None
        if self._write_timeout_timer:
            self._write_timeout_timer.stop()
            self._write_timeout_timer = None
        
        if should_verify and self._written_params:
            self._log(f"  🔍 VERIFY MODE: comparing {len(self._written_params)} written params")
            self._verify_parameters(received_params)
            return
        
        # Normal update mode - skip if verification is ongoing
        if self._verify_mode:
            self._log(f"  ⏭️ Skipping normal update - verification mode active")
            return
        
        update_count = 0
        for i in range(self.MAX_PARAMS):
            if i in self.params:
                widget = self.params[i]
                try:
                    value = received_params.get(i, 0)
                    if isinstance(widget, QDoubleSpinBox):
                        mult = self._display_mult(i)
                        widget.setValue(float(value) * mult)
                        update_count += 1
                    elif isinstance(widget, QComboBox):
                        cb_idx = self._resolve_combo_index(widget, int(value))
                        widget.setCurrentIndex(cb_idx)
                        update_count += 1
                except Exception as e:
                    self._log(f"  ⚠️ Failed to update param {i}: {e}")
        
        # Update config values from param data
        for cfg_idx in (ParamIndex.CONFIG1_BITS, ParamIndex.CONFIG2_BITS):
            ci = int(cfg_idx)
            if ci < len(params_list):
                self._config_values[ci] = params_list[ci]
        
        self._log(f"  ✅ Updated {update_count} parameters")
        self.update_config_display()
        self.sync_setup_from_advanced()
        self.dirty_params.clear()
        self._params_received = True
        self.status_label.setText(f"✅ Parameters loaded from FC (Set={param_set})")
        self.status_label.setStyleSheet("color: #27ae60;")
        self.reset_read_button()
        
        if self._read_timeout_timer:
            self._read_timeout_timer.stop()
            self._read_timeout_timer = None
        self._read_request_id = None
    
    def _verify_parameters(self, received_params, received_float=None):
        """Verify written parameters against received values
        
        For FLOAT params with available float data, compares in float32 space.
        Falls back to uint8 comparison for U8 params and when float data is missing.
        """
        self._log(f"  🔍 Verifying {len(self._written_params)} params...")

        self.write_progress.setValue(100)
        self.write_progress.setFormat("✅ Written to flash")
        QTimer.singleShot(2500, self.write_progress.hide)

        if len(self._written_params) == 0:
            self._log("  ⚠️ WARNING: No written params to verify!")
            self._verify_mode = False
            self._write_in_progress = False
            self._flash_write_pending = False
            self._expecting_param_packet = False
            self._write_timestamp = 0
            if self._write_timeout_timer:
                self._write_timeout_timer.stop()
                self._write_timeout_timer = None
            QMessageBox.warning(
                self,
                "⚠️ Verification Issue",
                "No parameters were stored for verification.\n\n"
                "This means the write may not have completed correctly.\n"
                "Please try writing again."
            )
            self.reset_write_button()
            return
        
        mismatches = []
        derived_warnings = []
        clamped_warnings = []
        mismatch_indices = []
        clamped_indices = set()

        # Derived params that FC recalculates via ApplyParameters
        derived_params = {
            ParamIndex.MAX_ROLL_RATE: "MaxRollRate",
            ParamIndex.MAX_PITCH_RATE: "MaxPitchRate",
            ParamIndex.HORIZON: "Horizon",
            ParamIndex.MADGWICK_KP_ACC: "MadgwickKpAcc",
            ParamIndex.ACC_CONF_SD: "AccConfSD",
            ParamIndex.FW_ROLL_PITCH_FF: "FWRollPitchFF",
        }

        # FC hard-clamps are detected generically: written value lands outside
        # PARAM_LIMITS (raw) and the received value equals the lo/hi ceiling.
        # No per-param hardcoded range table needed anymore.
        for idx, written_value in self._written_params.items():
            received_value = received_params.get(idx)
            if received_value is None:
                mismatches.append(f"{ParamIndex(idx).name}: Written={written_value}, Not received")
                mismatch_indices.append(idx)
                self._log(f"    ❌ {ParamIndex(idx).name}: Not received")
                continue

            # For FLOAT params with float data, compare in float32 space
            is_float_param = PARAM_TYPES.get(idx, 'U8') == 'FLOAT'
            float_available = (received_float is not None and idx in received_float
                               and idx in self._written_float)

            if is_float_param and float_available:
                wf = self._written_float[idx]
                rf = received_float[idx]
                if abs(wf - rf) < 1e-6:
                    self._log(f"    ✅ {ParamIndex(idx).name}: float {wf} == {rf}")
                    continue
                if idx in derived_params:
                    derived_warnings.append(
                        f"{ParamIndex(idx).name}: Written={wf} → "
                        f"Recalculated to {rf} (FC derived parameter)"
                    )
                    self._log(f"    ℹ️ {ParamIndex(idx).name}: {wf} → {rf} (derived)")
                    continue
                lo, hi = PARAM_LIMITS.get(idx, (0.0, 255.0))
                if (wf < lo and abs(rf - lo) < 1e-6) or (wf > hi and abs(rf - hi) < 1e-6):
                    clamped_warnings.append(
                        f"{ParamIndex(idx).name}: Written={wf} → "
                        f"Clamped to {rf} (range {lo:.6g}-{hi:.6g})")
                    clamped_indices.add(idx)
                    self._log(f"    ⚠️ {ParamIndex(idx).name}: {wf} → {rf} (clamped)")
                    continue
                mismatches.append(f"{ParamIndex(idx).name}: Written={wf}, Received={rf}")
                mismatch_indices.append(idx)
                self._log(f"    ❌ {ParamIndex(idx).name}: Written={wf}, Received={rf}")
                continue

            # Fall back to uint8 comparison (U8 params, or float data unavailable)
            if written_value != received_value:
                if idx in derived_params:
                    derived_warnings.append(
                        f"{ParamIndex(idx).name}: Written={written_value} → "
                        f"Recalculated to {received_value} (FC derived parameter)"
                    )
                    self._log(f"    ℹ️ {ParamIndex(idx).name}: {written_value} → {received_value} (derived)")
                    continue

                lo, hi = PARAM_LIMITS.get(idx, (0.0, 255.0))
                if (written_value < lo and received_value == int(lo)) or \
                        (written_value > hi and received_value == int(hi)):
                    clamped_warnings.append(
                        f"{ParamIndex(idx).name}: Written={written_value} → "
                        f"Clamped to {received_value} (range {lo:.6g}-{hi:.6g})")
                    clamped_indices.add(idx)
                    self._log(f"    ⚠️ {ParamIndex(idx).name}: {written_value} → {received_value} (clamped)")
                    continue

                mismatches.append(f"{ParamIndex(idx).name}: Written={written_value}, Received={received_value}")
                mismatch_indices.append(idx)
                self._log(f"    ❌ {ParamIndex(idx).name}: Written={written_value}, Received={received_value}")
            else:
                self._log(f"    ✅ {ParamIndex(idx).name}: {written_value} == {received_value}")
        
        self._verify_mode = False
        self._write_in_progress = False
        self._flash_write_pending = False
        self.reset_read_button()
        
        # Sync UI widgets with FC's actual values (in case of mismatches, clamps, or derived)
        self._suppress_sim_auto = True
        try:
            for i, value in received_params.items():
                if i not in self.params:
                    continue
                widget = self.params[i]
                try:
                    widget.blockSignals(True)
                    try:
                        if isinstance(widget, QDoubleSpinBox):
                            mult = self._display_mult(i)
                            if received_float is not None and i in received_float:
                                widget.setValue(received_float[i] * mult)
                            else:
                                widget.setValue(float(value) * mult)
                            if i in self._PROTECTED_PARAMS:
                                display_val = widget.value()
                                self._committed_values[i] = display_val
                        elif isinstance(widget, QComboBox):
                            cb_idx = self._resolve_combo_index(widget, int(value))
                            widget.setCurrentIndex(cb_idx)
                            if i in self._PROTECTED_PARAMS:
                                self._committed_values[i] = float(int(value))
                    finally:
                        widget.blockSignals(False)
                except Exception:
                    pass
                # Update _config_values cache for config bit params
                if i == int(ParamIndex.CONFIG1_BITS) or i == int(ParamIndex.CONFIG2_BITS):
                    old = self._config_values.get(i, -1)
                    if old != value:
                        self._log(f"  🔧 Config{i}: _config_values {old} → {value} (synced from FC)")
                    self._config_values[i] = int(value)
        finally:
            self._suppress_sim_auto = False
        if hasattr(self, '_update_fw_style'):
            self._update_fw_style()
        if hasattr(self, 'sync_setup_from_advanced'):
            self.sync_setup_from_advanced()

        # Red highlight = FC hard-clamped this param's write to the ceiling.
        # Persistent until the user edits the widget (param_changed restyles)
        # or a fresh load clears it (see _set_widgets_from_raw).
        for idx in clamped_indices:
            if idx not in self.params:
                continue
            widget = self.params[idx]
            if isinstance(widget, QDoubleSpinBox):
                widget.setStyleSheet("""
                    QDoubleSpinBox {
                        padding: 1px 2px;
                        background-color: #f2a0a0;
                        border: 1px solid #e74c3c;
                    }
                    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                        width: 12px;
                    }
                """)
        self._clamped_widgets = set(clamped_indices)

        self.update_config_display()
        self.dirty_params.clear()
        
        if self._verification_timer:
            self._verification_timer.stop()
            self._verification_timer = None
        
        # Show results
        if mismatches:
            msg = QMessageBox(self)
            msg.setWindowTitle("❌ Parameter Verification Failed")
            msg.setIcon(QMessageBox.Critical)
            msg.setText(f"Found {len(mismatches)} parameter mismatches!")
            
            detail_text = "MISMATCHES:\n" + "\n".join(mismatches[:20])
            if len(mismatches) > 20:
                detail_text += f"\n... and {len(mismatches) - 20} more"
            
            if derived_warnings:
                detail_text += "\n\nDERIVED (FC recalculated):\n" + "\n".join(derived_warnings[:10])
                if len(derived_warnings) > 10:
                    detail_text += f"\n... and {len(derived_warnings) - 10} more"
            
            if clamped_warnings:
                detail_text += "\n\nCLAMPED (FC applied limits):\n" + "\n".join(clamped_warnings[:10])
                if len(clamped_warnings) > 10:
                    detail_text += f"\n... and {len(clamped_warnings) - 10} more"
            
            msg.setDetailedText(detail_text)
            
            for idx in mismatch_indices[:10]:
                if idx in self.params:
                    widget = self.params[idx]
                    widget.setStyleSheet("background-color: #e74c3c; color: white; border: 2px solid #c0392b;")
                    QTimer.singleShot(5000, lambda w=widget: w.setStyleSheet(""))
            
            msg.exec_()
            self.status_label.setText(f"❌ Verification failed: {len(mismatches)} mismatches")
            self.status_label.setStyleSheet("color: #e74c3c;")

        elif derived_warnings or clamped_warnings:
            warning_text = ""
            if derived_warnings and clamped_warnings:
                warning_text = (
                    f"All parameters were written successfully.\n\n"
                    f"⚠️ {len(derived_warnings)} parameter(s) were recalculated by the FC:\n"
                    + "\n".join(derived_warnings[:5])
                    + (f"\n... and {len(derived_warnings)-5} more" if len(derived_warnings) > 5 else "")
                    + f"\n\n⚠️ {len(clamped_warnings)} value(s) were clamped by the FC to valid ranges."
                )
            elif derived_warnings:
                warning_text = (
                    f"All parameters were written successfully.\n\n"
                    f"⚠️ {len(derived_warnings)} parameter(s) were recalculated by the FC:\n"
                    + "\n".join(derived_warnings[:10])
                    + (f"\n... and {len(derived_warnings)-10} more" if len(derived_warnings) > 10 else "")
                )
            else:
                warning_text = (
                    f"All parameters were written successfully.\n\n"
                    f"⚠️ {len(clamped_warnings)} value(s) were clamped by the FC to valid ranges.\n\n"
                    + "\n".join(clamped_warnings[:10])
                    + (f"\n... and {len(clamped_warnings)-10} more" if len(clamped_warnings) > 10 else "")
                )
            
            QMessageBox.warning(
                self,
                "⚠️ Parameters Written with Adjustments",
                warning_text
            )
            self.status_label.setText(f"⚠️ Written OK - {len(derived_warnings)} derived, {len(clamped_warnings)} clamped")
            self.status_label.setStyleSheet("color: #f39c12;")

        else:
            QMessageBox.information(
                self,
                "✅ Verification Successful",
                f"All {len(self._written_params)} parameters verified successfully!"
            )
            self.status_label.setText(f"✅ All {len(self._written_params)} parameters verified OK!")
            self.status_label.setStyleSheet("color: #27ae60;")

    def reset_read_button(self):
        # Read button was removed (connect auto-populates the page via tag-71);
        # kept as a no-op so the state-reset call sites in the read/verify flow
        # remain unchanged.
        pass
    
    def on_param_commit_ack(self, ok):
        """Handle the tag-72 flash-commit ACK from the FC.

        ok=True  — config confirmed written to flash; the FC is about to reboot,
                   so the widget returns to the waiting state; the deferred
                   post-reboot tag-71 verification then confirms the flash load.
        ok=False — commit was NOT persisted (write/erase failed, or FC is in
                   flight). The FC does NOT reboot. Un-arm the deferred verify,
                   reset the button and show the failure loudly.
        """
        if ok:
            self._log("  ✅ Commit ACK — config confirmed in flash; waiting for FC reboot...")
            return

        self._log("  ❌ Commit ACK NACK — params were NOT saved to flash!")
        self.write_progress.hide()
        self._flash_write_pending = False
        self.reset_write_button()
        if self.parent_window:
            self.parent_window._pending_param_verification = False

    def reset_write_button(self):
        self._write_in_progress = False
        self.write_progress.hide()
    
    def _read_timeout(self):
        self._log("⏱️ Read timeout")
        self.status_label.setText("⚠️ Read timeout - will retry when FC data arrives")
        self.status_label.setStyleSheet("color: #e74c3c;")
        self.reset_read_button()
        self._read_request_id = None
        self._read_timeout_timer = None
        self._read_retry_pending = True

    def retry_read_if_pending(self):
        if getattr(self, '_read_retry_pending', False):
            self._log("🔄 FC data received - retrying param read")
            self._read_retry_pending = False
            self.read_params()
    
    def _write_timeout(self):
        self._log("⏱️ Write timeout - resetting")
        self._write_in_progress = False
        self._flash_write_pending = False
        if self._verify_mode:
            self._verify_mode = False
        self._written_params = {}
        self._written_float = {}
        self._expecting_param_packet = False
        self._write_timestamp = 0

        self.status_label.setText("⚠️ Write timeout - no response from FC")
        self.status_label.setStyleSheet("color: #e74c3c;")
        self.write_progress.setValue(0)
        self.write_progress.hide()
        self.reset_write_button()
        self._write_timeout_timer = None
    
    _CATEGORY_LABELS = {
        'MR': 'MR', 'FW': 'FW', 'VTOL': 'VTOL', 'LAND': 'Land', 'NONE': 'Sensor',
    }

    def _category_for_af_file(self, path):
        """Read AF_TYPE from an .af file and return its class label ('MR'/'FW'/...)"""
        try:
            with open(path) as f:
                for line in f:
                    line = line.strip()
                    if not line.startswith('AF_TYPE') or '=' not in line:
                        continue
                    val = line.split('=', 1)[1].strip()
                    try:
                        member = af_module._resolve_enum_member(AirframeType, val)
                        if member is not None:
                            af = member
                        else:
                            af = AirframeType(int(val))
                    except (KeyError, ValueError):
                        return 'MR'
                    cat = self._category_for_af(af.value)
                    return self._CATEGORY_LABELS.get(cat, 'MR')
        except OSError:
            pass
        return 'MR'


    
    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(4)
        main_layout.setContentsMargins(4, 4, 4, 4)
        
        toolbar = QHBoxLayout()
        toolbar.setSpacing(4)

        self.save_btn = QPushButton("Save Params")
        self.save_btn.setStyleSheet("padding: 4px 12px;")
        toolbar.addWidget(self.save_btn)
        
        self.load_btn = QPushButton("Load Airframe…")
        self.load_btn.setStyleSheet("padding: 4px 12px;")
        self.load_btn.setToolTip("Open the airframe library and load defaults + tuning limits")
        toolbar.addWidget(self.load_btn)
        
        self.airframe_name_label = QLabel("")
        self.airframe_name_label.setStyleSheet("color: #000; font-weight: normal;")
        self.airframe_name_label.setToolTip("Airframe name persisted in the FC config flash")
        self.airframe_name_label.hide()
        toolbar.addWidget(self.airframe_name_label)
        
        toolbar.addStretch()
        
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("color: #666;")
        toolbar.addWidget(self.status_label)

        self.write_progress = QProgressBar()
        self.write_progress.setRange(0, 100)
        self.write_progress.setValue(0)
        self.write_progress.setTextVisible(True)
        self.write_progress.setFormat("%p%")
        self.write_progress.setFixedWidth(200)
        self.write_progress.hide()
        toolbar.addWidget(self.write_progress)
        
        main_layout.addLayout(toolbar)
        
        # RC + Motors always visible (not scrollable)
        rc_motor_row = QHBoxLayout()
        rc_motor_row.setSpacing(0)
        rc_motor_row.setContentsMargins(0, 0, 0, 0)
        rc_combined = self._create_rc_combined_group()
        rc_combined.setFixedHeight(160)
        rc_motor_row.addWidget(rc_combined, 3)
        motor_group = self._create_motor_group()
        motor_group.setFixedHeight(160)
        rc_motor_row.addWidget(motor_group, 2)
        main_layout.addLayout(rc_motor_row)
        
        # Params page — Setup+Physics, Advanced toggle, and grid all inside here
        params_container = QWidget()
        self.param_layout = QVBoxLayout(params_container)
        self.param_layout.setSpacing(4)
        self.param_layout.setContentsMargins(2, 2, 2, 2)
        
        # Create the 2×4 grid FIRST (hidden until Advanced checked)
        # Must come before Setup group so self.params widgets exist
        self.create_parameter_groups()
        
        # Setup + Physics side by side
        setup_physics_row = QHBoxLayout()
        setup_physics_row.setSpacing(4)
        setup_group = self._create_setup_group()
        setup_physics_row.addWidget(setup_group, 1)
        physics_group = self._create_physics_group()
        setup_physics_row.addWidget(physics_group, 1)
        self.param_layout.addLayout(setup_physics_row)
        
        # AF_TYPE combo controls FW group styling
        af_combo = self._setup_widgets.get(int(ParamIndex.AF_TYPE))
        if af_combo and isinstance(af_combo, QComboBox):
            af_combo.currentIndexChanged.connect(self._update_fw_style)
        
        # Advanced toggle + Legacy, side by side
        advanced_row = QHBoxLayout()
        advanced_row.setSpacing(4)
        self._advanced_checkbox = QCheckBox("Advanced Parameters")
        self._advanced_checkbox.setToolTip("Show all parameter groups (PID, General, Config, etc.)")
        self._advanced_checkbox.setStyleSheet("font-weight: bold; padding: 2px;")
        self._advanced_checkbox.toggled.connect(self._on_advanced_toggled)
        advanced_row.addWidget(self._advanced_checkbox)

        self.legacy_check = QCheckBox("Legacy")
        self.legacy_check.setToolTip("Display legacy-tagged params as raw/scale (classic FC units)")
        self.legacy_check.toggled.connect(self._on_legacy_toggled)
        # Initial state mirrors __init__'s _legacy_mode (False = unified raw
        # display). Widgets are already built in unified units via _display_mult,
        # so set the visual state WITHOUT firing a redundant refresh (would
        # double-convert when the user later toggles legacy on).
        self.legacy_check.blockSignals(True)
        self.legacy_check.setChecked(self._legacy_mode)
        self.legacy_check.blockSignals(False)
        advanced_row.addWidget(self.legacy_check)
        advanced_row.addStretch(1)
        self.param_layout.addLayout(advanced_row)
        
        # Advanced grid goes below the checkbox (Aircraft Config is above it)
        if hasattr(self, '_grid_container'):
            self.param_layout.addWidget(self._grid_container)
        
        main_layout.addWidget(params_container)
        
        # Prevent RC/Motors from stretching vertically
        main_layout.addStretch(0)
        
        info_box = QGroupBox()
        info_box.setTitle("Info")
        info_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 4px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        info_layout = QVBoxLayout()
        info_layout.setContentsMargins(4, 8, 4, 4)
        self.info_text = QLabel("Hover over a parameter for description")
        self.info_text.setWordWrap(True)
        self.info_text.setStyleSheet("padding: 3px; background-color: #f8f8f8; color: #555;")
        info_layout.addWidget(self.info_text)
        info_box.setLayout(info_layout)
        main_layout.addWidget(info_box)
    
    def _on_advanced_toggled(self, checked):
        """Show/hide the advanced parameter grid. Window expands when Advanced is shown."""
        if hasattr(self, '_grid_container'):
            self._grid_container.setVisible(checked)
        if checked:
            self.adjustSize()
        else:
            self.adjustSize()
    
    def _show_param_info(self, idx):
        param = PARAMETER_DEFS.get(int(idx))
        if param and param.description:
            self.info_text.setText(f"<b>{param.name}</b>: {param.description}")
        elif param:
            self.info_text.setText(f"<b>{param.name}</b>")
        else:
            self.info_text.setText(f"Parameter {idx}")
    
    def eventFilter(self, obj, event):
        if event.type() == QEvent.Enter:
            idx = obj.property("param_index")
            if idx is not None:
                self._show_param_info(int(idx))
        elif event.type() == QEvent.Leave:
            self.info_text.setText("Hover over a parameter for description")
        return super().eventFilter(obj, event)
    
    def create_parameter_groups(self):
        """Create all parameter groups using enum references"""
        # Initialize all params with hidden spinboxes for backup
        for i in range(self.MAX_PARAMS):
            if i not in self.params:
                spin = QDoubleSpinBox()
                spin.setRange(0.0, 255.0)
                spin.setValue(0)
                spin.setProperty("param_index", i)
                spin.valueChanged.connect(lambda v, idx=i: self.param_changed(idx, v))
                spin.setMaximumWidth(70)
                spin.setMinimumWidth(62)
                spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                spin.hide()
                self.params[i] = spin
        
        # Main grid layout — two rows
        grid = QGridLayout()
        grid.setHorizontalSpacing(0)
        grid.setVerticalSpacing(2)
        self._param_grid = grid

        # Row 0: PID | General | Config | Battery | Filters
        pid_group = self._create_pid_group()
        grid.addWidget(pid_group, 0, 0)

        gen_group = self._create_general_group()
        grid.addWidget(gen_group, 0, 1)

        config_group = self._create_config_group()
        grid.addWidget(config_group, 0, 2)

        batt_group = self._create_compact_group("Battery", self.MISC2_PARAMS)
        grid.addWidget(batt_group, 0, 3)

        filt_group = self._create_filters_single_box()
        grid.addWidget(filt_group, 0, 4)

        # Row 1: Navigation | Altitude (2 cols) | Fixed Wing (2 cols)
        nav_group = self._create_two_col_compact_group("Navigation", self.NAVIGATION_PARAMS)
        grid.addWidget(nav_group, 1, 0)

        # Altitude - make two columns using the two-col compact helper
        alt_group = self._create_two_col_compact_group("Altitude", self.ALTITUDE_PARAMS)
        alt_group.setMaximumWidth(440)  # 2x 220
        grid.addWidget(alt_group, 1, 1)

        # Fixed Wing - double column box
        fw_group = self._create_fixed_wing_two_col_box()
        grid.addWidget(fw_group, 1, 2, 1, 2)

        # Make grid 4 columns
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 0)
        grid.setColumnStretch(2, 1)
        grid.setColumnStretch(3, 1)
        grid.setColumnStretch(4, 1)

        # Wrap grid in a widget so Advanced toggle can show/hide it
        grid_container = QWidget()
        grid_container.setLayout(grid)
        self._grid_container = grid_container
        grid_container.setVisible(False)
        # NOTE: added to param_layout in setup_ui after the Advanced checkbox so
        # the grid renders below it (RC Config -> Aircraft Config -> checkbox -> grid)
        
        # Connect AF_TYPE combo to FW styling
        QTimer.singleShot(0, self._update_fw_style)
    
    def _update_fw_style(self):
        for fw_group in (self._fw_left, self._fw_right):
            if fw_group is None:
                continue
            af_combo = self._setup_widgets.get(int(ParamIndex.AF_TYPE)) if hasattr(self, '_setup_widgets') else None
            if not isinstance(af_combo, QComboBox):
                af_combo = self.params.get(int(ParamIndex.AF_TYPE))
            if not isinstance(af_combo, QComboBox):
                fw_group.setVisible(False)
                continue
            data = af_combo.currentData()
            if data is None:
                fw_group.setVisible(False)
                continue
            fw_group.setVisible(AirframeType.eElevonAF <= int(data) <= AirframeType.eVTOL2AF)
        self._apply_pid_view()

    def _create_rc_combined_group(self):
        """Create combined RC configuration + live monitoring group"""
        group = QGroupBox("RC Configuration")
        group.setStyleSheet("""
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
        layout = QGridLayout()
        layout.setHorizontalSpacing(4)
        layout.setVerticalSpacing(2)
        layout.setContentsMargins(4, 10, 4, 10)

        for half in range(3):
            ci = half * 4
            layout.addWidget(self._header("µs"), 0, ci, Qt.AlignRight | Qt.AlignVCenter)
            layout.addWidget(self._header("Ch"), 0, ci + 1, Qt.AlignLeft | Qt.AlignVCenter)
            layout.addWidget(self._header("Func"), 0, ci + 3)
            layout.setColumnMinimumWidth(ci, 36)
            layout.setColumnMinimumWidth(ci + 2, 42)

        # Bar-column header doubles as the unit flip button (percent default)
        self.rc_unit_header = self._header("%")
        layout.addWidget(self.rc_unit_header, 0, 1, Qt.AlignLeft | Qt.AlignVCenter)
        self.rc_unit_btn = QToolButton()
        self.rc_unit_btn.setText("%")
        self.rc_unit_btn.setCheckable(True)
        self.rc_unit_btn.setAutoRaise(True)
        self.rc_unit_btn.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self.rc_unit_btn.setToolTip("Toggle Rx value display: % (default) or raw µs")
        self.rc_unit_btn.toggled.connect(self._toggle_rc_units)
        layout.addWidget(self.rc_unit_btn, 0, 5, Qt.AlignLeft | Qt.AlignVCenter)

        self.rc_channel_spins = []
        self.rc_value_labels = {}
        self.rc_progress_bars = {}
        for fn_idx, (param_idx, func_name, default_ch) in enumerate(self.RC_MAP_PARAMS):
            col_offset = 4 * (fn_idx // 6)
            row = (fn_idx % 6) + 1

            val = QLabel("---")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setFixedWidth(36)
            layout.addWidget(val, row, col_offset)
            self.rc_value_labels[fn_idx] = val

            bar = TickBar()
            bar.setRange(0, 1000, ticks=[500])
            bar.setValue(0)
            bar.setMinimumWidth(50)
            layout.addWidget(bar, row, col_offset + 1)
            self.rc_progress_bars[fn_idx] = bar

            spin = QSpinBox()
            spin.setRange(0, 15)
            spin.setValue(max(0, min(15, default_ch)))
            spin.setProperty("param_index", int(param_idx))
            spin.setProperty("fn_index", fn_idx)
            spin.valueChanged.connect(lambda v, i=int(param_idx), fn=fn_idx: self._rc_channel_changed(i, fn, v))
            spin.setToolTip(f"{func_name} channel slot (0 = first channel, 1..15 = subsequent)")
            spin.setMaximumWidth(42)
            spin.setMinimumWidth(38)
            spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            spin.setMaximumHeight(18)
            spin.setStyleSheet(self.RC_CH_STYLE)
            layout.addWidget(spin, row, col_offset + 2)
            self.params[int(param_idx)] = spin
            self.rc_channel_spins.append(spin)

            func_label = QLabel(func_name)
            func_label.setStyleSheet("font-weight: bold;")
            func_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            layout.addWidget(func_label, row, col_offset + 3)

        # Third column: spare channels 12-15, same cell layout, no assignment
        self.rc_spare_labels = []
        self.rc_spare_bars = []
        for i in range(4):
            row = i + 1
            val = QLabel("---")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setFixedWidth(36)
            layout.addWidget(val, row, 8)
            self.rc_spare_labels.append(val)

            bar = TickBar()
            bar.setRange(0, 1000, ticks=[500])
            bar.setValue(0)
            bar.setMinimumWidth(50)
            layout.addWidget(bar, row, 9)
            self.rc_spare_bars.append(bar)

            spin = QSpinBox()
            spin.setRange(0, 15)
            spin.setValue(12 + i)
            spin.setEnabled(False)
            spin.setMaximumWidth(42)
            spin.setMinimumWidth(38)
            spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            spin.setMaximumHeight(18)
            spin.setStyleSheet(self.RC_CH_STYLE)
            spin.setToolTip("Unmapped Rx channel (no function assigned)")
            layout.addWidget(spin, row, 10)

            func_label = QLabel(f"CH{13 + i}")
            func_label.setStyleSheet("font-weight: bold;")
            func_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            layout.addWidget(func_label, row, 11)

        layout.setColumnStretch(2, 1)
        layout.setColumnStretch(6, 1)
        layout.setColumnStretch(10, 1)
        layout.setRowStretch(7, 1)

        group.setLayout(layout)
        return group

    def _toggle_rc_units(self, checked):
        """Flip RC value display between percent (default) and raw microseconds"""
        unit = "µs" if checked else "%"
        self.rc_unit_btn.setText(unit)
        self.rc_unit_header.setText(unit)
        self.update_rc_display()

    def _create_motor_group(self):
        """Create motor/servo bargraph group"""
        group = QGroupBox("Motors / Servos")
        group.setStyleSheet("""
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
        motor_layout = QGridLayout()
        motor_layout.setHorizontalSpacing(3)
        motor_layout.setVerticalSpacing(1)
        motor_layout.setContentsMargins(4, 8, 4, 4)

        col_widths = [35, 55, 22]
        col_stretch = [0, 1, 0]
        for half in range(2):
            for c, w in enumerate(col_widths):
                ci = half * 3 + c
                motor_layout.setColumnMinimumWidth(ci, w)
                motor_layout.setColumnStretch(ci, col_stretch[c])

        motor_layout.addWidget(self._header("%"), 0, 0)
        # Bar column header doubles as the unit flip button (percent default)
        self.motor_unit_btn = QToolButton()
        self.motor_unit_btn.setText("%")
        self.motor_unit_btn.setCheckable(True)
        self.motor_unit_btn.setAutoRaise(True)
        self.motor_unit_btn.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self.motor_unit_btn.setToolTip("Toggle motor display: % (default) or raw µs")
        self.motor_unit_btn.toggled.connect(self._toggle_motor_units)
        motor_layout.addWidget(self.motor_unit_btn, 0, 1, Qt.AlignLeft | Qt.AlignVCenter)
        motor_layout.addWidget(self._header("Ch"), 0, 2)
        motor_layout.addWidget(self._header("%"), 0, 3)
        motor_layout.addWidget(self._header(""), 0, 4)
        motor_layout.addWidget(self._header("Ch"), 0, 5)

        # left-align Ch headers
        for col in [2, 5]:
            item = motor_layout.itemAtPosition(0, col)
            if item and item.widget():
                item.widget().setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        self.motor_names = ["FL", "BL", "FR", "BR", "M4", "M5", "M6", "M7", "M8", "M9", "M10", "M11"]
        for i in range(12):
            col_offset = 3 * (i // 6)
            row = (i % 6) + 1

            pct = QLabel("0%")
            pct.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            pct.setStyleSheet("font-weight: bold;")
            pct.setFixedWidth(35)
            motor_layout.addWidget(pct, row, col_offset)

            bar = TickBar()
            bar.setRange(0, 1000, ticks=[500])
            bar.setValue(0)
            bar.setMinimumWidth(50)
            bar.setMinimumHeight(18)
            motor_layout.addWidget(bar, row, col_offset + 1)

            name = QLabel(self.motor_names[i])
            name.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            name.setStyleSheet("font-weight: bold;")
            name.setMinimumWidth(22)
            motor_layout.addWidget(name, row, col_offset + 2)

            self.motor_name_labels.append(name)
            self.motor_pct_labels.append(pct)
            self.motor_bars.append(bar)

        motor_layout.setRowStretch(7, 1)
        group.setLayout(motor_layout)
        return group

    def _toggle_motor_units(self, checked):
        """Flip motor display between percent (default) and raw microseconds"""
        unit = "µs" if checked else "%"
        self.motor_unit_btn.setText(unit)
        if self._last_motor_fd is not None:
            self._update_motor_display(self._last_motor_fd)

    def _update_motor_display(self, flight_data):
        """Update motor/servo bargraphs from flight data"""
        self._last_motor_fd = flight_data
        show_us = self.motor_unit_btn.isChecked()
        has_nan = False
        if hasattr(flight_data, 'pwm') and flight_data.pwm:
            num_motors = min(len(flight_data.pwm), 10)
            for i in range(12):
                if i < num_motors:
                    # RawPW idle-zero-referenced: 0=1000uS, 0.5=1500uS,
                    # 1.0=2000uS
                    raw = flight_data.pwm[i]
                    if not math.isfinite(raw):
                        # A non-finite actuator value (seen 2026-09-18: the FC's
                        # emulator shipped NaN pwm) must never reach int()/round()
                        # here — this runs in a 50 ms QTimer slot, so an unhandled
                        # ValueError aborts the whole GCS. Show it as invalid and
                        # report it once per episode instead.
                        has_nan = True
                        self.motor_bars[i].setValue(0)
                        self.motor_pct_labels[i].setText("---")
                        self.motor_name_labels[i].setText(self.motor_names[i] if i < 4 else f"M{i}")
                        self.motor_pct_labels[i].setStyleSheet(
                            "background-color: #cc0000; color: white; font-weight: bold;"
                        )
                        continue
                    pwm_us = (raw + 1.0) * 1000.0
                    scaled_val = max(0, min(1000, int(pwm_us - 1000)))
                    pct = int(scaled_val / 10)
                    self.motor_bars[i].setRange(0, 1000, ticks=[500])
                    self.motor_bars[i].setValue(scaled_val)
                    label = f"{round(pwm_us)}" if show_us else f"{pct}%"
                    self.motor_pct_labels[i].setText(label)
                    self.motor_name_labels[i].setText(self.motor_names[i] if i < 4 else f"M{i}")
                    if pct < 10 or pct > 80:
                        bg = "#f39c12"
                    else:
                        bg = "#27ae60"
                    self.motor_pct_labels[i].setStyleSheet(
                        f"background-color: {bg}; color: white; font-weight: bold;"
                    )
                else:
                    self.motor_bars[i].setValue(0)
                    self.motor_pct_labels[i].setText("---" if show_us else "0%")
                    self.motor_name_labels[i].setText(self.motor_names[i] if i < 4 else f"M{i}")
                    self.motor_pct_labels[i].setStyleSheet(
                        "background-color: #999; font-weight: bold;"
                    )
            if has_nan:
                if not self._motor_nan_warned:
                    self._motor_nan_warned = True
                    self._log("Non-finite (NaN) motor/servo value from FC — "
                              "bars flagged invalid", "Error")
            else:
                self._motor_nan_warned = False
        else:
            for i in range(12):
                self.motor_bars[i].setValue(0)
                self.motor_pct_labels[i].setText("---" if show_us else "0%")
                self.motor_name_labels[i].setText(self.motor_names[i] if i < 4 else f"M{i}")
                self.motor_pct_labels[i].setStyleSheet(
                    "background-color: #999; font-weight: bold;"
                )
    
    def _create_setup_group(self):
        """Create Setup section with FC param widgets only"""
        group = QGroupBox("Aircraft Config")
        group.setStyleSheet("""
            QGroupBox { font-weight: bold; border: 1px solid #e74c3c; border-radius: 4px; margin-top: 6px; }
            QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; color: #e74c3c; }
        """)
        layout = QVBoxLayout()
        layout.setSpacing(3)
        layout.setContentsMargins(6, 10, 6, 6)

        self._setup_widgets = {}
        self._phys_widgets = {}
        self._setup_group = group
        self._setup_layout = layout

        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(2)
        grid.setContentsMargins(0, 0, 0, 0)

        setup_params = [
            (ParamIndex.RX_TYPE, "Rx Type", "combo", 0,
             list(RX_TYPE_NAMES.values()), None),
            (ParamIndex.FAILSAFE_ACTION, "FS Action", "combo", 0,
             list(FAILSAFE_ACTION_NAMES.values()), None),
            (ParamIndex.FAILSAFE_DELAY, "FS Delay (s)", "spin", 0, None, (2, 15, 5)),
            (ParamIndex.ESC_TYPE, "ESC Protocol", "combo", 0,
             list(ESC_TYPE_NAMES.values()), None),
            (ParamIndex.RF_SENSOR_TYPE, "RF Module", "combo", 0,
             list(RF_TYPE_NAMES.values()), None),
            (ParamIndex.AS_SENSOR_TYPE, "Airspeed Sensor", "combo", 0,
             list(AS_SENSOR_TYPE_NAMES.values()), None),
            (ParamIndex.BATTERY_CAPACITY, "Battery (mAh)", "spin", 0, None, None),
            (ParamIndex.NAV_MAG_VAR, "Mag Decl (°)", "spin", 1, None, None),
            (ParamIndex.VOLT_SCALE, "Volt Scale", "spin", 1, None, None),
            (ParamIndex.CURRENT_SCALE, "Curr Scale (A)", "spin", 1, None, None),
        ]
        for i, (idx, name, wtype, dec, items, data) in enumerate(setup_params):
            row = i // 3
            col = i % 3
            idx_int = int(idx)
            if wtype == "combo":
                combo = QComboBox()
                for j, item in enumerate(items):
                    d = data[j] if data is not None else j
                    combo.addItem(item, d)
                combo.setMaximumWidth(120)
                combo.setEditable(True)
                combo.lineEdit().setReadOnly(True)
                combo.lineEdit().setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                combo.setStyleSheet("QComboBox { padding: 1px 2px; } QComboBox::drop-down { width: 16px; }")
                combo.currentIndexChanged.connect(lambda v, ii=idx_int: self._setup_sync_to_advanced(ii))
                self._setup_widgets[idx_int] = combo
                grid.addWidget(combo, row, col * 2, Qt.AlignRight | Qt.AlignVCenter)
                lbl = QLabel(name)
                lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                lbl.setStyleSheet("font-weight: normal;")
                grid.addWidget(lbl, row, col * 2 + 1)
            else:
                spin = QDoubleSpinBox()
                if isinstance(data, tuple) and len(data) == 3:
                    spin.setRange(data[0], data[1])
                    spin.setValue(data[2])
                else:
                    spin.setRange(0, 9999)
                spin.setDecimals(dec)
                spin.setMaximumWidth(80)
                spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                spin.setStyleSheet("QDoubleSpinBox { padding: 1px 2px; }")
                spin.valueChanged.connect(lambda v, ii=idx_int: self._setup_sync_to_advanced(ii))
                self._setup_widgets[idx_int] = spin
                grid.addWidget(spin, row, col * 2, Qt.AlignRight | Qt.AlignVCenter)
                lbl = QLabel(name)
                lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                lbl.setStyleSheet("font-weight: normal;")
                grid.addWidget(lbl, row, col * 2 + 1)

        layout.addLayout(grid)

        # Character slider — dev-gated (2026-09-13 decision: HIDDEN in the shipped
        # GCS, see wiki/Session_Report_CascadeCrossCheck_Sep13.md §5.1c). The
        # fleet character-curve machinery computes gains from AF type + slider +
        # physics, but angle gains are now locked-derived
        # (QKp = RateMax/(2·sin(AngleMax/2))) and rate gains are pot-bracketed
        # (Ch10) + trace-measured per airframe, so the slider has no seat in the
        # field workflow. Set UAVXGS_DEV_TUNE_SLIDER=1 to restore the full row
        # (all widgets are still constructed so compute_defaults /
        # _get_scale_factors / reset_to_computed / metadata round-trip stay
        # intact and reachable in a dev session).
        self._tune_slider_dev = os.environ.get("UAVXGS_DEV_TUNE_SLIDER", "0") == "1"

        slider_row = QHBoxLayout()
        slider_row.setSpacing(4)
        lbl_con = QLabel("Steady")
        lbl_con.setStyleSheet("font-weight: normal; color: #666;")
        lbl_con.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        slider_row.addWidget(lbl_con)

        self._character_slider = QSlider(Qt.Horizontal)
        self._character_slider.setRange(0, 100)
        self._character_slider.setValue(50)
        self._character_slider.setTickPosition(QSlider.TicksBelow)
        self._character_slider.setTickInterval(10)
        self._character_slider.setMinimumWidth(80)
        self._character_slider.valueChanged.connect(self._on_character_slider)
        slider_row.addWidget(self._character_slider, 1)

        lbl_ag = QLabel("Frisky")
        lbl_ag.setStyleSheet("font-weight: normal; color: #666;")
        lbl_ag.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        slider_row.addWidget(lbl_ag)

        self._character_value_label = QLabel("50%")
        self._character_value_label.setMinimumWidth(28)
        self._character_value_label.setAlignment(Qt.AlignCenter)
        self._character_value_label.setStyleSheet("font-weight: bold; color: #3498db;")
        slider_row.addWidget(self._character_value_label)

        compute_btn = QPushButton("Compute")
        compute_btn.setToolTip("Compute tuning params from slider + physical descriptors")
        compute_btn.setStyleSheet("padding: 2px 6px; font-weight: bold;")
        compute_btn.clicked.connect(self.compute_defaults)
        slider_row.addWidget(compute_btn)

        reset_btn = QPushButton("Reset")
        reset_btn.setToolTip("Reset to last computed values")
        reset_btn.setStyleSheet("padding: 2px 6px;")
        reset_btn.clicked.connect(self.reset_to_computed)
        slider_row.addWidget(reset_btn)

        sim_btn = QPushButton("Sim")
        sim_btn.setToolTip("Run the full robustness report (2 seeds, detailed) — "
                           "tuning edits & Compute already auto-check in the status bar")
        sim_btn.setStyleSheet("padding: 2px 6px; font-weight: bold; color: #8e44ad;")
        sim_btn.clicked.connect(self.run_robustness_sim)
        self._sim_btn = sim_btn
        slider_row.addWidget(sim_btn)

        # Hidden by default: slider + Steady/Frisky/% + Compute/Reset. The Sim
        # button stays visible — it runs the robustness report, which is a
        # genuine tuning-check instrument independent of the fleet curves.
        if not self._tune_slider_dev:
            for w in (lbl_con, self._character_slider, lbl_ag,
                      self._character_value_label, compute_btn, reset_btn):
                w.setVisible(False)

        layout.addLayout(slider_row)

        group.setLayout(layout)
        return group

    def _create_physics_group(self):
        """Create Physics box: AF_TYPE on top, physical descriptors below"""
        group = QGroupBox("Physics")
        group.setStyleSheet("""
            QGroupBox { font-weight: bold; border: 1px solid #9b59b6; border-radius: 4px; margin-top: 6px; }
            QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; color: #9b59b6; }
        """)
        layout = QVBoxLayout()
        layout.setSpacing(3)
        layout.setContentsMargins(6, 10, 6, 6)

        self._phys_widgets = {}
        self._phys_group = group

        # AF_TYPE combo — label AFTER combo
        af_row = QHBoxLayout()
        af_row.setSpacing(4)
        af_combo = QComboBox()
        for af in ALL_AIRFRAMES:
            af_combo.addItem(AIRFRAME_NAMES[af], af.value)
        af_combo.setEditable(True)
        af_combo.lineEdit().setReadOnly(True)
        af_combo.lineEdit().setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        af_combo.setStyleSheet("QComboBox { padding: 1px 2px; } QComboBox::drop-down { width: 16px; }")
        af_combo.currentIndexChanged.connect(lambda v, ii=int(ParamIndex.AF_TYPE): self._setup_sync_to_advanced(ii))
        self._setup_widgets[int(ParamIndex.AF_TYPE)] = af_combo
        af_row.addWidget(af_combo)
        lbl = QLabel("Airframe Type")
        lbl.setStyleSheet("font-weight: normal;")
        af_row.addWidget(lbl)
        af_row.addStretch(1)
        layout.addLayout(af_row)

        # Physical descriptor grid — rebuilt when AF_TYPE changes
        self._phys_grid = QGridLayout()
        self._phys_grid.setHorizontalSpacing(6)
        self._phys_grid.setVerticalSpacing(2)
        self._phys_grid.setContentsMargins(0, 0, 0, 0)
        self._phys_row = 0
        self._build_phys_row()
        layout.addLayout(self._phys_grid)

        self._phys_hover_lbl = QLabel("")
        self._phys_hover_lbl.setStyleSheet("font-weight: bold; font-size: 10px;")
        self._phys_hover_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._phys_hover_lbl.hide()
        layout.addWidget(self._phys_hover_lbl)

        group.setLayout(layout)
        return group

    def _build_phys_row(self):
        """Build or rebuild the physical descriptor row based on current AF_TYPE category"""
        if not hasattr(self, '_phys_grid') or not hasattr(self, '_phys_row'):
            return
        grid = self._phys_grid
        row = self._phys_row

        # Clear old widgets from grid
        for key, widget in self._phys_widgets.items():
            grid.removeWidget(widget)
            widget.setParent(None)
            widget.deleteLater()
        self._phys_widgets.clear()

        # Clear old labels too
        while grid.count():
            item = grid.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)
                w.deleteLater()

        af_combo = self._setup_widgets.get(int(ParamIndex.AF_TYPE))
        cat = 'MR'
        af_type_int = 4  # default to Quad
        if af_combo and isinstance(af_combo, QComboBox):
            af_data = af_combo.currentData()
            if af_data is not None:
                af_type_int = int(af_data)
                cat = self._category_for_af(af_type_int)
        self._current_af_type = cat
        self._current_af_type_int = af_type_int

        if cat == 'FW':
            descriptors = self._FW_PHYS_DESCRIPTORS
        elif cat == 'LAND':
            descriptors = self._LAND_PHYS_DESCRIPTORS
        elif cat == 'NONE':
            descriptors = []
        else:
            # Use airframe-specific defaults if available, otherwise fall back to generic MR
            af_specific = self._MR_PHYS_DEFAULTS.get(self._current_af_type_int)
            if af_specific:
                descriptors = af_specific
            else:
                descriptors = self._MR_PHYS_DESCRIPTORS

        for i, (key, label, dec, default, tooltip) in enumerate(descriptors):
            row = i // 3
            col = i % 3
            if dec is None:
                edit = QLineEdit()
                edit.setText(str(default))
                edit.setMaximumWidth(70)
                edit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                edit.setToolTip(tooltip)
                edit.setStyleSheet("QLineEdit { padding: 1px 2px; }")
                edit.textChanged.connect(lambda t, k=key: self._phys_changed(k))
                self._phys_widgets[key] = edit
                grid.addWidget(edit, row, col * 2)
            else:
                spin = QDoubleSpinBox()
                spin.setDecimals(dec)
                spin.setMaximumWidth(70)
                spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                spin.setToolTip(tooltip)
                spin.setStyleSheet("QDoubleSpinBox { padding: 1px 2px; }")
                spin.valueChanged.connect(lambda v, k=key: self._phys_changed(k))
                # Sensible ranges to catch obviously wrong values
                if 'AUW' in key:
                    spin.setRange(50, 20000)
                elif 'ARM' in key or 'WING' in key or 'CHORD' in key or 'TRACK' in key:
                    spin.setRange(10, 5000)
                elif 'WHEEL' in key:
                    spin.setRange(1, 50)
                elif 'THRUST_G' in key:
                    spin.setRange(1, 5000)
                elif 'THRUST_A' in key:
                    spin.setRange(0.1, 500)
                elif 'THRUST_V' in key:
                    spin.setRange(1, 60)
                elif 'COUNT' in key:
                    spin.setRange(1, 16)
                else:
                    spin.setRange(0, 99999)
                spin.setValue(default)
                self._phys_widgets[key] = spin
                grid.addWidget(spin, row, col * 2)
            lbl = QLabel(label)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            lbl.setStyleSheet("font-weight: normal;")
            lbl.setToolTip(tooltip)
            grid.addWidget(lbl, row, col * 2 + 1)

    @staticmethod
    def _parse_prop(text: str) -> tuple:
        """Parse prop string 'DxP' or 'DxPxB' → (diameter, pitch, blades)."""
        text = text.strip()
        parts = text.split('x')
        try:
            dia = float(parts[0])
            pitch = float(parts[1]) if len(parts) > 1 else dia * 0.5
            blades = int(parts[2]) if len(parts) > 2 else 2
        except (ValueError, IndexError):
            return 10.0, 5.0, 2
        return dia, pitch, blades

    @staticmethod
    def _format_prop(dia: float, pitch: float, blades: int = 2) -> str:
        """Format (diameter, pitch, blades) → 'DxP' or 'DxPxB'."""
        dia_s = f"{dia:.0f}" if dia == int(dia) else f"{dia:.1f}"
        pitch_s = f"{pitch:.1f}" if pitch != int(pitch) else f"{pitch:.0f}"
        if blades == 2:
            return f"{dia_s}x{pitch_s}"
        return f"{dia_s}x{pitch_s}x{blades}"

    def _phys_changed(self, key):
        """Handle change to a physical descriptor — recompute derived values and check sanity"""
        phys_meta = {}
        for k, widget in self._phys_widgets.items():
            if isinstance(widget, QDoubleSpinBox):
                phys_meta[k] = str(int(widget.value())) if widget.decimals() == 0 else str(widget.value())
            elif isinstance(widget, QLineEdit):
                dia, pitch, blades = self._parse_prop(widget.text())
                phys_meta['PHYS_PROP_INCH'] = str(dia)
                phys_meta['PHYS_PROP_PITCH_INCH'] = str(pitch)
                phys_meta['PHYS_PROP_BLADES'] = str(blades)
        self._current_phys_meta = self._compute_physics_from_descriptors(phys_meta)

        if hasattr(self, '_phys_hover_lbl'):
            cat = self._category_for_af(self._current_af_type)
            if cat == 'NONE':
                self._phys_hover_lbl.hide()
                return
            thr = float(self._current_phys_meta.get('PHYS_CRUISE_THR') or self._current_phys_meta.get('PHYS_HOVER_THR', 0.5))
            label = "Est Cruise Throttle"
            self._phys_hover_lbl.setText(f"{label}: {thr:.0%}")
            if thr < self._HOVER_THR_MIN:
                self._phys_hover_lbl.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 10px;")
                self._phys_hover_lbl.show()
            elif thr > self._HOVER_THR_MAX:
                self._phys_hover_lbl.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 10px;")
                self._phys_hover_lbl.show()
            else:
                self._phys_hover_lbl.setStyleSheet("color: #27ae60; font-weight: bold; font-size: 10px;")
                self._phys_hover_lbl.show()

    # Old → new metadata key migration (backward compat with pre-existing .af files)
    _PHYS_KEY_MAP = {
        'PHYS_MASS_KG': 'PHYS_AUW_G',
        'PHYS_ARM_LEN_M': 'PHYS_ARM_MM',
        'PHYS_PROP_DIAMETER_INCH': 'PHYS_PROP_INCH',
    }
    _PHYS_CONVERSIONS = {
        'PHYS_MASS_KG': lambda v: float(v) * 1000,      # kg → g
        'PHYS_ARM_LEN_M': lambda v: float(v) * 1000,     # m → mm
        'PHYS_PROP_DIAMETER_INCH': lambda v: float(v),    # same units
    }

    def _load_phys_from_metadata(self, meta):
        """Populate physical descriptor widgets from .af metadata.

        Handles backward compatibility: converts old PHYS_MASS_KG→PHYS_AUW_G,
        PHYS_ARM_LEN_M→PHYS_ARM_MM, etc.
        """
        normalized = {}
        for k, v in meta.items():
            if k in self._PHYS_KEY_MAP:
                new_key = self._PHYS_KEY_MAP[k]
                try:
                    converted = self._PHYS_CONVERSIONS[k](v)
                    if new_key == 'PHYS_AUW_G':
                        normalized[new_key] = str(int(converted))
                    elif new_key == 'PHYS_ARM_MM':
                        normalized[new_key] = str(int(converted))
                    else:
                        normalized[new_key] = str(converted)
                except (ValueError, TypeError):
                    normalized[new_key] = v
            else:
                normalized[k] = v
        # Backward compat: old PHYS_MOTOR_W → derive thrust/current/voltage
        if 'PHYS_MOTOR_W' in normalized and 'PHYS_MOTOR_THRUST_G' not in normalized:
            try:
                motor_w = float(normalized.pop('PHYS_MOTOR_W'))
                normalized['PHYS_MOTOR_THRUST_G'] = str(int(motor_w * 3.0))
                normalized['PHYS_MOTOR_THRUST_A'] = f'{motor_w / 14.8:.1f}'
                normalized['PHYS_MOTOR_THRUST_V'] = '14.8'
            except (ValueError, TypeError):
                pass
        normalized.setdefault('PHYS_PROP_BLADES', '2')
        if 'PHYS_PROP_INCH' in normalized and 'PHYS_PROP_PITCH_INCH' not in normalized:
            try:
                normalized['PHYS_PROP_PITCH_INCH'] = str(float(normalized['PHYS_PROP_INCH']) / 2)
            except (ValueError, TypeError):
                pass
        for key, widget in self._phys_widgets.items():
            if isinstance(widget, QLineEdit) and key == 'PHYS_PROP':
                prop_in = normalized.get('PHYS_PROP_INCH')
                pitch_in = normalized.get('PHYS_PROP_PITCH_INCH')
                blades = normalized.get('PHYS_PROP_BLADES', '2')
                if prop_in and pitch_in:
                    text = self._format_prop(float(prop_in), float(pitch_in), int(blades))
                    widget.blockSignals(True)
                    widget.setText(text)
                    widget.blockSignals(False)
            elif key in normalized:
                try:
                    val = float(normalized[key])
                    widget.blockSignals(True)
                    widget.setValue(val)
                    widget.blockSignals(False)
                except (ValueError, TypeError):
                    pass
        self._phys_changed(next(iter(self._phys_widgets.keys()), None) or '')

    def _phys_to_metadata(self):
        """Collect current physical descriptor values as metadata dict"""
        result = {}
        for key, widget in self._phys_widgets.items():
            if isinstance(widget, QDoubleSpinBox):
                val = widget.value()
                result[key] = str(int(val)) if widget.decimals() == 0 else str(val)
            elif isinstance(widget, QLineEdit) and key == 'PHYS_PROP':
                dia, pitch, blades = self._parse_prop(widget.text())
                result['PHYS_PROP_INCH'] = str(dia)
                result['PHYS_PROP_PITCH_INCH'] = str(pitch)
                result['PHYS_PROP_BLADES'] = str(blades)
        if result:
            result = self._compute_physics_from_descriptors(result)
        return result

    def _setup_sync_to_advanced(self, idx_int):
        """Sync setup widget value → advanced grid widget"""
        setup_w = self._setup_widgets.get(idx_int)
        adv_w = self.params.get(idx_int)
        if not setup_w or not adv_w:
            return
        if isinstance(setup_w, QComboBox) and isinstance(adv_w, QComboBox):
            data = setup_w.currentData()
            if data is not None:
                adv_idx = self._resolve_combo_index(adv_w, int(data))
                adv_w.blockSignals(True)
                adv_w.setCurrentIndex(adv_idx)
                adv_w.blockSignals(False)
            self.dirty_params.add(idx_int)
            self.combo_changed(idx_int, int(data) if data is not None else 0)
            if idx_int == int(ParamIndex.AF_TYPE):
                self._build_phys_row()
        elif isinstance(setup_w, QDoubleSpinBox) and isinstance(adv_w, QDoubleSpinBox):
            val = setup_w.value()
            adv_w.blockSignals(True)
            adv_w.setValue(val)
            adv_w.blockSignals(False)
            self.dirty_params.add(idx_int)
            self.param_changed(idx_int, val)

    def sync_setup_from_advanced(self):
        """Sync advanced grid widget values → setup widgets (call after load/read)"""
        for idx_int, setup_w in self._setup_widgets.items():
            adv_w = self.params.get(idx_int)
            if not adv_w:
                continue
            if isinstance(setup_w, QComboBox) and isinstance(adv_w, QComboBox):
                adv_data = adv_w.currentData()
                if adv_data is not None:
                    s_idx = self._resolve_combo_index(setup_w, int(adv_data))
                    setup_w.blockSignals(True)
                    setup_w.setCurrentIndex(s_idx)
                    setup_w.blockSignals(False)
            elif isinstance(setup_w, QDoubleSpinBox) and isinstance(adv_w, QDoubleSpinBox):
                setup_w.blockSignals(True)
                setup_w.setValue(adv_w.value())
                setup_w.blockSignals(False)
        if hasattr(self, '_phys_grid'):
            self._build_phys_row()

    def _on_character_slider(self, value):
        """Update the character label when slider moves"""
        self._character_value_label.setText(f"{value}%")

    def compute_defaults(self):
        """Compute all tuning params from AF type + character slider position + physical descriptors.

        The _PARAM_CURVES define base ranges for a reference MR/FW.
        Physics scaling adjusts these ranges based on the actual aircraft's
        mass, arm/wingspan, etc. from the Setup physical descriptor fields.
        """
        slider_pct = self._character_slider.value() / 100.0
        af_combo = self.params.get(int(ParamIndex.AF_TYPE))
        cat = 'MR'
        af_type_int = 4
        if af_combo and isinstance(af_combo, QComboBox):
            af_data = af_combo.currentData()
            if af_data is not None:
                af_type_int = int(af_data)
                cat = self._category_for_af(af_type_int)

        scale_factors = self._get_scale_factors(cat)

        # Type-conditioned curves: FW airframes need different yaw/comp references
        # than the MR-anchored base dict (evidence: Phoenix yaw family).
        curves = dict(self._PARAM_CURVES)
        if cat == 'FW':
            curves.update(self._PARAM_CURVES_FW)

        computed = {}
        for idx_str, (conservative, aggressive) in curves.items():
            base_val = conservative + slider_pct * (aggressive - conservative)
            sf = scale_factors.get(idx_str, 1.0)
            computed[idx_str] = base_val * sf

        self._computed_values = computed.copy()
        self._computed_slider_pos = self._character_slider.value()

        for idx_str, display_val in computed.items():
            widget = self.params.get(idx_str)
            if widget is None:
                continue
            widget.blockSignals(True)
            if isinstance(widget, QDoubleSpinBox):
                widget.setValue(display_val)
            widget.blockSignals(False)
            self.dirty_params.add(idx_str)

        caveats = self._cascade_integrity_caveats(computed)
        self._cascade_warnings = caveats
        if caveats:
            caveat_str = "\n".join(caveats)
            print(f"⚠️ Cascade integrity caveats: {len(caveats)}")
            for c in caveats:
                print(f"   - {c}")
            if hasattr(self, '_sim_btn') and self._sim_btn:
                self._sim_btn.setText(f"Cascade caveats: {len(caveats)}")
            new_msg = self._last_caveat_msg != caveat_str
            self._last_caveat_msg = caveat_str
            self.status_label.setText(f"⚡ {len(caveats)} cascade caveat(s) — see console / sim button tooltip")
            self.status_label.setStyleSheet("color: #e67e22; font-weight: bold;")
            if self._warn_cascade_dialog and new_msg:
                QMessageBox.warning(self, "Cascade Integrity Caveats", caveat_str
                                    + "\n\nValues applied as-is — verify before flight, or reduce "
                                      "gains toward authority within the 20–30% band.")
        else:
            self._last_caveat_msg = ""
            self.status_label.setText(f"Computed {len(computed)} params (Character={self._character_slider.value()}%)")
            self.status_label.setStyleSheet("color: #3498db;")

        self.update_config_display()
        self._schedule_sim_check()

    def _cascade_integrity_caveats(self, computed):
        """Report cascade-integrity caveats for the just-computed tuning.

        Mirrors control.c:584 — the outer quaternion angle loop's rate setpoint is
        clamped to A[axis].R.Max for roll/pitch and  min(A[Yaw].R.Max,
        Nav.MaxHeadingRate) for yaw.  Authored rate limits are stored in FC raw
        rad/s with PARAM_DISPLAY_MULT 57.296 (deg/s display), so divide back.

        Checks:
          1. 2·sin(Θmax/2)·Kp  ≤ maxCmdRate   (P-path must not exceed the cascade cap)
          2. IntLim           ≤ maxCmdRate  (hard windup guard: an I-term above the
             clamp can never discharge → saturates the loop)
          3. IntLim           ≥ 0.20·maxCmdRate *and* ≤ 0.30·maxCmdRate
                       (tune rule — keep QP the main driver, I just trims)

        Returns a list of caveat strings (empty = OK).
        """
        RAD2DEG = 57.29577951308232

        # 3 angle-axes: tag of QP, QI-limit, max-angle (yaw: use π, Qa saturates 1)
        axes = [
            # (axis-key, Kp-tag, I-tag, intLim-tag, maxAngle-tag, maxRate-tag(s))
            ("Roll", int(ParamIndex.ROLL_ANGLE_Q_KP), int(ParamIndex.ROLL_ANGLE_Q_KI),
             int(ParamIndex.ROLL_ANGLE_Q_INT_LIMIT), int(ParamIndex.MAX_ROLL_ANGLE),
             (int(ParamIndex.MAX_ROLL_RATE),)),
            ("Pitch", int(ParamIndex.PITCH_ANGLE_Q_KP), int(ParamIndex.PITCH_ANGLE_Q_KI),
             int(ParamIndex.PITCH_ANGLE_Q_INT_LIMIT), int(ParamIndex.MAX_PITCH_ANGLE),
             (int(ParamIndex.MAX_PITCH_RATE),)),
            ("Yaw", int(ParamIndex.YAW_ANGLE_Q_KP), int(ParamIndex.YAW_ANGLE_Q_KI),
             int(ParamIndex.YAW_ANGLE_Q_INT_LIMIT), None,
             (int(ParamIndex.MAX_YAW_RATE), int(ParamIndex.MAX_HEADING_RATE))),
        ]
        caveats = []
        for name, kp_tag, ki_tag, il_tag, ang_tag, rate_tags in axes:
            kp = computed.get(kp_tag)
            il = computed.get(il_tag)
            if kp is None or il is None:
                continue
            # max commanded rate (rad/s) = min of the rate caps
            caps = []
            for rt in rate_tags:
                v = computed.get(rt)
                if v is None:
                    v = self._param_widget_display(rt)
                if v is not None and v > 0:
                    caps.append(v)
            if not caps:
                continue
            max_cmd = min(caps) / RAD2DEG
            if max_cmd <= 0:
                continue
            # angle max (rad)
            if ang_tag is not None:
                a_deg = computed.get(ang_tag)
                if a_deg is not None:
                    ang_rad = a_deg / RAD2DEG
                    qp_demand = 2.0 * math.sin(ang_rad / 2.0) * kp
                else:
                    qp_demand = 2.0 * kp
            else:
                # Yaw: up to 180° heading error → sin(π/2)=1 → Qa=1
                qp_demand = 2.0 * kp
            # Item 1: P-path
            if qp_demand > max_cmd * 1.02:
                caveats.append(
                    f"{name}: P-path can demand {qp_demand:.2f} rad/s at max error "
                    f"> max commanded {max_cmd:.2f} rad/s — gain saturates the rate cap")
            # Item 2: hard windup clamp
            if il > max_cmd:
                caveats.append(
                    f"{name}: I-limit {il:.2f} rad/s exceeds max commanded "
                    f"{max_cmd:.2f} rad/s — integrator can never discharge (windup)")
            # Item 3: 20–30% band tune rule
            elif il > 0.30 * max_cmd:
                caveats.append(
                    f"{name}: I-limit {il:.2f} rad/s = {100.0*il/max_cmd:.0f}% of "
                    f"max commanded {max_cmd:.2f} rad/s — above the 20–30% band "
                    f"(keep QP the main driver)")
            elif il < 0.20 * max_cmd:
                # I-limit below the band is acceptable (I only trims) — informational
                pass
        return caveats

    def _param_widget_display(self, idx: int) -> Optional[float]:
        """Read a param's current display-float if it isn't in the computed set."""
        if not hasattr(self, 'params'):
            return None
        w = self.params.get(idx)
        if w is None:
            return None
        if isinstance(w, QDoubleSpinBox):
            return w.value()
        if isinstance(w, QComboBox):
            try:
                return float(w.currentData())
            except (TypeError, ValueError):
                return None
        return None

    def _check_cascade_sanity(self):
        """Trigger a cascade caveat recheck when any angle/rate spinbox fires.

        Reads the LIVE widget values (manual edits, not just Compute) and
        reports whether the current slider/tune pushes past the 20–30% I rule
        or the P-path rate cap.
        """
        tag_values = {}
        for t in (int(ParamIndex.ROLL_ANGLE_Q_KP), int(ParamIndex.ROLL_ANGLE_Q_KI),
                  int(ParamIndex.ROLL_ANGLE_Q_INT_LIMIT), int(ParamIndex.MAX_ROLL_ANGLE),
                  int(ParamIndex.PITCH_ANGLE_Q_KP), int(ParamIndex.PITCH_ANGLE_Q_KI),
                  int(ParamIndex.PITCH_ANGLE_Q_INT_LIMIT), int(ParamIndex.MAX_PITCH_ANGLE),
                  int(ParamIndex.YAW_ANGLE_Q_KP), int(ParamIndex.YAW_ANGLE_Q_KI),
                  int(ParamIndex.YAW_ANGLE_Q_INT_LIMIT),
                  int(ParamIndex.MAX_ROLL_RATE), int(ParamIndex.MAX_PITCH_RATE),
                  int(ParamIndex.MAX_YAW_RATE), int(ParamIndex.MAX_HEADING_RATE)):
            w = self.params.get(t)
            if w is not None and isinstance(w, QDoubleSpinBox):
                tag_values[t] = w.value()
        caveats = self._cascade_integrity_caveats(tag_values)
        self._cascade_warnings = caveats
        if hasattr(self, '_sim_btn') and self._sim_btn:
            default_text = getattr(self, '_DEFAULT_SIM_BTN_TEXT', "Run Robustness Sim")
            self._sim_btn.setText(f"Cascade caveats: {len(caveats)}" if caveats else default_text)
        return caveats

    def _get_scale_factors(self, cat):
        """Compute per-param scaling factors from physical descriptors.

        Returns {param_idx: scale_factor}. Base curves are for the reference
        aircraft; scale_factor adjusts for actual aircraft physics.

        MR scaling (direction only — magnitudes are clamped to 0.5–2.0, see
        the evidence note at the tail of this function):
          - Rate Kp/Kd: ∝ 1/inertia → lighter/shorter arms → lower inertia → HIGHER gains
          - Angle Kp/Ki: ∝ mass → heavier → more authority
          - Rate limits: ∝ 1/mass → lighter → faster response possible
          - Alt Kp/Ki: NOT scaled — fleet runs 1.83/0.005 flat across 30–1344 g
            (matches critic-validated .af files; do not add a mass term)
          - Nav Kp/Ki: ∝ 1/mass → lighter → faster nav response

        FW scaling:
          - Rate Kp/Kd: ∝ 1/Ixx → larger wingspan → more inertia → LOWER gains
          - Angle Kp/Ki: ∝ mass → heavier → more authority
          - Rate limits: ∝ 1/mass → lighter → faster response
          - Alt Kp/Ki: ∝ mass
          - Nav Kp/Ki: ∝ 1/cruise_speed → faster cruise → quicker nav
        """
        sf = {}
        try:
            phys = self._current_phys_meta if hasattr(self, '_current_phys_meta') else {}
        except AttributeError:
            phys = {}

        try:
            auw_g = float(phys.get('PHYS_AUW_G', 0))
        except (ValueError, TypeError):
            auw_g = 0

        if auw_g <= 0:
            return sf

        mass_kg = auw_g / 1000.0

        if cat == 'MR':
            ref_mass = self._REF_MR_AUW_G / 1000.0
            try:
                arm_mm = float(phys.get('PHYS_ARM_MM', self._REF_MR_ARM_MM))
            except (ValueError, TypeError):
                arm_mm = self._REF_MR_ARM_MM
            ref_arm = self._REF_MR_ARM_MM / 1000.0
            actual_arm = arm_mm / 1000.0

            mass_ratio = mass_kg / ref_mass if ref_mass > 0 else 1.0
            inertia_ratio = (actual_arm / ref_arm) ** 2 * mass_ratio if ref_arm > 0 else 1.0

            for idx in self._PARAM_CURVES:
                pname = ''
                try:
                    pname = ParamIndex(idx).name
                except (ValueError, AttributeError):
                    pass

                if 'RATE_KP' in pname or 'RATE_KD' in pname:
                    sf[idx] = 1.0 / inertia_ratio if inertia_ratio > 0 else 1.0
                elif 'ANGLE_Q_KP' in pname or 'ANGLE_Q_KI' in pname or 'ANGLE_Q_INT_LIMIT' in pname:
                    sf[idx] = mass_ratio
                elif 'MAX_ROLL_RATE' in pname or 'MAX_PITCH_RATE' in pname or 'MAX_HEADING' in pname:
                    sf[idx] = 1.0 / mass_ratio if mass_ratio > 0 else 1.0

        elif cat == 'FW':
            ref_mass = self._REF_FW_AUW_G / 1000.0
            try:
                wingspan_mm = float(phys.get('PHYS_WINGSPAN_MM', self._REF_FW_WINGSPAN_MM))
            except (ValueError, TypeError):
                wingspan_mm = self._REF_FW_WINGSPAN_MM
            ref_wingspan = self._REF_FW_WINGSPAN_MM / 1000.0
            actual_wingspan = wingspan_mm / 1000.0

            mass_ratio = mass_kg / ref_mass if ref_mass > 0 else 1.0
            inertia_ratio = (actual_wingspan / ref_wingspan) ** 2 * mass_ratio if ref_wingspan > 0 else 1.0

            try:
                cruise_ms = float(phys.get('PHYS_CRUISE_MS', 12))
            except (ValueError, TypeError):
                cruise_ms = 12

            for idx in self._PARAM_CURVES:
                pname = ''
                try:
                    pname = ParamIndex(idx).name
                except (ValueError, AttributeError):
                    pass

                if 'RATE_KP' in pname or 'RATE_KD' in pname:
                    sf[idx] = 1.0 / inertia_ratio if inertia_ratio > 0 else 1.0
                elif 'ANGLE_Q_KP' in pname or 'ANGLE_Q_KI' in pname or 'ANGLE_Q_INT_LIMIT' in pname:
                    sf[idx] = mass_ratio
                elif 'MAX_ROLL_RATE' in pname or 'MAX_PITCH_RATE' in pname or 'MAX_HEADING' in pname:
                    sf[idx] = 1.0 / mass_ratio if mass_ratio > 0 else 1.0

        # Fleet evidence (Aug 29 audit, wiki/Session_Report_CritiqueRetune_Aug29.md):
        # the pure ratio model spans 800x across our 30–1344 g fleet, but actual
        # critic-validated gains span <6x (roll rate gains are nearly flat — thrust
        # scales with mass, so plant gain is roughly size-invariant). Clamp the
        # factor so compute_defaults stays near-flyable for extreme frames; the
        # .af files + critic remain the real per-frame authority.
        for idx in sf:
            sf[idx] = min(2.0, max(0.5, sf[idx]))
        return sf

    def reset_to_computed(self):
        """Revert all params and slider back to last computed values"""
        if not self._computed_values:
            QMessageBox.information(self, "No Computed Values",
                "Click 'Compute' first to generate values from the slider.")
            return
        # Restore slider position
        if hasattr(self, '_computed_slider_pos') and self._character_slider:
            self._character_slider.setValue(self._computed_slider_pos)
        for idx_str, display_val in self._computed_values.items():
            widget = self.params.get(idx_str)
            if widget is None:
                continue
            widget.blockSignals(True)
            if isinstance(widget, QDoubleSpinBox):
                widget.setValue(display_val)
            widget.blockSignals(False)
            self.dirty_params.add(idx_str)
        self.status_label.setText(f"Reset {len(self._computed_values)} params to computed values")
        self.status_label.setStyleSheet("color: #3498db;")
        self.update_config_display()

    def run_robustness_sim(self):
        """Run the robustness simulator for the currently selected airframe.

        Spawns a background thread (the sim is pure-Python) and posts the
        result back to the UI via QTimer. Uses tests/test_robustness_sim.py.
        """
        af_path = getattr(self, '_current_airframe_path', None)
        if not af_path or not os.path.exists(af_path):
            af_path = os.path.join(self._generic_dir, 'Quad.af')
        if not os.path.exists(af_path):
            QMessageBox.warning(self, "Robustness Sim",
                f"Airframe file not found:\n{af_path}\n\n"
                "Run 'Compute' first or load an airframe from the pulldown.")
            return

        btn = getattr(self, '_sim_btn', None)
        if btn is not None:
            btn.setEnabled(False)
        self._set_sim_button_state('running')
        self._sim_busy = True
        self.status_label.setText(f"Running robustness sim for {os.path.basename(af_path)} ...")
        self.status_label.setStyleSheet("color: #8e44ad;")

        import threading

        def worker():
            lines = ["<no result>"]
            all_pass = False
            try:
                from tests.test_robustness_sim import quick_check
                all_pass, lines, _summary = quick_check(af_path, seeds=2)
            except Exception as e:
                all_pass = False
                lines = [f"Simulator error: {e}"]
            self.sim_finished.emit(all_pass, lines, "", True)

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    def _on_sim_finished(self, all_pass, lines, summary, manual):
        self._sim_busy = False
        self._set_sim_button_state('pass' if all_pass else 'fail')
        mark = "OK" if all_pass else "FAIL"
        color = "#27ae60" if all_pass else "#e74c3c"
        self.status_label.setText(f"Robustness {mark} — {summary}")
        self.status_label.setStyleSheet(f"color: {color};")
        if manual:
            btn = getattr(self, '_sim_btn', None)
            if btn is not None:
                btn.setEnabled(True)
            if not all_pass:
                msg = ("\n".join(lines)
                       + "\n\n"
                       + "❌ SOME CHECKS FAIL")
                QMessageBox.warning(self, "Robustness Sim Failed", msg)

    def _schedule_sim_check(self):
        """Debounced auto-trigger of the robustness safety check.

        Fired after Compute and on tuning spinbox edits; re-arms so the sim
        only runs once the user pauses. Skipped during bulk widget
        population (FC reads, .af loads) via _suppress_sim_auto.
        """
        if not hasattr(self, '_sim_debounce') or self._suppress_sim_auto:
            return
        self._sim_debounce.start()

    def _set_sim_button_state(self, state):
        """Color the Sim button: 'running' orange, 'pass' green, 'fail' red."""
        if not hasattr(self, '_sim_btn') or self._sim_btn is None:
            return
        if state == 'running':
            self._sim_btn.setStyleSheet(
                "padding: 2px 6px; font-weight: bold; color: #6b4f00; background-color: #f5c46b;")
        elif state == 'pass':
            self._sim_btn.setStyleSheet(
                "padding: 2px 6px; font-weight: bold; color: white; background-color: #27ae60;")
        elif state == 'fail':
            self._sim_btn.setStyleSheet(
                "padding: 2px 6px; font-weight: bold; color: white; background-color: #e74c3c;")
        else:
            self._sim_btn.setStyleSheet(
                "padding: 2px 6px; font-weight: bold; color: #8e44ad;")

    def _run_auto_sim(self):
        """Non-intrusive robustness check (debounce timeout).

        Background thread runs quick_check(seeds=1, ~0.6s) and posts a
        compact PASS/FAIL readout to the status label — no dialog. Deferred
        (re-armed) while a manual sim is running.
        """
        if self._sim_busy:
            self._sim_debounce.start(300)
            return
        af_path = getattr(self, '_current_airframe_path', None)
        if not af_path or not os.path.exists(af_path):
            af_path = os.path.join(self._generic_dir, 'Quad.af')
        if not os.path.exists(af_path):
            return
        self._sim_busy = True
        self._set_sim_button_state('running')
        self.status_label.setText(f"Robustness check: {os.path.basename(af_path)} ...")
        self.status_label.setStyleSheet("color: #8e44ad;")

        import threading

        def worker():
            all_pass, summary = False, "sim error"
            try:
                from tests.test_robustness_sim import quick_check
                all_pass, _lines, summary = quick_check(af_path, seeds=1)
            except Exception as e:
                summary = f"sim error: {e}"
            self.sim_finished.emit(all_pass, [], summary, False)

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    def _category_for_af(self, af_type):
        """Return 'MR', 'FW', 'VTOL', 'LAND', or 'NONE' for a given AF type enum value.

        Delegates to the single authority category_of() in protocol_enums.py
        (mirror of FC ClassifyAFType() in params.c). 'NONE' is a GCS-only
        refinement for eInstrumentation (sensor pack — drives no motors).
        """
        try:
            from protocol_enums import AirframeType, category_of, AirframeCategory
            af = AirframeType(af_type)
            if af is AirframeType.eInstrumentation:
                return 'NONE'
            return {
                AirframeCategory.eCatMr: 'MR',
                AirframeCategory.eCatFw: 'FW',
                AirframeCategory.eCatVtol: 'VTOL',
                AirframeCategory.eCatLand: 'LAND',
            }[category_of(af)]
        except (ValueError, AttributeError, KeyError):
            return 'MR'

    def _compute_physics_from_descriptors(self, phys_meta):
        """Compute derived quantities from physical descriptors.

        Takes a dict of PHYS_* metadata strings, computes inertias, TWR, etc.
        Returns a dict of all PHYS_* values (user inputs + computed).
        Updates the input dict in-place with computed values.
        """
        result = dict(phys_meta)
        try:
            auw_g = float(result.get('PHYS_AUW_G', 800))
        except (ValueError, TypeError):
            auw_g = 800
        mass_kg = auw_g / 1000.0

        # Determine category from AF type
        af_combo = self.params.get(int(ParamIndex.AF_TYPE))
        cat = 'MR'
        af_type_int = 4
        if af_combo and isinstance(af_combo, QComboBox):
            af_data = af_combo.currentData()
            if af_data is not None:
                af_type_int = int(af_data)
                cat = self._category_for_af(af_type_int)

        if cat == 'MR':
            try:
                arm_mm = float(result.get('PHYS_ARM_MM', 220))
            except (ValueError, TypeError):
                arm_mm = 220
            arm_m = arm_mm / 1000.0
            try:
                prop_in = float(result.get('PHYS_PROP_INCH', 11))
            except (ValueError, TypeError):
                prop_in = 11
            try:
                thrust_g = float(result.get('PHYS_MOTOR_THRUST_G', 300))
            except (ValueError, TypeError):
                thrust_g = 300
            try:
                current_a = float(result.get('PHYS_MOTOR_THRUST_A', 20))
            except (ValueError, TypeError):
                current_a = 20
            try:
                voltage_v = float(result.get('PHYS_MOTOR_THRUST_V', 14.8))
            except (ValueError, TypeError):
                voltage_v = 14.8
            motor_w = current_a * voltage_v

            try:
                n_motors = int(result.get('PHYS_MOTOR_COUNT', 0))
            except (ValueError, TypeError):
                n_motors = 0
            if n_motors == 0:
                n_motors = self._MR_MOTOR_COUNT.get(af_type_int, 4)
            shape_kf = self._MR_SHAPE_KF.get(n_motors, 0.55)

            # Moments of inertia (flat plate approximation)
            ixx = mass_kg * arm_m * arm_m * shape_kf
            iyy = ixx
            izz = ixx * 1.6

            # Direct thrust-based physics
            thrust_per_motor_n = thrust_g * 9.81 / 1000.0
            total_thrust_n = thrust_per_motor_n * n_motors
            twr = total_thrust_n / (mass_kg * 9.81) if mass_kg > 0 else 5.0

            # Hover throttle from thrust fraction (thrust ∝ power^(2/3))
            thrust_hover_n = (mass_kg * 9.81) / n_motors if n_motors > 0 else mass_kg * 9.81
            hover_thr = (thrust_hover_n / thrust_per_motor_n) ** 1.5 if thrust_per_motor_n > 0 else 0.5

            result['PHYS_MOTOR_COUNT'] = str(n_motors)
            result['PHYS_SHAPE_KF'] = f'{shape_kf:.3f}'
            result['PHYS_IROLL'] = f'{ixx:.6f}'
            result['PHYS_IPITCH'] = f'{iyy:.6f}'
            result['PHYS_IYAW'] = f'{izz:.6f}'
            result['PHYS_MAX_THRUST_N'] = f'{total_thrust_n:.2f}'
            result['PHYS_TWR'] = f'{twr:.2f}'
            result['PHYS_HOVER_THR'] = f'{hover_thr:.3f}'
            result['PHYS_MASS_KG'] = f'{mass_kg:.3f}'
            result['PHYS_ARM_LEN_M'] = f'{arm_m:.4f}'
            result['PHYS_PROP_DIAMETER_INCH'] = str(int(prop_in))
            result['PHYS_MOTOR_THRUST_G'] = str(int(thrust_g))
            result['PHYS_MOTOR_THRUST_A'] = f'{current_a:.1f}'
            result['PHYS_MOTOR_THRUST_V'] = f'{voltage_v:.1f}'

        elif cat == 'FW':
            try:
                wingspan_mm = float(result.get('PHYS_WINGSPAN_MM', 1800))
            except (ValueError, TypeError):
                wingspan_mm = 1800
            wingspan_m = wingspan_mm / 1000.0
            try:
                chord_mm = float(result.get('PHYS_CHORD_MM', 250))
            except (ValueError, TypeError):
                chord_mm = 250
            chord_m = chord_mm / 1000.0
            try:
                prop_in = float(result.get('PHYS_PROP_INCH', 10))
            except (ValueError, TypeError):
                prop_in = 10
            try:
                thrust_g = float(result.get('PHYS_MOTOR_THRUST_G', 800))
            except (ValueError, TypeError):
                thrust_g = 800
            try:
                current_a = float(result.get('PHYS_MOTOR_THRUST_A', 30))
            except (ValueError, TypeError):
                current_a = 30
            try:
                voltage_v = float(result.get('PHYS_MOTOR_THRUST_V', 14.8))
            except (ValueError, TypeError):
                voltage_v = 14.8
            motor_w = current_a * voltage_v

            wing_area = wingspan_m * chord_m

            # Moments of inertia (thin plate approximation)
            ixx = mass_kg * wingspan_m * wingspan_m / 12.0
            iyy = mass_kg * (wingspan_m * wingspan_m + chord_m * chord_m) / 12.0
            izz = mass_kg * wingspan_m * wingspan_m / 6.0

            # Cruise speed: V = sqrt(2mg / (rho * S * CL))
            rho = 1.225
            cl_cruise = 0.5
            v_cruise = (2.0 * mass_kg * 9.81 / (rho * wing_area * cl_cruise)) ** 0.5

            # Dynamic pressure at cruise
            qbar = 0.5 * rho * v_cruise * v_cruise

            # Direct thrust-based physics
            thrust_n = thrust_g * 9.81 / 1000.0

            result['PHYS_WINGSPAN_M'] = f'{wingspan_m:.4f}'
            result['PHYS_CHORD_M'] = f'{chord_m:.4f}'
            result['PHYS_WING_AREA'] = f'{wing_area:.4f}'
            result['PHYS_IROLL'] = f'{ixx:.6f}'
            result['PHYS_IPITCH'] = f'{iyy:.6f}'
            result['PHYS_IYAW'] = f'{izz:.6f}'
            result['PHYS_CRUISE_MS'] = f'{v_cruise:.1f}'
            result['PHYS_QBAR'] = f'{qbar:.1f}'
            result['PHYS_MAX_THRUST_N'] = f'{thrust_n:.2f}'
            result['PHYS_MASS_KG'] = f'{mass_kg:.3f}'
            result['PHYS_PROP_DIAMETER_INCH'] = str(int(prop_in))
            result['PHYS_MOTOR_THRUST_G'] = str(int(thrust_g))
            result['PHYS_MOTOR_THRUST_A'] = f'{current_a:.1f}'
            result['PHYS_MOTOR_THRUST_V'] = f'{voltage_v:.1f}'

            # FW cruise power and endurance
            # Typical drag coefficient for clean airframe
            cd = 0.04
            drag_n = qbar * wing_area * cd
            cruise_power_w = drag_n * v_cruise / 0.30  # η=0.30 (ESC×motor×prop)
            cruise_thr = cruise_power_w / max(motor_w, 10.0)
            result['PHYS_CRUISE_POWER_W'] = f'{cruise_power_w:.1f}'
            result['PHYS_CRUISE_THR'] = f'{cruise_thr:.3f}'

            # Endurance if battery capacity and voltage known
            try:
                bat_mah = float(result.get('PHYS_BATTERY_MAH', 0))
                bat_v = float(result.get('PHYS_BATT_V', 3.7))
                if bat_mah > 0 and bat_v > 0:
                    bat_wh = bat_mah * bat_v / 1000.0
                    endurance_min = bat_wh / cruise_power_w * 60.0
                    result['PHYS_ENDURANCE_MIN'] = f'{endurance_min:.1f}'
                    result['PHYS_CRUISE_CURRENT_A'] = f'{cruise_power_w / bat_v:.2f}'
            except (ValueError, TypeError):
                pass

        elif cat == 'LAND':
            try:
                track_mm = float(result.get('PHYS_TRACK_MM', 300))
            except (ValueError, TypeError):
                track_mm = 300
            track_m = track_mm / 1000.0
            try:
                wheel_dia_mm = float(result.get('PHYS_WHEEL_DIA', 100))
            except (ValueError, TypeError):
                wheel_dia_mm = 100

            # Simple box inertia for land vehicle
            ixx = mass_kg * track_m * track_m / 12.0
            iyy = mass_kg * track_m * track_m / 12.0
            izz = mass_kg * (track_m * track_m + track_m * track_m) / 12.0

            result['PHYS_TRACK_M'] = f'{track_m:.4f}'
            result['PHYS_IROLL'] = f'{ixx:.6f}'
            result['PHYS_IPITCH'] = f'{iyy:.6f}'
            result['PHYS_IYAW'] = f'{izz:.6f}'
            result['PHYS_MASS_KG'] = f'{mass_kg:.3f}'

        return result

    def _add_pid_row(self, layout, row, entry):
        """Add the three axis spins + parameter label for one PID_PARAMS row."""
        label, roll_idx, pitch_idx, yaw_idx, roll_def, pitch_def, yaw_def = entry
        self.add_spin(layout, roll_idx, roll_def, row, 0)
        self.add_spin(layout, pitch_idx, pitch_def, row, 1)
        self.add_spin(layout, yaw_idx, yaw_def, row, 2)
        lbl = QLabel(label)
        lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(lbl, row, 3)

    def _create_pid_group(self):
        """Create PID group using enum definitions.

        Layout (top to bottom): column labels (Roll/Pitch/Yaw/Parameter) at the very
        top, then the Max Rate and Max Angle rows (the derivation inputs — always
        visible/editable), then the View: flick switch, then the gain rows. Two view
        modes behind the switch:
        - Angle view (default): every gain/law spin editable (readback/sync/live-write
          exactly as always).
        - Rate view: the roll/pitch angle spinboxes (Q Angle, Ki Angle, I-Limit) are
          hidden and shown as read-only derived labels computed live from the editable
          MaxRate / MaxAngle spins (QAngleKp = RateMax/(2·sin(AngleMax/2)),
          KiAngle = k·Kp_angle, I-Limit = 0.01·AngleMax). Yaw stays editable (no yaw
          angle-max input exists). Derived labels are pure display — the same spins stay
          in self.params so readback/sync/live-write see both views identically (no
          widget duplication, per the two-view design, Session_Report_TuningCascade_Sep11).
        """
        group = QGroupBox("PID (Roll / Pitch / Yaw)")
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(2)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(6, 10, 6, 6)
        
        layout.setColumnMinimumWidth(0, 50)
        layout.setColumnMinimumWidth(1, 50)
        layout.setColumnMinimumWidth(2, 50)
        layout.setColumnMinimumWidth(3, 85)

        # Column labels at the very top
        layout.addWidget(self._header("Roll"), 0, 0)
        layout.addWidget(self._header("Pitch"), 0, 1)
        layout.addWidget(self._header("Yaw"), 0, 2)
        layout.addWidget(self._header("Parameter"), 0, 3)

        # Derivation-input rows first (always visible/editable, above everything else)
        pid_rest = [e for e in self.PID_PARAMS if e[0] != "Max Rate (Deg/Sec)"]
        mr = next(e for e in self.PID_PARAMS if e[0] == "Max Rate (Deg/Sec)")
        self._add_pid_row(layout, 1, mr)
        # Max Angle row (yaw: no max-angle input — dash)
        ma_roll = self.add_spin(layout, ParamIndex.MAX_ROLL_ANGLE, 45, 2, 0)
        ma_pitch = self.add_spin(layout, ParamIndex.MAX_PITCH_ANGLE, 45, 2, 1)
        dash = QLabel("--")
        dash.setStyleSheet("color: #999;")
        dash.setAlignment(Qt.AlignCenter)
        layout.addWidget(dash, 2, 2)
        lbl = QLabel("Max Angle")
        lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(lbl, 2, 3)

        # View flick switch (below the derivation-input rows)
        view_lbl = QLabel("View:")
        view_lbl.setStyleSheet("font-weight: bold; color: #555;")
        view_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(view_lbl, 3, 0)
        self._pid_view_combo = QComboBox()
        self._pid_view_combo.setObjectName("pidViewCombo")
        self._pid_view_combo.addItem("Angle (edit)", "angle")
        self._pid_view_combo.addItem("Rate (derived)", "rate")
        self._pid_view_combo.setToolTip(
            "Angle view: tune every gain directly (default).\n"
            "Rate view: roll/pitch angle spins (Q Angle Kp, Ki Angle, I-Limit) hidden,\n"
            "shown read-only as values derived from Max Rate / Max Angle; yaw Ki Angle\n"
            "derived from yaw Q Angle Kp (fleet constant 0.083), yaw Q Angle Kp and\n"
            "yaw I-Limit stay editable (no yaw angle-max input).\n"
            "On fixed-wing frames the derived Ki / I-Limit rows are disabled "
            "(not-applied \u2014 FW angle loop is P-only).")
        self._pid_view_combo.currentIndexChanged.connect(lambda *_: self._apply_pid_view())
        layout.addWidget(self._pid_view_combo, 3, 1, 1, 2)

        # Apply-Derived button: materialise the Rate-view derived values into the
        # real spins (via the normal param_changed live-write path), but only after
        # an explicit confirmation — the one gated write. After it succeeds the
        # page is a normal editable grid (auto-flip to Angle view).
        self._apply_derived_btn = QPushButton("Apply Derived")
        self._apply_derived_btn.setObjectName("pidApplyDerived")
        self._apply_derived_btn.setToolTip(
            "Write the currently derived values into the real parameter spins "
            "(one confirmed write to the FC; afterwards the page is a normal "
            "editable grid \u2014 subsequent edits are live-written as usual).\n"
            "Dormant rows (Ki / I-Limit on fixed-wing frames) are skipped.")
        self._apply_derived_btn.clicked.connect(self._on_apply_pid_derived)
        layout.addWidget(self._apply_derived_btn, 3, 3)

        # Gain rows (dependent on the view mode)
        self._pid_derived = []  # (label, col, derived_label, spin, param_idx) roll/pitch Q-gain rows
        self._pid_yaw_derived = []  # (label, derived_label, spin, param_idx) yaw Q-gain derived rows (Ki, I-Limit)
        qgain_row_labels = {"Q Angle", "Ki Angle", "I-Limit"}
        yaw_kp_spin = None  # the yaw QAngleKp spin (derivation input for yaw Ki)
        for off, (label, roll_idx, pitch_idx, yaw_idx, roll_def, pitch_def, yaw_def) in enumerate(pid_rest, 4):
            roll_spin = self.add_spin(layout, roll_idx, roll_def, off, 0)
            pitch_spin = self.add_spin(layout, pitch_idx, pitch_def, off, 1)
            yaw_spin = self.add_spin(layout, yaw_idx, yaw_def, off, 2)
            if label == "Q Angle":
                yaw_kp_spin = yaw_spin
            lbl = QLabel(label)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            layout.addWidget(lbl, off, 3)

            if label in qgain_row_labels:
                for col, spin, pidx in ((0, roll_spin, roll_idx), (1, pitch_spin, pitch_idx)):
                    dlbl = QLabel("")
                    dlbl.setObjectName("pidDerived")
                    dlbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                    dlbl.setStyleSheet("color: #2c6fb3; background: transparent;")
                    dlbl.setMinimumWidth(62)
                    dlbl.setMaximumWidth(70)
                    layout.addWidget(dlbl, off, col)
                    dlbl.hide()
                    self._pid_derived.append((label, col, dlbl, spin, int(pidx)))
                # Yaw soaks BOTH angle rows from the fleet: Ki = 0.083·Kp_yaw
                # (yaw Qa authority ratio constant) and I-Limit = 0.03 (fleet
                # mode; the yaw heading-hold integral cap, independent of Kp).
                # No yaw angle-max exists (heading unbounded) so yaw QAngleKp
                # stays the editable derivation input (the "Q Angle" row).
                if label in ("Ki Angle", "I-Limit") and yaw_kp_spin is not None:
                    dlbl = QLabel("")
                    dlbl.setObjectName("pidDerived")
                    dlbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                    dlbl.setStyleSheet("color: #2c6fb3; background: transparent;")
                    dlbl.setMinimumWidth(62)
                    dlbl.setMaximumWidth(70)
                    layout.addWidget(dlbl, off, 2)
                    dlbl.hide()
                    self._pid_yaw_derived.append((label, dlbl, yaw_spin, int(yaw_idx)))
                    if label == "Ki Angle":
                        yaw_kp_spin.valueChanged.connect(self._refresh_pid_derived)

        group.setLayout(layout)

        # Derived labels track the editable Max Angle / Max Rate spins live
        # (these are the derivation inputs, and they drive both views).
        for idx in (ParamIndex.MAX_ROLL_RATE, ParamIndex.MAX_PITCH_RATE,
                    ParamIndex.MAX_ROLL_ANGLE, ParamIndex.MAX_PITCH_ANGLE):
            spin = self.params.get(int(idx))
            if isinstance(spin, QDoubleSpinBox):
                spin.valueChanged.connect(self._refresh_pid_derived)

        self._apply_pid_view()
        return group

    def _current_af_category(self) -> str:
        """Current airframe category ('MR', 'FW', 'VTOL', 'LAND', 'NONE'),
        mirroring FC ClassifyAFType(). Falls back to 'MR' (the default)."""
        combo = None
        if hasattr(self, "_setup_widgets"):
            combo = self._setup_widgets.get(int(ParamIndex.AF_TYPE))
        if not isinstance(combo, QComboBox):
            combo = self.params.get(int(ParamIndex.AF_TYPE))
        if isinstance(combo, QComboBox):
            data = combo.currentData()
            if data is not None:
                try:
                    return self._category_for_af(int(data))
                except (ValueError, TypeError):
                    pass
        return 'MR'

    def _apply_pid_view(self):
        """Apply the flick-switch view mode to the PID grid.

        Angle view: derived labels hidden, Q-gain spins visible (editable).
        Rate view: roll/pitch Q-gain spins hidden, derived labels shown. Yaw Ki Angle
        and Yaw I-Limit are derived from the fleet (0.083·Kp_yaw and 0.03) from the
        editable yaw QAngleKp spin — still visible as the derivation input (no yaw
        angle-max exists). FW frames: derived Ki / I-Limit rows disabled
        (not-applied — FW angle loop is P-only, dormant Ki).
        """
        rate = False
        combo = getattr(self, "_pid_view_combo", None)
        if isinstance(combo, QComboBox):
            rate = combo.currentData() == "rate"
        self._pid_view_is_rate = rate
        cat = self._current_af_category()

        self._refresh_pid_derived()

        for label, _col, dlbl, spin, _idx in getattr(self, "_pid_derived", []):
            if not rate:
                dlbl.hide()
                spin.show()
                continue
            spin.hide()
            dlbl.show()
            if label in ("Ki Angle", "I-Limit") and cat == 'FW':
                dlbl.setEnabled(False)
                dlbl.setToolTip("Not applied on the FW angle loop (P-only \u2014 dormant Ki).")
            else:
                dlbl.setEnabled(True)
                dlbl.setToolTip("Derived in Rate view from Max Angle / Max Rate.")

        for label, dlbl, spin, _idx in getattr(self, "_pid_yaw_derived", []):
            if not rate:
                dlbl.hide()
                spin.show()
                continue
            spin.hide()
            dlbl.show()
            if cat == 'FW':
                dlbl.setEnabled(False)
                dlbl.setToolTip("Not applied on the FW angle loop (P-only \u2014 dormant Ki).")
            else:
                dlbl.setEnabled(True)
                if label == "Ki Angle":
                    dlbl.setToolTip("Yaw angle Ki derived from the fleet (0.083 \u00b7 yaw QAngleKp).")
                else:
                    dlbl.setToolTip("Yaw I-Limit derived from the fleet (0.03 \u2014 yaw heading-hold integral cap).")

    def _refresh_pid_derived(self):
        """Recompute the Rate-view derived labels from the editable Max Angle / Max
        Rate spins (display deg / deg/s converted to rad / rad/s), in the same display
        units as the spins they replace: QAngleKp = RateMax/(2·sin(AngleMax/2)),
        KiAngle = k·Kp_angle (MR/VTOL 0.026, FW 0.05), I-Limit = 0.01·AngleMax.
        Yaw: Ki = 0.083·Kp_yaw and I-Limit = 0.03 (fleet constants), both from the
        editable yaw QAngleKp spin. Values are also cached in `_pid_derived_last`
        ({param_idx: display_value}) so the Apply-Derived button can materialise
        exactly what the labels show.
        """
        derived = getattr(self, "_pid_derived", None)
        yaw_derived = getattr(self, "_pid_yaw_derived", None)
        if not derived and not yaw_derived:
            return

        self._pid_derived_last = {}

        def spin_val(pidx):
            w = self.params.get(int(pidx))
            return w.value() if isinstance(w, QDoubleSpinBox) else 0.0

        deg2rad = math.pi / 180.0
        cat = self._current_af_category()
        k = 0.026 if cat != 'FW' else 0.05

        for label, col, dlbl, _spin, pidx in derived:
            if col == 0:
                rmax = spin_val(ParamIndex.MAX_ROLL_RATE) * deg2rad
                amax = spin_val(ParamIndex.MAX_ROLL_ANGLE) * deg2rad
            else:
                rmax = spin_val(ParamIndex.MAX_PITCH_RATE) * deg2rad
                amax = spin_val(ParamIndex.MAX_PITCH_ANGLE) * deg2rad
            if amax <= 1e-9:
                dlbl.setText("")
                continue
            denom = 2.0 * math.sin(amax * 0.5)
            if label == "Q Angle":
                val = (rmax / denom) if denom > 1e-12 else 0.0
            elif label == "Ki Angle":
                qkp = (rmax / denom) if denom > 1e-12 else 0.0
                val = k * qkp
            else:  # I-Limit
                val = 0.01 * amax
            dlbl.setText(f"{val:.4f}")
            self._pid_derived_last[int(pidx)] = val

        yaw_kp = spin_val(ParamIndex.YAW_ANGLE_Q_KP)
        for label, dlbl, _spin, pidx in yaw_derived:
            if label == "Ki Angle":
                val = self._YAW_ANGLE_KI_KP_RATIO * yaw_kp
            else:  # I-Limit — fleet mode, independent of Kp
                val = 0.03
            dlbl.setText(f"{val:.4f}")
            self._pid_derived_last[int(pidx)] = val

    def _on_apply_pid_derived(self):
        """Apply-Derived button: materialise the currently derived values into the
        real parameter spins, after an explicit confirmation.

        One gated write: the spin setValue calls fire the normal
        valueChanged -> param_changed live-write path (debounced, echo-ACK), so the
        derived values reach the FC via the standard transport. No confirmation is
        shown for subsequent edits — after Apply the page is a normal editable grid.
        Dormant rows (FW Ki/I-Limit, and all yaw angle rows on FW) are skipped.
        """
        derived = getattr(self, "_pid_derived", [])
        yaw_derived = getattr(self, "_pid_yaw_derived", [])
        last = getattr(self, "_pid_derived_last", {})
        if not last:
            self._refresh_pid_derived()
            last = getattr(self, "_pid_derived_last", {})

        cat = self._current_af_category()
        fw = cat == 'FW'

        changes = []  # (axis, label, param_idx, spin, old_display, new_display)
        for label, col, dlbl, spin, pidx in derived:
            if fw and label in ("Ki Angle", "I-Limit"):
                continue  # dormant on the FW angle loop
            idx = int(pidx)
            if idx not in last:
                continue
            new_d = last[idx]
            old_d = spin.value() if isinstance(spin, QDoubleSpinBox) else 0.0
            if abs(new_d - old_d) < 1e-9:
                continue  # already equal — nothing to write
            axis = "Roll" if col == 0 else "Pitch"
            changes.append((axis, label, idx, spin, old_d, new_d))

        for label, dlbl, spin, pidx in yaw_derived:
            if fw:
                continue  # yaw angle rows dormant on FW (aMax = eRoll)
            idx = int(pidx)
            if idx not in last:
                continue
            new_d = last[idx]
            old_d = spin.value() if isinstance(spin, QDoubleSpinBox) else 0.0
            if abs(new_d - old_d) < 1e-9:
                continue
            changes.append(("Yaw", label, idx, spin, old_d, new_d))

        if not changes:
            QMessageBox.information(self, "Apply Derived",
                                    "Derived values already match the current "
                                    "settings \u2014 nothing to write.")
            return

        lines = ["Overwrite these angle gains in the FC (one confirmed write)?\n"]
        for axis, label, _idx, _spin, old_d, new_d in changes:
            lines.append(f"  {axis} {label}: {old_d:.4f} \u2192 {new_d:.4f}")
        lines.append("")
        lines.append("After applying, the page returns to normal editing \u2014 "
                     "subsequent changes are live-written without confirmation.")

        reply = QMessageBox.question(
            self, "Apply Derived", "\n".join(lines),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        for _axis, _label, idx, spin, _old_d, new_d in changes:
            if isinstance(spin, QDoubleSpinBox):
                spin.setValue(new_d)

        combo = getattr(self, "_pid_view_combo", None)
        if isinstance(combo, QComboBox) and combo.currentData() != "angle":
            combo.setCurrentIndex(combo.findData("angle"))

        self._refresh_pid_derived()

    def _create_fixed_wing_group(self):
        group = QGroupBox("Fixed Wing")
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(2)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(4, 8, 4, 4)

        for i, (idx, name, default) in enumerate(self.FW_PARAMS, 1):
            self.add_spin(layout, idx, default, i, 0)
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            layout.addWidget(lbl, i, 1)

        group.setLayout(layout)
        return group

    def _create_general_group(self):
        group = QGroupBox("General")
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(4, 8, 4, 4)

        for row, param in enumerate(self.GENERAL_PARAMS):
            if len(param) == 5:
                idx, name, default, items, data = param
                self.add_combo(layout, name, idx, default, items, row, data=data)
            elif len(param) == 4:
                idx, name, default, items = param
                self.add_combo(layout, name, idx, default, items, row)
            else:
                idx, name, default = param
                self.add_spin_label(layout, name, idx, default, row)

        for row, param in enumerate(self.GENERAL_CAMERA_PARAMS):
            if len(param) == 5:
                idx, name, default, items, data = param
                self.add_combo(layout, name, idx, default, items, row, col_offset=2, data=data)
            elif len(param) == 4:
                idx, name, default, items = param
                self.add_combo(layout, name, idx, default, items, row, col_offset=2)
            else:
                idx, name, default = param
                self.add_spin_label(layout, name, idx, default, row, col_offset=2)

        group.setLayout(layout)
        return group

    def _create_filters_group(self):
        group = QGroupBox("Filters")
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(2)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(4, 8, 4, 4)

        combo_idx = 0
        for param in self.FILTER_PARAMS:
            if len(param) == 4:
                idx, name, default, items = param
                self.add_combo(layout, name, idx, default, items, combo_idx)
            else:
                idx, name, default = param
                self.add_spin_label(layout, name, idx, default, combo_idx)
            combo_idx += 1

        group.setLayout(layout)
        return group

    def _create_fixed_wing_two_col_box(self):
        # Group FF parameters together in left column, others in right
        ff_params = []
        other_params = []
        for p in self.FW_PARAMS:
            name = p[1]
            if "FF" in name or "Mix" in name or "Diff" in name or "Spoiler" in name or "Decay" in name:
                ff_params.append(p)
            else:
                other_params.append(p)

        group = QGroupBox("Fixed Wing")
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(2)      # Match Altitude
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(4, 8, 4, 4)  # Match Altitude
        layout.setColumnMinimumWidth(0, 50)
        layout.setColumnMinimumWidth(2, 50)

        # Left column: FF params
        for i, (idx, name, default) in enumerate(ff_params):
            self.add_spin(layout, idx, default, i, 0)
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            layout.addWidget(lbl, i, 1)

        # Right column: other params
        for i, (idx, name, default) in enumerate(other_params):
            self.add_spin(layout, idx, default, i, 2)
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            layout.addWidget(lbl, i, 3)

        # Servo Sense — FC pServoSense bitmask (tag 51), surfaced as labelled
        # checkboxes instead of a raw 0-127 number.  InitServoSense() maps each
        # bit to one servo output via SM[] = {RightAileronC, LeftAileronC,
        # ElevatorC, RudderC, SpoilerC, CamRollC, Aux1CamPitchC}: bit SET =
        # reversed sense (PWSense = -1.0), bit clear = normal (+1.0).  Now
        # recomputed immediately on a live tag-17 write (telem.c
        # ProcessParamsWrite), so no boot/commit is needed to take effect.
        ss_row = max(len(ff_params), len(other_params)) + 1
        ss_title = QLabel("Servo Sense")
        ss_title.setStyleSheet("font-weight: bold;")
        layout.addWidget(ss_title, ss_row, 0, 1, 4)

        self.servo_sense_checks = []
        servo_sense_bits = [
            ("Right Aileron", 0), ("Left Aileron", 1), ("Elevator", 2),
            ("Rudder", 3), ("Spoiler", 4), ("Cam Roll", 5), ("Cam Pitch", 6),
        ]
        for i, (name, bit) in enumerate(servo_sense_bits):
            cb = QCheckBox(name)
            cb.setProperty("bit", bit)
            cb.setToolTip("Reverses this servo output's sense on the FC.\n"
                          "Checked = bit SET = reversed (PWSense = -1).\n"
                          "Applies immediately (live param write).")
            cb.stateChanged.connect(lambda s, b=bit: self.servo_sense_bit_changed(b, s))
            layout.addWidget(cb, ss_row + 1 + i // 2, (i % 2) * 2, 1, 2)
            self.servo_sense_checks.append(cb)

        group.setLayout(layout)
        return group

    def _create_filters_single_box(self):
        group = QGroupBox("Filters")
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(2)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(4, 8, 4, 4)
        layout.setColumnMinimumWidth(0, 50)
        layout.setColumnMinimumWidth(1, 80)

        for i, param in enumerate(self.FILTER_PARAMS, 1):
            if len(param) == 4:
                idx, name, default, items = param
                self.add_combo(layout, name, idx, default, items, i)
            else:
                idx, name, default = param
                self.add_spin(layout, idx, default, i, 0)
                lbl = QLabel(name)
                lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                layout.addWidget(lbl, i, 1)

        group.setLayout(layout)
        return group

    def _create_filters_two_col_box(self):
        # Split FILTER_PARAMS into two columns
        mid = len(self.FILTER_PARAMS) // 2 + len(self.FILTER_PARAMS) % 2
        left_params = self.FILTER_PARAMS[:mid]
        right_params = self.FILTER_PARAMS[mid:]

        group = QGroupBox("Filters")
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(20)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(8, 8, 8, 8)

        # Left column
        for i, param in enumerate(left_params):
            if len(param) == 4:
                idx, name, default, items = param
                self.add_combo(layout, name, idx, default, items, i, 0)
            else:
                idx, name, default = param
                self.add_spin_label(layout, name, idx, default, i, 0)

        # Right column
        for i, param in enumerate(right_params):
            if len(param) == 4:
                idx, name, default, items = param
                self.add_combo(layout, name, idx, default, items, i, 2)
            else:
                idx, name, default = param
                self.add_spin_label(layout, name, idx, default, i, 2)

        group.setLayout(layout)
        return group

    def _create_estimator_group(self):
        group = QGroupBox("Estimator")
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(2)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(4, 8, 4, 4)

        for i, (idx, name, default) in enumerate(self.ESTIMATOR_PARAMS):
            self.add_spin_label(layout, name, idx, default, i)

        group.setLayout(layout)
        return group

    def _create_config_group(self):
        group = QGroupBox("Configuration")
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(2)
        layout.setContentsMargins(4, 8, 4, 4)

        self.config1_checks = []
        config1_bits = [
            ("Ext Mag", 0), ("Autoland", 1), ("Use Mag", 2),
            ("Emulation", 3), ("AH Alarm", 4), ("GPS Alt", 5),
            ("WP Test", 6), ("Clamp", 7),
        ]
        for i, item in enumerate(config1_bits):
            name, bit = item[0], item[1]
            cb = QCheckBox(name)
            cb.setProperty("bit", bit)
            cb.stateChanged.connect(lambda s, b=bit: self.bit_changed(ParamIndex.CONFIG1_BITS, b, s))
            layout.addWidget(cb, i // 2, (i % 2) + 1)
            self.config1_checks.append(cb)

        self.config2_checks = []
        config2_bits = [
            ("Batt Comp", 0), ("Fast Start", 1), ("Unused 2-2", 2, True),
            ("Have GPS", 3), ("Prop In", 4), ("Turn WP", 5), ("Beep WP", 6),
            ("Unused 2-7", 7, True),
        ]
        for i, item in enumerate(config2_bits):
            name, bit = item[0], item[1]
            unused = len(item) > 2 and item[2]
            cb = QCheckBox(name)
            cb.setProperty("bit", bit)
            cb.stateChanged.connect(lambda s, b=bit: self.bit_changed(ParamIndex.CONFIG2_BITS, b, s))
            if name == "Have GPS":
                cb.setToolTip("Sets/clears FC HaveGPS (drives InitialisingGPS / no-GPS boot):\n"
                              "bit SET = GPS required (waits for a fix),\n"
                              "bit CLEAR = no GPS - FC skips InitialisingGPS and flies without a satellite fix")
            elif unused:
                cb.setEnabled(False)
                cb.setToolTip(f"Unused — free config bit (Config2 bit {bit}). Available for a future feature.")
            layout.addWidget(cb, i // 2 + 4, (i % 2) + 1)
            self.config2_checks.append(cb)

        self.reset_config_btn = QPushButton("Reset Config")
        self.reset_config_btn.clicked.connect(self.reset_config)
        layout.addWidget(self.reset_config_btn, 8, 0, 1, 3)

        group.setLayout(layout)
        return group

    def _create_compact_group(self, title, params):
        group = QGroupBox(title)
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(2)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(4, 8, 4, 4)

        layout.setColumnMinimumWidth(0, 50)
        layout.setColumnMinimumWidth(1, 80)

        for i, p in enumerate(params, 1):
            if len(p) == 4:
                idx, name, default, items = p
                self.add_combo(layout, name, idx, default, items, i)
            else:
                idx, name, default = p
                self.add_spin(layout, idx, default, i, 0)
                lbl = QLabel(name)
                lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                layout.addWidget(lbl, i, 1)

        group.setLayout(layout)
        return group

    def _create_two_col_compact_group(self, title, params):
        group = QGroupBox(title)
        group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
        layout = QGridLayout()
        layout.setHorizontalSpacing(2)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(4, 8, 4, 4)

        half = (len(params) + 1) // 2
        for i, p in enumerate(params):
            col_base = 0 if i < half else 2
            row = (i if i < half else i - half) + 1
            if len(p) == 4:
                idx, name, default, items = p
                self.add_combo(layout, name, idx, default, items, row, col_base)
            else:
                idx, name, default = p
                self.add_spin(layout, idx, default, row, col_base)
                lbl = QLabel(name)
                lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                layout.addWidget(lbl, row, col_base + 1)

        group.setLayout(layout)
        return group

    def add_spin(self, layout, idx: ParamIndex, default: float, row: int, col: int):
        """Always a QDoubleSpinBox. Step = range/50."""
        idx_int = int(idx)
        mult = self._display_mult(idx_int)
        limits = self._param_limits(idx_int)

        lo = limits[0] * mult
        hi = limits[1] * mult
        span = hi - lo
        step = max(0.001, span / 50.0)
        if mult == 100:
            step = 1.0
        if idx_int in (68,) and step < 1.0:
            step = 1.0
        dec = max(0, min(6, -int(math.floor(math.log10(step)))))
        if mult not in (1.0, 100.0) and dec == 0 and step > 0.1:
            step = 0.1
            dec = 1
        if idx_int == 91 and step > 0.5:
            step = 0.5
        if idx_int in (84, 85) and step > 0.01:
            step = 0.01
            dec = 2
        if idx_int in ONE_DECIMAL_LIMIT_TAGS:
            step = 0.1
            dec = 1
        dec = self._min_decimals(idx_int, dec)
        # Built-in default is expressed in unified (normal) display units;
        # project it through the classic scaling so the initial value matches
        # this box's (possibly legacy-scaled) range.  Untagged params are
        # unaffected (mult == norm_mult), and normal mode has mult ==
        # PARAM_DISPLAY_MULT == norm_mult, so val stays the raw default.
        norm_mult = PARAM_DISPLAY_MULT.get(idx_int, 1.0)
        val = (float(default) * (mult / norm_mult)
               if norm_mult else float(default))

        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(dec)
        spin.setSingleStep(step)
        spin.setValue(val)
        spin.setProperty("param_index", idx_int)
        spin.installEventFilter(self)
        spin.valueChanged.connect(lambda v, i=idx_int: self.param_changed(i, v))
        spin.setToolTip(f"{idx.name} ({lo:.4g}-{hi:.4g})")
        spin.setMaximumWidth(70)
        spin.setMinimumWidth(62)
        spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        if lo >= hi:
            spin.setEnabled(False)
            spin.setStyleSheet("""
                QDoubleSpinBox {
                    background-color: #ffe0e0;
                    color: #999;
                    padding: 1px 2px;
                }
            """)
            spin.setToolTip(f"{idx.name} — [unused]")
        else:
            spin.setStyleSheet("""
                QDoubleSpinBox {
                    padding: 1px 2px;
                }
                QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                    width: 12px;
                }
            """)
        layout.addWidget(spin, row, col)
        self.params[idx_int] = spin
        return spin

    def _header(self, text):
        label = QLabel(text)
        label.setStyleSheet("font-weight: bold; color: #555;")
        label.setAlignment(Qt.AlignCenter)
        label.setFixedHeight(20)
        return label

    def add_spin_label(self, layout, name: str, idx: ParamIndex, default: int, row: int, col_offset: int = 0):
        """Add a spinbox with label (spinbox first)"""
        spin = self.add_spin(layout, idx, default, row, col_offset)
        lbl = QLabel(name)
        lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(lbl, row, 1 + col_offset)
        return spin
    
    def add_combo(self, layout, name: str, idx: ParamIndex, default: int, items: list, row: int, col_offset: int = 0, data: list = None):
        """Add a combobox with label (combo first).
        If data is provided, each item stores the corresponding raw FC value as itemData.
        The default parameter is treated as a raw FC value (matched against data list)."""
        combo = QComboBox()
        for i, item in enumerate(items):
            d = data[i] if data is not None else i
            combo.addItem(item, d)
        # Find default by data value, not by positional index
        if data is not None:
            try:
                default_idx = data.index(default)
            except ValueError:
                default_idx = 0
        else:
            default_idx = max(0, min(len(items)-1, default))
        combo.setCurrentIndex(default_idx)
        combo.setProperty("param_index", int(idx))
        combo.installEventFilter(self)
        combo.currentIndexChanged.connect(lambda v, i=int(idx): self.combo_changed(i))
        combo.setToolTip(f"Parameter {int(idx)+1}: {idx.name}")
        combo.setMaximumWidth(100)
        combo.setEditable(True)
        combo.lineEdit().setReadOnly(True)
        combo.lineEdit().setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        combo.setStyleSheet("""
            QComboBox { padding: 1px 2px; }
            QComboBox::drop-down { width: 16px; }
        """)
        layout.addWidget(combo, row, 0 + col_offset)
        
        lbl = QLabel(name)
        lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(lbl, row, 1 + col_offset)
        
        self.params[int(idx)] = combo
        return combo
    
    def bit_changed(self, idx: ParamIndex, bit: int, state: int):
        """Handle configuration bit checkbox changes"""
        current = self._config_values.get(int(idx), 0)
        if state == Qt.Checked:
            new_value = current | (1 << bit)
        else:
            new_value = current & ~(1 << bit)
        
        new_value = max(0, min(255, new_value))
        self._config_values[int(idx)] = new_value
        
        # Sync the hidden widget so write_params/send_params_typed use the corrected value
        pi = int(idx)
        if pi in self.params and isinstance(self.params[pi], QDoubleSpinBox):
            self.params[pi].setValue(new_value)
            self._log(f"  🔧 Config bit toggled: Param[{pi}] = {current} → {new_value} (synced)")
        else:
            self._log(f"  ⚠️ Config bit toggled: Param[{pi}] = {current} → {new_value} (widget NOT FOUND)")
        
        self.param_changed(pi, new_value)
        self.update_config_display()

    def servo_sense_bit_changed(self, bit: int, state: int):
        """Handle Servo Sense (tag 51) checkbox changes.

        Checked = bit SET = that servo output is reversed (PWSense = -1).
        Mirror of bit_changed but reads the current bitmask from the
        spinbox backing the param, since SERVO_SENSE has no config-register
        cache.  The spinbox's valueChanged -> param_changed connection
        (hidden spinbox wiring) handles dirty-marking, the protected-param
        confirm dialog and the write queue, so we only push the new bitmask
        into the spinbox here.
        """
        pi = int(ParamIndex.SERVO_SENSE)
        spin = self.params.get(pi)
        if not isinstance(spin, QDoubleSpinBox):
            self._log(f"  ⚠️ Servo Sense widget NOT FOUND — cannot toggle bit {bit}")
            return

        current = int(round(spin.value()))
        if state == Qt.Checked:
            new_value = current | (1 << bit)
        else:
            new_value = current & ~(1 << bit)

        new_value = max(0, min(127, new_value))
        spin.setValue(new_value)
        self._log(f"  🔧 Servo Sense bit{bit} toggled: 0b{current:07b} → 0b{new_value:07b} (Param[{pi}] = {new_value})")
        # Re-sync the checkboxes from the spinbox: on an accepted change the
        # valueChanged->param_changed->update_config_display chain already did
        # this, but if the protected-param confirm is declined param_changed
        # reverts the spinbox and returns before its update call — so the
        # checkboxes must be re-read here to stay consistent.
        self.update_config_display()

    def _rc_channel_changed(self, idx_int, fn_idx, value):
        """Handle RC channel spin box change — mark param dirty + check for clashes"""
        self.dirty_params.add(idx_int)
        self.status_label.setText(f"P{idx_int+1} changed ({len(self.dirty_params)})")
        self.status_label.setStyleSheet("color: #f39c12;")

        if value == 0:
            return
        clashes = []
        for other_spin in self.rc_channel_spins:
            if other_spin is self.params.get(idx_int):
                continue
            if other_spin.value() == value:
                other_idx = other_spin.property("param_index")
                other_fn = other_spin.property("fn_index")
                if other_fn is not None and other_fn < len(self.RC_MAP_PARAMS):
                    other_name = self.RC_MAP_PARAMS[other_fn][1]
                else:
                    other_name = f"P{other_idx+1}"
                clashes.append(other_name)
        if clashes:
            func_name = self.RC_MAP_PARAMS[fn_idx][1] if fn_idx < len(self.RC_MAP_PARAMS) else f"P{idx_int+1}"
            QMessageBox.warning(
                self, "Channel Clash",
                f"{func_name} is assigned to channel {value},\n"
                f"which is also used by: {', '.join(clashes)}\n\n"
                "Duplicate channel assignments will cause conflicts."
            )

    def update_config_display(self):
        """Update config value displays and checkboxes"""
        val1 = self._config_values.get(ParamIndex.CONFIG1_BITS, 0)
        val2 = self._config_values.get(ParamIndex.CONFIG2_BITS, 0)

        for cb in self.config1_checks:
            bit = cb.property("bit")
            cb.blockSignals(True)
            cb.setChecked((val1 & (1 << bit)) != 0)
            cb.blockSignals(False)
        
        for cb in self.config2_checks:
            bit = cb.property("bit")
            cb.blockSignals(True)
            cb.setChecked((val2 & (1 << bit)) != 0)
            cb.blockSignals(False)

        # Servo Sense (tag 51) is a bitmask surfacing as labelled checkboxes in
        # the Fixed Wing pgroup.  Sync from the spinbox holding the raw value.
        ss_spin = self.params.get(int(ParamIndex.SERVO_SENSE))
        if ss_spin is not None and isinstance(ss_spin, QDoubleSpinBox):
            val_ss = int(round(ss_spin.value()))
            for cb in self.servo_sense_checks:
                bit = cb.property("bit")
                cb.blockSignals(True)
                cb.setChecked((val_ss & (1 << bit)) != 0)
                cb.blockSignals(False)

    def reset_config(self):
        """Reset both config registers to 0"""
        for idx in (ParamIndex.CONFIG1_BITS, ParamIndex.CONFIG2_BITS):
            pi = int(idx)
            self._config_values[pi] = 0
            if pi in self.params and isinstance(self.params[pi], QDoubleSpinBox):
                self.params[pi].setValue(0)
            self.param_changed(idx, 0)
        self._log("  🔧 Config reset: both Config1 and Config2 set to 0 (synced to spinbox)")
        self.update_config_display()

    def _raw_float_values(self) -> List[float]:
        """Collect current widget values as raw float32 values (already scaled)."""
        vals = []
        for i in range(self.MAX_PARAMS):
            if i in self.params:
                widget = self.params[i]
                if isinstance(widget, QDoubleSpinBox):
                    mult = self._display_mult(i)
                    vals.append(widget.value() / mult)
                elif isinstance(widget, QComboBox):
                    data = widget.itemData(widget.currentIndex())
                    if data is None:
                        raise ValueError(
                            f"Combo param {i} selection has no data "
                            f"(index {widget.currentIndex()}) — refusing to save "
                            f"a positional index as the value")
                    vals.append(float(data))
                elif isinstance(widget, QSpinBox):
                    vals.append(float(widget.value()))
                else:
                    vals.append(0.0)
            else:
                vals.append(0.0)
        return vals

    def _apply_airframe_limits(self):
        """Re-apply spinbox ranges from the current .af [LIMITS] block (raw
        limits × display mult), falling back to the class-ceiling PARAM_LIMITS
        for tags without a [LIMITS] entry. If a loaded value already sits
        outside the new range the bound is widened to contain it, never the
        value silently clamped (setRange would otherwise inflate/deflate a
        stale .af gain into the range — the Shadow 'way too high' trap).
        """
        for idx, widget in self.params.items():
            if not isinstance(widget, QDoubleSpinBox):
                continue
            idx_int = int(idx)
            mult = self._display_mult(idx_int)
            lo_r, hi_r = self._param_limits(idx_int)
            lo = lo_r * mult
            hi = hi_r * mult
            span = hi - lo
            step = max(0.001, span / 50.0)
            if mult == 100:
                step = 1.0
            if idx_int in (68,) and step < 1.0:
                step = 1.0
            dec = max(0, min(6, -int(math.floor(math.log10(step)))))
            if mult not in (1.0, 100.0) and dec == 0 and step > 0.1:
                step = 0.1
                dec = 1
            if idx_int == 91 and step > 0.5:
                step = 0.5
            if idx_int in (84, 85) and step > 0.01:
                step = 0.01
                dec = 2
            dec = self._min_decimals(idx_int, dec)
            current = widget.value()
            if current < lo:
                lo = current
            elif current > hi:
                hi = current
            span = hi - lo
            step = max(0.001, span / 50.0)
            if mult == 100:
                step = 1.0
            if idx_int in (68,) and step < 1.0:
                step = 1.0
            dec = max(0, min(6, -int(math.floor(math.log10(step)))))
            if mult not in (1.0, 100.0) and dec == 0 and step > 0.1:
                step = 0.1
                dec = 1
            if idx_int == 91 and step > 0.5:
                step = 0.5
            if idx_int in (84, 85) and step > 0.01:
                step = 0.01
                dec = 2
            dec = self._min_decimals(idx_int, dec)
            widget.blockSignals(True)
            widget.setRange(lo, hi)
            widget.setDecimals(dec)
            widget.setSingleStep(step)
            widget.blockSignals(False)

    def _set_widgets_from_raw(self, raw_values: Dict[int, float]):
        """Set all UI widget values from raw float32 dictionary {idx: raw_float}."""
        # A fresh bulk load clears any stale FC-clamp red highlights
        for idx in self._clamped_widgets:
            widget = self.params.get(idx)
            if isinstance(widget, QDoubleSpinBox):
                widget.setStyleSheet("""
                    QDoubleSpinBox {
                        padding: 1px 2px;
                    }
                    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                        width: 12px;
                    }
                """)
        self._clamped_widgets.clear()
        # A programmatic bulk set has changed widgets without going through the
        # live-edit queue — the FC's RAM image no longer matches the UI. Any
        # later commit must push the whole image first (only the load path
        # calls this; FC readbacks update widgets in place, never here).
        self._image_dirty = True
        self._suppress_sim_auto = True
        try:
            for i, raw in raw_values.items():
                if i not in self.params:
                    continue
                widget = self.params[i]
                try:
                    # Block signals so programmatic bulk loads never trigger the
                    # protected-param confirm dialogs (combo_changed/param_changed)
                    # or their revert-on-No — a load is not a user edit. Without
                    # this, loading a file whose AF_TYPE differs from the previous
                    # value popped "Confirm Parameter Change" and reverted the
                    # freshly loaded type (flying wing displayed/saved as X Quad).
                    widget.blockSignals(True)
                    try:
                        if isinstance(widget, QDoubleSpinBox):
                            mult = self._display_mult(i)
                            display_val = raw * mult
                            if i in self._PROTECTED_PARAMS:
                                self._committed_values[i] = display_val
                            widget.setValue(display_val)
                        elif isinstance(widget, QComboBox):
                            if i in self._PROTECTED_PARAMS:
                                self._committed_values[i] = float(int(raw))
                            cb_idx = self._resolve_combo_index(widget, int(raw))
                            widget.setCurrentIndex(cb_idx)
                        elif isinstance(widget, QSpinBox):
                            widget.setValue(int(round(raw)))
                    finally:
                        widget.blockSignals(False)
                except Exception:
                    pass
            # Sync _config_values cache for config bit registers
            for cfg_idx in (ParamIndex.CONFIG1_BITS, ParamIndex.CONFIG2_BITS):
                ci = int(cfg_idx)
                if ci in raw_values:
                    self._config_values[ci] = int(raw_values[ci])
            # Re-run UI side-effects that the (now-suppressed) change signals
            # previously drove for an AF_TYPE change. Sync setup widget first so
            # _update_fw_style reads the freshly loaded value.
            if hasattr(self, 'sync_setup_from_advanced'):
                self.sync_setup_from_advanced()
            if hasattr(self, '_update_fw_style'):
                self._update_fw_style()
            if hasattr(self, '_build_phys_row'):
                self._build_phys_row()
        finally:
            self._suppress_sim_auto = False

    def set_flash_airframe_name(self, name: str):
        """Record the airframe name reported by the FC config flash (tag-63
        response). This is the SINGLE displayed name — it survives GCS restarts
        because it lives in the FC config, not in a local GCS setting.
        """
        self._flash_airframe_name = name or ""
        self._log(f"  🏷️ FC flash airframe name: \"{name}\"")
        self._update_airframe_name_label()

    def _update_airframe_name_label(self):
        """Show the current airframe name next to the Load button.

        The name comes from the FC config flash (single source of truth). The
        label stays hidden until a name is present so it does not waste layout
        space on first open.
        """
        name = (self._flash_airframe_name or "").strip()
        if name:
            self.airframe_name_label.setText(name)
            self.airframe_name_label.show()
        else:
            self.airframe_name_label.setText("")
            self.airframe_name_label.hide()

    def _load_airframe(self, path: str):
        """Load an airframe file's defaults + [LIMITS] into the UI.

        Shared by the toolbar 'Load Airframe…' button and the Load Params
        dialog. Replaces the current values in the UI and, when connected,
        live-writes the loaded set to the FC's RAM image immediately — Load IS
        the write (no separate Write button in the live-write model).
        Boot-scoped params in the loaded set trigger the one-shot
        Apply & Reboot offer once the writes drain.
        """
        name = os.path.splitext(os.path.basename(path))[0] if path else ""

        # Save any pending tweaks before switching airframe
        if not self._save_if_dirty():
            return

        if not path or not os.path.exists(path):
            self._log(f"  ❌ Airframe file not found: {path}")
            return

        reply = QMessageBox.question(
            self,
            "Airframe Defaults",
            f"Load and write defaults for \"{name}\"?\n\n"
            "This will replace current parameter values in the UI and\n"
            "write the new set to the flight controller immediately "
            "(when connected).\n"
            "Boot-only parameters will then ask whether to apply and reboot.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.No:
            return

        try:
            read_name, raw_values, meta = af_module.parse_af_file(path)
            self._log(f"  📖 Loaded {len(raw_values)} params from \"{read_name}\"")
        except Exception as e:
            self._log(f"  ❌ Failed to parse {path}: {e}")
            QMessageBox.critical(self, "Parse Error", str(e))
            return

        # Reset any stuck verification/write state
        self._current_airframe_path = path
        self._verify_mode = False
        self._write_in_progress = False
        self._flash_write_pending = False
        self._written_params = {}
        self._written_float = {}
        self._expecting_param_packet = False
        self._write_timestamp = 0
        if self._verification_timer:
            self._verification_timer.stop()
            self._verification_timer = None

        if self._write_timeout_timer:
            self._write_timeout_timer.stop()
            self._write_timeout_timer = None
        self.reset_write_button()
        self.reset_read_button()

        # Widen every spinbox to the class ceiling FIRST so no loaded value is
        # clamped by the previous-load (stale) or factory range, THEN set
        # values, then narrow to the .af [LIMITS] tuning range (the shrink
        # guard in _apply_airframe_limits keeps any out-of-range stale value
        # instead of silently moving it to lo).
        self._airframe_limits = {}
        self._apply_airframe_limits()
        self._set_widgets_from_raw(raw_values)
        # Apply .af [LIMITS] tuning ranges (fallback = class-ceiling PARAM_LIMITS)
        self._airframe_limits = dict(meta.get('LIMITS', {}) or {})
        self._apply_airframe_limits()
        self._ensure_config2_fast_start()
        self.update_config_display()
        self.update_rc_display()
        # Restore character slider from .af metadata
        if self._character_slider and 'Character' in meta:
            try:
                self._character_slider.setValue(int(meta['Character']))
            except (ValueError, TypeError):
                pass
        self.sync_setup_from_advanced()
        # Rebuild physics descriptor row for the loaded AF_TYPE (sync blocks signals)
        self._build_phys_row()
        # Load physical descriptors from .af metadata
        phys_meta = {k: v for k, v in meta.items() if k.startswith('PHYS_')}
        if phys_meta:
            self._load_phys_from_metadata(phys_meta)
        connected = bool(self.parent_window
                         and getattr(self.parent_window, 'connected', False))
        if connected and hasattr(self.parent_window, 'send_params_typed'):
            # Load = a single serialized batch write of the whole widget image
            # via the tag-17 batch sender. `_write_in_progress` makes every
            # echo ACK inert (update_params_from_typed returns early), so the
            # UI stays static during the ~1 s push — no widget churn, no
            # per-param live-queue interleaving, no offer mid-stream. The
            # Apply & Reboot offer appears exactly once, after the batch drains.
            self._write_in_progress = True

            def _on_load_write_complete():
                self._write_in_progress = False
                self._image_dirty = False  # the full batch landed — RAM == UI
                boot_idx = next((b for b in sorted(PARAM_BOOT_REQUIRED)
                                 if b in self.params), None)
                if boot_idx is not None:
                    self._offer_apply_reboot_for_boot_param(boot_idx)
                self.status_label.setText(
                    f"📝 Loaded \"{name}\" — written to FC ({len(raw_values)} params)")
                self.status_label.setStyleSheet("color: #27ae60;")

            self.parent_window.send_params_typed(on_complete=_on_load_write_complete)
        else:
            self.status_label.setText(
                f"📝 Loaded \"{name}\" defaults in UI — "
                + ("(not connected)" if not connected else "(no parent write path)"))
            self.status_label.setStyleSheet("color: #f39c12;")

        # The loaded .af name is the current airframe for the next flash commit
        # (a later connect refreshes it from the FC config flash).
        self._flash_airframe_name = name
        self._update_airframe_name_label()

        # Reset protected param warnings for new airframe
        self._warned_protected.clear()

        # Persist the selection
        settings = QSettings("UAVX", "Groundstation")
        settings.setValue("airframe_path", path)

    def _restore_airframe_selection(self):
        settings = QSettings("UAVX", "Groundstation")
        saved_path = settings.value("airframe_path", "")
        if not saved_path:
            # Default: retuned Ken_450_1165 (proposed tuning set — read-only
            # fully-populated defaults from the retune campaign).
            default_name = "Ken_450_1165"
            candidate = os.path.join(self._proposed_dir, f"{default_name}.af")
            if os.path.exists(candidate):
                settings.setValue("airframe_path", candidate)
                self._current_airframe_path = candidate
                self._log(f"  Auto-selected default airframe: {default_name}")
            return
        self._current_airframe_path = saved_path

    def _ensure_config2_fast_start(self):
        """Ensure Config2 Fast Start bit (bit 1) is set — preserves all other bits
        (bit 0 BatteryComp is now a fleet-wide default and must survive loading)."""
        idx = int(ParamIndex.CONFIG2_BITS)
        config2 = self._config_values.get(idx, 0)
        corrected = config2 | 2  # Set bit 1 (Fast Start), keep everything else
        if config2 != corrected:
            self._log(f"  🔧 FIXING Config2 at {ParamIndex.CONFIG2_BITS.name}: {config2} -> {corrected}")
            self._config_values[idx] = corrected
            if idx in self.params and isinstance(self.params[idx], QDoubleSpinBox):
                self.params[idx].setValue(corrected)
                self._log(f"  🔧 Config2 widget synced to {corrected}")
            self.dirty_params.add(idx)
    

    
    def combo_changed(self, idx: int, value: int = None):
        # Derive authoritative FC value from widget data
        if value is None:
            widget = self.params.get(idx)
            if widget:
                data = widget.currentData()
                value = int(data) if data is not None else widget.currentIndex()
        # Confirmation dialog for safety-critical combo params (only once per session)
        if idx in self._PROTECTED_PARAMS and idx not in self._warned_protected:
            old_val = self._committed_values.get(idx, float(value))
            if abs(float(value) - old_val) > 0.001:
                name = self._PROTECTED_NAMES.get(idx, f"P{idx+1}")
                warning = self._PROTECTED_WARNINGS.get(idx, "This parameter affects critical behavior.")
                reply = QMessageBox.question(
                    self, "Confirm Parameter Change",
                    f"{name}\n\n"
                    f"{warning}\n\n"
                    f"Are you sure you want to change this?",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No
                )
                if reply == QMessageBox.No:
                    widget = self.params[idx]
                    widget.blockSignals(True)
                    cb_idx = self._resolve_combo_index(widget, int(old_val))
                    widget.setCurrentIndex(cb_idx)
                    widget.blockSignals(False)
                    # Keep the paired physics/setup combo consistent — otherwise
                    # the two AF_TYPE combos diverge and a later save writes the
                    # declined value.
                    if hasattr(self, '_setup_widgets'):
                        setup_w = self._setup_widgets.get(idx)
                        if isinstance(setup_w, QComboBox) and setup_w.currentData() != int(old_val):
                            setup_w.blockSignals(True)
                            cb_idx = self._resolve_combo_index(setup_w, int(old_val))
                            setup_w.setCurrentIndex(cb_idx)
                            setup_w.blockSignals(False)
                    if idx == int(ParamIndex.AF_TYPE):
                        self._update_fw_style()
                    return
                self._warned_protected.add(idx)
                self._committed_values[idx] = float(value)
        elif idx in self._PROTECTED_PARAMS:
            old_val = self._committed_values.get(idx, float(value))
            if abs(float(value) - old_val) > 0.001:
                self._committed_values[idx] = float(value)
        self.dirty_params.add(idx)
        self.status_label.setText(f"P{idx+1} changed ({len(self.dirty_params)})")
        self.status_label.setStyleSheet("color: #f39c12;")
        self._enqueue_live_write(idx)
        # Sync back to setup widget if this param has one (bidirectional tracking)
        if hasattr(self, '_setup_widgets') and idx in self._setup_widgets:
            setup_w = self._setup_widgets.get(idx)
            if isinstance(setup_w, QComboBox):
                adv_w = self.params.get(idx)
                if isinstance(adv_w, QComboBox):
                    data = adv_w.currentData()
                    if data is not None:
                        cb_idx = self._resolve_combo_index(setup_w, int(data))
                        if cb_idx != setup_w.currentIndex():
                            setup_w.blockSignals(True)
                            setup_w.setCurrentIndex(cb_idx)
                            setup_w.blockSignals(False)
            if idx == int(ParamIndex.AF_TYPE):
                self._update_fw_style()

    def param_changed(self, idx: int, value: float):
        # Confirmation dialog for safety-critical params (only once per session)
        if idx in self._PROTECTED_PARAMS and idx not in self._warned_protected:
            old_val = self._committed_values.get(idx, value)
            if abs(value - old_val) > 0.001:
                name = self._PROTECTED_NAMES.get(idx, f"P{idx+1}")
                warning = self._PROTECTED_WARNINGS.get(idx, "This parameter affects critical behavior.")
                reply = QMessageBox.question(
                    self, "Confirm Parameter Change",
                    f"{name}\n\n"
                    f"{warning}\n\n"
                    f"Current: {old_val:.4f}\nProposed: {value:.4f}\n\n"
                    f"Are you sure you want to change this?",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No
                )
                if reply == QMessageBox.No:
                    widget = self.params[idx]
                    widget.blockSignals(True)
                    widget.setValue(old_val)
                    widget.blockSignals(False)
                    return
                self._warned_protected.add(idx)
                self._committed_values[idx] = value
        elif idx in self._PROTECTED_PARAMS:
            old_val = self._committed_values.get(idx, value)
            if abs(value - old_val) > 0.001:
                self._committed_values[idx] = value

        self.dirty_params.add(idx)
        self.status_label.setText(f"P{idx+1} changed ({len(self.dirty_params)})")
        self.status_label.setStyleSheet("color: #f39c12;")
        self._enqueue_live_write(idx)
        self.update_config_display()

        # Tuning param edit → debounced auto robustness check
        if idx in self._PARAM_CURVES:
            self._schedule_sim_check()
            self._check_cascade_sanity()

        # Orange highlight if value outside PARAM_LIMITS (in display units)
        if idx in self.params:
            widget = self.params[idx]
            if isinstance(widget, QDoubleSpinBox):
                mult = self._display_mult(idx)
                limits = PARAM_LIMITS.get(idx, (0.0, 255.0))
                display_lo, display_hi = limits[0] * mult, limits[1] * mult
                if value < display_lo or value > display_hi:
                    widget.setStyleSheet("""
                        QDoubleSpinBox {
                            padding: 1px 2px;
                            background-color: #ffe0b0;
                        }
                    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                        width: 12px;
                    }
                """)
                else:
                    widget.setStyleSheet("""
                        QDoubleSpinBox {
                            padding: 1px 2px;
                        }
                        QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                            width: 12px;
                        }
                    """)

        # Sync back to setup widget if this param has one (bidirectional tracking)
        if hasattr(self, '_setup_widgets') and idx in self._setup_widgets:
            setup_w = self._setup_widgets[idx]
            adv_w = self.params.get(idx)
            if setup_w and adv_w:
                if isinstance(setup_w, QComboBox) and isinstance(adv_w, QComboBox):
                    adv_data = adv_w.currentData()
                    if adv_data is not None:
                        s_idx = self._resolve_combo_index(setup_w, int(adv_data))
                        if s_idx != setup_w.currentIndex():
                            setup_w.blockSignals(True)
                            setup_w.setCurrentIndex(s_idx)
                            setup_w.blockSignals(False)
                elif isinstance(setup_w, QDoubleSpinBox) and isinstance(adv_w, QDoubleSpinBox):
                    adv_val = adv_w.value()
                    if abs(setup_w.value() - adv_val) > 0.001:
                        setup_w.blockSignals(True)
                        setup_w.setValue(adv_val)
                        setup_w.blockSignals(False)

    def _enqueue_live_write(self, idx):
        """Debounce a param change for live-write to the FC RAM image.

        Also arms the one-shot Apply & Reboot offer for boot-scoped params
        (effective only after reboot). Shared by both the spinbox path
        (param_changed) and the combo path (combo_changed) — boot-scoped
        selectors like AF_TYPE are combos, so both must route here.
        """
        if not hasattr(self, '_live_pending'):
            self._live_pending = set()
            self._live_timer = QTimer(self)
            self._live_timer.setSingleShot(True)
            self._live_timer.timeout.connect(self._flush_live_writes)
        self._live_pending.add(idx)
        self._live_timer.start(350)
        if idx in PARAM_BOOT_REQUIRED:
            self._boot_reboot_offer = idx

    def _pending_live_float(self, idx):
        """Raw fc-float for a live write — mirrors the send_params_typed logic."""
        widget = self.params.get(idx)
        if isinstance(widget, QDoubleSpinBox):
            mult = self._display_mult(idx)
            return widget.value() / mult
        if isinstance(widget, QSpinBox):
            return float(widget.value())
        if isinstance(widget, QComboBox):
            data = widget.currentData()
            if data is None:
                raise ValueError(
                    f"Combo param {idx} selection has no data "
                    f"(index {widget.currentIndex()}) — refusing to write a "
                    f"positional index as the value")
            return float(data)
        return 0.0

    def _flush_live_writes(self):
        """Push all debounced live-write indices to the FC's RAM image.

        After a boot-scoped param (PARAM_BOOT_REQUIRED) drains, offer the
        explicit Apply & Reboot — its effect can only appear at next power-on.
        """
        if not hasattr(self, '_live_pending') or not self._live_pending:
            return
        pending = sorted(self._live_pending)
        self._live_pending.clear()
        if self.parent_window and hasattr(self.parent_window, '_write_params_live'):
            self.parent_window._write_params_live(
                [(i, self._pending_live_float(i)) for i in pending])
        if self.parent_window and getattr(self.parent_window, 'connected', False):
            boot_idx = getattr(self, '_boot_reboot_offer', None)
            if boot_idx is not None and boot_idx in pending:
                self._boot_reboot_offer = None
                self._boot_reboot_offer_param = boot_idx
                self._when_param_write_drained(self._offer_apply_reboot_for_boot_param)

    def _offer_apply_reboot_for_boot_param(self, idx=None):
        """Ask once whether to persist + reboot after a boot-scoped change."""
        if idx is None:
            if not hasattr(self, '_boot_reboot_offer_param'):
                return
            idx = self._boot_reboot_offer_param
        self._boot_reboot_offer_param = None
        name = self._PROTECTED_NAMES.get(idx, f"P{idx+1}")
        reply = QMessageBox.question(
            self,
            "Apply & Reboot?",
            f"{name}\n\n"
            "This parameter only takes effect after the FC restarts.\n"
            "Save all parameters to flash and restart the FC now?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )
        if reply == QMessageBox.Yes:
            self.apply_and_reboot(confirm=False)

    def setup_connections(self):
        self.load_btn.clicked.connect(self._pick_and_load_airframe)
        self.save_btn.clicked.connect(self.save_params)

    def _snapshot_baseline(self):
        """Capture current widget values as the baseline for dirty detection"""
        self._baseline_snapshot = {}
        for idx, widget in self.params.items():
            if isinstance(widget, QDoubleSpinBox):
                self._baseline_snapshot[idx] = widget.value()
            elif isinstance(widget, QComboBox):
                self._baseline_snapshot[idx] = widget.currentData()
            elif isinstance(widget, QSpinBox):
                self._baseline_snapshot[idx] = widget.value()

    def _is_dirty_from_baseline(self):
        """Check if any widget differs from the last-loaded baseline"""
        for idx, widget in self.params.items():
            if isinstance(widget, QDoubleSpinBox):
                current = widget.value()
            elif isinstance(widget, QComboBox):
                current = widget.currentData()
            elif isinstance(widget, QSpinBox):
                current = widget.value()
            else:
                continue
            baseline = self._baseline_snapshot.get(idx)
            if baseline is None:
                if current is not None and current != 0:
                    return True
            elif abs(float(current) - float(baseline)) > 0.001:
                return True
        return False

    def _source_airframe_name(self):
        """Base name of the current airframe for default save filenames.

        The currently loaded .af is the airframe actually being tuned, so its
        filename is the save-name source. The FC config flash name (the single
        source of truth for DISPLAY) is only a fallback for a fresh session with
        no .af loaded — otherwise a stale flashed name would mislabel saves made
        after a different airframe was loaded (e.g. quad params written as
        "Shadow_..."). Greg 2026-09-17.
        """
        if self._current_airframe_path:
            return os.path.splitext(os.path.basename(self._current_airframe_path))[0]
        if self._flash_airframe_name:
            return self._flash_airframe_name
        return "Params"

    def _base_airframe_name(self):
        """Airframe name with trailing '_Tuned' and save-timestamp tags stripped.

        Repeated saves used to compound the suffix (name_Tuned_Tuned...) because
        the previously-saved source name was fed straight back into the next
        save. Stripping to the true base keeps save filenames clean; a
        date/time tag is appended by the save paths below instead. Since the
        loaded .af is now the name source (see _source_airframe_name), a
        previously saved file's trailing _YYYYMMDD_HHMMSS tag(s) are stripped
        too so loading and re-saving does not compound date tags.
        """
        name = self._source_airframe_name().strip()
        while name.endswith("_Tuned") or name.endswith("_tuned"):
            name = name[: -len("_Tuned")]
        while re.search(r"_\d{8}_\d{6}$", name):
            name = re.sub(r"_\d{8}_\d{6}$", "", name)
        # Guard against a last-resort empty stem so we never write "_.af".
        return name if name else "Params"

    def _default_save_path(self):
        """Build default save path: airframes/user/<name>_<YYYYMMDD_HHMMSS>.af

        An edited airframe is NEVER saved over its read-only source; a date/
        time tag is appended and the result lands in the user airframes dir.
        Read-only sources (generic/original) can therefore never be
        overwritten, and every save produces a fresh, uniquely named file.
        """
        from datetime import datetime
        airframe_name = self._base_airframe_name()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        tuned_dir = self._user_dir
        os.makedirs(tuned_dir, exist_ok=True)
        return os.path.join(tuned_dir, f"{airframe_name}_{stamp}.af")

    def _save_if_dirty(self):
        """Save dirty params before proceeding. Returns True to proceed.

        Greg 2026-09-07: the "Save/Discard/Cancel?" prompt was wasted clicks —
        Save was always the right answer (and the default button), and a
        parameter window read-only-close loses nothing that a timestamped .af
        in airframes/user/ wouldn't recover. So the prompt is gone: if any
        tweaks are pending they are saved unconditionally. The write can never
        hit a read-only source (`_default_save_path` always name-tags into the
        user dir). Returns False ONLY if the save itself failed.
        """
        if not self._is_dirty_from_baseline():
            return True
        return self._save_current_to_af()

    def _save_current_to_af(self):
        """Snapshot current widget values to a timestamped user .af."""
        path = self._default_save_path()
        try:
            raw_vals = self._raw_float_values()
            raw_dict = {i: raw_vals[i] for i in range(self.MAX_PARAMS)}
            metadata = {}
            if self._character_slider:
                metadata['Character'] = str(self._character_slider.value())
            metadata.update(self._phys_to_metadata())
            metadata['LIMITS'] = dict(self._airframe_limits)
            text = af_module.format_af("Tuned Parameters", raw_dict, metadata=metadata)
            with open(path, 'w') as f:
                f.write(text)
            self._log(f"✅ Saved tuned params to {path}")
            self.status_label.setText(f"✅ Saved to {os.path.basename(path)}")
            self.status_label.setStyleSheet("color: #27ae60;")
            return True
        except Exception as e:
            self._log(f"❌ Save failed: {e}")
            QMessageBox.critical(self, "Save Failed", str(e))
            return False
    
    def save_params(self):
        from datetime import datetime
        raw_vals = self._raw_float_values()
        raw_dict = {i: raw_vals[i] for i in range(self.MAX_PARAMS)}
        
        # Generate physics summary for filename
        phys_meta = self._phys_to_metadata()
        phys_summary = ""
        if phys_meta:
            parts = []
            if 'PHYS_AUW_G' in phys_meta:
                parts.append(f"{phys_meta['PHYS_AUW_G']}g")
            if 'PHYS_PROP_INCH' in phys_meta:
                parts.append(f"{phys_meta['PHYS_PROP_INCH']}in")
            if 'PHYS_MOTOR_COUNT' in phys_meta:
                parts.append(f"{phys_meta['PHYS_MOTOR_COUNT']}M")
            if 'PHYS_WINGSPAN_MM' in phys_meta:
                parts.append(f"{phys_meta['PHYS_WINGSPAN_MM']}mm")
            phys_summary = "_".join(parts)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suggested = f"{self._base_airframe_name()}_{timestamp}.af"
        
        dialog = AfSaveDialog(self, suggested, phys_summary, self._user_dir)
        if dialog.exec_() != QDialog.Accepted:
            return
        chosen = dialog.get_selected_name()
        if not chosen:
            return
        # Always write into the user airframes directory (only the filename is
        # taken from the dialog) so saved files appear in the load pulldown
        os.makedirs(self._user_dir, exist_ok=True)
        path = os.path.join(self._user_dir, os.path.basename(chosen))
        if not path.lower().endswith(".af"):
            path += ".af"
        try:
            metadata = {}
            if self._character_slider:
                metadata['Character'] = str(self._character_slider.value())
            metadata.update(phys_meta)
            metadata['LIMITS'] = dict(self._airframe_limits)
            text = af_module.format_af("Saved Parameters", raw_dict, metadata=metadata)
            with open(path, 'w') as f:
                f.write(text)
            self._log(f"✅ Saved {self.MAX_PARAMS} parameters to {path}")
            self.status_label.setText(f"✅ Saved to {os.path.basename(path)}")
            self.status_label.setStyleSheet("color: #27ae60;")
        except Exception as e:
            self._log(f"❌ Save failed: {e}")
            QMessageBox.critical(self, "Save Failed", str(e))
    
    def _pick_and_load_airframe(self):
        """Open the airframe library dialog and load the selected file.
        This is the single Load button — it opens the airframe library. The
        button's label shows the current airframe name (from the FC config
        flash on connect, or the file just loaded)."""
        dialog = AfLoadDialog(self, classify_fn=self._category_for_af_file)
        if dialog.exec_() != QDialog.Accepted:
            return
        path = dialog.get_selected_path()
        if not path:
            return
        self._load_airframe(path)

    def _get_param_value(self, idx):
        """Safely get parameter value from widget"""
        if idx not in self.params:
            return 0
        widget = self.params[idx]
        if isinstance(widget, (QDoubleSpinBox, QSpinBox)):
            return int(widget.value())
        elif isinstance(widget, QComboBox):
            data = widget.itemData(widget.currentIndex())
            if data is None:
                raise ValueError(
                    f"Combo param {idx} selection has no data "
                    f"(index {widget.currentIndex()}) — refusing to read a "
                    f"positional index as the value")
            return data
        else:
            return 0
    
    def update_rc_display(self):
        """Update RC display using function-to-channel mapping"""
        if not hasattr(self, 'rc_value_labels'):
            return

        discovered = 0
        flight_data = self.parent_window.flight_data if self.parent_window else None
        if flight_data:
            discovered = getattr(flight_data, 'discovered_channels', 0)

        for spin in self.rc_channel_spins:
            slot = spin.value()
            bad = discovered > 0 and slot >= discovered
            old = spin.property("_rc_hot") is True
            if bool(bad) != old:
                spin.setStyleSheet(self.RC_CH_BAD_STYLE if bad else self.RC_CH_STYLE)
                spin.setProperty("_rc_hot", bool(bad))

        # Physical Rx channel values (RCInp[] uS as received), one per channel.
        phys = list(getattr(flight_data, 'rc_physical', []) or []) if flight_data else []
        ch_values = {}
        for i, val in enumerate(phys):
            ch_values[i] = val
            if i < 16:
                self.rc_channels[i] = val
        for i in range(16):
            self.rc_channels[i] = ch_values.get(i, 0)

        show_us = self.rc_unit_btn.isChecked()

        def style_label(lbl, val):
            if val > 0.01:
                us = round(val)
                lbl.setText(f"{us}" if show_us else f"{round((us - 1500) / 5.0):+d}%")
                bg = "#27ae60" if 1450 <= us <= 1550 else "#f39c12" if us < 1200 or us > 1800 else "#8bc34a"
                style = f"background-color: {bg}; font-weight: bold; padding: 0px 3px;"
            else:
                lbl.setText("---")
                style = "background-color: #999; font-weight: bold; padding: 0px 3px;"
            if lbl.styleSheet() != style:
                lbl.setStyleSheet(style)

        def style_bar(bar, val):
            bar.setRange(900, 2100)
            bar.setValue(min(2100, max(900, round(val))) if val > 0.01 else 1500)

        # Function rows: look up each assigned channel's physical value
        for fn_idx, (param_idx, func_name, tc) in enumerate(self.RC_MAP_PARAMS):
            assigned_ch = self._get_param_value(int(param_idx))
            val = ch_values.get(assigned_ch, 0)
            style_label(self.rc_value_labels[fn_idx], val)
            style_bar(self.rc_progress_bars[fn_idx], val)

        # Spare channels 12-15 (third column)
        for i in range(4):
            val = ch_values.get(12 + i, 0)
            style_label(self.rc_spare_labels[i], val)
            style_bar(self.rc_spare_bars[i], val)

        # Update motor/servo bargraphs
        flight_data = self.parent_window.flight_data if self.parent_window else None
        if flight_data:
            self._update_motor_display(flight_data)
    
    def _set_config_default(self, idx: ParamIndex, value: int):
        """Set a config register to a default value — updates _config_values and spinbox."""
        pi = int(idx)
        if pi in self.params and isinstance(self.params[pi], QDoubleSpinBox):
            self.params[pi].setValue(float(value))
        self._config_values[pi] = value

    def load_default_params(self):
        """Load default parameters — zeros until auto-populated from FC on connect."""
        self._log("Starting with zeros — auto-populated from FC on connect")
        default_values = [0] * self.MAX_PARAMS
        
        for i, val in enumerate(default_values):
            if i in self.params:
                clamped = max(0, min(255, val))
                if isinstance(self.params[i], QDoubleSpinBox):
                    pass  # Keep initial value from add_spin
                elif isinstance(self.params[i], QComboBox):
                    max_idx = self.params[i].count() - 1
                    self.params[i].setCurrentIndex(max(0, min(max_idx, clamped)))
        
        # Defaults: Config1 = safe (no emulation) + use mag, Config2 = Batt Comp +
        # Fast Start + GPS + Nav Beep (matches the fleet-wide retune sweep)
        self._set_config_default(ParamIndex.CONFIG1_BITS, int(Config1Bits.eEnforceDriveSymmetry | Config1Bits.eUsingMag))
        self._set_config_default(ParamIndex.CONFIG2_BITS, int(Config2Bits.eUseBatteryComp | Config2Bits.eUseFastStart | Config2Bits.eUseGPS | Config2Bits.eUseNavBeep))
        
        self.update_config_display()
        self.update_rc_display()
        self.sync_setup_from_advanced()
        self._log("✅ Default parameters loaded")
    
    def read_params(self):
        """Request a full param download from the FC.

        Called automatically on connect (on_connected → tag-71) and by the
        post-reboot write verification; the manual Read button was removed.
        """
        self._log("📖 Requesting parameters from FC")

        if self._flash_write_pending:
            self._log("  ⛔ Read refused — flash write/commit outstanding (interlock)")
            self.status_label.setText("⏳ Reading disabled until flash write completes...")
            self.status_label.setStyleSheet("color: #f39c12;")
            return
        
        if self.parent_window and hasattr(self.parent_window, 'can_read_parameters'):
            safe_rd, reason_rd = self.parent_window.can_read_parameters()
            if not safe_rd:
                self._log(f"  ⛔ Read refused — {reason_rd}")
                self.status_label.setText("❌ Cannot read - check connection")
                self.status_label.setStyleSheet("color: #e74c3c;")
                return
        
        if self.parent_window and hasattr(self.parent_window, 'flight_data'):
            if self.parent_window.flight_data and self.parent_window.flight_data.flight_state == FlightState.eInFlight:
                reply = QMessageBox.question(
                    self,
                    "⚠️ Warning: Aircraft is Flying",
                    "You are requesting parameters while the aircraft is in flight.\n\n"
                    "This is generally safe but may cause timing issues.\n"
                    "Do you want to continue?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No
                )
                if reply == QMessageBox.No:
                    return

        self.status_label.setText("⏳ Requesting ALL 128 parameters from FC...")
        self.status_label.setStyleSheet("color: #f39c12;")

        self._read_timeout_timer = QTimer()
        self._read_timeout_timer.setSingleShot(True)
        self._read_timeout_timer.timeout.connect(self._read_timeout)
        self._read_timeout_timer.start(5000)

        if self.parent_window and hasattr(self.parent_window, 'send_request'):
            self.parent_window.send_request(PacketTag.PARAM_TAGGED_READ, 255, 0, self._read_request_id)
            self._log(f"  Sent request with request_id={self._read_request_id}")
        else:
            self.status_label.setText("❌ Cannot send request - no connection")
            self.status_label.setStyleSheet("color: #e74c3c;")
            self.reset_read_button()
            if self._read_timeout_timer:
                self._read_timeout_timer.stop()
                self._read_timeout_timer = None
    
    def apply_and_reboot(self, confirm=True):
        """Flush any pending live writes, then commit RAM→flash and reboot (tag-72).

        Live-writes already keep the FC's RAM image in sync, so there is no
        batch of param packets to send — the commit just needs the debounced
        write queue drained first, otherwise the packed flash block could miss
        the newest value. Reuses the existing commit tail (airframe-name persist
        + tag-72 + deferred post-reboot verification).

        confirm=True (default) asks the caller-facing "Are you sure?" — the
        auto-offer from a boot-scoped param edit passes confirm=False because
        its own prompt has already asked.
        """
        if self._write_in_progress:
            self._log("⚠️ Apply & Reboot already in progress")
            return
        if not (self.parent_window and getattr(self.parent_window, 'connected', False)):
            self.status_label.setText("❌ Cannot commit - no connection")
            self.status_label.setStyleSheet("color: #e74c3c;")
            return

        # Never commit while the aircraft is flying — a flash erase+program
        # mid-air stalls the control loop and motors.
        if self.parent_window and hasattr(self.parent_window, 'can_write_parameters'):
            safe, reason = self.parent_window.can_write_parameters()
            if not safe:
                QMessageBox.critical(
                    self,
                    "🚫 Safety Blocked!",
                    f"Cannot commit parameters:\n\n{reason}\n\n"
                    "Committing parameters to flash while the aircraft is flying "
                    "is EXTREMELY DANGEROUS!\n"
                    "Please land the aircraft before making changes."
                )
                self.status_label.setText(f"🚫 Blocked: {reason}")
                self.status_label.setStyleSheet("color: #e74c3c;")
                return

        if confirm:
            state_name = "Unknown"
            if self.parent_window and hasattr(self.parent_window, 'flight_data'):
                if self.parent_window.flight_data:
                    state_name = FLIGHT_STATE_NAMES.get(
                        self.parent_window.flight_data.flight_state, "Unknown")

            reply = QMessageBox.question(
                self,
                "⚠️ Confirm Apply & Reboot",
                "Save the current parameter set to FC flash and restart the FC?\n\n"
                f"Aircraft State: {state_name}\n"
                "The FC reboots only after the write is verified in flash.\n\n"
                "Are you sure you want to continue?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.No:
                self.status_label.setText("Apply & Reboot cancelled")
                self.status_label.setStyleSheet("color: #666;")
                return

        self._write_in_progress = True
        self._flash_write_pending = True
        self.status_label.setText("💾 Committing parameters to flash and restarting FC...")
        self.status_label.setStyleSheet("color: #f39c12;")

        # Snapshot the whole current image so the post-reboot readback can be
        # verified against it (catches a failed flash commit/verify mismatch).
        self._written_float = self._compute_written_float()
        self._written_params = {}
        for i in range(self.MAX_PARAMS):
            if i not in self.params:
                continue
            widget = self.params[i]
            if isinstance(widget, QDoubleSpinBox):
                mult = self._display_mult(i)
                self._written_params[i] = int(widget.value() / mult)
            elif isinstance(widget, QSpinBox):
                self._written_params[i] = int(widget.value())
            elif isinstance(widget, QComboBox):
                data = widget.currentData()
                self._written_params[i] = int(data) if data is not None else widget.currentIndex()
        self._verify_mode = True
        self._expecting_param_packet = True
        self._write_timestamp = time.time()

        # Any live-write sitting in the 350 ms debounce window must hit RAM
        # before the tag-72 commit packs the flash block.
        #
        # A bulk-set (load) that never reached the FC — offline load, or a load
        # whose batch did not drain — leaves the FC RAM stale vs the UI. If one
        # is pending, push the WHOLE widget image first: it supersedes whatever
        # the FC held, and the debounced live-edit flush that follows overrides
        # just the indices the user edited afterwards. Without this the commit
        # flashes stale RAM (old values / factory defaults) while the verify
        # compares against the loaded widget values — the 10-slot mismatch.
        if getattr(self, '_image_dirty', False):
            self._image_dirty = False
            self._log("  ⚙️ Bulk-set pending — pushing full param image before commit")
            if hasattr(self.parent_window, 'send_params_typed'):
                self.parent_window.send_params_typed()
        self._flush_live_writes()
        self._when_param_write_drained(self._commit_to_flash)

    def _when_param_write_drained(self, callback, attempts=200):
        """Invoke callback once the tag-17 live-write queue has drained."""
        parent = self.parent_window
        if not parent or not getattr(parent, '_param_write_list', []):
            callback()
            return
        n = [attempts]

        def _poll():
            n[0] -= 1
            if not getattr(parent, '_param_write_list', []) or n[0] <= 0:
                callback()
                return
            QTimer.singleShot(20, _poll)

        QTimer.singleShot(20, _poll)

    def _commit_to_flash(self):
        """Persist the airframe name and commit RAM→flash via tag-72."""
        self.dirty_params.clear()
        self.write_progress.setValue(100)
        self.write_progress.setFormat("Committing to flash...")
        self.write_progress.show()
        # Persist the airframe name into FC config flash BEFORE commit so both
        # land in the same config write — flash is the single source of truth.
        if self.parent_window and hasattr(self.parent_window, 'send_afname'):
            self.parent_window.send_afname(self._airframe_name_to_persist())
        if self.parent_window and hasattr(self.parent_window, 'send_param_commit'):
            self.parent_window.send_param_commit()
        # Don't verify immediately — FC resets after commit.
        # Defer to when the next flight packet arrives (FC reconnected).
        self._log("  ⏳ Waiting for FC to reconnect after commit...")
        if self.parent_window:
            self.parent_window._pending_param_verification = True

    def _compute_written_float(self, indices=None):
        """Compute the float32 values that will be sent — mirrors send_params_typed logic.
        If indices is provided, only compute for those indices (matching _written_params).
        """
        param_range = indices if indices is not None else range(self.MAX_PARAMS)
        floats = {}
        for i in param_range:
            if i in self.params:
                widget = self.params[i]
                if isinstance(widget, QDoubleSpinBox):
                    mult = self._display_mult(i)
                    floats[i] = widget.value() / mult
                elif isinstance(widget, QSpinBox):
                    floats[i] = float(widget.value())
                elif isinstance(widget, QComboBox):
                    data = widget.currentData()
                    if data is None:
                        raise ValueError(
                            f"Combo param {i} selection has no data "
                            f"(index {widget.currentIndex()}) — refusing to write "
                            f"a positional index as the value")
                    floats[i] = float(data)
                else:
                    floats[i] = 0.0
            else:
                floats[i] = 0.0
        return floats

    def _airframe_name_to_persist(self):
        """Name to persist into the FC config flash on commit.

        Uses the currently loaded .af file's base name (the file is the source
        of the parameter set being written). The flash then becomes the single
        source of truth displayable on GCS restart.
        """
        src = self._base_airframe_name()
        return src if src and src != "Params" else ""

    def _request_write_verify(self):
        """Request param readback via tagged protocol (subtag 71) for verification"""
        self._write_timestamp = time.time()
        if self.parent_window and hasattr(self.parent_window, 'send_request'):
            self.parent_window.send_request(PacketTag.PARAM_TAGGED_READ, 255, 0)
            self._log("  📤 Requested tagged param readback for verification")
        # Start verification timeout — resets buttons if FC never responds
        if self._verification_timer:
            self._verification_timer.stop()
        self._verification_timer = QTimer()
        self._verification_timer.setSingleShot(True)
        self._verification_timer.timeout.connect(self._verification_timeout)
        self._verification_timer.start(5000)

    def _verification_timeout(self):
        self._log("⏱️ Verification timeout - no automatic param packet received from FC")
        self._verification_timer = None
        # If verification timed out, the param packet might still arrive later
        # Don't clear the verification state immediately - let the next param packet handle it
        if self._verify_mode:
            # Check if we've been waiting too long
            if time.time() - self._write_timestamp > 5.0:
                self._log("  ⏱️ Verification timeout - giving up")
                self._verify_mode = False
                self._written_params = {}
                self._written_float = {}
                self._expecting_param_packet = False
                self._write_in_progress = False
                self._flash_write_pending = False
                self._write_timestamp = 0
                
                self.status_label.setText("⚠️ Verification timeout - no param packet from FC")
                self.status_label.setStyleSheet("color: #e74c3c;")
            else:
                self._log("  ⏳ Still waiting for param packet...")
                # Restart timer
                self._verification_timer = QTimer()
                self._verification_timer.setSingleShot(True)
                self._verification_timer.timeout.connect(self._verification_timeout)
                self._verification_timer.start(1000)

    def closeEvent(self, event):
        data_manager.unregister_observer(self.on_data_updated)
        
        if self._read_timeout_timer:
            self._read_timeout_timer.stop()
            self._read_timeout_timer = None
        if self._write_timeout_timer:
            self._write_timeout_timer.stop()
            self._write_timeout_timer = None
        if self._verification_timer:
            self._verification_timer.stop()
            self._verification_timer = None
        
        if not self._save_if_dirty():
            event.ignore()
            return
        event.accept()
    
    def set_buttons_enabled(self, enabled: bool):
        pass
