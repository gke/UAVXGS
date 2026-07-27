# ui/parameter_window.py - Refactored with enums, no magic numbers
"""
UAVX Parameter Editor Window
"""

import sys
import os
from typing import Optional, Dict, List, Tuple
import time
import math


from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol_enums import (
    ParamIndex,
    RC_MAPPING_NAMES,
    RCControl,
    AirframeType,
    ESCType,
    RxType,
    ArmingMode,
    TelemetryType,

    RX_TYPE_NAMES,
    RF_TYPE_NAMES,
    AS_SENSOR_TYPE_NAMES,
    IMU_FILTER_NAMES,
    GYRO_LPF_NAMES,
    ACC_LPF_NAMES,
    BB_LOG_NAMES,
    MOTOR_STOP_NAMES,
    ACTIVE_AIRFRAMES,
    AIRFRAME_NAMES,
)
from protocol_constants import FlightState, FLIGHT_STATE_NAMES
from packet_parser import *
from core.data_manager import data_manager
from parameters import PARAMETER_DEFS, PARAM_LIMITS, PARAM_TYPES, PARAM_DISPLAY_MULT, PARAM_SCALES, LEGACY_TAGS
from airframes import airframes as af_module


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


class ParameterWindow(QMainWindow):
    """Parameter editor window - refactored with enums"""
    
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
        # Angle quaternion gains (display = raw, scale=1.0 for legacy)
        int(ParamIndex.ROLL_ANGLE_KP):    (5.0, 9.0),
        int(ParamIndex.PITCH_ANGLE_KP):   (5.0, 9.0),
        int(ParamIndex.YAW_ANGLE_KP):     (5.0, 10.0),
        # Angle integral gains (display = raw / PARAM_SCALE, scale=0.05)
        int(ParamIndex.ROLL_ANGLE_KI):    (1.0, 10.0),
        int(ParamIndex.PITCH_ANGLE_KI):   (1.0, 10.0),
        int(ParamIndex.YAW_ANGLE_KI):     (1.0, 10.0),
        # Angle integral limits (display = raw / PARAM_SCALE)
        int(ParamIndex.ROLL_ANGLE_INT_LIMIT): (19.1, 114.6),
        int(ParamIndex.PITCH_ANGLE_INT_LIMIT): (19.1, 114.6),
        int(ParamIndex.YAW_ANGLE_INT_LIMIT): (11.5, 68.8),
        # Rate proportional gains (display = raw / PARAM_SCALE, scale=0.005)
        int(ParamIndex.ROLL_RATE_KP):     (25.0, 100.0),
        int(ParamIndex.PITCH_RATE_KP):    (25.0, 100.0),
        int(ParamIndex.YAW_RATE_KP):      (25.0, 150.0),
        # Rate derivative gains (display = raw / PARAM_SCALE)
        int(ParamIndex.ROLL_RATE_KD):     (50.0, 200.0),
        int(ParamIndex.PITCH_RATE_KD):    (50.0, 200.0),
        int(ParamIndex.YAW_RATE_KD):      (200.0, 800.0),
        # Rate limits (display = deg/s)
        int(ParamIndex.MAX_ROLL_RATE):    (80.0, 360.0),
        int(ParamIndex.MAX_PITCH_RATE):   (60.0, 240.0),
        int(ParamIndex.MAX_COMPASS_YAW_RATE): (15.0, 120.0),
        # Altitude
        int(ParamIndex.ALT_POS_KP):       (10.9, 27.3),
        int(ParamIndex.ALT_POS_KI):       (2.2, 10.9),
        int(ParamIndex.ALT_THROTTLE_COMP_LIMIT): (10.0, 35.0),
        # Navigation
        int(ParamIndex.NAV_POS_KP):       (4.5, 18.2),
        int(ParamIndex.NAV_POS_KI):       (1.5, 6.3),
        int(ParamIndex.NAV_VEL_KP):       (1.7, 6.7),
        int(ParamIndex.HORIZON):          (2.0, 5.0),
        # Angle limits (display = deg)
        int(ParamIndex.MAX_PITCH_ANGLE):  (20.0, 40.0),
        int(ParamIndex.MAX_ROLL_ANGLE):   (20.0, 40.0),
    }

    # Physical descriptor definitions for Setup row (dynamic per AF category)
    # Each entry: (metadata_key, label, decimals, default, tooltip)
    _MR_PHYS_DESCRIPTORS = [
        ("PHYS_AUW_G",      "AUW (g)",         0, 800,    "All-up weight in grams — weigh the aircraft on a scale"),
        ("PHYS_ARM_MM",     "Arm (mm)",        0, 220,    "Motor shaft to center distance"),
        ("PHYS_PROP_INCH",  "Prop (in)",       1, 11.0,   "Prop diameter in inches"),
        ("PHYS_MOTOR_W",    "Motor W",         0, 100,    "Watts per motor — from ESC or power meter"),
        ("PHYS_MOTOR_COUNT","Motors",          0, 4,      "Number of motors"),
    ]
    _FW_PHYS_DESCRIPTORS = [
        ("PHYS_AUW_G",       "AUW (g)",        0, 1000,  "All-up weight in grams — weigh the aircraft on a scale"),
        ("PHYS_WINGSPAN_MM", "Wing (mm)",      0, 1800,  "Wingtip to wingtip"),
        ("PHYS_CHORD_MM",    "Chord (mm)",     0, 250,   "Average root chord"),
        ("PHYS_PROP_INCH",   "Prop (in)",      1, 10.0,  "Prop diameter in inches"),
        ("PHYS_MOTOR_W",     "Motor W",        0, 150,   "Watts — from ESC or power meter"),
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

    # Motor count lookup from AF type (MR only)
    _MR_MOTOR_COUNT = {
        3: 4, 4: 4, 5: 4, 6: 4,   # QUAD, QUAD_X, QUAD_COAX, QUAD_COAX_X
        7: 6, 8: 6,                 # HEX, HEX_X
        9: 8, 10: 8,                # OCT, OCT_X
    }

    # MR shape factors for inertia from arm length
    _MR_SHAPE_KF = {4: 0.55, 6: 0.50, 8: 0.45}

    # Default base curves (unscaled) for computing scale factors
    # These represent a "reference" MR at 800g AUW, 220mm arm, 1000kv, 11"
    _REF_MR_AUW_G = 800
    _REF_MR_ARM_MM = 220
    _REF_MR_PROP_INCH = 11.0
    _REF_FW_AUW_G = 1000
    _REF_FW_WINGSPAN_MM = 1800

    # Parameter groups defined with enums
    PID_PARAMS = [
        ("Q Angle", ParamIndex.ROLL_ANGLE_KP, ParamIndex.PITCH_ANGLE_KP, ParamIndex.YAW_ANGLE_KP, 7.0, 7.0, 3.0),
        ("Ki Angle", ParamIndex.ROLL_ANGLE_KI, ParamIndex.PITCH_ANGLE_KI, ParamIndex.YAW_ANGLE_KI, 3, 3, 3),
        ("I-Limit", ParamIndex.ROLL_ANGLE_INT_LIMIT, ParamIndex.PITCH_ANGLE_INT_LIMIT, ParamIndex.YAW_ANGLE_INT_LIMIT, 10, 10, 10),
        ("Kp Rate", ParamIndex.ROLL_RATE_KP, ParamIndex.PITCH_RATE_KP, ParamIndex.YAW_RATE_KP, 0.30, 0.45, 0.75),
        ("Kd Rate", ParamIndex.ROLL_RATE_KD, ParamIndex.PITCH_RATE_KD, ParamIndex.YAW_RATE_KD, 0.015, 0.0225, 0.0375),
        ("Max Rate (Deg/Sec)", ParamIndex.MAX_ROLL_RATE, ParamIndex.MAX_PITCH_RATE, ParamIndex.MAX_COMPASS_YAW_RATE, 240, 120, 120),
    ]
    
    RC_MAP_PARAMS = [
        (ParamIndex.RX_THROTTLE_CH, "Throttle", 0),
        (ParamIndex.RX_ROLL_CH, "Roll", 1),
        (ParamIndex.RX_PITCH_CH, "Pitch", 2),
        (ParamIndex.RX_YAW_CH, "Yaw", 3),
        (ParamIndex.RX_GEAR_CH, "NavMode", 4),
        (ParamIndex.RX_AUX1_CH, "AttMode", 5),
        (ParamIndex.RX_AUX2_CH, "NavQual", 6),
        (ParamIndex.RX_AUX3_CH, "CamPitch", 10),
        (ParamIndex.RX_AUX4_CH, "Aux2", 8),
        (ParamIndex.RX_AUX5_CH, "Transition", 9),
        (ParamIndex.RX_AUX6_CH, "PassThru", 7),
        (ParamIndex.RX_AUX7_CH, "Dive", 11),
    ]
    
    ALTITUDE_PARAMS = [
        (ParamIndex.ALT_POS_KP, "Kp Alt", 28),
        (ParamIndex.ALT_POS_KI, "Ki Alt", 10),
        (ParamIndex.ALT_POS_INT_LIMIT, "I-Limit", 10),
        (ParamIndex.MAX_YAW_RATE, "Max ROC (m/s)", 2),  # MAX_YAW_RATE is used for Max ROC in params.h
        (ParamIndex.ALT_LPF, "Alt LPF (Hz)", 5),
        (ParamIndex.ALT_HOLD_THR_COMP_DECAY_PERCENT_PS, "Thr Decay (%/s)", 3),
        (ParamIndex.ALT_THROTTLE_COMP_LIMIT, "Thr Comp Limit", 20),
        (ParamIndex.AH_THROTTLE_MOVING_TRIGGER, "Thr Move Trig", 10),
    ]
    
    NAVIGATION_PARAMS = [
        (ParamIndex.NAV_POS_KP, "Kp Pos", 20),
        (ParamIndex.NAV_POS_KI, "Ki Pos", 5),
        (ParamIndex.NAV_VEL_KP, "Kp Vel", 20),
        (ParamIndex.NAV_POS_INT_LIMIT, "Max Vel (m/s)", 3),
        (ParamIndex.NAV_MAX_ANGLE, "Max Angle (Deg)", 30),
        (ParamIndex.NAV_RTH_ALT, "RTH Alt (m)", 15),
        (ParamIndex.NAV_MAG_VAR, "Mag Var (Deg)", 12.8),
        (ParamIndex.NAV_CROSS_TRACK_KP, "XTrack Kp", 4),
        (ParamIndex.NAV_HEADING_TURNOUT, "Heading Turnout", 50),
        (ParamIndex.MAX_COMPASS_YAW_RATE, "Max Yaw Rate (°/s)", 30),
        (ParamIndex.NAV_PROX_ALT_M, "Prox Alt (m)", 4),
        (ParamIndex.NAV_PROX_RADIUS_M, "Prox Radius (m)", 3),
        (ParamIndex.NAV_FENCE_RADIUS_M, "Fence Rad (m)", 100),
        (ParamIndex.EST_CRUISE_THR, "Cruise Thr (%)", 45),
        (ParamIndex.DESCENT_DELAY_S, "Land Delay (s)", 15),
        (ParamIndex.MAX_DESCENT_RATE_DMP_S, "Land Rate (m/s)", 7),
        (ParamIndex.MOTOR_STOP_SEL, "Motor Stop", 0, list(MOTOR_STOP_NAMES.values())),
        (ParamIndex.VRS_ROC, "VRS ROC (m/s)", 3),
        (ParamIndex.AH_ROC_WINDOW_MPS, "AH ROC Win (m/s)", 1),
    ]
    
    GENERAL_PARAMS = [
        (ParamIndex.AF_TYPE, "Airframe", 4, [AIRFRAME_NAMES[af] for af in sorted(ACTIVE_AIRFRAMES, key=lambda x: AIRFRAME_NAMES[x].lower())], [af.value for af in sorted(ACTIVE_AIRFRAMES, key=lambda x: AIRFRAME_NAMES[x].lower())]),
        (ParamIndex.ESC_TYPE, "ESC Type", 2, list(ESC_TYPE_NAMES.values())),
        (ParamIndex.RX_TYPE, "Rx Type", 0, list(RX_TYPE_NAMES.values())),
        (ParamIndex.ARMING_MODE, "Arming Mode", 1, list(ARMING_MODE_NAMES.values())),
        (ParamIndex.TELEMETRY_TYPE, "Telemetry", 1, list(TELEMETRY_TYPE_NAMES.values())),
        (ParamIndex.RF_SENSOR_TYPE, "RF Type", 0, list(RF_TYPE_NAMES.values())),
        (ParamIndex.AS_SENSOR_TYPE, "Airspeed", 4, list(AS_SENSOR_TYPE_NAMES.values())),
        (ParamIndex.PERCENT_IDLE_THR, "Idle Thr (%)", 10),
        (ParamIndex.STICK_HYSTERESIS, "Hysteresis", 2),
    ]

    GENERAL_CAMERA_PARAMS = [
        (ParamIndex.ROLL_CAM_KP, "Roll Cam Kp", 50),
        (ParamIndex.PITCH_CAM_KP, "Pitch Cam Kp", 50),
        (ParamIndex.ROLL_CAM_TRIM, "Roll Cam Trim", 0),
        (ParamIndex.SERVO_SENSE, "Servo Sense", 0),
        (ParamIndex.BB_LOG_TYPE, "BB Log", 0, list(BB_LOG_NAMES.values())),
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
        (ParamIndex.MADGWICK_KP_MAG, "Kp Mag", 50),
        (ParamIndex.MADGWICK_KP_ACC, "Kp Acc", 20),
        (ParamIndex.ACC_CONF_SD, "Acc Conf (SD)", 4),
        (ParamIndex.HORIZON, "Horizon", 30),
        (ParamIndex.TILT_THROTTLE_FF, "Tilt Thr (%)", 25),
    ]
    
    MISC1_PARAMS = []

    MISC2_PARAMS = [
        (ParamIndex.LOW_VOLT_THRES, "Low Volt (V)", 16),
        (ParamIndex.VOLT_SCALE, "Volt Scale", 19),
        (ParamIndex.CURRENT_SCALE, "Curr Scale", 0),
        (ParamIndex.BATTERY_CAPACITY, "Batt mAH", 2200),
        (ParamIndex.THROTTLE_GAIN_RATE, "Thr Gain (%)", 0),
    ]
    
    FW_PARAMS = [
        (ParamIndex.FW_MAX_CLIMB_ANGLE, "Climb Angle (Deg)", 10),
        (ParamIndex.BALANCE, "Trim Angle (Deg)", 5),
        (ParamIndex.FW_CLIMB_THROTTLE, "Climb Thr (%)", 70),
        (ParamIndex.FW_ROLL_PITCH_FF, "Roll/Pitch FF (%)", 25),
        (ParamIndex.FW_PITCH_THROTTLE_FF, "Pitch Thr FF (%)", 50),
        (ParamIndex.FW_AILERON_RUDDER_MIX, "Ail/Rud Mix (%)", 10),
        (ParamIndex.FW_ALT_SPOILER_FF, "Alt Spoiler FF (%)", 50),
        (ParamIndex.FW_SPOILER_DECAY_PERCENT_PS, "Spoiler Decay (%/s)", 15),
        (ParamIndex.FW_AILERON_DIFFERENTIAL, "Aileron Diff (%)", 0),
        (ParamIndex.FW_STICK_SCALE, "Stick Scale(%)", 30),
        (ParamIndex.FW_ROLL_CONTROL_PITCH_LIMIT, "Roll/Pitch Limit (Deg)", 60),
        (ParamIndex.FW_BOARD_PITCH_ANGLE, "Board Pitch (Deg)", 0),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.params = {}
        self.dirty_params = set()
        self.rc_channels = [0] * 16
        self._param_grid = None
        self._config_group = None
        self._fw_group = None
        self.motor_bars = []
        self.motor_pct_labels = []
        self.motor_name_labels = []
        self.parent_window = parent
        self.voltage_trim = 1.0
        self._params_received = False
        self._read_request_id = None
        self._committed_values = {}  # last FC-loaded values for protected param revert
        self._baseline_snapshot = {}  # param values at last load — for dirty detection
        self._baseline_source_path = None  # .af path that was loaded (for default save path)
        self._write_request_id = None
        self._read_timeout_timer = None
        self._write_timeout_timer = None
        self._verify_mode = False
        self._written_params = {}
        self._written_float = {}
        self._write_in_progress = False
        self._verification_timer = None
        self._expecting_param_packet = False
        self._write_timestamp = 0
        self.MAX_PARAMS = 128
        self._config_values = {}
        self._current_airframe_path = None
        self._legacy_mode = False
        self._character_slider = None
        self._character_value_label = None
        self._advanced_checkbox = None
        self._computed_values = {}  # last computed values for reset
        self.setWindowTitle("UAVX Parameters")
        self.setMinimumSize(1200, 900)
        
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
        
        print("🔵 Parameter window initialized (refactored with enums)")
        
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

    def _on_legacy_toggled(self, checked: bool):
        self._legacy_mode = checked
        self._refresh_legacy_display()

    def _refresh_legacy_display(self):
        """Re-apply display multiplier to all QDoubleSpinBox params in-place.

        For each spinbox: read current value, divide by the OLD mult to
        recover the raw float32, then set the new range and value using
        the NEW (possibly legacy) mult.  The raw value is preserved.
        """
        # prev_mult is the multiplier that was active before the toggle.
        # _legacy_mode has already been updated, so:
        #   - if legacy is now ON,  prev was the normal mult
        #   - if legacy is now OFF, prev was the legacy mult (for LEGACY_TAGS)
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
            limits = PARAM_LIMITS.get(idx, (0.0, 255.0))
            lo = limits[0] * new_mult
            hi = limits[1] * new_mult
            span = hi - lo
            step = max(0.001, span / 50.0)
            if new_mult == 100:
                step = 1.0
            if idx in (68, 113) and step < 1.0:
                step = 1.0
            dec = max(0, min(6, -int(math.floor(math.log10(step)))))
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
        
        # Normal update
        for idx in received_u8:
            if idx not in self.params:
                continue
            widget = self.params[idx]
            try:
                if isinstance(widget, QDoubleSpinBox):
                    mult = self._display_mult(idx)
                    fc_float = received_float.get(idx, float(received_u8[idx]))
                    widget.setValue(fc_float * mult)
                elif isinstance(widget, QComboBox):
                    cb_idx = widget.findData(int(received_u8[idx]))
                    if cb_idx >= 0:
                        widget.setCurrentIndex(cb_idx)
                    elif received_u8[idx] < widget.count():
                        widget.setCurrentIndex(received_u8[idx])
            except Exception:
                pass

        # Update config values from FC typed data
        for cfg_idx in (ParamIndex.CONFIG1_BITS, ParamIndex.CONFIG2_BITS):
            ci = int(cfg_idx)
            if ci in received_float:
                self._config_values[ci] = int(received_float[ci])
        self.update_config_display()

        # Don't touch UI state during write — echo ACKs update widgets in-place
        if self._verify_mode or self._write_in_progress:
            return

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
                        cb_idx = widget.findData(int(value))
                        if cb_idx >= 0:
                            widget.setCurrentIndex(cb_idx)
                        elif value < widget.count():
                            widget.setCurrentIndex(value)
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
        
        if len(self._written_params) == 0:
            self._log("  ⚠️ WARNING: No written params to verify!")
            self._verify_mode = False
            self._write_in_progress = False
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
        
        # Derived params that FC recalculates via ApplyParameters
        derived_params = {
            ParamIndex.MAX_ROLL_RATE: "MaxRollRate",
            ParamIndex.MAX_PITCH_RATE: "MaxPitchRate",
            ParamIndex.HORIZON: "Horizon",
            ParamIndex.MADGWICK_KP_ACC: "MadgwickKpAcc",
            ParamIndex.ACC_CONF_SD: "AccConfSD",
            ParamIndex.FW_ROLL_PITCH_FF: "FWRollPitchFF",
        }
        
        # Params with clamping ranges - use raw ints to avoid enum issues
        clamped_indices = [70, 68, 69, 53, 31, 46, 80, 111, 112, 115, 116]
        clamped_ranges = {
            70: (10, 50),   # FWAileronDifferential
            68: (10, 180),  # FWMaxClimbAngle
            69: (10, 60),   # NavMaxAngle
            53: (1, 127),   # BatteryCapacity
            31: (30, 50),   # MadgwickKpMag
            46: (1, 10),    # MaxDescentRateDmpS
            80: (0, 1),  # YawSymmetryFactor
            111: (2, 200),  # KFAccUVar
            112: (2, 200),  # FWStickScale
             115: (20, 20000), # NavFenceRadiusM
             116: (0, 250),  # DiveRecoverAlt
        }
        
        for idx, written_value in self._written_params.items():
            received_value = received_params.get(idx)
            if received_value is None:
                mismatches.append(f"P{idx+1}: Written={written_value}, Not received")
                mismatch_indices.append(idx)
                self._log(f"    ❌ P{idx+1}: Not received")
                continue

            # For FLOAT params with float data, compare in float32 space
            is_float_param = PARAM_TYPES.get(idx, 'U8') == 'FLOAT'
            float_available = (received_float is not None and idx in received_float
                               and idx in self._written_float)

            if is_float_param and float_available:
                wf = self._written_float[idx]
                rf = received_float[idx]
                if abs(wf - rf) < 1e-6:
                    self._log(f"    ✅ P{idx+1}: float {wf} == {rf}")
                    continue
                if idx in derived_params:
                    derived_warnings.append(
                        f"P{idx+1} ({derived_params[idx]}): Written={wf} → "
                        f"Recalculated to {rf} (FC derived parameter)"
                    )
                    self._log(f"    ℹ️ P{idx+1}: {wf} → {rf} (derived)")
                    continue
                mismatches.append(f"P{idx+1}: Written={wf}, Received={rf}")
                mismatch_indices.append(idx)
                self._log(f"    ❌ P{idx+1}: Written={wf}, Received={rf}")
                continue

            # Fall back to uint8 comparison (U8 params, or float data unavailable)
            if written_value != received_value:
                if idx in derived_params:
                    derived_warnings.append(
                        f"P{idx+1} ({derived_params[idx]}): Written={written_value} → "
                        f"Recalculated to {received_value} (FC derived parameter)"
                    )
                    self._log(f"    ℹ️ P{idx+1}: {written_value} → {received_value} (derived)")
                    continue

                if idx in clamped_indices:
                    min_val, max_val = clamped_ranges.get(idx, (0, 255))
                    if written_value < min_val and received_value == min_val:
                        clamped_warnings.append(f"P{idx+1}: Written={written_value} → Clamped to {received_value} (min={min_val})")
                        self._log(f"    ⚠️ P{idx+1}: {written_value} → {received_value} (clamped)")
                        continue
                    elif written_value > max_val and received_value == max_val:
                        clamped_warnings.append(f"P{idx+1}: Written={written_value} → Clamped to {received_value} (max={max_val})")
                        self._log(f"    ⚠️ P{idx+1}: {written_value} → {received_value} (clamped)")
                        continue

                mismatches.append(f"P{idx+1}: Written={written_value}, Received={received_value}")
                mismatch_indices.append(idx)
                self._log(f"    ❌ P{idx+1}: Written={written_value}, Received={received_value}")
            else:
                self._log(f"    ✅ P{idx+1}: {written_value} == {received_value}")
        
        self._verify_mode = False
        self._write_in_progress = False
        
        # Sync UI widgets with FC's actual values (in case of mismatches, clamps, or derived)
        for i, value in received_params.items():
            if i not in self.params:
                continue
            widget = self.params[i]
            try:
                if isinstance(widget, QDoubleSpinBox):
                    mult = self._display_mult(i)
                    if received_float is not None and i in received_float:
                        widget.setValue(received_float[i] * mult)
                    else:
                        widget.setValue(float(value) * mult)
                elif isinstance(widget, QComboBox):
                    cb_idx = widget.findData(int(value))
                    if cb_idx >= 0:
                        widget.setCurrentIndex(cb_idx)
                    else:
                        widget.setCurrentIndex(max(0, min(widget.count()-1, int(value))))
            except Exception:
                pass
            # Update _config_values cache for config bit params
            if i == int(ParamIndex.CONFIG1_BITS) or i == int(ParamIndex.CONFIG2_BITS):
                old = self._config_values.get(i, -1)
                if old != value:
                    self._log(f"  🔧 Config{i}: _config_values {old} → {value} (synced from FC)")
                self._config_values[i] = int(value)
        
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
            
            self.WriteParamsButton.setStyleSheet("""
                QPushButton {
                    background-color: #e74c3c;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #c0392b;
                    border-radius: 4px;
                    padding: 4px 8px;
                }
                QPushButton:hover {
                    background-color: #c0392b;
                }
            """)
            self.WriteParamsButton.setText("❌ Failed")
            self.WriteParamsButton.setEnabled(True)
            QTimer.singleShot(3000, self.reset_write_button)
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
            
            self.WriteParamsButton.setStyleSheet("""
                QPushButton {
                    background-color: #f39c12;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #e67e22;
                    border-radius: 4px;
                    padding: 4px 8px;
                }
                QPushButton:hover {
                    background-color: #e67e22;
                }
            """)
            self.WriteParamsButton.setText("⚠️ Adjusted")
            self.WriteParamsButton.setEnabled(True)
            QTimer.singleShot(3000, self.reset_write_button)
        else:
            QMessageBox.information(
                self,
                "✅ Verification Successful",
                f"All {len(self._written_params)} parameters verified successfully!"
            )
            self.status_label.setText(f"✅ All {len(self._written_params)} parameters verified OK!")
            self.status_label.setStyleSheet("color: #27ae60;")
            
            self.WriteParamsButton.setStyleSheet("""
                QPushButton {
                    background-color: #27ae60;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #229954;
                    border-radius: 4px;
                    padding: 4px 8px;
                }
                QPushButton:hover {
                    background-color: #229954;
                }
            """)
            self.WriteParamsButton.setText("✅ Success")
            self.WriteParamsButton.setEnabled(True)
            QTimer.singleShot(2000, self.reset_write_button)
    
    def reset_read_button(self):
        self.ReadParamsButton.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                font-weight: bold;
                border: 2px solid #229954;
                border-radius: 4px;
                padding: 4px 8px;
            }
            QPushButton:hover {
                background-color: #229954;
            }
        """)
        self.ReadParamsButton.setText("Read")
        self.ReadParamsButton.setEnabled(True)
    
    def reset_write_button(self):
        self._write_in_progress = False
        self.WriteParamsButton.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                font-weight: bold;
                border: 2px solid #229954;
                border-radius: 4px;
                padding: 4px 8px;
            }
            QPushButton:hover {
                background-color: #229954;
            }
        """)
        self.WriteParamsButton.setText("Write")
        self.WriteParamsButton.setEnabled(True)
    
    def _read_timeout(self):
        self._log("⏱️ Read timeout")
        self.status_label.setText("⚠️ Read timeout - no response from FC")
        self.status_label.setStyleSheet("color: #e74c3c;")
        self.reset_read_button()
        self._read_request_id = None
        self._read_timeout_timer = None
    
    def _write_timeout(self):
        self._log("⏱️ Write timeout - resetting")
        self._write_in_progress = False
        if self._verify_mode:
            self._verify_mode = False
        self._written_params = {}
        self._written_float = {}
        self._expecting_param_packet = False
        self._write_timestamp = 0

        self.status_label.setText("⚠️ Write timeout - no response from FC")
        self.status_label.setStyleSheet("color: #e74c3c;")
        self.reset_write_button()
        self._write_request_id = None
        self._write_timeout_timer = None
    
    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(4)
        main_layout.setContentsMargins(4, 4, 4, 4)
        
        toolbar = QHBoxLayout()
        toolbar.setSpacing(4)
        
        self.ReadParamsButton = QPushButton("Read")
        self.ReadParamsButton.setStyleSheet("""
            QPushButton {
                font-weight: bold; 
                padding: 4px 12px;
                background-color: #27ae60;
                color: white;
                border: 2px solid #229954;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #229954;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
                color: #666;
            }
        """)
        self.ReadParamsButton.setProperty('original_text', 'Read')
        toolbar.addWidget(self.ReadParamsButton)
        
        self.WriteParamsButton = QPushButton("Write")
        self.WriteParamsButton.setStyleSheet("""
            QPushButton {
                font-weight: bold; 
                padding: 4px 12px;
                background-color: #27ae60;
                color: white;
                border: 2px solid #229954;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #229954;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
                color: #666;
            }
        """)
        self.WriteParamsButton.setProperty('original_text', 'Write')
        toolbar.addWidget(self.WriteParamsButton)
        
        self.load_btn = QPushButton("Load Params")
        self.load_btn.setStyleSheet("padding: 4px 12px;")
        toolbar.addWidget(self.load_btn)
        
        self.save_btn = QPushButton("Save Params")
        self.save_btn.setStyleSheet("padding: 4px 12px;")
        toolbar.addWidget(self.save_btn)
        
        self.airframe_combo = QComboBox()
        self.airframe_combo.setMinimumWidth(160)
        self.airframe_combo.setToolTip("Select airframe to load defaults")
        self.airframe_combo.addItem("— Defaults —", "__DEFAULTS__")
        airframe_list = af_module.list_airframes()
        for name, path in airframe_list:
            self.airframe_combo.addItem(name, path)
        toolbar.addWidget(self.airframe_combo)
        
        self.legacy_check = QCheckBox("Legacy")
        self.legacy_check.setToolTip("Display legacy-tagged params as raw/scale (classic FC units)")
        self.legacy_check.toggled.connect(self._on_legacy_toggled)
        toolbar.addWidget(self.legacy_check)
        
        toolbar.addStretch()
        
        self.status_label = QLabel("Ready - Click Read to get parameters from FC")
        self.status_label.setStyleSheet("color: #666;")
        toolbar.addWidget(self.status_label)
        
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
        
        # Advanced toggle
        self._advanced_checkbox = QCheckBox("Advanced Parameters")
        self._advanced_checkbox.setToolTip("Show all parameter groups (PID, General, Config, etc.)")
        self._advanced_checkbox.setStyleSheet("font-weight: bold; padding: 2px;")
        self._advanced_checkbox.toggled.connect(self._on_advanced_toggled)
        self.param_layout.addWidget(self._advanced_checkbox)
        
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
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(4)
        self._param_grid = grid

        # Row 0: PID, General, Configuration, Battery
        pid_group = self._create_pid_group()
        grid.addWidget(pid_group, 0, 0)

        gen_group = self._create_general_group()
        grid.addWidget(gen_group, 0, 1)

        config_group = self._create_config_group()
        grid.addWidget(config_group, 0, 2)

        batt_group = self._create_compact_group("Battery", self.MISC2_PARAMS)
        grid.addWidget(batt_group, 0, 3)

        # Row 1: Navigation, Altitude, Filters, Fixed Wing
        nav_group = self._create_two_col_compact_group("Navigation", self.NAVIGATION_PARAMS)
        grid.addWidget(nav_group, 1, 0)

        alt_group = self._create_compact_group("Altitude", self.ALTITUDE_PARAMS)
        grid.addWidget(alt_group, 1, 1)

        filt_group = self._create_filters_group()
        grid.addWidget(filt_group, 1, 2)

        self._fw_group = self._create_fixed_wing_group()
        grid.addWidget(self._fw_group, 1, 3)

        # Wrap grid in a widget so Advanced toggle can show/hide it
        grid_container = QWidget()
        grid_container.setLayout(grid)
        self._grid_container = grid_container
        self.param_layout.addWidget(grid_container)
        grid_container.setVisible(False)
        
        # Connect AF_TYPE combo to FW styling
        QTimer.singleShot(0, self._update_fw_style)
    
    def _update_fw_style(self):
        """Style Fixed Wing group: normal when active, faint when inactive"""
        if self._fw_group is None:
            return
        af_combo = self.params.get(int(ParamIndex.AF_TYPE))
        if af_combo is None or not isinstance(af_combo, QComboBox):
            self._fw_group.setEnabled(False)
            return
        # Reverse-lookup display name → enum value (combo items lack itemData)
        text = af_combo.currentText()
        idx = next((af.value for af, name in AIRFRAME_NAMES.items() if name == text), None)
        if idx is None:
            self._fw_group.setEnabled(False)
            return
        is_fw = AirframeType.ELEVON <= idx <= AirframeType.VTOL2
        if is_fw:
            self._fw_group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; }")
            self._fw_group.setEnabled(True)
        else:
            self._fw_group.setStyleSheet("QGroupBox { font-weight: bold; color: #bbb; border: 1px solid black; border-radius: 4px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px 0 3px; color: #bbb; }")
            self._fw_group.setEnabled(False)
    
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
        layout.setHorizontalSpacing(3)
        layout.setVerticalSpacing(1)
        layout.setContentsMargins(4, 8, 4, 4)

        # Column widths: [µs, bar, Ch, Func] repeated for left and right halves
        col_widths = [35, 35, 42, 35]
        col_stretch = [0, 1, 0, 0]
        for half in range(2):
            for c, w in enumerate(col_widths):
                ci = half * 4 + c
                layout.setColumnMinimumWidth(ci, w)
                layout.setColumnStretch(ci, col_stretch[c])

        # Left header
        layout.addWidget(self._header("µs"), 0, 0)
        layout.addWidget(self._header(""), 0, 1)
        layout.addWidget(self._header("Ch"), 0, 2)
        layout.addWidget(self._header("Func"), 0, 3)
        # Right header
        layout.addWidget(self._header("µs"), 0, 4)
        layout.addWidget(self._header(""), 0, 5)
        layout.addWidget(self._header("Ch"), 0, 6)
        layout.addWidget(self._header("Func"), 0, 7)

        # right-align µs and Ch headers, left-align Func headers
        for col in [0, 2, 4, 6]:
            item = layout.itemAtPosition(0, col)
            if item and item.widget():
                item.widget().setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        for col in [3, 7]:
            item = layout.itemAtPosition(0, col)
            if item and item.widget():
                item.widget().setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        self.rc_value_labels = {}
        self.rc_progress_bars = {}
        self.rc_channel_spins = []

        for fn_idx, (param_idx, func_name, default_ch) in enumerate(self.RC_MAP_PARAMS):
            col_offset = 4 * (fn_idx // 6)
            row = (fn_idx % 6) + 1

            val_label = QLabel("---")
            val_label.setStyleSheet("font-weight: bold;")
            val_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val_label.setFixedWidth(35)
            layout.addWidget(val_label, row, col_offset)
            self.rc_value_labels[fn_idx] = val_label

            bar = TickBar()
            bar.setRange(900, 2100)
            bar.setValue(1500)
            bar.setMinimumWidth(35)
            bar.setMinimumHeight(18)
            layout.addWidget(bar, row, col_offset + 1)
            self.rc_progress_bars[fn_idx] = bar

            spin = QSpinBox()
            spin.setRange(0, 15)
            spin.setValue(max(0, min(15, default_ch)))
            spin.setProperty("param_index", int(param_idx))
            spin.setProperty("fn_index", fn_idx)
            spin.valueChanged.connect(lambda v, i=int(param_idx), fn=fn_idx: self._rc_channel_changed(i, fn, v))
            spin.setToolTip(f"{func_name} channel (0=none, 1-15=physical channel)")
            spin.setMaximumWidth(42)
            spin.setMinimumWidth(38)
            spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            spin.setMaximumHeight(18)
            spin.setStyleSheet("""
                QSpinBox { padding: 1px 2px; }
                QSpinBox::up-button, QSpinBox::down-button { width: 12px; }
            """)
            layout.addWidget(spin, row, col_offset + 2)
            self.params[int(param_idx)] = spin
            self.rc_channel_spins.append(spin)

            func_label = QLabel(func_name)
            func_label.setStyleSheet("font-weight: bold;")
            func_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            layout.addWidget(func_label, row, col_offset + 3)

        group.setLayout(layout)
        return group
    
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
        motor_layout.addWidget(self._header(""), 0, 1)
        motor_layout.addWidget(self._header("Ch"), 0, 2)
        motor_layout.addWidget(self._header("%"), 0, 3)
        motor_layout.addWidget(self._header(""), 0, 4)
        motor_layout.addWidget(self._header("Ch"), 0, 5)

        # left-align Ch headers
        for col in [2, 5]:
            item = motor_layout.itemAtPosition(0, col)
            if item and item.widget():
                item.widget().setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        motor_names = ["M0", "M1", "M2", "M3", "M4", "M5", "M6", "M7", "M8", "M9", "M10", "M11"]
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

            name = QLabel(motor_names[i])
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

    def _update_motor_display(self, flight_data):
        """Update motor/servo bargraphs from flight data"""
        if hasattr(flight_data, 'pwm') and flight_data.pwm:
            num_motors = min(len(flight_data.pwm), 10)
            for i in range(12):
                if i < num_motors:
                    pwm_val = flight_data.pwm[i]
                    scaled_val = max(0, min(1000, int((pwm_val - 1000) * 1.0)))
                    self.motor_bars[i].setRange(0, 1000, ticks=[500])
                    self.motor_bars[i].setValue(scaled_val)
                    self.motor_pct_labels[i].setText(f"{int(scaled_val / 10)}%")
                    self.motor_name_labels[i].setText(f"M{i}")
                    pct = int(scaled_val / 10)
                    if pct < 10 or pct > 80:
                        bg = "#f39c12"
                    else:
                        bg = "#555"
                    self.motor_pct_labels[i].setStyleSheet(
                        f"background-color: {bg}; font-weight: bold;"
                    )
                else:
                    self.motor_bars[i].setValue(0)
                    self.motor_pct_labels[i].setText("0%")
                    self.motor_name_labels[i].setText(f"M{i}")
                    self.motor_pct_labels[i].setStyleSheet(
                        "background-color: #999; font-weight: bold;"
                    )
        else:
            for i in range(12):
                self.motor_bars[i].setValue(0)
                self.motor_pct_labels[i].setText("0%")
                self.motor_name_labels[i].setText(f"M{i}")
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
                grid.addWidget(combo, row, col * 2)
            else:
                spin = QDoubleSpinBox()
                spin.setRange(0, 9999)
                spin.setDecimals(dec)
                spin.setMaximumWidth(80)
                spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                spin.setStyleSheet("QDoubleSpinBox { padding: 1px 2px; }")
                spin.valueChanged.connect(lambda v, ii=idx_int: self._setup_sync_to_advanced(ii))
                self._setup_widgets[idx_int] = spin
                grid.addWidget(spin, row, col * 2)
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            lbl.setStyleSheet("font-weight: normal;")

        layout.addLayout(grid)

        # Character slider — single compact row
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
        for af in sorted(ACTIVE_AIRFRAMES, key=lambda x: AIRFRAME_NAMES[x].lower()):
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
        if af_combo and isinstance(af_combo, QComboBox):
            af_data = af_combo.currentData()
            if af_data is not None:
                cat = self._category_for_af(int(af_data))

        if cat == 'FW':
            descriptors = self._FW_PHYS_DESCRIPTORS
        elif cat == 'LAND':
            descriptors = self._LAND_PHYS_DESCRIPTORS
        else:
            descriptors = self._MR_PHYS_DESCRIPTORS

        for i, (key, label, dec, default, tooltip) in enumerate(descriptors):
            row = i // 3
            col = i % 3
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
            elif 'PROP' in key or 'WHEEL' in key:
                spin.setRange(1, 50)
            elif 'MOTOR_W' in key:
                spin.setRange(1, 5000)
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

    def _phys_changed(self, key):
        """Handle change to a physical descriptor — recompute derived values and check sanity"""
        phys_meta = {}
        for k, widget in self._phys_widgets.items():
            if isinstance(widget, QDoubleSpinBox):
                phys_meta[k] = str(int(widget.value())) if widget.decimals() == 0 else str(widget.value())
        self._current_phys_meta = self._compute_physics_from_descriptors(phys_meta)

        if hasattr(self, '_phys_hover_lbl'):
            thr = float(self._current_phys_meta.get('PHYS_HOVER_THR', 0.5))
            self._phys_hover_lbl.setText(f"Est Hover Throttle: {thr:.0%}")
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
        for key, widget in self._phys_widgets.items():
            if key in normalized:
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
                adv_idx = adv_w.findData(int(data))
                if adv_idx >= 0:
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
                    s_idx = setup_w.findData(int(adv_data))
                    if s_idx >= 0:
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

        computed = {}
        for idx_str, (conservative, aggressive) in self._PARAM_CURVES.items():
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

        self.status_label.setText(f"Computed {len(computed)} params (Character={self._character_slider.value()}%)")
        self.status_label.setStyleSheet("color: #3498db;")
        self.update_config_display()

    def _get_scale_factors(self, cat):
        """Compute per-param scaling factors from physical descriptors.

        Returns {param_idx: scale_factor}. Base curves are for the reference
        aircraft; scale_factor adjusts for actual aircraft physics.

        MR scaling:
          - Rate Kp/Kd: ∝ 1/inertia → lighter/shorter arms → lower gains
          - Angle Kp/Ki: ∝ mass → heavier → more authority
          - Rate limits: ∝ 1/mass → lighter → faster response possible
          - Alt Kp/Ki: ∝ mass → heavier → more authority
          - Nav Kp/Ki: ∝ 1/mass → lighter → faster nav response

        FW scaling:
          - Rate Kp/Kd: ∝ 1/Ixx → larger wingspan → more inertia → higher gains
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
                elif 'ANGLE_KP' in pname or 'ANGLE_KI' in pname or 'INT_LIMIT' in pname:
                    sf[idx] = mass_ratio
                elif 'MAX_ROLL_RATE' in pname or 'MAX_PITCH_RATE' in pname or 'MAX_COMPASS' in pname:
                    sf[idx] = 1.0 / mass_ratio if mass_ratio > 0 else 1.0
                elif 'ALT_POS' in pname:
                    sf[idx] = mass_ratio
                elif 'NAV_POS' in pname or 'NAV_VEL' in pname:
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
                elif 'ANGLE_KP' in pname or 'ANGLE_KI' in pname or 'INT_LIMIT' in pname:
                    sf[idx] = mass_ratio
                elif 'MAX_ROLL_RATE' in pname or 'MAX_PITCH_RATE' in pname or 'MAX_COMPASS' in pname:
                    sf[idx] = 1.0 / mass_ratio if mass_ratio > 0 else 1.0
                elif 'ALT_POS' in pname:
                    sf[idx] = mass_ratio

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

    def _category_for_af(self, af_type):
        """Return 'MR', 'FW', 'VTOL', or 'LAND' for a given AF type enum value"""
        try:
            from protocol_enums import AirframeType
            name = AirframeType(af_type).name
            if 'FW' in name or 'WING' in name or 'DELTA' in name or 'HELI' in name or 'AILERON' in name or 'SPOILERON' in name:
                return 'FW'
            if 'VTOL' in name:
                return 'VTOL'
            if 'TRACKED' in name or 'GLIDER' in name or 'CAR' in name:
                return 'LAND'
        except (ValueError, AttributeError):
            pass
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
                motor_w = float(result.get('PHYS_MOTOR_W', 100))
            except (ValueError, TypeError):
                motor_w = 100

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

            # Thrust estimation from actuator disk theory + watts per motor
            # η = electrical→thrust efficiency: ESC(93%) × motor(70%) × prop(50%) ≈ 0.30 at hover
            prop_m = prop_in * 0.0254
            disk_area = 3.14159 * (prop_m / 2.0) ** 2
            rho = 1.225
            eta = 0.30

            # Static thrust from power: T = (η × P × √(2ρA))^(2/3)
            p_shaft = max(motor_w, 10.0) * eta
            thrust_per_motor = (p_shaft * (2.0 * rho * disk_area) ** 0.5) ** (2.0 / 3.0)
            total_thrust_n = thrust_per_motor * n_motors
            twr = total_thrust_n / (mass_kg * 9.81) if mass_kg > 0 else 5.0

            # Hover throttle: electrical power needed / electrical power available
            thrust_hover = (mass_kg * 9.81) / n_motors if n_motors > 0 else mass_kg * 9.81
            v_induced = (thrust_hover / (2.0 * rho * disk_area)) ** 0.5 if disk_area > 0 else 5.0
            p_hover_shaft = thrust_hover * v_induced
            p_hover_elec = p_hover_shaft / eta
            hover_thr = p_hover_elec / max(motor_w, 10.0)

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
            result['PHYS_MOTOR_W'] = str(int(motor_w))

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
                motor_w = float(result.get('PHYS_MOTOR_W', 150))
            except (ValueError, TypeError):
                motor_w = 150

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

            # Max thrust from actuator disk theory (same η as MR)
            prop_m = prop_in * 0.0254
            disk_area = 3.14159 * (prop_m / 2.0) ** 2
            eta = 0.30
            p_shaft = max(motor_w, 10.0) * eta
            thrust_n = (p_shaft * (2.0 * rho * disk_area) ** 0.5) ** (2.0 / 3.0)

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
            result['PHYS_MOTOR_W'] = str(int(motor_w))

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

    def _create_pid_group(self):
        """Create PID group using enum definitions"""
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
        
        layout.addWidget(self._header("Roll"), 0, 0)
        layout.addWidget(self._header("Pitch"), 0, 1)
        layout.addWidget(self._header("Yaw"), 0, 2)
        layout.addWidget(self._header("Parameter"), 0, 3)
        
        for row_idx, (label, roll_idx, pitch_idx, yaw_idx, roll_def, pitch_def, yaw_def) in enumerate(self.PID_PARAMS, 1):
            self.add_spin(layout, roll_idx, roll_def, row_idx, 0)
            self.add_spin(layout, pitch_idx, pitch_def, row_idx, 1)
            self.add_spin(layout, yaw_idx, yaw_def, row_idx, 2)
            lbl = QLabel(label)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            layout.addWidget(lbl, row_idx, 3)
        
        # Max Angle row
        row_idx = len(self.PID_PARAMS) + 1
        self.add_spin(layout, ParamIndex.MAX_ROLL_ANGLE, 60, row_idx, 0)
        self.add_spin(layout, ParamIndex.MAX_PITCH_ANGLE, 60, row_idx, 1)
        dash = QLabel("--")
        dash.setStyleSheet("color: #999;")
        dash.setAlignment(Qt.AlignCenter)
        layout.addWidget(dash, row_idx, 2)
        lbl = QLabel("Max Angle")
        lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(lbl, row_idx, 3)
        
        group.setLayout(layout)
        return group

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
            ("Ext Mag", 0), ("Autoland", 1), ("No LEDs", 2),
            ("Emulation", 3), ("AH Alarm", 4), ("GPS Alt", 5), ("Disable VRS", 6),
            ("Clamp", 7),
        ]
        for i, (name, bit) in enumerate(config1_bits):
            cb = QCheckBox(name)
            cb.setProperty("bit", bit)
            cb.stateChanged.connect(lambda s, b=bit: self.bit_changed(ParamIndex.CONFIG1_BITS, b, s))
            layout.addWidget(cb, i // 2, (i % 2) + 1)
            self.config1_checks.append(cb)

        self.config2_checks = []
        config2_bits = [
            ("Batt Comp", 0), ("Fast Start", 1), ("BLHeli", 2),
            ("Glider", 3), ("Rev Props", 4), ("Turn WP", 5), ("Beep WP", 6),
        ]
        for i, (name, bit) in enumerate(config2_bits):
            cb = QCheckBox(name)
            cb.setProperty("bit", bit)
            cb.stateChanged.connect(lambda s, b=bit: self.bit_changed(ParamIndex.CONFIG2_BITS, b, s))
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
        limits = PARAM_LIMITS.get(idx_int, (0.0, 255.0))

        lo = limits[0] * mult
        hi = limits[1] * mult
        span = hi - lo
        step = max(0.001, span / 50.0)
        if mult == 100:
            step = 1.0
        if idx_int in (68, 113) and step < 1.0:
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
        # Defaults in the param lists are in legacy display units (old
        # uint8-style integers).  Convert through raw for legacy-tagged
        # params so the spinbox shows the correct scaled value.
        if idx_int in LEGACY_TAGS:
            scale = PARAM_SCALES.get(idx_int, 1.0)
            val = float(default) * scale * mult
        else:
            val = float(default)

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
        combo.currentIndexChanged.connect(lambda v, i=int(idx): self.combo_changed(i, v))
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
        
        # Sync the hidden widget so write_params/send_params use the corrected value
        pi = int(idx)
        if pi in self.params and isinstance(self.params[pi], QDoubleSpinBox):
            self.params[pi].setValue(new_value)
            self._log(f"  🔧 Config bit toggled: Param[{pi}] = {current} → {new_value} (synced)")
        else:
            self._log(f"  ⚠️ Config bit toggled: Param[{pi}] = {current} → {new_value} (widget NOT FOUND)")
        
        self.param_changed(pi, new_value)
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
                    vals.append(float(data) if data is not None else float(widget.currentIndex()))
                else:
                    vals.append(0.0)
            else:
                vals.append(0.0)
        return vals

    def _set_widgets_from_raw(self, raw_values: Dict[int, float]):
        """Set all UI widget values from raw float32 dictionary {idx: raw_float}."""
        for i, raw in raw_values.items():
            if i not in self.params:
                continue
            widget = self.params[i]
            try:
                if isinstance(widget, QDoubleSpinBox):
                    mult = self._display_mult(i)
                    display_val = raw * mult
                    widget.setValue(display_val)
                    if i in self._PROTECTED_PARAMS:
                        self._committed_values[i] = display_val
                elif isinstance(widget, QComboBox):
                    cb_idx = widget.findData(int(raw))
                    if cb_idx >= 0:
                        widget.setCurrentIndex(cb_idx)
                    else:
                        idx = max(0, min(widget.count() - 1, int(raw)))
                        widget.setCurrentIndex(idx)
                    if i in self._PROTECTED_PARAMS:
                        self._committed_values[i] = float(int(raw))
            except Exception:
                pass
        # Sync _config_values cache for config bit registers
        for cfg_idx in (ParamIndex.CONFIG1_BITS, ParamIndex.CONFIG2_BITS):
            ci = int(cfg_idx)
            if ci in raw_values:
                self._config_values[ci] = int(raw_values[ci])

    def on_airframe_selected(self, index: int):
        """User selected an airframe from the combo box → write its defaults to FC."""
        if index < 0:
            return
        name = self.airframe_combo.currentText()
        path = self.airframe_combo.itemData(index)

        # Prompt to save if dirty before switching
        if not self._prompt_save_if_dirty():
            # Revert combo to previous selection
            self.airframe_combo.blockSignals(True)
            for i in range(self.airframe_combo.count()):
                if self.airframe_combo.itemData(i) == self._current_airframe_path:
                    self.airframe_combo.setCurrentIndex(i)
                    break
            self.airframe_combo.blockSignals(False)
            return

        if path == "__DEFAULTS__":
            reply = QMessageBox.question(
                self,
                "FC Factory Defaults",
                "Load FC factory defaults into the UI?\n\n"
                "This will replace current parameter values.\n"
                "You will need to press Write to send them to the FC.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.No:
                self.airframe_combo.setCurrentIndex(-1)
                return
            raw_values = {i: PARAMETER_DEFS[i].default for i in range(self.MAX_PARAMS)}
            self._log(f"  📖 Generated {len(raw_values)} FC factory defaults")
        else:
            if not path or not os.path.exists(path):
                self._log(f"  ❌ Airframe file not found: {path}")
                return
            reply = QMessageBox.question(
                self,
                "Airframe Defaults",
                f"Load and write defaults for \"{name}\"?\n\n"
                "This will replace current parameter values in the UI.\n"
                "You will need to press Write to send them to the FC.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.No:
                self.airframe_combo.setCurrentIndex(-1)
                return
            try:
                read_name, raw_values, meta = af_module.parse_af_file(path)
                self._log(f"  📖 Loaded {len(raw_values)} params from \"{read_name}\"")
            except Exception as e:
                self._log(f"  ❌ Failed to parse {path}: {e}")
                QMessageBox.critical(self, "Parse Error", str(e))
                self.airframe_combo.setCurrentIndex(-1)
                return

        # Reset any stuck verification/write state
        self._verify_mode = False
        self._write_in_progress = False
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

        self._set_widgets_from_raw(raw_values)
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
        # Load physical descriptors from .af metadata
        phys_meta = {k: v for k, v in meta.items() if k.startswith('PHYS_')}
        if phys_meta:
            self._load_phys_from_metadata(phys_meta)
        self.status_label.setText(f"📝 Loaded \"{name}\" defaults — writing to FC")
        self.status_label.setStyleSheet("color: #f39c12;")

        # Persist the selection
        settings = QSettings("UAVX", "Groundstation")
        settings.setValue("airframe_path", path)
        QTimer.singleShot(100, lambda: self.write_params(silent=True))

    def _restore_airframe_selection(self):
        settings = QSettings("UAVX", "Groundstation")
        saved_path = settings.value("airframe_path", "")
        if not saved_path:
            # Default: select moderate lower-powered airframe (the one actively flight-tested)
            default_name = "Ecks_800g_Moderate"
            for i in range(self.airframe_combo.count()):
                path = self.airframe_combo.itemData(i)
                if path and default_name in path:
                    self.airframe_combo.blockSignals(True)
                    self.airframe_combo.setCurrentIndex(i)
                    self.airframe_combo.blockSignals(False)
                    settings.setValue("airframe_path", path)
                    self._current_airframe_path = path
                    self._log(f"  Auto-selected default airframe: {default_name}")
                    break
            return
        for i in range(self.airframe_combo.count()):
            if self.airframe_combo.itemData(i) == saved_path:
                self.airframe_combo.blockSignals(True)
                self.airframe_combo.setCurrentIndex(i)
                self.airframe_combo.blockSignals(False)
                break

    def _ensure_config2_fast_start(self):
        """Ensure Config2 has Fast Start bit set (bit 1) and bit 0 clear"""
        idx = int(ParamIndex.CONFIG2_BITS)
        config2 = self._config_values.get(idx, 0)
        corrected = (config2 & ~1) | 2  # Set bit 1, clear bit 0
        if config2 != corrected:
            self._log(f"  🔧 FIXING Config2 at {ParamIndex.CONFIG2_BITS.name}: {config2} -> {corrected}")
            self._config_values[idx] = corrected
            if idx in self.params and isinstance(self.params[idx], QDoubleSpinBox):
                self.params[idx].setValue(corrected)
                self._log(f"  🔧 Config2 widget synced to {corrected}")
            self.dirty_params.add(idx)
    

    
    def combo_changed(self, idx: int, value: int):
        # Confirmation dialog for safety-critical combo params
        if idx in self._PROTECTED_PARAMS:
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
                    cb_idx = widget.findData(int(old_val))
                    if cb_idx >= 0:
                        widget.setCurrentIndex(cb_idx)
                    widget.blockSignals(False)
                    return
                self._committed_values[idx] = float(value)
        self.dirty_params.add(idx)
        self.status_label.setText(f"P{idx+1} changed ({len(self.dirty_params)})")
        self.status_label.setStyleSheet("color: #f39c12;")
    
    def param_changed(self, idx: int, value: float):
        # Confirmation dialog for safety-critical params
        if idx in self._PROTECTED_PARAMS:
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
                self._committed_values[idx] = value

        self.dirty_params.add(idx)
        self.status_label.setText(f"P{idx+1} changed ({len(self.dirty_params)})")
        self.status_label.setStyleSheet("color: #f39c12;")
        self.update_config_display()

        # Orange highlight if value outside PARAM_LIMITS (in display units)
        if idx in self.params and isinstance(self.params[idx], QDoubleSpinBox):
            mult = self._display_mult(idx)
            limits = PARAM_LIMITS.get(idx, (0.0, 255.0))
            display_lo, display_hi = limits[0] * mult, limits[1] * mult
            widget = self.params[idx]
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
    def setup_connections(self):
        from core.ack_handler import ack_handler
        
        self.ReadParamsButton.clicked.connect(self.read_params)
        self.WriteParamsButton.clicked.connect(self.write_params)
        self.load_btn.clicked.connect(self.load_params)
        self.save_btn.clicked.connect(self.save_params)
        self.airframe_combo.currentIndexChanged.connect(self.on_airframe_selected)
        self._read_request_id = ack_handler.register_button(17, self.ReadParamsButton, self._on_read_success)
        self._log(f"  Read button request_id: {self._read_request_id}")
        
        self._write_request_id = ack_handler.register_button(17, self.WriteParamsButton, self._on_write_success)
        self._log(f"  Write button request_id: {self._write_request_id}")
    


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

    def _default_save_path(self):
        """Build default save path: ~/UAVX/<airframe_name>_Tuned.af"""
        airframe_name = "Params"
        if self._current_airframe_path:
            airframe_name = os.path.splitext(os.path.basename(self._current_airframe_path))[0]
        tuned_dir = os.path.expanduser("~/UAVX")
        os.makedirs(tuned_dir, exist_ok=True)
        return os.path.join(tuned_dir, f"{airframe_name}_Tuned.af")

    def _prompt_save_if_dirty(self):
        """Prompt to save if dirty. Returns True if safe to proceed, False to cancel."""
        if not self._is_dirty_from_baseline():
            return True
        reply = QMessageBox.question(
            self, "Unsaved Changes",
            "You have unsaved parameter changes.\n\n"
            "Save to ~/.af file before continuing?",
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
            QMessageBox.Save
        )
        if reply == QMessageBox.Save:
            path = self._default_save_path()
            try:
                raw_vals = self._raw_float_values()
                raw_dict = {i: raw_vals[i] for i in range(self.MAX_PARAMS)}
                metadata = {}
                if self._character_slider:
                    metadata['Character'] = str(self._character_slider.value())
                metadata.update(self._phys_to_metadata())
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
        elif reply == QMessageBox.Discard:
            return True
        else:
            return False
    
    def save_params(self):
        from datetime import datetime
        raw_vals = self._raw_float_values()
        raw_dict = {i: raw_vals[i] for i in range(self.MAX_PARAMS)}
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suggested = f"Params_{timestamp}.af"
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Parameters", suggested, "Airframe Files (*.af);;All Files (*)")
        if not path:
            return
        try:
            metadata = {}
            if self._character_slider:
                metadata['Character'] = str(self._character_slider.value())
            metadata.update(self._phys_to_metadata())
            text = af_module.format_af("Saved Parameters", raw_dict, metadata=metadata)
            with open(path, 'w') as f:
                f.write(text)
            self._log(f"✅ Saved {self.MAX_PARAMS} parameters to {path}")
            self.status_label.setText(f"✅ Saved to {os.path.basename(path)}")
            self.status_label.setStyleSheet("color: #27ae60;")
        except Exception as e:
            self._log(f"❌ Save failed: {e}")
            QMessageBox.critical(self, "Save Failed", str(e))
    
    def load_params(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Parameters", "", "Airframe Files (*.af);;All Files (*)")
        if not path:
            return
        try:
            name, raw_values, meta = af_module.parse_af_file(path)
            self._log(f"  📖 Loaded {len(raw_values)} params from \"{name}\"")
            self._set_widgets_from_raw(raw_values)
            self._ensure_config2_fast_start()
            self.update_config_display()
            self.update_rc_display()
            self.dirty_params.update(raw_values.keys())
            # Restore character slider position from .af metadata
            if self._character_slider and 'Character' in meta:
                try:
                    self._character_slider.setValue(int(meta['Character']))
                except (ValueError, TypeError):
                    pass
            self.sync_setup_from_advanced()
            self._log(f"✅ Loaded {len(raw_values)} parameters from {path}")
            self.status_label.setText(f"✅ Loaded from {os.path.basename(path)}")
            self.status_label.setStyleSheet("color: #27ae60;")
        except Exception as e:
            self._log(f"❌ Import failed: {e}")
            QMessageBox.critical(self, "Import Failed", str(e))
    
    def _on_read_success(self):
        self._log("✅ Read ACK received (fallback)")
        self.reset_read_button()
        self._read_request_id = None
        
        if self._read_timeout_timer:
            self._read_timeout_timer.stop()
            self._read_timeout_timer = None
    
    def _on_write_success(self):
        self._log("✅ Write ACK received - automatic param packet should arrive shortly")
        self._write_request_id = None
        
        if self._write_timeout_timer:
            self._write_timeout_timer.stop()
            self._write_timeout_timer = None
        
        if self._verification_timer:
            self._verification_timer.stop()
        
        self._verification_timer = QTimer()
        self._verification_timer.setSingleShot(True)
        self._verification_timer.timeout.connect(self._verification_timeout)
        self._verification_timer.start(3000)
        self._log("  ⏳ Waiting for automatic param packet from FC...")
    
    def get_rc_data(self):
        if self.parent_window and hasattr(self.parent_window, 'flight_data'):
            flight_data = self.parent_window.flight_data
            if flight_data and hasattr(flight_data, 'rc_channels'):
                if flight_data.rc_channels:
                    return flight_data.rc_channels
                else:
                    print(f"[RC] parent_window rc_channels is empty list")
        
        flight_data = data_manager.get_flight_data()
        if flight_data and hasattr(flight_data, 'rc_channels'):
            if flight_data.rc_channels:
                return flight_data.rc_channels
            else:
                print(f"[RC] data_manager rc_channels is empty list")
        
        return None
    
    def _get_param_value(self, idx):
        """Safely get parameter value from widget"""
        if idx not in self.params:
            return 0
        widget = self.params[idx]
        if isinstance(widget, (QDoubleSpinBox, QSpinBox)):
            return int(widget.value())
        elif isinstance(widget, QComboBox):
            data = widget.itemData(widget.currentIndex())
            return data if data is not None else widget.currentIndex()
        else:
            return 0
    
    def update_rc_display(self):
        """Update RC display using function-to-channel mapping"""
        rc_data = self.get_rc_data()
        
        # Build channel-to-value map from live data
        ch_values = {}
        if rc_data:
            for i, val in enumerate(rc_data[:16]):
                ch_values[i] = val
                self.rc_channels[i] = val
        else:
            for i in range(16):
                self.rc_channels[i] = 0
        
        # For each function, find its assigned channel and display the live value
        for fn_idx, (param_idx, func_name, default_ch) in enumerate(self.RC_MAP_PARAMS):
            assigned_ch = self._get_param_value(int(param_idx))
            val = ch_values.get(assigned_ch, 0)
            
            if val > 0:
                self.rc_value_labels[fn_idx].setText(f"{val}")
                self.rc_progress_bars[fn_idx].setRange(900, 2100)
                self.rc_progress_bars[fn_idx].setValue(val)
                
                if 1450 <= val <= 1550:
                    bg = "#27ae60"
                elif val < 1200 or val > 1800:
                    bg = "#f39c12"
                else:
                    bg = "#8bc34a"
                self.rc_value_labels[fn_idx].setStyleSheet(
                    f"background-color: {bg}; font-weight: bold; padding: 0px 3px;"
                )
            else:
                self.rc_value_labels[fn_idx].setText("---")
                self.rc_progress_bars[fn_idx].setRange(900, 2100)
                self.rc_progress_bars[fn_idx].setValue(1500)
                self.rc_value_labels[fn_idx].setStyleSheet(
                    "background-color: #999; font-weight: bold; padding: 0px 3px;"
                )
        
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
        """Load default parameters — zeros until Read from FC or Import"""
        self._log("Starting with zeros - click Read to get params from FC")
        default_values = [0] * self.MAX_PARAMS
        
        for i, val in enumerate(default_values):
            if i in self.params:
                clamped = max(0, min(255, val))
                if isinstance(self.params[i], QDoubleSpinBox):
                    pass  # Keep initial value from add_spin
                elif isinstance(self.params[i], QComboBox):
                    max_idx = self.params[i].count() - 1
                    self.params[i].setCurrentIndex(max(0, min(max_idx, clamped)))
        
        # Defaults: Config1 = Emulation, Config2 = Fast Start
        self._set_config_default(ParamIndex.CONFIG1_BITS, int(Config1Bits.EMULATION_ENABLE | Config1Bits.ENFORCE_DRIVE_SYMMETRY))
        self._set_config_default(ParamIndex.CONFIG2_BITS, int(Config2Bits.USE_FAST_START))
        
        self.update_config_display()
        self.update_rc_display()
        self.sync_setup_from_advanced()
        self._log("✅ Default parameters loaded")
    
    def read_params(self):
        self._log("📖 Read button clicked - requesting parameters from FC")
        
        if self.parent_window and hasattr(self.parent_window, 'can_read_parameters'):
            if not self.parent_window.can_read_parameters():
                self.status_label.setText("❌ Cannot read - check connection")
                self.status_label.setStyleSheet("color: #e74c3c;")
                return
        
        if self.parent_window and hasattr(self.parent_window, 'flight_data'):
            if self.parent_window.flight_data and self.parent_window.flight_data.flight_state == FlightState.FLYING:
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

        self.ReadParamsButton.setStyleSheet("""
            QPushButton {
                background-color: #f39c12;
                color: white;
                font-weight: bold;
            }
        """)
        self.ReadParamsButton.setText("⏳ Waiting...")
        self.ReadParamsButton.setEnabled(False)

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
    
    def write_params(self, silent=False):
        if self._write_in_progress:
            self._log("⚠️ Write already in progress")
            return

        if not self.dirty_params:
            self._log("📝 No changed parameters to write")
            self.status_label.setText("✅ No changed parameters")
            self.status_label.setStyleSheet("color: #27ae60;")
            QMessageBox.information(self, "No Changes", "No parameters have been changed.\n\nAdjust a parameter value first, then press Write.")
            return

        n_dirty = len(self.dirty_params)
        self._log(f"📝 Write button clicked - writing {n_dirty} changed parameters to FC")

        if not silent:
            if self.parent_window and hasattr(self.parent_window, 'can_write_parameters'):
                safe, reason = self.parent_window.can_write_parameters()
                if not safe:
                    QMessageBox.critical(
                        self,
                        "🚫 Safety Blocked!",
                        f"Cannot write parameters:\n\n{reason}\n\n"
                        "Writing parameters while the aircraft is flying is EXTREMELY DANGEROUS!\n"
                        "Please land the aircraft before making changes."
                    )
                    self.status_label.setText(f"🚫 Blocked: {reason}")
                    self.status_label.setStyleSheet("color: #e74c3c;")
                    return

            state_name = "Unknown"
            if self.parent_window and hasattr(self.parent_window, 'flight_data'):
                if self.parent_window.flight_data:
                    state_name = FLIGHT_STATE_NAMES.get(
                        self.parent_window.flight_data.flight_state,
                        "Unknown"
                    )

            reply = QMessageBox.question(
                self,
                "⚠️ Confirm Write",
                f"About to write {n_dirty} changed parameter(s) to the FC.\n\n"
                f"Aircraft State: {state_name}\n"
                f"Verification: Will read back and verify after write.\n\n"
                f"Are you sure you want to continue?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.No:
                self.status_label.setText("Write cancelled")
                self.status_label.setStyleSheet("color: #666;")
                return

        self._write_in_progress = True

        current_params = {}
        for i in self.dirty_params:
            if i in self.params:
                widget = self.params[i]
                if isinstance(widget, QDoubleSpinBox):
                    mult = self._display_mult(i)
                    current_params[i] = int(widget.value() / mult)
                elif isinstance(widget, QSpinBox):
                    current_params[i] = widget.value()
                elif isinstance(widget, QComboBox):
                    data = widget.currentData()
                    current_params[i] = data if data is not None else widget.currentIndex()
                else:
                    current_params[i] = 0

        self._written_params = current_params.copy()
        self._written_float = self._compute_written_float(indices=current_params.keys())
        self._verify_mode = True
        self._expecting_param_packet = True
        self._write_timestamp = time.time()

        # Log config bit values: what we read from widget vs what _config_values cache holds
        for ck, cn in ((ParamIndex.CONFIG1_BITS, "Config1"), (ParamIndex.CONFIG2_BITS, "Config2")):
            ci = int(ck)
            widget_val = current_params.get(ci, -1)
            cache_val = self._config_values.get(ci, -1)
            match = "✅ MATCH" if widget_val == cache_val else "❌ MISMATCH"
            self._log(f"  📋 {cn}[{ci}]: widget_spinbox={widget_val}, _config_values={cache_val} {match}")
        
        self._log(f"  📦 Stored {len(self._written_params)} params for verification")
        first_10 = {k: v for k, v in list(self._written_params.items())[:10]}
        self._log(f"  📝 First 10 params: {first_10}")
        self._log(f"  🔑 _verify_mode={self._verify_mode}, _expecting_param_packet={self._expecting_param_packet}")
        self._log(f"  ⏱️ Write timestamp: {self._write_timestamp}")

        self.status_label.setText(f"⏳ Writing {n_dirty} changed parameter(s) to FC...")
        self.status_label.setStyleSheet("color: #f39c12;")

        self.WriteParamsButton.setStyleSheet("""
            QPushButton {
                background-color: #f39c12;
                color: white;
                font-weight: bold;
            }
        """)
        self.WriteParamsButton.setText("⏳ Writing...")
        self.WriteParamsButton.setEnabled(False)

        self._write_timeout_timer = QTimer()
        self._write_timeout_timer.setSingleShot(True)
        self._write_timeout_timer.timeout.connect(self._write_timeout)
        self._write_timeout_timer.start(5000)

        # Send only dirty params; _on_params_typed_sent will also add Config1/Config2 if needed
        dirty_indices = sorted(self.dirty_params)
        if self.parent_window and hasattr(self.parent_window, 'send_params_typed'):
            self.parent_window.send_params_typed(on_complete=self._on_params_typed_sent, indices=dirty_indices)
        else:
            self.status_label.setText("❌ Cannot write - no connection")
            self.status_label.setStyleSheet("color: #e74c3c;")
            self._write_in_progress = False
            self._verify_mode = False
            self._written_params = {}
            self._written_float = {}
            self._expecting_param_packet = False
            self._write_timestamp = 0
            self.reset_write_button()
            if self._write_timeout_timer:
                self._write_timeout_timer.stop()
                self._write_timeout_timer = None

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
                    floats[i] = float(data) if data is not None else float(widget.currentIndex())
                else:
                    floats[i] = 0.0
            else:
                floats[i] = 0.0
        return floats

    def _on_params_typed_sent(self):
        """Callback when all param packets have been sent"""
        self._log(f"  ✅ All param packets sent ({len(self._written_params)} values)")
        self.dirty_params.clear()
        if self._write_timeout_timer:
            self._write_timeout_timer.stop()
            self._write_timeout_timer = None
        # Commit to flash
        if self.parent_window and hasattr(self.parent_window, 'send_param_commit'):
            self.parent_window.send_param_commit()
        # Don't verify immediately — FC resets after commit.
        # Defer to when the next flight packet arrives (FC reconnected).
        self._log("  ⏳ Waiting for FC to reconnect after commit...")
        if self.parent_window:
            self.parent_window._pending_param_verification = True

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
                self._write_timestamp = 0
                
                self.status_label.setText("⚠️ Verification timeout - no param packet from FC")
                self.status_label.setStyleSheet("color: #e74c3c;")
                
                self.WriteParamsButton.setStyleSheet("""
                    QPushButton {
                        background-color: #e74c3c;
                        color: white;
                        font-weight: bold;
                        border: 2px solid #c0392b;
                        border-radius: 4px;
                        padding: 4px 8px;
                    }
                    QPushButton:hover {
                        background-color: #c0392b;
                    }
                """)
                self.WriteParamsButton.setText("⏱️ Timeout")
                self.WriteParamsButton.setEnabled(True)
                QTimer.singleShot(3000, self.reset_write_button)
            else:
                self._log("  ⏳ Still waiting for param packet...")
                # Restart timer
                self._verification_timer = QTimer()
                self._verification_timer.setSingleShot(True)
                self._verification_timer.timeout.connect(self._verification_timeout)
                self._verification_timer.start(1000)
    

            self.status_label.setText("Saved")
    
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
        
        if not self._prompt_save_if_dirty():
            event.ignore()
            return
        event.accept()
    
    def set_buttons_enabled(self, enabled: bool):
        pass
