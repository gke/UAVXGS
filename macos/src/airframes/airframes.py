"""
Airframe parameter file (.af) format — human-readable raw float values.

Format:
  # comment
  ParamName = value    # value is raw float32 or enum name / decimal for bitmasks
  ParamName = 31       # decimal for bitmask/uint8

Lines starting with '#' are comments.
Blank lines are ignored.
ParamName matches ParamIndex enum member name (case-insensitive).
Enum values are looked up by name from the matching protocol enum.
"""

import os
import re
import struct
from typing import Any, Dict, List, Optional, Tuple

from protocol_enums import (
    ParamIndex, AirframeType, ESCType, RxType, ArmingMode,
    TelemetryType, RangefinderType, AirspeedSensorType,
    IMUFilterType, Config1Bits, Config2Bits, FailsafeAction,
    AIRFRAME_NAMES, ESC_TYPE_NAMES, RX_TYPE_NAMES, ARMING_MODE_NAMES,
    RF_TYPE_NAMES, AS_SENSOR_TYPE_NAMES,
    MOTOR_STOP_NAMES, BB_LOG_NAMES, FAILSAFE_ACTION_NAMES,
)
from parameters import PARAM_DISPLAY_MULT

AIRFRAMES_DIR = os.path.join(os.path.dirname(__file__))

LINE_RE = re.compile(r'^\s*(\w+)\s*=\s*(.+?)\s*$')

# Transitional bridge for .af files saved under the pre-rename 128-param
# schema: params moved out of the param block into the FC Config struct were
# renamed to UNUSED_NN. Their values are still meaningful in old files (e.g.
# the sim cruise baseline), so map the old names forward instead of dropping
# them. Pass-through only — a re-save writes them under the UNUSED_NN names.
_LEGACY_PARAM_NAME_TO_TAG = {
    'EST_CRUISE_THR': 19,       # → Unused20 (FC Config.CruiseThrottleFF)
    'KF_ACC_U_BIAS_VAR': 72,    # → Unused73
    'KF_BARO_VAR': 110,         # → Unused111
    'KF_ACC_U_VAR': 111,        # → Unused112
    'FW_ROLL_CONTROL_PITCH_LIMIT': 113,  # → Unused114 (Euler-era FW roll guard)
}



ENUM_MAP: Dict[str, type] = {
    'AF_TYPE': AirframeType,
    'ESC_TYPE': ESCType,
    'RX_TYPE': RxType,
    'ARMING_MODE': ArmingMode,
    'TELEMETRY_TYPE': TelemetryType,
    'RF_SENSOR_TYPE': RangefinderType,
    'AS_SENSOR_TYPE': AirspeedSensorType,
    'IMU_FILT_TYPE': IMUFilterType,
    'CONFIG1_BITS': Config1Bits,
    'CONFIG2_BITS': Config2Bits,
    'FAILSAFE_ACTION': FailsafeAction,
    'BB_LOG_TYPE': None,
    'SERVO_SENSE': None,
    'THROTTLE_GAIN_RATE': None,
    'MOTOR_STOP_SEL': None,
    'VOLT_SCALE': None,
    'CURRENT_SCALE': None,
    'UNUSED_109': None,
    'UNUSED_110': None,
    'POWER_RESET_CAUSE': None,
}

# Maps param index → (enum_class, display_names_dict) for combo box resolution
_COMBO_ENUM_MAP: Dict[int, tuple] = {
    ParamIndex.AF_TYPE: (AirframeType, AIRFRAME_NAMES),
    ParamIndex.ESC_TYPE: (ESCType, ESC_TYPE_NAMES),
    ParamIndex.RX_TYPE: (RxType, RX_TYPE_NAMES),
    ParamIndex.ARMING_MODE: (ArmingMode, ARMING_MODE_NAMES),
    ParamIndex.RF_SENSOR_TYPE: (RangefinderType, RF_TYPE_NAMES),
    ParamIndex.AS_SENSOR_TYPE: (AirspeedSensorType, AS_SENSOR_TYPE_NAMES),
    ParamIndex.MOTOR_STOP_SEL: (None, MOTOR_STOP_NAMES),
    ParamIndex.BB_LOG_TYPE: (None, BB_LOG_NAMES),
    ParamIndex.FAILSAFE_ACTION: (FailsafeAction, FAILSAFE_ACTION_NAMES),
}


def _enum_for_param(param_name: str):
    """Find the enum class for a given parameter name."""
    # Exact match first
    if param_name.upper() in ENUM_MAP:
        return ENUM_MAP[param_name.upper()]
    # Check by prefix/suffix
    up = param_name.upper()
    if up.endswith('_LPF_SEL') or up.startswith('GYRO_') or up.startswith('ACC_'):
        return None
    return None

_LEGACY_ENUM_TOKENS = {
    cls_name: {old: new for old, new in m.items()}
    for cls_name, m in {
        'AirframeType': {
            'TRI': 'eTriAF', 'TRI_COAX': 'eTriCoaxAF', 'VTAIL': 'eVTailAF',
            'QUAD': 'eQuadAF', 'QUAD_X': 'eQuadXAF', 'QUAD_COAX': 'eQuadCoaxAF',
            'QUAD_COAX_X': 'eQuadCoaxXAF', 'HEX': 'eHexAF', 'HEX_X': 'eHexXAF',
            'OCT': 'eOctAF', 'OCT_X': 'eOctXAF', 'HELI_90': 'eHeli90AF',
            'BI': 'eBiAF', 'ELEVON': 'eElevonAF', 'DELTA': 'eDeltaAF',
            'AILERON': 'eAileronAF', 'AILERON_SPOILER_FLAPS': 'eAileronSpoilerFlapsAF',
            'AILERON_VTAIL': 'eAileronVTailAF', 'RUDDER_ELEVATOR': 'eRudderElevatorAF',
            'DIFFERENTIAL_TWIN': 'eDifferentialTwinAF', 'VTOL': 'eVTOLAF',
            'VTOL2': 'eVTOL2AF', 'TRACKED': 'eTrackedAF', 'FOUR_WHEEL': 'eFourWheelAF',
            'TWO_WHEEL': 'eTwoWheelAF', 'INSTRUMENTATION': 'eInstrumentation',
            'UNKNOWN': 'eAFUnknown',
        },
        'ESCType': {'FAST_PWM': 'eESCPWM', 'DC_MOTORS': 'eDCMotors',
                    'DRIVES_DISARMED': 'eMotorsOff'},
        'RxType': {'CPPM': 'eCPPMRx', 'FUTABA_SBUS': 'eFutabaSBusRx',
                   'SPEKTRUM_1024': 'eSpektrum1024Rx', 'SPEKTRUM_2048': 'eSpektrum2048Rx',
                   'CRSF': 'eCRSFRx', 'UNKNOWN': 'eUnknownRx'},
        'ArmingMode': {'TX_ARMING': 'eTxArming', 'SWITCH_ARMING': 'eSwitchArming',
                       'UNUSED_ARMING_2': 'eUnusedArming2',
                       'UNUSED_ARMING_3': 'eUnusedArming3'},
        'TelemetryType': {'UAVX_DJT': 'eUAVXDJTTelemetry', 'INAV_LUA': 'eINavLUATelemetry',
                          'NO_TELEMETRY': 'eNoTelemetry', 'U4': 'eU4Telemetry',
                          'U5': 'eU5Telemetry', 'U6': 'eU6Telemetry', 'U7': 'eU7Telemetry',
                          'U8': 'eU8Telemetry'},
        'RangefinderType': {'MAX_SONAR_CM': 'eMaxSonarcm', 'SRF_I2C_CM': 'eSRFI2Ccm',
                            'MAX_SONAR_I2C_CM': 'eMaxSonarI2Ccm',
                            'SHARP_IR_GP2Y0A02YK': 'eSharpIRGP2Y0A02YK',
                            'SHARP_IR_GP2Y0A710K': 'eSharpIRGP2Y0A710K', 'NO_RF': 'eNoRF'},
        'AirspeedSensorType': {'MS4525D0_I2C': 'eMS4525D0I2C',
                               'MPXV7002DP_ANALOG': 'eMPXV7002DPAnalog',
                               'AS_THERMOPILE_ANALOG': 'eASThermopileAnalog',
                               'AS_GPS_DERIVED': 'eASGPSDerived', 'NO_AS': 'eNoAS'},
        'IMUFilterType': {'LPFILT': 'eLP2Filt', 'MPUFILT': 'eHDLPFilt',
                          'PT1FILT': 'eF1', 'IMU_FILT_3': 'eF2', 'IMU_FILT_4': 'eF3',
                          'IMU_FILT_5': 'eF4'},
        'Config1Bits': {'USE_INVERT_MAG': 'eUseInvertMag', 'USE_RTH_DESCEND': 'eUseRTHDescend',
                        'DISABLE_LEDS_IN_FLIGHT': 'eUsingMag',
                        'USE_MAG': 'eUsingMag',
                        'EMULATION_ENABLE': 'eEmulationEnable',
                        'USE_ALT_HOLD_ALARM': 'eUseAltHoldAlarm',
                        'USE_OFFSET_HOME': 'eUseOffsetHome',
                        'TEST_MISSION': 'eUnused1_6',
                        'ENFORCE_DRIVE_SYMMETRY': 'eEnforceDriveSymmetry'},
        'Config2Bits': {'USE_BATTERY_COMP': 'eUseBatteryComp', 'USE_FAST_START': 'eUseFastStart',
                        'USE_ESC_PROG': 'eUnused2_2', 'USE_BLHELI': 'eUnused2_2', 'USE_HAVE_GPS': 'eUseGPS',
                        'USE_PROP_SENSE': 'ePropsInwards', 'USE_TURN_TO_WP': 'eUseTurnToWP',
                        'USE_NAV_BEEP': 'eUseNavBeep'},
        'FailsafeAction': {'FS_RTH': 'eFsRth', 'FS_Land': 'eFsLand',
                           'FS_MOTORS_OFF': 'eFsMotorsOff'},
    }.items()
}


def _resolve_enum_member(enum_cls, token: str):
    """Resolve an .af enum token to an enum member.

    Order: exact match -> case-insensitive match -> legacy UPPER token -> None.
    Handles both new e-* tokens and pre-sweep UPPER-SNAKE tokens.
    """
    if enum_cls is None:
        return None
    t = token.strip()
    try:
        return enum_cls[t]
    except (KeyError, ValueError):
        pass
    for name, member in enum_cls.__members__.items():
        if name.lower() == t.lower():
            return member
    legacy = _LEGACY_ENUM_TOKENS.get(enum_cls.__name__)
    if legacy and t in legacy:
        try:
            return enum_cls[legacy[t]]
        except (KeyError, ValueError):
            pass
    return None


def _find_enum(param_name: str, raw_value: float) -> str:
    """Convert raw float value → enum name for a known enum param."""
    for enum_cls in ENUM_MAP.values():
        if enum_cls is None:
            continue
        try:
            return enum_cls(int(raw_value)).name
        except (ValueError, TypeError):
            continue
    return None


def _parse_value(param_name: str, text: str) -> float:
    """Parse a human-readable value string back to raw float."""
    text = text.strip()
    # Hex
    if text.startswith('0x') or text.startswith('0X'):
        return float(int(text, 16))
    # Pipe-separated IntFlag combination
    if '|' in text:
        total = 0
        for part in text.split('|'):
            part = part.strip()
            found = False
            for enum_cls in ENUM_MAP.values():
                member = _resolve_enum_member(enum_cls, part)
                if member is not None:
                    total += member.value
                    found = True
                    break
            if not found:
                # If any part unrecognized, fall through to plain float
                break
        else:
            return float(total)
    # Try enum name lookup
    for enum_cls in ENUM_MAP.values():
        member = _resolve_enum_member(enum_cls, text)
        if member is not None:
            return float(member.value)
    # Plain float
    return float(text)


def _format_value(param_name: str, raw_value: float) -> str:
    """Format a raw float value as human-readable text."""
    up = param_name.upper()
    enum_cls = _enum_for_param(up)
    if enum_cls is not None:
        try:
            name = enum_cls(int(raw_value)).name
            if name is not None:
                return name
        except (ValueError, TypeError):
            pass
    if raw_value == int(raw_value) and 0 <= raw_value <= 255:
        i = int(raw_value)
        if i > 9:
            return str(i)
    if abs(raw_value) < 0.001:
        return f'{raw_value:.8g}'
    if raw_value == int(raw_value) and abs(raw_value) < 1e9:
        return str(int(raw_value))
    return f'{raw_value:.8g}'


def parse_af(text: str) -> Tuple[str, Dict[int, float], Dict[str, Any]]:
    """Parse .af file text → (name, {tag: raw_float_value}, metadata_dict).

    metadata_dict contains:
      - 'Name', 'Character' from '# Key: Value' comment lines
      - 'PHYS_*' keys from PHYS_* = value lines (physical descriptors)
      - 'LIMITS': {tag: (lo, hi)} from an optional [LIMITS] block (raw units)
    """
    name = 'Unknown'
    values: Dict[int, float] = {}
    metadata: Dict[str, Any] = {}
    limits: Dict[int, Tuple[float, float]] = {}
    in_limits = False
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            if line.startswith('# ') and name == 'Unknown':
                rest = line[2:].strip()
                if ':' in rest and not rest.startswith(' '):
                    key, val = rest.split(':', 1)
                    key = key.strip()
                    val = val.strip()
                    if key in ('Name', 'Character'):
                        metadata[key] = val
                else:
                    name = rest
            continue
        if line.upper() == '[LIMITS]':
            in_limits = True
            continue
        m = LINE_RE.match(line)
        if not m:
            continue
        param_name = m.group(1).strip().upper()
        value_text = m.group(2).strip()
        if param_name.startswith('PHYS_'):
            metadata[param_name] = value_text
            continue
        tag = None
        try:
            tag = ParamIndex[param_name].value
        except KeyError:
            tag = _LEGACY_PARAM_NAME_TO_TAG.get(param_name)
        if tag is None:
            continue
        if in_limits:
            parts = value_text.split(',')
            if len(parts) != 2:
                continue
            try:
                limits[tag] = (float(parts[0].strip()), float(parts[1].strip()))
            except ValueError:
                continue
            continue
        val = _parse_value(param_name, value_text)
        values[tag] = val
    if limits:
        metadata['LIMITS'] = limits
    return name, values, metadata


def parse_af_file(path: str) -> Tuple[str, Dict[int, float], Dict[str, Any]]:
    """Read an .af file from disk and parse it → (name, {tag: raw_float}, metadata)."""
    with open(path) as f:
        return parse_af(f.read())


def format_af(name: str, values: Dict[int, float], metadata: Dict[str, Any] = None) -> str:
    """Format param values → .af file text.

    Optional metadata stored as:
      - '# Key: Value' comment lines for Name, Character
      - 'PHYS_KEY = Value' lines for physical descriptors and derived quantities
      - 'LIMITS': {tag: (lo, hi)} emitted as a [LIMITS] block (raw units)
    """
    lines = [f'# {name}', '']
    phys_lines = []
    limits = None
    if metadata:
        limits = metadata.get('LIMITS')
        for k, v in metadata.items():
            if k == 'LIMITS':
                continue
            if k.startswith('PHYS_'):
                phys_lines.append(f'{k} = {v}')
            else:
                lines.append(f'# {k}: {v}')
        if phys_lines:
            lines.append('')
            lines.extend(phys_lines)
            lines.append('')
    for i in range(128):
        if i not in values:
            continue
        try:
            param_name = ParamIndex(i).name
        except ValueError:
            param_name = f'UNUSED_{i}'
        raw = values[i]
        formatted = _format_value(param_name, raw)
        lines.append(f'{param_name} = {formatted}')
    if limits:
        lines.append('')
        lines.append('[LIMITS]')
        for tag in sorted(limits):
            lo, hi = limits[tag]
            try:
                param_name = ParamIndex(tag).name
            except ValueError:
                param_name = f'UNUSED_{tag}'
            lines.append(f'{param_name} = {lo:.8g}, {hi:.8g}')
    lines.append('')
    return '\n'.join(lines)


def _raw_float_for_display(tag: int, display_val: float,
                           mult: Optional[float] = None) -> float:
    """Convert GCS display value to raw float for .af storage / FC write.

    `mult` is the effective display multiplier (mode-aware: the UI passes
    the legacy `1/scale` in Legacy mode, else `PARAM_DISPLAY_MULT`). A
    legacy-scaled spinbox value MUST be rescaled by its mode multiplier
    before any write — these functions never assume raw == display.
    """
    if mult is None:
        mult = PARAM_DISPLAY_MULT.get(tag, 1.0)
    return display_val / mult


def _display_for_raw_float(tag: int, raw_val: float,
                           mult: Optional[float] = None) -> float:
    """Convert raw float to GCS display value."""
    if mult is None:
        mult = PARAM_DISPLAY_MULT.get(tag, 1.0)
    return raw_val * mult


def export_af_from_widgets(name: str, params: Dict[int, object],
                           limits: Dict[int, Tuple[float, float]] = None,
                           mult_fn=None) -> str:
    """Export current widget values as .af text.

    Optional `limits` {tag: (lo, hi)} raw is emitted as a [LIMITS] block.
    `mult_fn(tag) -> float` returns the effective display multiplier
    (mode-aware). If omitted, plain PARAM_DISPLAY_MULT is used — callers in
    Legacy mode MUST pass the legacy `1/scale` mapping or the exported
    values will be legacy-scaled raw, not raw.
    """
    from parameter_window import QDoubleSpinBox, QComboBox
    raw_values: Dict[int, float] = {}
    for tag, widget in params.items():
        if isinstance(widget, QDoubleSpinBox):
            mult = mult_fn(tag) if mult_fn else PARAM_DISPLAY_MULT.get(tag, 1.0)
            raw = _raw_float_for_display(tag, widget.value(), mult)
            raw_values[tag] = raw
        elif isinstance(widget, QComboBox):
            # Reverse-resolve display name → enum value for alphabetically-sorted combos
            mapping = _COMBO_ENUM_MAP.get(tag)
            if mapping is not None:
                enum_cls, names = mapping
                display = widget.currentText()
                # Invert names dict: display_name → enum_value
                inv = {v: k for k, v in names.items()}
                key = inv.get(display)
                if key is not None:
                    raw_values[tag] = float(key.value) if hasattr(key, 'value') else float(key)
                else:
                    raw_values[tag] = float(widget.currentIndex())
            else:
                raw_values[tag] = float(widget.currentIndex())
    return format_af(name, raw_values, metadata={'LIMITS': limits} if limits else None)


def import_to_widgets(name: str, af_text: str, params: Dict[int, object],
                      mult_fn=None) -> str:
    """Parse .af text and set widget values. Returns airframe name."""
    af_name, raw_values, _meta = parse_af(af_text)
    from parameter_window import QDoubleSpinBox, QComboBox
    for tag, raw_val in raw_values.items():
        if tag not in params:
            continue
        widget = params[tag]
        if isinstance(widget, QDoubleSpinBox):
            mult = mult_fn(tag) if mult_fn else PARAM_DISPLAY_MULT.get(tag, 1.0)
            widget.setValue(_display_for_raw_float(tag, raw_val, mult))
        elif isinstance(widget, QComboBox):
            # Resolve raw enum value → display name for alphabetically-sorted combos
            display = None
            mapping = _COMBO_ENUM_MAP.get(tag)
            if mapping is not None:
                enum_cls, names = mapping
                try:
                    key = enum_cls(int(raw_val)) if enum_cls else int(raw_val)
                    display = names.get(key)
                except (ValueError, KeyError, TypeError):
                    pass
            if display is not None:
                idx = widget.findText(display)
            else:
                idx = -1
            if idx < 0:
                idx = max(0, min(widget.count() - 1, int(raw_val)))
            widget.setCurrentIndex(idx)
    return af_name if name == 'Unknown' else name


def list_airframes() -> List[Tuple[str, str]]:
    """List all .af files in the airframes directory → [(name, path)]."""
    results = []
    if not os.path.isdir(AIRFRAMES_DIR):
        return results
    for fn in sorted(os.listdir(AIRFRAMES_DIR)):
        if not fn.endswith('.af'):
            continue
        path = os.path.join(AIRFRAMES_DIR, fn)
        name, _, _meta = parse_af_file(path)
        results.append((name, path))
    return results
