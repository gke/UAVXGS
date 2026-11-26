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

Calibration and flight-safety work 2026-09-27 by OpenCode, Big Pickle & Friends.
"""

import os
import math
import re
import struct
from typing import Any, Dict, List, Optional, Tuple

from protocol_enums import (
    ParamIndex, AirframeType, ESCType, RxType, ArmingMode,
    TelemetryType, RangefinderType, AirspeedSensorType,
    IMUFilterType, Config1Bits, Config2Bits, FailsafeAction,
    AIRFRAME_NAMES, ESC_TYPE_NAMES, RX_TYPE_NAMES, ARMING_MODE_NAMES,
    RF_TYPE_NAMES, AS_SENSOR_TYPE_NAMES,
    MOTOR_STOP_NAMES, FAILSAFE_ACTION_NAMES,
)
from parameters import PARAM_DISPLAY_MULT

# Airframe library layout (read-only, ships inside the app):
#   generic/      curated, supported defaults
#   contributed/  user submissions promoted by the maintainer at a release (best-effort)
#   proposed/     developer staging -- present but NOT flight-ready
#   original/     frozen pre-fork snapshots (never tuned)
# The user's own airframes are NOT stored here; they live in the user's
# workspace on their machine. See wiki/docs/Airframe_Library_Layout.md.
AIRFRAMES_DIR = os.path.join(os.path.dirname(__file__))

LINE_RE = re.compile(r'^\s*(\w+)\s*=\s*(.+?)\s*$')

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
    'UNUSED_14': None,
    'SERVO_SENSE': None,
    'UNUSED_37': None,
    'UNUSED_58': None,
    'UNUSED_93': None,
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

def _resolve_enum_member(enum_cls, token: str):
    """Resolve an .af enum token to an enum member.

    Order: exact match -> case-insensitive match -> None. An unresolvable token
    is an error: .af files carry canonical member names.
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


def _resolve_owned_or_any(enum_cls: type, token: str):
    """Resolve a token against the param's own enum first, then any other.

    Bare ambiguity-prone tokens (e.g. `DEFAULT`) exist on more than one
    IntFlag, so the param's owning enum must take precedence, else
    `CONFIG2_BITS = DEFAULT` resolves as Config1Bits.DEFAULT (0x16) instead
    of Config2Bits.DEFAULT (0x49).
    """
    if enum_cls is not None:
        member = _resolve_enum_member(enum_cls, token)
        if member is not None:
            return member
    for cls in ENUM_MAP.values():
        if cls is None or cls is enum_cls:
            continue
        member = _resolve_enum_member(cls, token)
        if member is not None:
            return member
    return None


def _parse_value(param_name: str, text: str) -> float:
    """Parse a human-readable value string back to raw float."""
    owned = _enum_for_param(param_name)
    text = text.strip()
    # Hex
    if text.startswith('0x') or text.startswith('0X'):
        return float(int(text, 16))
    # Pipe-separated IntFlag combination
    if '|' in text:
        total = 0
        found_all = True
        for part in text.split('|'):
            part = part.strip()
            member = _resolve_owned_or_any(owned, part)
            if member is None:
                # If any part unrecognized, fall through to plain float
                found_all = False
                break
            total += member.value
        if found_all:
            return float(total)
    # Try enum name lookup
    member = _resolve_owned_or_any(owned, text)
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


def require_finite(tag: int, value: float, context: str) -> float:
    """Refuse a non-finite parameter value at the GCS boundary.

    Python's float() silently accepts 'nan'/'inf'/'-inf', so a corrupted or
    hand-edited .af loads clean today and re-ships on every save. The GCS is
    a source of badness as well as a display sink; this is the guard for
    that role. Raises ValueError with the offending parameter so the caller
    can report and refuse the load/write. Never masks or substitutes.
    """
    try:
        ok = math.isfinite(value)
    except TypeError:
        ok = False
    if not ok:
        raise ValueError(
            f"{context}: {ParamIndex(tag).name} (tag {tag}) = {value!r} "
            f"is not finite — refusing to use it")
    return value


def parse_af(text: str) -> Tuple[str, Dict[int, float], Dict[str, Any]]:
    """Parse .af file text → (name, {tag: raw_float_value}, metadata_dict).

    metadata_dict contains:
      - 'Name', 'Character' from '# Key: Value' comment lines
      - 'PHYS_*' keys from PHYS_* = value lines (physical descriptors)
      - 'LIMITS': {tag: (lo, hi)} from an optional [LIMITS] block (raw units)
      - 'UNKNOWN_KEYS': [param_name, ...] keys absent from the current ParamIndex
    """
    name = 'Unknown'
    values: Dict[int, float] = {}
    metadata: Dict[str, Any] = {}
    limits: Dict[int, Tuple[float, float]] = {}
    unknown: List[str] = []
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
        try:
            tag = ParamIndex[param_name].value
        except KeyError:
            unknown.append(param_name)
            continue
        if tag is None:
            continue
        if in_limits:
            parts = value_text.split(',')
            if len(parts) != 2:
                continue
            try:
                lo = float(parts[0].strip())
                hi = float(parts[1].strip())
            except ValueError:
                continue
            limits[tag] = (require_finite(tag, lo, f".af '{name}' [LIMITS]"),
                           require_finite(tag, hi, f".af '{name}' [LIMITS]"))
            continue
        val = _parse_value(param_name, value_text)
        values[tag] = require_finite(tag, val, f".af '{name}'")
    if limits:
        metadata['LIMITS'] = limits
    if unknown:
        metadata['UNKNOWN_KEYS'] = unknown
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
