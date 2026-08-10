"""
Convert old uint8-based defaults.h airframes to new unified-float .af files.

The old system (originalparams.c) stored uint8 values in Config.P[] and
applied tag-specific scaling in InitPIDStructs(). This script applies
the SAME scaling to produce the raw float32 values that the new system
stores in Config.ParamData[i].f.

Usage:
    python3 convert_old_defaults.py /path/to/originaldefaults.h [output_dir]
"""

import re, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from protocol_enums import ParamIndex
from airframes import airframes

DEG_RAD = 0.017453292519943295

# Scaling factors from originalparams.c InitPIDStructs()
# Format: { tag: (scale_factor, note) }
# U8-type params have scale=1 (value stored as-is, cast to float32)
OLD_SCALES = {
    # ---- Rate PID gains ----
    0:  (0.005,       "RollRateKp: P() * 0.005"),
    5:  (0.005,       "PitchRateKp: P() * 0.005"),
    10: (0.005,       "YawRateKp: P() * 0.005"),
    11: (0.0001,      "RollRateKd: P() * 0.0001"),
    27: (0.0001,      "PitchRateKd: P() * 0.0001"),
    90: (0.000025,    "YawRateKd: P() * 0.000025"),

    # ---- Angle PID gains (P.Kp = uint8 * 0.25 = Q) ----
    2:  (0.25,        "RollAngleQKp → Roll Q: P() * 0.25"),
    7:  (0.25,        "PitchAngleQKp → Pitch Q: P() * 0.25"),

    # ---- Angle PID Ki ----
    23: (0.05,        "RollAngleQKi: P() * 0.05"),
    24: (0.05,        "PitchAngleQKi: P() * 0.05"),
    99: (0.05,        "YawAngleQKi: P() * 0.05"),

    # ---- Angle PID IntLim (uses DEG_RAD * 0.015) ----
    4:  (DEG_RAD * 0.015, "RollAngleQIntLimit: P() * DEG_RAD * 0.015"),
    9:  (DEG_RAD * 0.015, "PitchAngleQIntLimit: P() * DEG_RAD * 0.015"),
    100: (DEG_RAD * 0.05, "YawAngleQIntLimit: P() * DEG_RAD * 0.05"),

    # ---- Altitude PID ----
    1:  (0.00046,     "AltPosKi: P() * 0.00046"),
    6:  (0.0183,      "AltPosKp: P() * 0.0183"),
    29: (0.0026,      "AltVelKp/AltPosKd: P() * 0.0026"),
    66: (0.05,        "AltVelIntLimit: P() * 0.05"),
    101: (0.00027,    "AltVelKi: P() * 0.00027 (UNUSED — ROC loop removed)"),

    # ---- Nav PID ----
    56: (0.0165,      "NavPosKp: P() * 0.0165"),
    60: (0.004,       "NavPosKi: P() * 0.004"),
    28: (0.06,        "NavVelKp: P() * 0.06"),
    48: (0.01,        "NavCrossTrackKp: P() * 0.01"),

    # ---- Camera ----
    18: (0.1,         "RollCamKp: P() * 0.1"),
    25: (0.1,         "PitchCamKp: P() * 0.1"),

    # ---- Sensor fusion ----
    38: (0.01,        "MadgwickKpAcc: P() * 0.01 (= TwoKpAccBase/2)"),
    31: (0.01,        "MadgwickKpMag: P() * 0.01"),

    # ---- KF variances (stored as scaled float, now tracked live) ----
    72: (1e-6,        "KFAccUBiasVar: P() * 0.000001"),
    111: (0.01,       "KFBaroVar: P() * 0.01"),
    112: (0.1,        "KFAccUVar: P() * 0.1"),

    # ---- Misc FLOAT params with known scales ----
    17: (0.1,         "LowVoltThres: P() * 0.1 (= volts ×10 → volts)"),
    33: (DEG_RAD * 0.1, "NavMagVar: P() * DEG_RAD * 0.1"),
    45: (1.0,         "MaxDescentRateDmpS: direct uint8 (= m/s)"),
    52: (0.01,        "AccConfSD: P() * 0.01 (= 1/sdev)"),
    40: (1.0,         "NavPosIntLimit → MaxVelocity: direct uint8 (= m/s)"),
    63: (10.0,        "MaxYawRate: P() * 10 (°/s) → DEG_RAD in InitPIDStructs"),
    88: (10.0,        "MaxCompassYawRate → MaxHeadingRate: was P() * 10 (°/s) → DEG_RAD in InitPIDStructs"),
    67: (1.0,         "FWMaxClimbAngle: direct uint8 (= deg) → DEG_RAD"),
    68: (1.0,         "NavMaxAngle: direct uint8 (= deg) → DEG_RAD"),
    74: (1.0,         "MaxPitchAngle: direct uint8 (= deg) → DEG_RAD"),
    76: (1.0,         "MaxRollAngle: direct uint8 (= deg) → DEG_RAD"),
    78: (1.0,         "NavHeadingTurnout: direct uint8 (= deg) → DEG_RAD"),

    # ---- Percent-based params (old uint8 = percent, new stores fraction) ----
    19: (0.01,        "EstCruiseThr/P(): P() as percent → fraction"),
    20: (0.01,        "StickHysteresis: P() as percent → fraction"),
    21: (0.01,        "FWClimbThrottle: P() as percent → fraction"),
    22: (0.01,        "PercentIdleThr: P() as percent → fraction"),
    49: (1.0,         "BatteryCapacity: direct uint8 (×100 mAh?)"),
    57: (1.0,         "AltLPF: direct uint8 (UNUSED — KF provides denoising)"),
    58: (0.01,        "Balance: P() as percent → fraction (CG offset)"),
    62: (0.01,        "TiltThrottleFF: P() as percent → fraction"),
    65: (0.01,        "FWPitchThrottleFF: P() as percent → fraction"),
    69: (0.001,       "FWSpoilerDecayPercentPS: P() * 0.1 → fraction/s"),
    70: (0.01,        "FWAileronDifferential: P() as percent → fraction"),
    79: (0.001,       "AltHoldThrCompDecayPercentPS: P() * 0.1 → fraction/s"),
    86: (0.01,        "FWAileronRudderMix: P() as percent → fraction"),
    87: (0.01,        "FWAltSpoilerFF: P() as percent → fraction"),
    102: (0.01,       "AltThrottleCompLimit: P() as percent → fraction"),
    114: (0.01,       "FWStickScale: P() as percent → fraction"),
    113: (1.0,        "FWRollControlPitchLimit: direct deg → use DEG_RAD"),
    115: (10.0,       "NavFenceRadiusM: P() * 10 (= metres, from old pos 108)"),
    26: (1.0,         "ServoLPFHz: direct uint8 (= Hz)"),
    30: (0.01,        "Horizon: P() → 1/FromPercent"),
    32: (1.0,         "NavRTHAlt: direct uint8 (= metres)"),
    39: (0.01,        "RollCamTrim: P() * 0.01 (= degrees)"),
    64: (0.01,        "FWRollPitchFF: P() as percent → fraction (negated)"),
    77: (1.0,         "YawLPFHz: direct uint8 (= Hz)"),
    80: (0.01,         "YawSymmetryFactor: yaw symmetry factor (0-1)"),
    81: (1.0,         "FWBoardPitchAngle: direct deg → DEG_RAD"),
    82: (10.0,        "MaxRollRate: P() * 10 °/s → DEG_RAD"),
    83: (10.0,        "MaxPitchRate: P() * 10 °/s → DEG_RAD"),
    84: (1.0,         "CurrentScale: direct uint8"),
    85: (1.0,         "VoltScale: direct uint8"),
    89: (1.0,         "AccLPFSel: U8 enum"),
    96: (0.25,        "Yaw Q (was YawAngleQKp): approx P() * 0.25"),
    97: (0.05,        "YawAngleQKi: P() * 0.05"),
    98: (DEG_RAD * 0.05, "YawAngleQIntLimit: P() * DEG_RAD * 0.05"),
    104: (1.0,        "Unused105: U8 raw"),
    106: (1.0,        "NavProxAltM: direct uint8"),
    107: (1.0,        "NavProxRadiusM: direct uint8 (= metres)"),
    108: (1.0,        "NavFenceRadiusM: P() * 10 (= metres)"),

    # ---- U8 params (stored as-is, cast to float32) ----
    3:  (1.0, "ArmingMode — U8 enum"),
    8:  (1.0, "RFSensorType — U8 enum"),
    12: (1.0, "IMUFiltType — U8 enum"),
    13: (1.0, "BBLogType — U8 raw"),
    14: (1.0, "RxType — U8 enum"),
    15: (1.0, "Config1Bits — U8 bitmask"),
    16: (1.0, "RxThrottleCh — U8 enum"),
    34: (1.0, "UnusedSensorHint — U8"),
    35: (1.0, "ESCType — U8 enum"),
    36: (1.0, "RCChannels — U8"),
    37: (1.0, "RxRollCh — U8 enum"),
    41: (1.0, "RxPitchCh — U8 enum"),
    42: (1.0, "RxYawCh — U8 enum"),
    43: (1.0, "AFType — U8 enum"),
    44: (1.0, "TelemetryType — U8 enum"),
    46: (1.0, "DescentDelayS — U8 seconds"),
    47: (1.0, "GyroLPFSel — U8 enum"),
    50: (1.0, "RxGearCh — U8 enum"),
    51: (1.0, "RxAux1Ch — U8 enum"),
    53: (1.0, "ServoSense — U8 bitmask"),
    54: (1.0, "RxAux2Ch — U8 enum"),
    55: (1.0, "RxAux3Ch — U8 enum"),
    59: (1.0, "RxAux4Ch — U8 enum"),
    61: (1.0, "UnusedGPSProtocol — U8 raw"),
    71: (1.0, "ASSensorType — U8 enum"),
    73: (1.0, "Config2Bits — U8 bitmask"),
    91: (1.0, "AccLPFSel — U8 enum"),
    92: (1.0, "ThrottleGainRate — U8 percent (UNUSED)"),
    93: (1.0, "RxAux5Ch — U8 enum"),
    94: (1.0, "RxAux6Ch — U8 enum (PassThru)"),
    95: (1.0, "RxAux7Ch — U8 enum (Dive)"),
    103: (1.0, "MotorStopSel — U8 enum"),
    105: (1.0, "Unused105 — U8"),
    109: (1.0, "Unused109 — U8"),
    110: (1.0, "Unused110 — U8"),
    118: (1.0, "Unused118 — U8"),

    # ---- New params with no old equivalent — use new default ----
    # These are not in the old defaults.h arrays; skip or use new default.
}

def extract_airframes(path):
    with open(path) as f:
        text = f.read()
    text = re.sub(r'//.*', '', text)
    pattern = re.compile(r'\{\s*"([^"]+)"\s*,\s*\{\s*([0-9,\s]+)\s*\}\s*\}')
    results = []
    for m in pattern.finditer(text):
        name = m.group(1)
        nums = [int(x.strip()) for x in m.group(2).split(',') if x.strip()]
        results.append((name, nums))
    return results


def convert(name, u8_vals):
    lines = [f'# {name} — converted from originaldefaults.h', '#']
    issues = []
    for i in range(min(len(u8_vals), 128)):
        u8 = u8_vals[i]
        scale, note = OLD_SCALES.get(i, (None, None))

        try:
            pname = ParamIndex(i).name
        except ValueError:
            pname = f'UNUSED_{i}'

        if scale is None:
            # Param not in our table — likely a new param with no old equivalent
            if u8 == 0:
                continue
            lines.append(f'# {pname} = {airframes._format_value(pname, float(u8))}  (no old scale defined)')
            issues.append(f"Tag {i} ({pname}): old uint8={u8}, no scale defined — stored as-is")
            continue

        raw = u8 * scale

        # Convert degrees/°_per_s to radians/rad_per_s for params where
        # the old FC stored direct degree values but the unified float
        # system stores radian values.
        if '→ DEG_RAD' in note or '→ use DEG_RAD' in note:
            raw *= DEG_RAD

        # Round for readability
        if abs(raw) < 1e-10:
            raw = 0.0
        elif abs(raw) >= 1:
            raw = round(raw, 6)
        elif abs(raw) >= 0.001:
            raw = round(raw, 6)
        else:
            raw = float(f'{raw:.6g}')

        lines.append(f'{pname} = {airframes._format_value(pname, raw)}')

    return '\n'.join(lines), issues


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    src = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(__file__), 'original_defaults')

    os.makedirs(out_dir, exist_ok=True)

    airframes = extract_airframes(src)
    print(f"Found {len(airframes)} airframes in {src}")

    all_issues = {}
    for name, u8_vals in airframes:
        text, issues = convert(name, u8_vals)
        safe = re.sub(r'[^a-zA-Z0-9]+', '_', name).strip('_')
        out_path = os.path.join(out_dir, f'{safe}.af')
        with open(out_path, 'w') as f:
            f.write(text)
        print(f"  Wrote {out_path}")
        if issues:
            all_issues[name] = issues

    if all_issues:
        print("\n=== Issues by airframe ===")
        for name, issues in all_issues.items():
            print(f"\n{name}:")
            for iss in issues:
                print(f"  {iss}")


if __name__ == '__main__':
    main()
