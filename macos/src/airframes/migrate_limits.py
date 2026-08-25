"""One-time .af migration: fix unit conventions to FC-native units and add
[LIMITS] blocks (wiki/docs/ParamClass_Bounds_Design.md §5.2).

Unit fixes applied to shipped frames:
  - tag 17  LOW_VOLT_THRES: per-cell volts (<9) -> pack volts = per-cell * cells
  - tag 30  HORIZON: clamp to FC ceiling (50)
  - tag 53  BATTERY_CAPACITY: clamp to [1500, 10000]
  - tag 74/76 MAX_PITCH/ROLL_ANGLE: clamp to exact 60 deg radian value
  - tag 113 FW_ROLL_CONTROL_PITCH_LIMIT: degrees (>1.3) -> radians
  - tag 119 BATTERY_ALARM_PCT: percent (>1.0) -> fraction

[LIMITS] blocks are generated for every FLOAT param present in the file,
bracketed ~x2 around the (corrected) value and clamped to the class ceiling.

Usage:
  python3 airframes/migrate_limits.py [--dry-run]
"""
import glob
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from protocol_enums import AirframeType, ParamIndex
from airframes import airframes
from parameters import PARAM_LIMITS, PARAM_TYPES

AIRFRAMES_DIR = os.path.dirname(__file__)
RAD_60 = 1.0471975511965976
DEG2RAD = math.pi / 180.0

# Reference aircraft used by the GCS tuner (_get_scale_factors in
# parameter_window.py) — phys ratios are computed against these.
REF_MR_AUW_G = 800
REF_MR_ARM_MM = 220
REF_FW_AUW_G = 1000
REF_FW_WINGSPAN_MM = 1800


def category_for_af(af_type):
    """Return 'MR'/'FW' for an AF type value — mirrors _category_for_af()."""
    FW_TYPES = (AirframeType.eElevonAF, AirframeType.eDeltaAF, AirframeType.eAileronAF,
                AirframeType.eAileronSpoilerFlapsAF, AirframeType.eAileronVTailAF,
                AirframeType.eRudderElevatorAF)
    try:
        af = AirframeType(af_type)
        if af in FW_TYPES:
            return 'FW'
        if af in (AirframeType.eVTOLAF, AirframeType.eVTOL2AF):
            return 'FW'
    except (TypeError, ValueError):
        pass
    return 'MR'


def _to_float(s, default=0.0):
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


def phys_scale_factors(values, meta):
    """Compute per-tag tuning scale factor from the aircraft's physical
    descriptors, mirroring the GCS _get_scale_factors() ratios:

      rate gains (RATE_KP/RATE_KD) : 1/inertia_ratio
      angle gains (ANGLE_KP/KI/INT_LIMIT): mass_ratio
      max rates (MAX_ROLL/PITCH/HEADING_RATE): 1/mass_ratio

    The reference craft maps to scale 1.0 (so its sub-range is unchanged).
    Non-gain params and unknown/absent phys default to 1.0.
    """
    auw_g = _to_float(meta.get('PHYS_AUW_G'), 0)
    if auw_g <= 0:
        return {}

    mass_ratio = auw_g / 1000.0

    # Category from the AF_TYPE value stored in the file (tag 43)
    cat = 'MR'
    af_type = values.get(ParamIndex.AF_TYPE.value)
    if af_type is not None:
        cat = category_for_af(int(af_type))

    if cat == 'FW':
        ref_mass = REF_FW_AUW_G / 1000.0
        span = _to_float(meta.get('PHYS_WINGSPAN_MM'), REF_FW_WINGSPAN_MM)
        if span <= 0:
            span = REF_FW_WINGSPAN_MM
        ref_span = REF_FW_WINGSPAN_MM / 1000.0
        mass_ratio /= ref_mass
        inertia_ratio = (span / 1000.0 / ref_span) ** 2 * mass_ratio
    else:  # MR
        ref_mass = REF_MR_AUW_G / 1000.0
        arm = _to_float(meta.get('PHYS_ARM_MM'), REF_MR_ARM_MM)
        if arm <= 0:
            arm = REF_MR_ARM_MM
        ref_arm = REF_MR_ARM_MM / 1000.0
        mass_ratio /= ref_mass
        inertia_ratio = (arm / 1000.0 / ref_arm) ** 2 * mass_ratio

    inv_inertia = 1.0 / inertia_ratio if inertia_ratio > 0 else 1.0
    inv_mass = 1.0 / mass_ratio if mass_ratio > 0 else 1.0

    sf = {}
    for tag in values:
        try:
            pname = ParamIndex(tag).name
        except (ValueError, AttributeError):
            continue
        if 'RATE_KP' in pname or 'RATE_KD' in pname:
            sf[tag] = inv_inertia
        elif 'ANGLE_Q_KP' in pname or 'ANGLE_Q_KI' in pname or 'ANGLE_Q_INT_LIMIT' in pname:
            sf[tag] = mass_ratio
        elif 'MAX_ROLL_RATE' in pname or 'MAX_PITCH_RATE' in pname or 'MAX_HEADING' in pname:
            sf[tag] = inv_mass
    return sf


def cell_count(batt_v):
    """Approximate LiPo series cell count from pack voltage."""
    return max(1, round(batt_v / 3.7))


def siground(x, sig=4):
    if x == 0:
        return 0.0
    return round(x, sig - int(math.floor(math.log10(abs(x)))) - 1)


def fix_value(tag, raw, cells):
    """Return the FC-native raw value for a known-legacy .af quirk."""
    if tag == 17:                      # LOW_VOLT_THRES
        if raw < 9.0:                  # per-cell convention
            raw = raw * cells
        return min(max(raw, 9.0), 20.0)
    if tag == 30:                      # HORIZON
        return min(raw, 50.0)
    if tag == 53:                      # BATTERY_CAPACITY
        return min(max(raw, 1500.0), 10000.0)
    if tag in (74, 76):                # MAX_PITCH/ROLL_ANGLE (60 deg exact)
        return min(raw, RAD_60)
    if tag == 113:                     # FW_ROLL_CONTROL_PITCH_LIMIT (deg -> rad)
        if raw > 1.3:
            raw = raw * DEG2RAD
        return raw
    if tag == 119:                     # BATTERY_ALARM_PCT (% -> fraction)
        if raw > 1.0:
            raw = raw / 100.0
        return raw
    return raw


def gen_limits(values, phys_sf=None):
    """Build {tag: (lo, hi)} raw tuning ranges for FLOAT params.

    phys_sf: {tag: scale} from phys_scale_factors(). The sub-range is scaled
    in center AND width: v*sf*[0.5, 2.0] — a heavier craft gets higher angle
    gain ranges, a lighter craft lower rate gain ranges. The reference craft
    (sf=1.0) is unchanged.
    """
    phys_sf = phys_sf or {}
    limits = {}
    for tag, v in values.items():
        if PARAM_TYPES.get(tag, 'U8') != 'FLOAT':
            continue
        lo, hi = PARAM_LIMITS.get(tag, (0.0, 255.0))
        sf = phys_sf.get(tag, 1.0)
        vc = v * sf
        if vc == 0.0:
            limits[tag] = (lo, hi)
            continue
        l = max(lo, siground(vc * 0.5))
        h = min(hi, siground(vc * 2.0))
        if l < h:
            limits[tag] = (l, h)
    return limits


def main():
    dry = '--dry-run' in sys.argv
    paths = sorted(glob.glob(os.path.join(AIRFRAMES_DIR, 'generic', '*.af'))
                   + glob.glob(os.path.join(AIRFRAMES_DIR, 'user', '*.af')))
    total_changed = 0
    for path in paths:
        name, values, meta = airframes.parse_af_file(path)
        batt_v = 0.0
        try:
            batt_v = float(meta.get('PHYS_BATT_V', '0'))
        except (TypeError, ValueError):
            pass
        cells = cell_count(batt_v)

        changed = []
        for tag in sorted(values):
            raw = values[tag]
            fixed = fix_value(tag, raw, cells)
            if abs(fixed - raw) > 1e-9:
                changed.append((tag, raw, fixed))
                values[tag] = fixed

        limits = gen_limits(values, phys_scale_factors(values, meta))
        meta['LIMITS'] = limits
        new_text = airframes.format_af(name, values, metadata=meta)

        if changed:
            total_changed += len(changed)
            for tag, old, new in changed:
                print(f"  {os.path.basename(path)}: {airframes.ParamIndex(tag).name} "
                      f"{old} -> {new}")
        if not dry:
            with open(path, 'w') as f:
                f.write(new_text)
        print(f"{'DRY-RUN ' if dry else ''}{os.path.basename(path)}: "
              f"{len(limits)} [LIMITS] entries, {len(changed)} value fixes")

    print(f"\nTotal value fixes: {total_changed}")
    if dry:
        print("(dry run — no files written)")


if __name__ == '__main__':
    main()
