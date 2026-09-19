# orientation_solver.py
"""
UAVX Sensor Orientation Probe Classifier

Classifies bench probe observations into the porting decision-tree outcomes
used by wiki/imu-orientation-porting-report.md section 5, and derives the
next Quadrant/Flip knob delta from the DECLARED currently-flashed settings.

Design note (2026-08-24): a one-shot absolute inverse is deliberately NOT
attempted. ScaleAccAndRate() contributes its own fixed axis-role permutation
(single invariant mapping; the VTOL variant was removed), so "healthy" is a
relative notion; the tool therefore verifies and converges iteratively, one
knob change per reflash, exactly as a human porter does.

Pure logic, no Qt. Run directly for the self-test:

    python3 orientation_solver.py

Observation encoding (produced by the GCS probe runner):
    yaw   : (channel_name, sign)     channel in rate_pitch/rate_roll/rate_yaw
    tilt  : ('pitch'|'roll', sign)   displayed axis and direction

Tree outcomes:
    'pass'      : all three probes canonical
    'z_family'  : yaw reversed          -> flip family is wrong
    'q_offset'  : tilt crossed/inverted -> quadrant offset 1..3, see delta()
"""

PASS = 'pass'
Z_FAMILY = 'z_family'
Q_OFFSET = 'q_offset'

_CANONICAL_YAW = ('rate_yaw', 1)
_CANONICAL_PITCH = ('pitch', 1)
_CANONICAL_ROLL = ('roll', 1)


def _wrap4(x):
    return x % 4


def classify(yaw_obs, pitch_obs, roll_obs):
    """
    Returns dict:
        status        : PASS | Z_FAMILY | Q_OFFSET | 'mixed'
        yaw_reversed  : bool
        pitch_delta   : 0..3 quadrant steps to add (0 when fine)
        roll_delta    : 0..3 (consistency cross-check)
    """
    yaw_ok = yaw_obs == _CANONICAL_YAW
    pitch_cls = _tilt_class(pitch_obs, _CANONICAL_PITCH)
    roll_cls = _tilt_class(roll_obs, _CANONICAL_ROLL)

    out = {
        'status': PASS,
        'yaw_reversed': not yaw_ok,
        'pitch_delta': pitch_cls,
        'roll_delta': roll_cls,
    }

    if not yaw_ok:
        out['status'] = Z_FAMILY
    elif pitch_cls or roll_cls:
        out['status'] = Q_OFFSET if pitch_cls == roll_cls else 'mixed'
    return out


def _tilt_class(obs, canonical):
    """Quadrant offset reproducing the observed tilt, 0 if canonical.

    Horizontal group algebra (valid relative to any fixed downstream chain):
      +1 step CW    moves a stimulus one channel over with sign preserved
      +2            inverts in place
    Observed classes:
      ('pitch',+1)=0  ('roll',+1)=1  ('pitch',-1)=2  ('roll',-1)=3
    """
    table = {('pitch', 1): 0, ('roll', 1): 1,
             ('pitch', -1): 2, ('roll', -1): 3}
    return (table[obs] - table[canonical]) % 4


def next_knobs(result, quadrant_cur, flip_cur):
    """
    Suggested (quadrant, flip, note) after this probe round.
    Z_FAMILY dominates: fix the vertical first, re-probe, then trim quadrant.
    """
    if result['status'] == PASS:
        return quadrant_cur, flip_cur, 'orientation verified - done'
    if result['status'] == Z_FAMILY:
        return quadrant_cur, not flip_cur, ('yaw reversed - toggled '
                                            'SensorFlip; re-probe')
    d = result['pitch_delta']
    if result['status'] == 'mixed':
        return quadrant_cur, flip_cur, \
            'inconsistent tilt outcomes - repeat the probes'
    q_new = _wrap4(quadrant_cur + d)
    note = ('horizontal offset %d step(s) - SensorQuadrant %d -> %d'
            % (d, quadrant_cur, q_new))
    return q_new, flip_cur, note


# ---------------------------------------------------------------------------
# self-test

def _selftest():
    failures = []

    def check(name, cond):
        print('%-58s %s' % (name, 'ok' if cond else 'FAIL'))
        if not cond:
            failures.append(name)

    # fully healthy
    r = classify(_CANONICAL_YAW, _CANONICAL_PITCH, _CANONICAL_ROLL)
    check('all-canonical -> pass', r['status'] == PASS)

    # yaw reversed, tilts fine: z_family dominates, flip toggles once
    r = classify(('rate_yaw', -1), _CANONICAL_PITCH, _CANONICAL_ROLL)
    check('reversed yaw -> z_family', r['status'] == Z_FAMILY)
    q, f, note = next_knobs(r, 2, False)
    check('z_family suggests flip toggle keeping quadrant',
          f is True and q == 2)

    # 180 off: both tilts inverted, yaw fine
    r = classify(_CANONICAL_YAW, ('pitch', -1), ('roll', -1))
    check('double inversion -> q_offset 2', r['status'] == Q_OFFSET
          and r['pitch_delta'] == 2 and r['roll_delta'] == 2)
    q, f, note = next_knobs(r, 0, False)
    check('offset 2 applied to quadrant', q == 2 and f is False)

    # one-step cross (reachable state for quadrant off by 1):
    # pitch probe reads roll-right, roll probe reads pitch-down
    r = classify(_CANONICAL_YAW, ('roll', 1), ('pitch', -1))
    check('one-step cross classified consistently',
          r['status'] == Q_OFFSET and r['pitch_delta']
          == r['roll_delta'] == 1)
    q0 = 3
    q, f, note = next_knobs(r, q0, False)
    check('+1 wrap 3->0', q == 0 and f is False)

    # mixed tilts are reported, never guessed
    r = classify(_CANONICAL_YAW, ('roll', 1), ('roll', -1))
    check('inconsistent tilts flagged mixed', r['status'] == 'mixed')

    # delta arithmetic wraps all four quadrants
    ok = True
    for qc in range(4):
        for d in range(4):
            r2 = {'status': Q_OFFSET, 'pitch_delta': d, 'roll_delta': d}
            q, _, _ = next_knobs(r2, qc, False)
            ok &= q == ((qc + d) % 4)
    check('quadrant delta arithmetic wraps modulo 4', ok)

    print('\n%s' % ('ALL TESTS PASSED' if not failures else
                    '%d FAILURES: %s' % (len(failures), failures)))
    return 0 if not failures else 1


if __name__ == '__main__':
    raise SystemExit(_selftest())
