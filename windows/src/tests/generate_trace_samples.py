#!/usr/bin/env python3
"""
Generate synthetic TRAC snapshot dumps — one per trace type — to exercise the
GCS trace viewer without an aircraft on a bench. Writes the exact v2 header
(128 B: 32 B base + gain block + 16 B base-row records, offsets per
UAVXArmQ/src/trace.h), so the viewer (or a future unit test) can consume them
byte-identical to a live tag-54 dump.

Records are written Roll/Pitch/Yaw per sample tick (3 records per tick, the
capture layout). v2 carries the base rate-loop row for every trace type, so the
synthetic signals below simply populate that row with type-suggestive content;
specific extended rows arrive when each type's layout ships (recordSize > 16,
version bump).

Run:  python3 tests/generate_trace_samples.py
Output: tests/trace_samples/<trace_*.bin>

Adapted 2026-08-30 by GKE (matches ui/trace_viewer.py parse). v2 gain-block +
rate-gain pot header updated 2026-09-14 (matches FC TraceSnapshotGains).
"""

import math
import os
import random
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from critic.metrics import step_metrics_window

HDR = struct.Struct('<IHBBHHHHIIII')   # 32 B base, see trace.h TraceWriteHeader
GAIN = struct.Struct('<HH21f')          # 88 B: count u16 + version u16 + 21 f32
REC = struct.Struct('<Ifff')            # 16 B: tick, Rate, Desired, Out
CRITIC = struct.Struct('<IBBBBffff')    # 24 B, see trace.h TRACE_CRITIC_*
MAGIC = 0x43415254                      # "TRAC"
VERSION = 2
RECORD_SIZE = 16
BASE_HDR_SIZE = 32
HEADER_SIZE = 128                       # v2: base + gain block
CRITIC_MAGIC = 0x43495243               # "CRIC"
CRITIC_VERSION = 1
AXIS_MASK = 0x07                        # Roll | Pitch | Yaw
SNAP_START = HEADER_SIZE                # ring data base
GAIN_COUNT = 21
GAIN_VERSION = 0
RATE_GAIN_Q = 1000.0                    # fixed-point multiplier for the pot
RATE_GAIN_CODE = 1000                   # x1.0 (neutral pot) — 250..4000 live

ETRACE_NONE = 0
ETRACE_RATE = 1
ETRACE_ATTITUDE = 2
ETRACE_ALTHOLD = 3
ETRACE_ACTUATOR = 4
ETRACE_IMU = 5

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'trace_samples')


def gain_block():
    """A plausible fleet gain set in the FC's TraceSnapshotGains layout: per
    axis R.Kp R.Kd R.Max P.Kp P.Ki P.IntLim P.Max, indexed by the base
    TRACE_GAIN_VERSION layout. Match ui/trace_viewer._GAIN_FIELDS order."""
    values = []
    # exercise per axis roll/pitch/yaw rows (generic MR-class numbers)
    for axis in range(3):
        values.extend([
            0.34,                       # R.Kp
            0.008,                      # R.Kd
            3.6652,                     # R.Max (210 deg/s)
            4.5607658,                  # P.Kp (QKp)
            0.11857991,                 # P.Ki
            0.0078539816,               # P.IntLim
            0.78539816,                 # P.Max (45 deg angle limit)
        ])
    return GAIN.pack(GAIN_COUNT, GAIN_VERSION, *values)


def build(trace_type, period_ms, start_ms, per_sample):
    """per_sample(t_ms) -> (roll_r, roll_d, roll_o,
                            pitch_r, pitch_d, pitch_o,
                            yaw_r, yaw_d, yaw_o)."""
    t = start_ms
    count = 0
    out = bytearray()
    out += HDR.pack(MAGIC, VERSION, trace_type, RECORD_SIZE, 0, AXIS_MASK,
                    period_ms, RATE_GAIN_CODE, SNAP_START, 0, start_ms, 0)
    out += gain_block()
    out += bytes(HEADER_SIZE - BASE_HDR_SIZE - GAIN.size)   # pad to 128
    end_ms = start_ms
    while True:
        row = per_sample(t)
        if row is None:
            break
        for a in range(3):
            r, d, o = row[a]
            out += REC.pack(t, r, d, o)
        count += 3
        end_ms = t
        t += period_ms
    HDR.pack_into(out, 0, MAGIC, VERSION, trace_type, RECORD_SIZE, 0,
                  AXIS_MASK, period_ms, RATE_GAIN_CODE, SNAP_START, count,
                  start_ms, end_ms)
    return bytes(out)


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def append_critic_trailer(blob, axis, stim, rise, overshoot, settle):
    """Append a 24 B critic-result trailer (the FC writes one at disarm for the
    measured axis). Keeps the synthetic dump byte-identical to a live dump that
    carries the on-board measurement."""
    trailer = CRITIC.pack(CRITIC_MAGIC, CRITIC_VERSION, axis, 0, 0,
                          stim, rise, overshoot, settle)
    return blob + trailer


def noise(rng, sig):
    return rng.gauss(0.0, sig)


# ---------------------------------------------------------------------------
def rate_probe(period_ms, start_ms, duration_ms, rng):
    """eTraceRate: the V1 probe signature — settle baseline, then the FC steps
    Roll Desired to a constant (ds) at +100 ms (pre-roll); the gyro Rate follows
    as a lightly damped 2nd-order response. Faithful to the real FC probe, where
    TraceStimulusSetpoint writes a CONSTANT TraceStimValue into Desired and the
    measured Rate is the plant response."""
    ds = 2.0            # rad/s step, 25% of a ~8 rad/s roll rate limit
    wn = 25.0
    zeta = 0.62
    wd = wn * math.sqrt(1.0 - zeta * zeta)
    t_stim = start_ms + 100
    t_end = start_ms + duration_ms

    def resp(t):
        # gyro-rate response to the constant step
        te = (t - t_stim) * 0.001
        env = math.exp(-zeta * wn * te)
        return ds * (1.0 - env * (math.cos(wd * te)
                                  + zeta / math.sqrt(1.0 - zeta * zeta)
                                  * math.sin(wd * te)))

    prev_r = 0.0

    def per_sample(t):
        nonlocal prev_r
        if t > t_end:
            return None
        d = ds if t >= t_stim else 0.0   # constant step (Desired)
        r = resp(t)                       # plant response (Rate)
        r += (d - r) * 0.012 + noise(rng, 0.006)   # tiny lag + gyro noise
        r = clip(r, -ds * 1.1, ds * 1.1)
        o = clip((d - r) * 2.2 - (r - prev_r) * 0.08 + noise(rng, 0.01),
                 -1.0, 1.0)
        prev_r = r
        roll = (r, d, o)
        pitch = (noise(rng, 0.002), 0.0, clip(noise(rng, 0.004), -1.0, 1.0))
        yaw = (noise(rng, 0.002), 0.0, clip(noise(rng, 0.004), -1.0, 1.0))
        return roll, pitch, yaw

    return build(ETRACE_RATE, period_ms, start_ms, per_sample)


def attitude(period_ms, start_ms, duration_ms, rng):
    """eTraceAttitude: slow coupled Roll/Pitch sweep (the base row shows the
    rate-loop fields this capture is piggy-backing until the extended row)."""
    t_end = start_ms + duration_ms

    def per_sample(t):
        if t > t_end:
            return None
        ts = (t - start_ms) * 0.001
        d_roll = 0.55 * math.sin(ts * 0.8)
        r_roll = d_roll * 0.85 + noise(rng, 0.004)
        o_roll = clip((d_roll - r_roll) * 1.5, -1.0, 1.0)
        d_pitch = 0.22 * math.sin(ts * 0.45 + 0.6)
        r_pitch = d_pitch * 0.8 + noise(rng, 0.004)
        o_pitch = clip((d_pitch - r_pitch) * 1.5, -1.0, 1.0)
        roll = (r_roll, d_roll, o_roll)
        pitch = (r_pitch, d_pitch, o_pitch)
        yaw = (noise(rng, 0.002), 0.0, clip(noise(rng, 0.004), -1.0, 1.0))
        return roll, pitch, yaw

    return build(ETRACE_ATTITUDE, period_ms, start_ms, per_sample)


def althold(period_ms, start_ms, duration_ms, rng):
    """eTraceAltHold: hover trims — setpoints ~0, out = throttle duty riding on
    a slow disturbance, illustrative until the extended row."""
    t_end = start_ms + duration_ms

    def per_sample(t):
        if t > t_end:
            return None
        ts = (t - start_ms) * 0.001
        trim = 0.22 + 0.05 * math.sin(ts * 0.15)
        roll = (noise(rng, 0.004), 0.0,
                clip(trim + noise(rng, 0.02), -1.0, 1.0))
        pitch = (noise(rng, 0.004), 0.0,
                 clip(trim - 0.03 + noise(rng, 0.02), -1.0, 1.0))
        yaw = (noise(rng, 0.002), 0.0,
               clip(trim * 0.5 + noise(rng, 0.01), -1.0, 1.0))
        return roll, pitch, yaw

    return build(ETRACE_ALTHOLD, period_ms, start_ms, per_sample)


def actuator(period_ms, start_ms, duration_ms, rng):
    """eTraceActuator: commanded output (Out) ramps/steps per axis, the rate
    fields idle (base row), illustrative until the extended row."""
    t_end = start_ms + duration_ms

    def out_of(ts, base, step_at, level):
        if ts < step_at:
            return clip(base + 0.4 * math.sin(ts * 1.2), -1.0, 1.0)
        return clip(level + 0.15 * math.sin(ts * 2.0), -1.0, 1.0)

    def per_sample(t):
        if t > t_end:
            return None
        ts = (t - start_ms) * 0.001
        roll = (noise(rng, 0.003), 0.0, out_of(ts, 0.35, 8.0, -0.5))
        pitch = (noise(rng, 0.003), 0.0, out_of(ts, -0.2, 14.0, 0.45))
        yaw = (noise(rng, 0.002), 0.0,
               clip(0.6 * math.sin(ts * 0.5) + noise(rng, 0.01), -1.0, 1.0))
        return roll, pitch, yaw

    return build(ETRACE_ACTUATOR, period_ms, start_ms, per_sample)


def imu(period_ms, start_ms, duration_ms, rng):
    """eTraceIMU: fast, noisy gyro rates on all three axes (500 Hz row)."""
    t_end = start_ms + duration_ms

    def per_sample(t):
        if t > t_end:
            return None
        ts = (t - start_ms) * 0.001
        burst = 1.0 + 3.0 * math.exp(-((ts - 1.4) ** 2) / 0.02)
        roll = (noise(rng, 0.035) * burst, 0.0, 0.0)
        pitch = (noise(rng, 0.03) * burst, 0.0, 0.0)
        yaw = (noise(rng, 0.02), 0.0, 0.0)
        return roll, pitch, yaw

    return build(ETRACE_IMU, period_ms, start_ms, per_sample)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rng = random.Random(20260830)
    rate_blob = rate_probe(2, 1000, 3000, rng)

    # Append a self-consistent critic trailer to the rate dump: derive it from
    # the exact series we generated using the shared spec, so a viewer or test
    # sees the FC-trailer floats agree with what ground re-derivation yields.
    (magic, ver, ttype, rsize, flags, mask, period, rgain, snap,
     count, start, end) = HDR.unpack_from(rate_blob, 0)
    times = []
    rate = []
    desired = []
    for i in range(count // 3):
        tick, r, d, o = REC.unpack_from(
            rate_blob, HEADER_SIZE + i * 3 * RECORD_SIZE)
        times.append((tick - start) * 0.001)
        rate.append(r)
        desired.append(d)
    setpoint = max(desired)
    dur = (end - start) * 0.001
    rise, overshoot, settle = step_metrics_window(
        times, rate, setpoint, not_reached_s=dur)
    rate_blob = append_critic_trailer(rate_blob, 0, setpoint,
                                      rise, overshoot, settle)

    dumps = [
        ('trace_rate', rate_blob),
        ('trace_attitude', attitude(20, 1000, 27000, rng)),
        ('trace_althold', althold(20, 1000, 27000, rng)),
        ('trace_actuator', actuator(20, 1000, 27000, rng)),
        ('trace_imu', imu(2, 1000, 3000, rng)),
    ]

    print(f'Writing synthetic TRAC dumps to {OUT_DIR}')
    for name, blob in dumps:
        path = os.path.join(OUT_DIR, name + '.bin')
        with open(path, 'wb') as f:
            f.write(blob)
        (magic, ver, ttype, rsize, flags, mask, period, rgain, snap,
         count, start, end) = HDR.unpack_from(blob, 0)
        assert magic == MAGIC and ver == VERSION and rsize == RECORD_SIZE
        base = HEADER_SIZE + count * RECORD_SIZE
        assert len(blob) == base or len(blob) == base + 24  # optional trailer
        critic = 'critic' if len(blob) == base + 24 else ''
        ok = 'OK'
        print(f'  {name:16s} type={ttype} period={period}ms '
              f'records={count} dur={(end - start) * 0.001:.1f}s {ok} '
              f'pot={rgain / RATE_GAIN_Q:.3f} {len(blob)}B '
              f'{critic}-> {name}.bin')
    print('Open via GCS File > Open Trace Dump...  (or Dump from a live link)')


if __name__ == '__main__':
    sys.exit(main())