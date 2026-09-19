# ui/trace_viewer.py - TRAC snapshot dump viewer (no matplotlib/pyqtgraph).
"""
UAVX Trace Dump Viewer — parses the 32 B TRAC base header (v2: 128 B with a
gain block) + 16 B base-row records dumped from the FC (tag-54 chunk transport)
and renders one overlay strip per axis using plain QPainter.

Header (mirrors UAVXArmQ/src/trace.h TraceWriteHeader, all LE):
  0  u32  magic   0x43415254 ("TRAC")
  4  u16  version (2; v1 = 1)
  6  u8   traceType (TraceTypes enum; selects title/period)
  7  u8   recordSize (16)
  8  u16  flags    (bit1 = dumped from the committed flash copy)
 10  u16  axisMask (bits: Roll|Pitch|Yaw, 0x07)
 12  u16  samplePeriodMs (2 for rate/IMU, 20 for the slow types)
 14  u16  rateGainCode (RateGainScale x1000 fixed-point; 0 = legacy/not recorded)
 16  u32  snapStart   (ring index of record 0)
 20  u32  recordCount (count of 16 B records, i.e. 3 per sample tick)
 24  u32  startTickMs (wall ms at capture open)
 28  u32  endTickMs

v2 gain block (absent in v1; header bytes 32..119 — offsets 32/34 are the count
u16 (=21) + block version u16 (0), then 21 f32): the controller gains in effect
during the capture, per axis (Roll/Pitch/Yaw) R.Kp, R.Kd, R.Max, P.Kp (QKp),
P.Ki, P.IntLim, P.Max. Self-describing dumps: the gains are baked in so a
capture stays interpretable even if the GCS-locked values shift on the ground
before commit. v2 records start at offset 128 (TRACE_HEADER_SIZE); v1 at 32.
The parser branches on `version`.

Record (16 B, one per axis, written Roll/Pitch/Yaw per sample tick):
  0  u32  tick     wall ms (same clock as startTickMs)
  4  f32  Rate     measured gyro rate, rad/s  (the plant output)
8  f32  Desired  rate setpoint, rad/s       (pilot sticks / outer angle loop)
  12  f32  Out      control output, fraction ±1 (as written to the mixer)

Historical critic-result trailer (24 B, all LE): the FC's on-board step critic
and injected stimulus were REMOVED 2026-09-14 (the capture is now a passive
pilot-stimulus snapshot), so new dumps never carry a trailer. The parser below
still recognises one for old on-disk dumps and prefers it when present; otherwise
metrics are re-derived on the raw rows.
  0  u32  magic   0x43495243 ("CRIC")
  4  u8   version (1)
  5  u8   axis    (TRACE_TEST_AXIS, eRoll = 0 in v1)
  6  u8   flags   (0)
  7  u8   reserved(0)
  8  f32  StimValue   commanded rate setpoint, rad/s
 12  f32  RiseTimeS
 16  f32  OvershootPct
 20  f32  SettlingTimeS
   (NotReachedS is NOT stored — it is this capture's duration, in the header.)

Header offset 14-15 (rateGainCode): the Ch10 pot's RateGainScale — the live
multiplier the rate controllers applied — fixed-point x1000. Real range
0.25..4.0 -> code 250..4000, so 0 unambiguously means "legacy header / not
recorded". v2 ALSO bakes the static gains into the block: the pot code varies
in flight (the fast thing to fix), the gain block records what flew.

v1 captures the base 16 B row for EVERY trace type (the selector is locked but
the extended record layouts are not shipped yet), so this viewer renders the same
rate-loop overlay for all types and uses the header's samplePeriodMs + type name
for the time axis. When a type ships an extended row (recordSize > 16, version
bump) the generic record-driven layout switches on recordSize.

Adapted 2026-08-30 by GKE (based on the AGENTS.md trace spec agreed Aug30).
v2 header + gain block + rate-gain pot code added 2026-09-14 by GKE.
"""

import struct

from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget

from critic.metrics import step_metrics_window, STEP_NOT_REACHED_S

# Keep in lock-step with UAVXArmQ/src/trace.h.
TRACE_MAGIC = 0x43415254
TRACE_VERSION = 2
_BASE_HDR_SIZE = 32
TRACE_HEADER_SIZE = 128          # v2: 32 B base + gain block + pad
TRACE_RECORD_SIZE = 16

# Header gain block (v2): count u16 + version u16 at offset 32/34, then
# TRACE_GAIN_COUNT f32 in 3-axis order (Roll/Pitch/Yaw), each axis carrying the
# same field order the FC loop reads: R.Kp R.Kd R.Max P.Kp P.Ki P.IntLim P.Max.
# The block is the capture-time snapshot of the gains in effect during the
# capture — with the pot code (offset 14-15) it makes a dump self-describing.
# The static gains are also readable via the tag-71 readback /.af at a live
# dump session, but the block survives without the link. (The x1000-fixed-point
# u16 at offset 14-15 is kept for v1 dumps; v2 keeps writing it AND the block.)
TRACE_GAIN_Q = 1000.0            # fixed-point multiplier for the pot u16
TRACE_GAIN_COUNT = 21
_GAIN_FIELDS = ('rate_kp', 'rate_kd', 'rate_max',
                'angle_kp', 'angle_ki', 'angle_intlim', 'angle_max')

# Critic-result trailer (mirrors UAVXArmQ/src/trace.h TRACE_CRITIC_*).
TRACE_CRITIC_MAGIC = 0x43495243
TRACE_CRITIC_VERSION = 1
TRACE_CRITIC_SIZE = 24

# Header rateGainCode (offset 14-15) fixed-point multiplier (trace.h
# TRACE_RATE_GAIN_Q).
TRACE_RATE_GAIN_Q = 1000.0

_HDR = struct.Struct('<IHBBHHHHIIII')   # magic ver type rsize flags mask period rategain snap count start end
_REC = struct.Struct('<Ifff')
_CRITIC = struct.Struct('<IBBBBffff')
_GAIN_BLOCK = struct.Struct('<HH21f')    # count u16 + version u16 + 21 gains


def _parse_gain_block(data):
    """Decode the v2 header gain block (offset 32) into {axis: {field: value}}.

    Returns None if the block is missing/unknown (legacy v1 header). The count
    field guards the layout: a count other than TRACE_GAIN_COUNT flags a version
    we do not yet understand — treat as absent rather than mis-parsed."""
    gcount, gver, *vals = _GAIN_BLOCK.unpack_from(data, _BASE_HDR_SIZE)
    if gcount != TRACE_GAIN_COUNT:
        return None
    gains = {}
    off = 0
    for a in AXIS_NAMES:
        gains[a] = dict(zip(_GAIN_FIELDS,
                            vals[off:off + len(_GAIN_FIELDS)]))
        off += len(_GAIN_FIELDS)
    return gains


def rate_gain_scale(code):
    """Decode the header rateGainCode (RateGainScale x1000) to the multiplier
    the rate controllers applied. 0 = legacy header / not recorded -> None."""
    if not code:
        return None
    return code / TRACE_RATE_GAIN_Q

TRACE_TYPE_NAMES = {
    0: 'None (selector off)',
    1: 'Rate (eTraceRate)',
    2: 'Attitude (eTraceAttitude)',
    3: 'AltHold (eTraceAltHold)',
    4: 'Actuator (eTraceActuator)',
    5: 'IMU (eTraceIMU)',
}

AXIS_NAMES = ('Roll', 'Pitch', 'Yaw')

RAD2DEG = 57.29577951308232

# Series styling: (colour, width, pen-style)
_DESIRED = (QColor('#4a90d9'), 1.2, Qt.DashLine)   # grey-blue dashed
_RATE = (QColor('#27ae60'), 1.6, Qt.SolidLine)      # green solid
_OUT = (QColor('#c0392b'), 1.0, Qt.DotLine)         # red dotted, own axis


def _nice_step(span):
    """Rough 'nice' tick step for a span (1/2/5 x 10^k ladder)."""
    if span <= 0.0:
        return 1.0
    k = 10.0 ** int(span / 10.0)
    for m in (1.0, 2.0, 5.0, 10.0):
        if span / (m * k) <= 4.5:
            return m * k
    return 10.0 * k


class TraceStrip(QWidget):
    """One axis: Desired (dashed) and Rate (solid, rad->deg) share the left
    axis; Out (dotted) uses its own ±1 right axis. Mouse wheel zooms the time
    window, double-click resets it."""

    def __init__(self, title, t, rate, desired, out, parent=None):
        super().__init__(parent)
        self._title = title
        self._t = t
        self._rate = rate
        self._desired = desired
        self._out = out
        self._x0 = 0.0
        self._x1 = t[-1] if len(t) > 1 else 1.0
        self._left = 0
        self._right = 200
        self.setMinimumHeight(120)

    def _slice(self, tt):
        idx = [i for i, tv in enumerate(self._t) if self._x0 <= tv <= self._x1]
        return idx, [tt[i] for i in idx]

    def wheelEvent(self, ev):
        span_w = max(1, self._right - self._left)
        frac = (ev.x() - self._left) / span_w
        frac = min(1.0, max(0.0, frac))
        anchor = self._x0 + (self._x1 - self._x0) * frac
        factor = 0.85 if ev.angleDelta().y() > 0 else 1.18
        self._x0 = anchor - (anchor - self._x0) * factor
        self._x1 = self._x0 + (self._x1 - self._x0) * factor
        if self._x0 < 0.0:
            self._x1 -= self._x0
            self._x0 = 0.0
        self.update()
        ev.accept()

    def mouseDoubleClickEvent(self, ev):
        self._x0 = 0.0
        self._x1 = self._t[-1] if len(self._t) > 1 else 1.0
        self.update()
        ev.accept()

    def paintEvent(self, ev):
        try:
            self._draw()
        except Exception as e:
            qp = QPainter(self)
            qp.fillRect(self.rect(), QColor('#101418'))
            qp.setPen(QColor('#e74c3c'))
            qp.drawText(8, 24, f'TraceStrip render error: {e}')
            qp.end()

    def _draw(self):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing, True)
        qp.fillRect(self.rect(), QColor('#101418'))

        tl = int(self.fontMetrics().height() * 1.4)
        self._left = 54
        self._right = self.width() - 40
        bottom = self.height() - tl - 2
        top = 4
        pw = max(10, self._right - self._left)
        ph = max(10, bottom - top)

        idx, desired = self._slice(self._desired)
        _, rate = self._slice(self._rate)
        _, out = self._slice(self._out)
        if not idx:
            qp.setPen(QColor('#8899aa'))
            qp.drawText(self._left + 8, top + ph // 2, '(no samples in window)')
            qp.end()
            return

        tmin = max(self._t[idx[0]], self._x0)
        tmax = min(self._t[idx[-1]], self._x1)

        # Left axis auto-ranges over Desired+Rate, kept symmetric about 0 so
        # direction is obvious and the zero line always exists.
        lo = min(min(desired), min(rate)) if idx else 0.0
        hi = max(max(desired), max(rate)) if idx else 0.0
        if hi - lo < 1e-6:
            hi = 1.0
            lo = -1.0
        pad = (hi - lo) * 0.12
        ylo = lo - pad
        yhi = hi + pad
        span = yhi - ylo

        def x_of(tv):
            return self._left + (tv - tmin) / max(1e-9, tmax - tmin) * pw

        # Left (rate/desired) y-mapper and right (out) y-mapper: Out maps the
        # full ±1 duty onto the plot height, its own scale independent of the
        # (much larger) rate span.
        def y_of(v):
            return top + (1.0 - (v - ylo) / span) * ph

        def r_of(v):
            return top + (0.5 - v * 0.5) * ph

        def series_poly(vals, xm, ym):
            return QPolygonF([QPointF(xm(tv), ym(vals[i]))
                              for i, tv in enumerate(self._t)
                              if self._x0 <= tv <= self._x1])

        # Left-axis grid + labels (deg/s since v1 fields are rad/s).
        qp.setPen(QPen(QColor('#22303c'), 1))
        step = _nice_step(span)
        v = float(int(ylo / step)) * step
        while v <= yhi:
            if abs(v) <= 1e-9:
                qp.setPen(QPen(QColor('#41505e'), 1, Qt.DashLine))
            else:
                qp.setPen(QPen(QColor('#22303c'), 1))
            y = y_of(v)
            qp.drawLine(self._left, int(y), self._right, int(y))
            qp.setPen(QColor('#7f8c99'))
            qp.drawText(2, int(y) + 4, f'{v * RAD2DEG:.0f}')   # deg/s
            qp.setPen(QPen(QColor('#22303c'), 1))
            v += step

        # Right-axis gridlines at ±1 (Out duty) with labels.
        for frac in (-1.0, 1.0):
            y = r_of(frac)
            qp.setPen(QPen(QColor('#3d4a56'), 1, Qt.DashLine))
            qp.drawLine(self._left, int(y), self._right, int(y))
            qp.setPen(QColor('#7f8c99'))
            qp.drawText(self.width() - 34, int(y) + 4, f'{frac:+.0f}')

        # Time axis.
        span_s = tmax - tmin
        qp.setPen(QColor('#8899aa'))
        label_y = bottom + tl - 2
        qp.drawText(self._left, label_y, '0 s')
        mid = (self._left + self._right) // 2 - 20
        qp.drawText(mid, label_y, f'{span_s:.3f} s')

        # Series: Out on its own right axis, then Desired, then Rate on top.
        qp.setPen(QPen(_OUT[0], _OUT[1], _OUT[2]))
        qp.drawPolyline(series_poly(out, x_of, r_of))
        qp.setPen(QPen(_DESIRED[0], _DESIRED[1], _DESIRED[2]))
        qp.drawPolyline(series_poly(desired, x_of, y_of))
        qp.setPen(QPen(_RATE[0], _RATE[1], _RATE[2]))
        qp.drawPolyline(series_poly(rate, x_of, y_of))

        # Title + legend.
        qp.setFont(QFont(qp.font().family(), qp.font().pointSize(), QFont.Bold))
        qp.setPen(QColor('#d8dee4'))
        qp.drawText(self._left + 8, top + self.fontMetrics().height(),
                    self._title)
        qp.setFont(QFont(qp.font().family(), qp.font().pointSize()))
        lgx = self._left + 8
        lgy = top + self.fontMetrics().height() + 4
        for name, sty in (('Desired', _DESIRED), ('Rate', _RATE),
                          ('Out', _OUT)):
            qp.setPen(QPen(sty[0], 2, sty[2]))
            qp.drawLine(lgx, lgy, lgx + 14, lgy)
            qp.setPen(QColor('#b7c3cd'))
            qp.drawText(lgx + 17, lgy, name)
            lgx += self.fontMetrics().width(name + 'xx')
        qp.end()


def parse_trace(data):
    """Validate a TRAC blob and split it into per-axis time series.

    Raises ValueError on any layout/truncation problem. Returns a dict with the
    header fields plus t/rate/desired/out lists grouped per axis (Roll, Pitch,
    Yaw), so the parsing is testable without a Qt event loop."""
    if len(data) < _BASE_HDR_SIZE:
        raise ValueError('not a TRAC dump: shorter than the base header')
    (magic, version, ttype, rsize, flags, axis_mask, period,
     rate_gain_code, snap_start, count, start_ms, end_ms) = _HDR.unpack_from(
        data, 0)
    if magic != TRACE_MAGIC:
        raise ValueError('not a TRAC snapshot: bad magic bytes '
                         f'{magic:08x}')
    if rsize != TRACE_RECORD_SIZE:
        raise ValueError(f'recordSize {rsize} unsupported (v1 = 16)')

    # Header size depends on version: v1 = 32 B base only; v2 appends a gain
    # block (count u16 + version u16 + 21 f32) so the record region shifts to
    # the 128 B aligned slot. The base 32 B are the same in both.
    if version == 1:
        header_size = _BASE_HDR_SIZE
        gains = None
    elif version == 2:
        if len(data) < TRACE_HEADER_SIZE:
            raise ValueError('not a TRAC dump: shorter than the 128 B v2 header')
        header_size = TRACE_HEADER_SIZE
        gains = _parse_gain_block(data)
    else:
        raise ValueError(f'TRAC version {version} unsupported (1 or 2)')

    need = header_size + count * rsize
    if count == 0 or need > len(data):
        raise ValueError(f'truncated record region ({need} > {len(data)})')

    # Three records per sample tick (Roll, Pitch, Yaw in that order).
    t = [[], [], []]
    rate = [[], [], []]
    desired = [[], [], []]
    out = [[], [], []]
    for i in range(count):
        rec = _REC.unpack_from(data, header_size + i * rsize)
        a = i % 3
        t[a].append(rec[0])
        rate[a].append(rec[1])
        desired[a].append(rec[2])
        out[a].append(rec[3])

    # Optional critic-result trailer right after the records: the FC-authoritative
    # measurement the GCS reads directly (radio-independent).
    critic = None
    if need + TRACE_CRITIC_SIZE <= len(data):
        (cmagic, cver, caxis, cflags, _cres, cstim, crise, covs,
         csettle) = _CRITIC.unpack_from(data, need)
        if cmagic == TRACE_CRITIC_MAGIC and cver == TRACE_CRITIC_VERSION:
            critic = {
                'axis': caxis,
                'flags': cflags,
                'setpoint': cstim,
                'rise_s': crise,
                'overshoot_pct': covs,
                'settle_s': csettle,
            }

    return {
        'version': version,
        'type': ttype,
        'record_size': rsize,
        'flags': flags,
        'axis_mask': axis_mask,
        'period_ms': period,
        'rate_gain_code': rate_gain_code,
        'rate_gain_scale': rate_gain_scale(rate_gain_code),
        'gains': gains,
        'snap_start': snap_start,
        'record_count': count,
        'start_ms': start_ms,
        'end_ms': end_ms,
        'critic': critic,
        't': t,
        'rate': rate,
        'desired': desired,
        'out': out,
    }


# Test axis used for a rate-step measurement: the first record of each sample
# triplet (mirrors the legacy v1 TRACE_TEST_AXIS = eRoll). The FC no longer
# designates a test axis (passive snapshot recorder, 2026-09-14); the GCS
# measures whatever step the pilot actually flew, defaulting to Roll.
_TEST_AXIS = 0   # eRoll


def rate_step_metrics(p):
    """Rate-step metrics for a passive eTraceRate capture.

    Evaluates rise/overshoot/settle against the shared critic.metrics spec on
    this byte dump (the same shape the FC's retired on-board CriticEvaluateRateStep
    used to ship, so ground analysis remains comparable by construction):

    - times = ms since capture open -> s (matches the FC/C sim time base)
    - values = measured Roll Rate (rad/s)
    - setpoint = peak Roll Desired during the flown step (== what the pilot
      commanded, or the outer angle loop demanded)
    - not_reached_s = this capture's duration (the window sentinel)

    Returns a dict, or None if there is no measurable step (no samples, or
    setpoint not raised above the 0.01 rad/s threshold)."""
    dur = (p['end_ms'] - p['start_ms']) * 0.001
    if dur <= 0.0:
        dur = STEP_NOT_REACHED_S

    # The FC-authoritative measurement ships in the trailer (radio-independent);
    # prefer it verbatim when present. Fall back to re-deriving on the byte dump
    # when no trailer exists (e.g. a synthetic or older dump).
    critic = p.get('critic')
    if critic:
        return {
            'rise_s': critic['rise_s'],
            'overshoot_pct': critic['overshoot_pct'],
            'settle_s': critic['settle_s'],
            'setpoint': critic['setpoint'],
            'not_reached_s': dur,
            'axis': critic['axis'],
            'source': 'fc',
        }

    times = [(tv - p['start_ms']) * 0.001 for tv in p['t'][_TEST_AXIS]]
    rate = p['rate'][_TEST_AXIS]
    desired = p['desired'][_TEST_AXIS]
    if not rate or not desired:
        return None

    setpoint = max(desired) if desired else 0.0
    if setpoint <= 0.01:
        return None

    rise, overshoot, settle = step_metrics_window(
        times, rate, setpoint, not_reached_s=dur)

    return {
        'rise_s': rise,
        'overshoot_pct': overshoot,
        'settle_s': settle,
        'setpoint': setpoint,
        'not_reached_s': dur,
        'axis': _TEST_AXIS,
        'source': 're-derived',
    }


class TraceViewer(QWidget):
    """Parse a TRAC blob and show one TraceStrip per axis."""

    def __init__(self, data, source='', parent=None):
        super().__init__(parent)
        p = parse_trace(data)
        ttype = p['type']

        self.setWindowTitle(
            f'Trace Viewer — {TRACE_TYPE_NAMES.get(ttype, str(ttype))}')
        self.setMinimumSize(760, 480)

        lay = QVBoxLayout(self)
        dur = (p['end_ms'] - p['start_ms']) * 0.001 \
            if p['end_ms'] >= p['start_ms'] else 0.0
        info = (
            'Trace: {}   version {}   recordSize {}   period {} ms\n'
            'records {}   axes {}   duration {:.2f} s\n'
            'source: {}   flags 0x{:04x}'.format(
                TRACE_TYPE_NAMES.get(ttype, ttype), p['version'],
                p['record_size'], p['period_ms'], p['record_count'],
                '|'.join(AXIS_NAMES), dur, source, p['flags']))

        if p['version'] >= 2:
            rgs = p['rate_gain_scale']
            pot = f'rate-gain pot scale x{rgs:.3f} (code {p["rate_gain_code"]})' \
                if rgs is not None else 'rate-gain pot: not recorded (code 0)'
            gains = p.get('gains')
            if gains:
                info += '\n\n{}'.format(pot)
                for a in AXIS_NAMES:
                    g = gains[a]
                    info += (
                        '\n{}  R.kp {:<7.4f} R.kd {:<8.6f} R.max {:4.1f}/s'
                        '   Q.kp {:<8.4f} Q.ki {:<8.5f} Ilim {:<7.5f} Q.max {:4.1f}'.format(
                            a, g['rate_kp'], g['rate_kd'], g['rate_max'],
                            g['angle_kp'], g['angle_ki'], g['angle_intlim'],
                            g['angle_max']))
            else:
                info += '\n\n{} (no v2 gain block)'.format(pot)

        if ttype == 1:   # eTraceRate — rate-step metrics on the flown segment
            m = rate_step_metrics(p)
            if m is not None:
                info += (
                    '\n\ncritic ({}, {}):\n'
                    '  setpoint {:6.3f} rad/s   not-reached {:5.3f} s\n'
                    '  rise {:7.4f} s   overshoot {:6.2f} %   '
                    'settle {:6.3f} s'.format(
                        AXIS_NAMES[m.get('axis', _TEST_AXIS)
                                   if m.get('source') == 'fc' else _TEST_AXIS],
                        m['source'],
                        m['setpoint'], m['not_reached_s'], m['rise_s'],
                        m['overshoot_pct'], m['settle_s']))
            else:
                info += '\n\ncritic: no measurable rate step (flat Desired)'

        lab = QLabel(info)
        lab.setStyleSheet('color:#d8dee4; background:#0b0e13; padding:6px;'
                          ' border:1px solid #22303c;')
        lay.addWidget(lab)

        for a in range(3):
            ts = [(tv - p['start_ms']) * 0.001 for tv in p['t'][a]]
            lay.addWidget(TraceStrip(AXIS_NAMES[a], ts, p['rate'][a],
                                     p['desired'][a], p['out'][a]), 1)

        self.setStyleSheet('background: #101418;')