#!/usr/bin/env python3
"""
Pure-stdlib PNG figure renderer (no matplotlib/PIL available).

Draws line plots and bar charts into an RGBA raster and writes a PNG
using zlib. Text uses a built-in 5x7 bitmap font (ASCII only) so the
output embeds cleanly in xelatex/pandoc PDFs without rsvg-convert.
"""

import math
import struct
import zlib

# ═══════════════════════════════════════════
#  5x7 bitmap font (MSB = leftmost column)
# ═══════════════════════════════════════════
def _g(*rows):
    return [int(r, 2) for r in rows]

FONT = {
    'A': _g('01110', '10001', '10001', '11111', '10001', '10001', '10001'),
    'B': _g('11110', '10001', '10001', '11110', '10001', '10001', '11110'),
    'C': _g('01110', '10001', '10000', '10000', '10000', '10001', '01110'),
    'D': _g('11110', '10001', '10001', '10001', '10001', '10001', '11110'),
    'E': _g('11111', '10000', '10000', '11110', '10000', '10000', '11111'),
    'F': _g('11111', '10000', '10000', '11110', '10000', '10000', '10000'),
    'G': _g('01110', '10001', '10000', '10111', '10001', '10001', '01111'),
    'H': _g('10001', '10001', '10001', '11111', '10001', '10001', '10001'),
    'I': _g('11111', '00100', '00100', '00100', '00100', '00100', '11111'),
    'J': _g('00111', '00010', '00010', '00010', '00010', '10010', '01100'),
    'K': _g('10001', '10010', '10100', '11000', '10100', '10010', '10001'),
    'L': _g('10000', '10000', '10000', '10000', '10000', '10000', '11111'),
    'M': _g('10001', '11011', '10101', '10101', '10001', '10001', '10001'),
    'N': _g('10001', '11001', '10101', '10011', '10001', '10001', '10001'),
    'O': _g('01110', '10001', '10001', '10001', '10001', '10001', '01110'),
    'P': _g('11110', '10001', '10001', '11110', '10000', '10000', '10000'),
    'Q': _g('01110', '10001', '10001', '10001', '10101', '10010', '01101'),
    'R': _g('11110', '10001', '10001', '11110', '10100', '10010', '10001'),
    'S': _g('01111', '10000', '10000', '01110', '00001', '00001', '11110'),
    'T': _g('11111', '00100', '00100', '00100', '00100', '00100', '00100'),
    'U': _g('10001', '10001', '10001', '10001', '10001', '10001', '01110'),
    'V': _g('10001', '10001', '10001', '10001', '10001', '01010', '00100'),
    'W': _g('10001', '10001', '10001', '10101', '10101', '11011', '10001'),
    'X': _g('10001', '10001', '01010', '00100', '01010', '10001', '10001'),
    'Y': _g('10001', '10001', '01010', '00100', '00100', '00100', '00100'),
    'Z': _g('11111', '00001', '00010', '00100', '01000', '10000', '11111'),
    'a': _g('00000', '00000', '01110', '00001', '01111', '10001', '01111'),
    'b': _g('10000', '10000', '10110', '11001', '10001', '10001', '11110'),
    'c': _g('00000', '00000', '01110', '10001', '10000', '10001', '01110'),
    'd': _g('00001', '00001', '01101', '10011', '10001', '10001', '01111'),
    'e': _g('00000', '00000', '01110', '10001', '11111', '10000', '01110'),
    'f': _g('00110', '01001', '01000', '11100', '01000', '01000', '01000'),
    'g': _g('00000', '00000', '01111', '10001', '10001', '01111', '00001'),
    'h': _g('10000', '10000', '10110', '11001', '10001', '10001', '10001'),
    'i': _g('00100', '00000', '01100', '00100', '00100', '00100', '01110'),
    'j': _g('00010', '00000', '00110', '00010', '00010', '10010', '01100'),
    'k': _g('10000', '10000', '10010', '10100', '11000', '10100', '10010'),
    'l': _g('01100', '00100', '00100', '00100', '00100', '00100', '01110'),
    'm': _g('00000', '00000', '11010', '10101', '10101', '10001', '10001'),
    'n': _g('00000', '00000', '10110', '11001', '10001', '10001', '10001'),
    'o': _g('00000', '00000', '01110', '10001', '10001', '10001', '01110'),
    'p': _g('00000', '00000', '11110', '10001', '10001', '11110', '10000'),
    'q': _g('00000', '00000', '01101', '10011', '10001', '01111', '00001'),
    'r': _g('00000', '00000', '10110', '11001', '10000', '10000', '10000'),
    's': _g('00000', '00000', '01111', '10000', '01110', '00001', '11110'),
    't': _g('01000', '01000', '11100', '01000', '01000', '01001', '00110'),
    'u': _g('00000', '00000', '10001', '10001', '10001', '10011', '01101'),
    'v': _g('00000', '00000', '10001', '10001', '10001', '01010', '00100'),
    'w': _g('00000', '00000', '10001', '10001', '10101', '10101', '01010'),
    'x': _g('00000', '00000', '10001', '01010', '00100', '01010', '10001'),
    'y': _g('00000', '00000', '10001', '10001', '01111', '00001', '01110'),
    'z': _g('00000', '00000', '11111', '00010', '00100', '01000', '11111'),
    '0': _g('01110', '10001', '10011', '10101', '11001', '10001', '01110'),
    '1': _g('00100', '01100', '00100', '00100', '00100', '00100', '01110'),
    '2': _g('01110', '10001', '00001', '00110', '01000', '10000', '11111'),
    '3': _g('11111', '00010', '00100', '00110', '00001', '10001', '01110'),
    '4': _g('00010', '00110', '01010', '10010', '11111', '00010', '00010'),
    '5': _g('11111', '10000', '11110', '00001', '00001', '10001', '01110'),
    '6': _g('00110', '01000', '10000', '11110', '10001', '10001', '01110'),
    '7': _g('11111', '00001', '00010', '00100', '01000', '01000', '01000'),
    '8': _g('01110', '10001', '10001', '01110', '10001', '10001', '01110'),
    '9': _g('01110', '10001', '10001', '01111', '00001', '00010', '01100'),
    ' ': _g('00000', '00000', '00000', '00000', '00000', '00000', '00000'),
    '.': _g('00000', '00000', '00000', '00000', '00000', '01100', '01100'),
    ',': _g('00000', '00000', '00000', '00000', '01100', '01100', '01000'),
    '-': _g('00000', '00000', '00000', '11111', '00000', '00000', '00000'),
    '_': _g('00000', '00000', '00000', '00000', '00000', '00000', '11111'),
    '/': _g('00001', '00010', '00100', '01000', '10000', '00000', '00000'),
    '(': _g('00100', '01000', '10000', '10000', '10000', '01000', '00100'),
    ')': _g('00100', '00010', '00001', '00001', '00001', '00010', '00100'),
    ':': _g('00000', '01100', '01100', '00000', '01100', '01100', '00000'),
    '%': _g('11001', '11010', '00010', '00100', '01000', '01011', '10011'),
    '+': _g('00000', '00100', '00100', '11111', '00100', '00100', '00000'),
    '=': _g('00000', '00000', '11111', '00000', '11111', '00000', '00000'),
    "'": _g('00100', '00100', '01000', '00000', '00000', '00000', '00000'),
}

COLORS = {
    "Roll": (214, 39, 40),
    "Pitch": (31, 119, 180),
    "Yaw": (44, 160, 44),
}


class Canvas:
    def __init__(self, w: int, h: int):
        self.w = w
        self.h = h
        self.px = [[(255, 255, 255)] * w for _ in range(h)]

    def set_px(self, x, y, c):
        x, y = int(round(x)), int(round(y))
        if 0 <= x < self.w and 0 <= y < self.h:
            self.px[y][x] = c

    def line(self, x0, y0, x1, y1, c):
        x0, y0, x1, y1 = int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = (x1 > x0) - (x1 < x0)
        sy = (y1 > y0) - (y1 < y0)
        err = dx + dy
        x0, y0, x1, y1 = round(x0), round(y0), round(x1), round(y1)
        while True:
            self.set_px(x0, y0, c)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def polyline(self, pts, c):
        for i in range(len(pts) - 1):
            self.line(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], c)

    def rect(self, x0, y0, x1, y1, c):
        x0, y0 = int(x0), int(y0)
        x1, y1 = int(x1), int(y1)
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                self.set_px(x, y, c)

    def text(self, x, y, s, c, scale: int = 2):
        cx = x
        for ch in s:
            g = FONT.get(ch)
            if g is None:
                cx += 6 * scale
                continue
            for r in range(7):
                row = g[r]
                for col in range(5):
                    if row & (1 << (4 - col)):
                        for dy in range(scale):
                            for dx in range(scale):
                                self.set_px(cx + col * scale + dx,
                                            y + r * scale + dy, c)
            cx += 6 * scale


def write_png(path: str, canvas: Canvas) -> None:
    w, h = canvas.w, canvas.h
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        for (r, g, b) in canvas.px[y]:
            raw += bytes((r, g, b))

    def chunk(tag, data):
        c = struct.pack('>I', len(data)) + tag + data
        c += struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF)
        return c

    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(bytes(raw), 6))
    png += chunk(b'IEND', b'')
    with open(path, 'wb') as f:
        f.write(png)


def _fmt(v: float) -> str:
    if abs(v - round(v)) < 1e-9:
        return f"{v:.0f}"
    s = f"{v:.2f}".rstrip('0').rstrip('.')
    return s


def plot_lines(path: str, title: str, xlabel: str, ylabel: str,
               series, xlim=None, ylim=None,
               W: int = 1100, H: int = 420, scale: int = 2) -> None:
    """series: list of (label, xs, ys)"""
    if not series:
        return
    xs = [x for _, xs_, _ in series for x in xs_]
    ys = [y for _, _, ys_ in series for y in ys_]
    xlo, xhi = xlim if xlim else (min(xs), max(xs))
    ylo, yhi = ylim if ylim else (min(ys), max(ys))
    if xhi - xlo < 1e-9:
        xhi = xlo + 1
    if yhi - ylo < 1e-9:
        yhi = ylo + 1
    PL, PR, PT, PB = 90, 40, 64, 66
    plot_w = W - PL - PR
    plot_h = H - PT - PB

    def SX(v):
        return PL + (v - xlo) / (xhi - xlo) * plot_w

    def SY(v):
        return PT + (1 - (v - ylo) / (yhi - ylo)) * plot_h

    cv = Canvas(W, H)
    cv.text((W - len(title) * 6 * scale) // 2, 14, title, (40, 40, 40), scale)

    # gridlines + y ticks
    for k in range(6):
        v = ylo + (yhi - ylo) * k / 5.0
        y = SY(v)
        cv.line(PL, y, W - PR, y, (224, 224, 224))
        cv.text(PL - 8 - len(_fmt(v)) * 6 * scale, y - 3 * scale,
                _fmt(v), (100, 100, 100), scale)
    # x ticks
    for k in range(6):
        v = xlo + (xhi - xlo) * k / 5.0
        x = SX(v)
        cv.text(x - len(_fmt(v)) * 3 * scale, H - PB + 6,
                _fmt(v), (100, 100, 100), scale)

    # axes
    cv.line(PL, PT, PL, PT + plot_h, (60, 60, 60))
    cv.line(PL, PT + plot_h, W - PR, PT + plot_h, (60, 60, 60))

    cv.text(PL + 8, H - 24, xlabel, (70, 70, 70), scale)
    cv.text(10, (H + len(ylabel) * 6 * scale) // 2 - 10, ylabel,
            (70, 70, 70), scale)

    for label, xs_, ys_ in series:
        pts = [(SX(x), SY(y)) for x, y in zip(xs_, ys_)]
        c = COLORS.get(label, (80, 80, 80))
        cv.polyline(pts, c)
        if len(pts) > 1:
            lx, ly = pts[-1]
            cv.text(min(lx + 6, W - PR - len(label) * 6 * scale),
                    ly - 4 * scale, label, c, scale)

    write_png(path, cv)


def plot_bars(path: str, title: str, ylabel: str, items,
              ymax=None, W: int = 1100, H: int = 420, scale: int = 2) -> None:
    if not items:
        return
    if ymax is None:
        ymax = max(v for _, v in items) * 1.15
    PL, PR, PT, PB = 90, 40, 64, 90
    plot_w = W - PL - PR
    plot_h = H - PT - PB
    n = len(items)
    bw = min(70.0, plot_w / n * 0.6)

    def SY(v):
        return PT + (1 - v / ymax) * plot_h

    cv = Canvas(W, H)
    cv.text((W - len(title) * 6 * scale) // 2, 14, title, (40, 40, 40), scale)
    for k in range(6):
        v = ymax * k / 5.0
        y = SY(v)
        cv.line(PL, y, W - PR, y, (224, 224, 224))
        cv.text(PL - 8 - len(_fmt(v)) * 6 * scale, y - 3 * scale,
                _fmt(v), (100, 100, 100), scale)
    cv.line(PL, PT, PL, PT + plot_h, (60, 60, 60))
    cv.line(PL, PT + plot_h, W - PR, PT + plot_h, (60, 60, 60))
    cv.text(PL + 8, H - 24, ylabel, (70, 70, 70), scale)

    for i, (label, v) in enumerate(items):
        cx = PL + (i + 0.5) / n * plot_w
        x0 = cx - bw / 2
        y0 = SY(v)
        c = COLORS.get(label.split(":")[0], (31, 119, 180))
        cv.rect(x0, y0, x0 + bw, PT + plot_h, c)
        cv.text(cx - len(_fmt(v)) * 3 * scale, y0 - 6 * scale, _fmt(v),
                (40, 40, 40), scale)
        # rotated-ish label: draw vertically stacked characters
        lx = cx - 3 * scale
        ly = PT + plot_h + 6
        for ch in label:
            cv.text(lx, ly, ch, (70, 70, 70), scale)
            ly += 7 * scale
    write_png(path, cv)
