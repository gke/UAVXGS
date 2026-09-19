# ui/replay_window.py
"""
Logfile replay window.

Loads a `.rawlog` (the always-on timed raw telemetry capture) and re-drives
the GCS from it: the captured bytes are pushed through the same
`FrameDecoder` the live serial thread uses, and each completed frame is
handed to `main_window.process_packet()` — so replay drives the exact same
display path as a live connection, without a serial port.

Only raw telemetry ever enters via the serial thread / its `raw_bytes`
signal; replay goes straight to `process_packet` and never emits
`raw_bytes`, so replaying a log can never be re-captured into a new one
(`main_window.replaying` is set as an extra guard the raw-log writer checks).

Transport (REW / PLAY / FF):

  PLAY    play/stop toggle — click to play (green), click again to pause (red)
  REW     acts while depressed (auto-repeat) — scans back while held
  FF      acts while depressed (auto-repeat) — scans forward while held
  slider  0.25x..8x playback-speed multiplier (1x = real-time pacing)
  timeline
          scrubber showing playback position (drag to seek)

Rewind/scrub re-sends captured frames idempotently (process_packet just
re-writes the same display values), so looping back over already-played
data is safe and cheap.
"""

import os

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

from core.frame_decoder import FrameDecoder


RAWLOG_MAGIC = b"UAVXRAW\x01"
MAX_CHUNK = 1 << 20


class ReplayWindow(QMainWindow):
    TICK_MS = 20
    REW_STEP_S = 1.0
    FF_STEP_S = 2.0
    SPEED_MIN_X = 0.25
    SPEED_MAX_X = 8.0

    def __init__(self, main_window, log_path=None):
        super().__init__()

        self.main = main_window
        self._frames = []          # [(offset_us, bytearray), ...] sorted
        self._total_us = 0
        self._playing = False
        self._last_idx = 0         # index of the next frame to send
        self._offset_us = 0        # simulated playback position (us)

        self.setWindowTitle("Replay Log")
        self.setMinimumSize(560, 210)

        self._build_ui()
        self._build_menu()

        if log_path is not None:
            self.load_file(log_path)

    # --------------------------------------------------------------- widgets
    def _build_ui(self):
        self._label = QLabel("No logfile loaded")
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setStyleSheet("font-weight: bold;")

        self._file_combo = QComboBox()
        self._file_combo.setToolTip("Raw logfiles in the log folder")
        self._refresh_file_list()
        self._file_combo.activated.connect(self.load_current)

        self._load_btn = QPushButton("Load")
        self._load_btn.clicked.connect(self.pick_file)

        self._play_btn = QPushButton("PLAY")
        self._play_btn.setCheckable(True)
        self._play_btn.setToolTip("Play / Stop toggle (click to play, click again to pause)")
        self._play_btn.toggled.connect(self._on_play_toggled)
        self._apply_play_style()

        self._rew_btn = QPushButton("REW")
        self._rew_btn.setAutoRepeat(True)
        self._rew_btn.setAutoRepeatDelay(300)
        self._rew_btn.setAutoRepeatInterval(40)
        self._rew_btn.setToolTip("Hold to rewind")
        self._rew_btn.clicked.connect(self._on_rewind)

        self._ff_btn = QPushButton("FF")
        self._ff_btn.setAutoRepeat(True)
        self._ff_btn.setAutoRepeatDelay(300)
        self._ff_btn.setAutoRepeatInterval(40)
        self._ff_btn.setToolTip("Hold to fast-forward")
        self._ff_btn.clicked.connect(self._on_ff)

        self._speed_slider = QSlider(Qt.Horizontal)
        self._speed_slider.setRange(25, 800)   # 0.25x .. 8.0x
        self._speed_slider.setValue(100)
        self._speed_slider.setTickPosition(QSlider.TicksBelow)
        self._speed_slider.setTickInterval(100)
        self._speed_slider.valueChanged.connect(self._on_speed_changed)
        self._speed_label = QLabel("1.0x")
        self._speed_label.setMinimumWidth(44)
        self._speed_label.setAlignment(Qt.AlignCenter)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setEnabled(False)
        self._slider.valueChanged.connect(self._on_scrub)

        self._time_label = QLabel("00:00.0 / 00:00.0")
        self._time_label.setAlignment(Qt.AlignCenter)

        transport = QHBoxLayout()
        transport.addWidget(self._rew_btn)
        transport.addWidget(self._play_btn)
        transport.addWidget(self._ff_btn)
        transport.addSpacing(12)
        transport.addWidget(QLabel("Speed:"))
        transport.addWidget(self._speed_slider, 1)
        transport.addWidget(self._speed_label)

        file_row = QHBoxLayout()
        file_row.addWidget(QLabel("Logfile:"))
        file_row.addWidget(self._file_combo, 1)
        file_row.addWidget(self._load_btn)

        v = QVBoxLayout()
        v.addWidget(self._label)
        v.addLayout(file_row)
        v.addLayout(transport)
        v.addWidget(self._slider)
        v.addWidget(self._time_label)

        container = QWidget()
        container.setLayout(v)
        self.setCentralWidget(container)

        self._timer = QTimer(self)
        self._timer.setInterval(self.TICK_MS)
        self._timer.timeout.connect(self._on_tick)

    def _build_menu(self):
        m = self.menuBar().addMenu("&File")
        open_action = QAction("Open Log…", self)
        open_action.triggered.connect(self.pick_file)
        m.addAction(open_action)
        m.addSeparator()
        close_action = QAction("Close", self)
        close_action.setShortcut("Ctrl+W")
        close_action.triggered.connect(self.close)
        m.addAction(close_action)

    def _default_log_dir(self):
        return getattr(self.main, "log_dir", None) or os.path.expanduser("~/UAVX")

    def _refresh_file_list(self):
        self._file_combo.clear()
        log_dir = self._default_log_dir()
        try:
            names = sorted(f for f in os.listdir(log_dir) if f.endswith(".rawlog"))
        except OSError:
            names = []
        for n in names:
            self._file_combo.addItem(n, os.path.join(log_dir, n))
        if names:
            self._file_combo.setCurrentIndex(len(names) - 1)

    # -------------------------------------------------------------- loading
    def load_current(self):
        path = self._file_combo.currentData()
        if path:
            self.load_file(path)

    def pick_file(self):
        log_dir = self._default_log_dir()
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Telemetry Log",
            log_dir, "UAVX Raw Logs (*.rawlog);;All Files (*)")
        if path:
            self.load_file(path)

    def load_file(self, path):
        frames, total_bytes, total_us = self._decode(path)
        if frames is None:
            QMessageBox.warning(self, "Replay", f"Could not read:\n{path}")
            return

        # Never re-capture replay input into a new log.
        self.main.replaying = True

        self._frames = frames
        self._total_us = total_us if total_us else (frames[-1][0] if frames else 0)
        self.log_path = path
        self._playing = False
        self._last_idx = 0
        self._offset_us = 0
        self._play_btn.setChecked(False)
        self._timer.stop()
        self._apply_play_style()
        self._update_slider_limits()
        self._update_time_label()

        base = os.path.basename(path)
        if frames:
            self._label.setText(
                f"{base}  —  {len(frames):,} frames  ({total_bytes/1024:.0f} KiB, "
                f"{self._total_us/1e6:.1f} s)")
        else:
            self._label.setText(f"{base}  —  no frames")
        self._slider.setEnabled(True)

    def _decode(self, path):
        """Read the raw log, feed every chunk through the shared decoder, and
        build a flat frame timeline: list of (offset_us, frame_bytes), sorted
        by offset. Returns (frames, total_bytes, last_offset_us) or None."""
        if not os.path.exists(path):
            return None, 0, 0
        with open(path, "rb") as f:
            data = f.read()
        if not data.startswith(RAWLOG_MAGIC):
            return None, 0, 0

        dec = FrameDecoder()
        out = []
        p = len(RAWLOG_MAGIC)
        ts = 0
        while p + 8 <= len(data):
            marker = data[p]
            if marker != 0xAA:
                p += 1
                continue
            ts = int.from_bytes(data[p+1:p+5], "little")
            n = int.from_bytes(data[p+5:p+9], "little")
            if n > MAX_CHUNK:
                break
            payload = data[p+9:p+9+n]
            if len(payload) < n:
                break
            chunk_frames, _text = dec.feed(payload)
            for fr in chunk_frames:
                out.append((ts * 1000, bytearray(fr)))
            p += 9 + n
        out.sort(key=lambda t: t[0])
        return out, len(data), ts * 1000

    # -------------------------------------------------------------- transport
    def _on_play_toggled(self, checked):
        if not self._frames:
            self._play_btn.setChecked(False)
            return
        self._playing = checked
        if checked:
            self._timer.start()
        else:
            self._timer.stop()
        self._apply_play_style()

    def _apply_play_style(self):
        # Green while playing, red while paused/stopped.
        color = "#27ae60" if self._playing else "#e74c3c"
        self._play_btn.setStyleSheet(
            f"font-weight: bold; color: white; background-color: {color};")

    def _on_rewind(self):
        self._offset_us = max(0, self._offset_us - self.REW_STEP_S * 1e6)
        self._sync_after_seek()
        if self._offset_us == 0 and self._playing:
            self._last_idx = 0
            self._send_up_to(0)

    def _on_ff(self):
        self._offset_us = min(self._total_us, self._offset_us + self.FF_STEP_S * 1e6)
        self._sync_after_seek()
        if self._offset_us >= self._total_us and self._playing:
            self._playing = False
            self._play_btn.setChecked(False)

    def _speed(self) -> float:
        return self._speed_slider.value() / 100.0

    def _on_speed_changed(self):
        self._speed_label.setText(f"{self._speed():.2f}x")

    def _on_scrub(self, value):
        if not self._frames:
            return
        scaled = value / 2.0   # slider ms -> offset us (so we survive 2 ms ticks)
        self._seek_to(int(scaled * 1000))

    def _seek_to(self, target_us):
        target_us = max(0, min(self._total_us, target_us))
        self._offset_us = target_us
        self._last_idx = self._first_index_at_or_after(target_us)
        self._update_slider_value()
        self._update_time_label()

    def _sync_after_seek(self):
        self._last_idx = self._first_index_at_or_after(self._offset_us)
        self._update_slider_value()
        self._update_time_label()

    def _first_index_at_or_after(self, target_us):
        idx = 0
        for i, (off, _fr) in enumerate(self._frames):
            if off >= target_us:
                break
            idx = i + 1
        return min(idx, len(self._frames))

    # --------------------------------------------------------------- ticking
    def _on_tick(self):
        if not self._frames:
            self._playing = False
            self._play_btn.setChecked(False)
            return
        self._offset_us += self._speed() * self.TICK_MS * 1000
        if self._offset_us >= self._total_us:
            self._offset_us = self._total_us
            self._playing = False
            self._play_btn.setChecked(False)
        self._send_up_to(self._offset_us)
        self._update_slider_value()
        self._update_time_label()

    def _send_up_to(self, target_us):
        """Send every frame whose offset is <= target_us. Idempotent: rewind
        re-sends already-played frames (process_packet re-writes the same
        display values)."""
        frames = self._frames
        n = len(frames)
        while self._last_idx < n and frames[self._last_idx][0] <= target_us:
            _off, fr = frames[self._last_idx]
            self.main.process_packet(fr)
            self._last_idx += 1

    # ---------------------------------------------------------------- widgets
    def _update_slider_limits(self):
        self._slider.setRange(0, max(1, int(self._total_us / 500)))
        self._update_slider_value()

    def _update_slider_value(self):
        if self._total_us > 0:
            self._slider.blockSignals(True)
            self._slider.setValue(int(self._offset_us / 500))
            self._slider.blockSignals(False)

    def _update_time_label(self):
        def fmt(us):
            s = us / 1e6
            return f"{int(s // 60):02d}:{s % 60:04.1f}"
        self._time_label.setText(f"{fmt(self._offset_us)} / {fmt(self._total_us)}")

    def closeEvent(self, event):
        # Replay ends: clear the raw-log veto. The serial connection stays
        # down until the user reconnects; never re-open the FC link.
        main = getattr(self, "main", None)
        if main is not None:
            main.replaying = False
            if hasattr(main, "raw_logger"):
                main.raw_logger.forbidden = False
        self._playing = False
        self._timer.stop()
        event.accept()