# logger/rawlog.py
"""
Always-on raw telemetry log writer.

Captures every byte that arrives from the serial port, stamped with the
millisecond offset since the file was opened. The FreeFlight-style framing
is deliberately trivial:

    magic  8 bytes  b"UAVXRAW\\x01"
    chunk  12 bytes header + payload, per serial read:
            marker byte 0xAA            (resync anchor)
            u32 LE offset ms since open
            u32 LE payload length
            payload bytes (the exact raw serial read)

The raw stream is the complete UAVX wire format, ESC-stuffed packets AND
printable diagnostics interleaved, exactly as delivered at the byte level.
A GCS replay feeds these same bytes through the shared `FrameDecoder`, so
what is captured is, byte for byte, what a live connection decoded.

Never captures replay output: the writer is driven exclusively from the
serial thread's `raw_bytes` signal, and `write()` additionally refuses
while the GCS is in replay mode (belt and braces so no second log can ever
be produced from a replay).
"""

import os
import re
import time

__all__ = ["RawLogWriter"]


MAGIC = b"UAVXRAW\x01"
MARKER = 0xAA


def _safe_name(name):
    """Sanitize an airframe name into a filename-safe token."""
    clean = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._")
    return clean or "unnamed"


class RawLogWriter:
    def __init__(self, log_dir=None):
        self.log_dir = log_dir or os.path.expanduser("~/UAVX")
        self._file = None
        self._path = None
        self._t0 = None
        self._airframe = ""
        self._forbidden = False  # set while the GCS is replaying a log

    @property
    def forbidden(self):
        return self._forbidden

    @forbidden.setter
    def forbidden(self, value):
        self._forbidden = bool(value)

    # ------------------------------------------------------------------ fmt
    def _default_path(self):
        from datetime import datetime
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = self.log_dir
        return os.path.join(base, f"{ts}.rawlog")

    # ------------------------------------------------------------ lifecycle
    def start(self, path=None, airframe=""):
        """Open a new raw log. Existing open log is closed first (never two
        writers fighting over the same stream)."""
        self.stop()
        if path is None:
            path = self._default_path()
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self._file = open(path, "wb")
        self._file.write(MAGIC)
        self._file.flush()
        self._t0 = time.monotonic()
        self._path = path
        self._airframe = airframe
        if airframe:
            self.rename(airframe)

    def write(self, data):
        """Append one serial read with its ms offset since log start.

        Refuses while `forbidden` (replay mode): the replay path feeds
        `process_packet` directly and never goes through `raw_bytes`, so
        this guard is belt-and-braces against any path that might tee bytes
        into the log from a non-serial source."""
        if self._file is None or self._forbidden:
            return
        try:
            ms = int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF
            payload = bytes(data)
            self._file.write(bytes((MARKER,)))
            self._file.write(ms.to_bytes(4, "little"))
            self._file.write(len(payload).to_bytes(4, "little"))
            self._file.write(payload)
        except OSError:
            self.stop()

    def rename(self, airframe):
        """Rewrite the on-disk filename to include the airframe name once the
        FC's persisted name arrives over tag 63 (it is only known after
        connect). The open handle keeps writing to the same inode."""
        if self._file is None or not airframe:
            return
        safe = _safe_name(airframe)
        if safe == self._airframe:
            return
        self._airframe = safe
        old = self._path
        base = os.path.basename(old)
        stem = base[:-len(".rawlog")] if base.endswith(".rawlog") else base
        new = os.path.join(os.path.dirname(old), f"{stem}_{safe}.rawlog")
        try:
            os.rename(old, new)
            self._path = new
        except OSError:
            pass  # cosmetic; the file itself still receives data

    def stop(self):
        if self._file is not None:
            try:
                self._file.flush()
                self._file.close()
            except OSError:
                pass
            self._file = None
        # `_path` is intentionally kept: callers log the final filename.

    @property
    def active(self):
        return self._file is not None