# core/frame_decoder.py
"""
Shared UAVX wire-stream decoder.

The FC telemetry stream interleaves ESC-stuffed binary packets (SOH 0x01 ..
EOT 0x04, bytes in {0x01,0x04,0x1B} prefixed with ESC 0x1B) with plain-text
diagnostics (boot messages, encoded motor-trace CSV). This class is the
single byte-level state machine that both the live serial thread and the
logfile replay run through, so replay is decoded byte-identically to a real
connection.

Only the raw-input backlog, the in-progress frame and a parked ESC flag
survive between feed() calls, matching the original serial-loop behaviour
where a frame split across two serial reads reassembles correctly.

Conventions: this mirrors the loop originally inlined in
`ui/main_window.py::TelemetryThread.run()`; the extraction is behaviour-
preserving (frame bytes emitted on EOT are identical).
"""

__all__ = ["FrameDecoder"]


SOH = 0x01
EOT = 0x04
ESC = 0x1B


class FrameDecoder:
    """Feeds a raw byte chunk in, yields completed frames and printable text.

    - `feed(data)` returns a `(frames, text_lines)` tuple. Every `frames`
      element is the complete unstuffed `[SOH .. checksum]` byte string that
      `process_packet`/`parse_packet` expect; `text_lines` holds decoded
      printable-ASCII runs that arrived outside packet framing (a `None`
      entry marks a discarded non-printable run).
    - Frame emission matches the old serial thread exactly: EOT is the
      terminator only, never part of the emitted bytes.
    """

    def __init__(self):
        self._buffer = bytearray()
        self._packet = bytearray()
        self._esc_flag = False

    def feed(self, data):
        frames = []
        text = []
        self._buffer.extend(data)

        i = 0
        while i < len(self._buffer):
            ch = self._buffer[i]

            if self._esc_flag:
                self._packet.append(ch)
                self._esc_flag = False
                i += 1
                continue

            if ch == ESC:
                self._esc_flag = True
                i += 1
                continue

            if ch == SOH:
                if len(self._packet) > 3:
                    txt = bytes(self._packet).decode("ascii", "replace")
                    printable = all(32 <= b < 127 or b in (10, 13)
                                    for b in self._packet)
                    if printable:
                        text.append(txt.strip())
                    else:
                        text.append(None)  # discarded non-printable run
                self._packet.clear()
                self._packet.append(ch)
                i += 1
                continue

            if ch == EOT:
                if len(self._packet) >= 3:
                    frames.append(bytes(self._packet))
                self._packet.clear()
                i += 1
                continue

            self._packet.append(ch)
            i += 1

        del self._buffer[:i]  # keep only the not-yet-consumed tail
        return frames, text