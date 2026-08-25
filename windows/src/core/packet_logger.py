# core/packet_logger.py

import os
from datetime import datetime
from typing import Any


_TAG_NAMES = {
    13: "FLIGHT", 14: "NAV", 15: "STATS", 16: "CONTROL", 17: "PARAM",
    18: "MIN", 19: "ORIGIN", 20: "WP", 21: "MISSION", 22: "RC",
    50: "REQUEST", 51: "ACK", 52: "MISC", 53: "UNUSED", 54: "BB",
    55: "UNUSED", 56: "UNUSED", 57: "TUNING", 58: "UNUSED", 59: "GUIDANCE",
    60: "UNUSED", 61: "UNUSED", 62: "CALIB", 63: "AFNAME",
    64: "WIND", 65: "UNUSED", 66: "SERIAL", 67: "EXEC",
    68: "UNUSED", 69: "LINK",
}


def _fmt(val: Any) -> str:
    if isinstance(val, float):
        return f"{val:.4f}"
    if isinstance(val, bytes):
        return val.hex()
    if isinstance(val, list):
        return ",".join(str(x) for x in val[:12])
    return str(val)


def _obj_fields(obj: Any) -> str:
    if obj is None:
        return "(empty)"
    if isinstance(obj, (bytes, bytearray)):
        return obj.hex()
    if isinstance(obj, dict):
        return "  ".join(f"{k}={_fmt(v)}" for k, v in obj.items())
    if not hasattr(obj, "__dict__") and not hasattr(type(obj), "__dataclass_fields__"):
        return str(obj)
    parts = []
    for attr in sorted(dir(obj)):
        if attr.startswith("_"):
            continue
        v = getattr(obj, attr)
        if callable(v):
            continue
        if isinstance(v, (int, float, str, bytes, list)):
            parts.append(f"{attr}={_fmt(v)}")
    return "  ".join(parts) if parts else "(empty)"


class PacketLogger:
    def __init__(self):
        self.src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.log_file = os.path.join(self.src_dir, "log.txt")

    def log(self, direction: str, tag: int, data: Any) -> None:
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        name = _TAG_NAMES.get(tag, f"TAG{tag}")
        entry = _obj_fields(data) if data is not None else ""
        line = f"{ts} {direction} {name}  {entry}" if entry else f"{ts} {direction} {name}"
        try:
            with open(self.log_file, 'a') as f:
                f.write(line + '\n')
                f.flush()
        except Exception:
            pass

    def log_received(self, tag: int, tag_name: str, data: Any) -> None:
        self.log("R", tag, data)

    def log_sent(self, tag: int, tag_name: str, data: Any = None) -> None:
        self.log("T", tag, data)


packet_logger = PacketLogger()
