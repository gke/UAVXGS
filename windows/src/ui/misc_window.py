# ui/misc_window.py

from typing import Any, Optional
from datetime import datetime
from dataclasses import fields, is_dataclass

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


_TAG_NAMES = {
    15: "Stats", 21: "Mission", 50: "Request", 52: "Misc", 53: "Noise",
    54: "BB", 55: "Inertial", 56: "MinimOSD", 57: "Tuning", 58: "UKF",
    59: "Guidance", 60: "AltCtrl", 61: "Soaring", 64: "Wind", 65: "Track",
    67: "ExecTime", 68: "AttCtrl",
}


def _format_value(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.4f}"
    if isinstance(v, bytes):
        return v.hex()
    if isinstance(v, list):
        return ", ".join(str(x) for x in v[:16])
    return str(v)


def _decode_fields(tag: int, body: bytes, parsed: Any) -> str:
    if parsed is not None and not isinstance(parsed, bytes):
        parts = []
        if is_dataclass(parsed):
            for f in fields(parsed):
                val = getattr(parsed, f.name)
                if not callable(val):
                    parts.append(f"{f.name}={_format_value(val)}")
        else:
            for attr in dir(parsed):
                if attr.startswith("_"):
                    continue
                val = getattr(parsed, attr)
                if callable(val):
                    continue
                if val:
                    parts.append(f"{attr}={_format_value(val)}")
        return "  ".join(parts) if parts else "(empty)"
    return f"raw={body.hex()}" if body else "(empty)"


class MiscWindow(QMainWindow):
    """Displays packets not shown in dedicated windows"""

    MAX_ROWS = 200

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.setWindowTitle("UAVX Misc Packets")
        self.setMinimumSize(1200, 500)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Time", "Tag", "Fields"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        layout.addWidget(self.table)

        self._tag_row: dict[int, int] = {}
        self._populate_placeholders()

    def _populate_placeholders(self):
        for tag in sorted(_TAG_NAMES):
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem("—"))
            tag_name = _TAG_NAMES[tag]
            self.table.setItem(row, 1, QTableWidgetItem(f"R {tag_name}"))
            item = QTableWidgetItem("(waiting)")
            item.setForeground(QColor("#999"))
            self.table.setItem(row, 2, item)
            self._tag_row[tag] = row

    def add_packet(self, tag: int, data: Any, direction: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        tag_name = _TAG_NAMES.get(tag, f"Tag{tag}")
        header = f"{direction} {tag_name}"
        fields = _decode_fields(tag, data if isinstance(data, bytes) else b"", data)

        if tag in self._tag_row:
            row = self._tag_row[tag]
            self.table.item(row, 0).setText(ts)
            self.table.item(row, 1).setText(header)
            self.table.item(row, 2).setText(fields)
            self.table.item(row, 2).setForeground(QColor("#000"))
        else:
            self.table.insertRow(0)
            if self.table.rowCount() > self.MAX_ROWS:
                self.table.removeRow(self.table.rowCount() - 1)
            self.table.setItem(0, 0, QTableWidgetItem(ts))
            self.table.setItem(0, 1, QTableWidgetItem(header))
            self.table.setItem(0, 2, QTableWidgetItem(fields))
