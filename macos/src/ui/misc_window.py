# ui/misc_window.py

from typing import Any, Optional
from datetime import datetime
from dataclasses import fields, is_dataclass

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from protocol_enums import FlightState, NavState


class _WrappedLabel(QLabel):
    """QLabel whose sizeHint reflects the actual wrapped-text height."""

    def __init__(self, text="", parent=None):
        super().__init__(text, parent)
        self.setWordWrap(True)
        self.setTextInteractionFlags(Qt.TextSelectableByMouse)

    def sizeHint(self):
        fm = self.fontMetrics()
        w = self.width() if self.width() > 0 else 200
        rect = fm.boundingRect(0, 0, w, 10000,
                               Qt.TextWordWrap, self.text())
        return QSize(rect.width() + 8, rect.height() + 4)


_TAG_NAMES = {
    15: "Stats", 21: "Mission", 50: "Request", 52: "Misc", 53: "Unused",
    54: "BB", 55: "Unused", 56: "Unused", 57: "Tuning", 58: "Unused",
    59: "Guidance", 60: "Unused", 61: "Unused", 64: "Wind", 65: "Unused",
    67: "ExecTime", 68: "Unused", 74: "TestRsp",
    76: "I2CErrors",
}

# The cooperative diagnostic test suite (tag 73 I2C-scan / mag-hammer /
# IMU-stats, its testId/phase field tables and run controls) was stripped
# 2026-09-24 (FC + GCS). Tag 74 remains and now carries ONLY PollDiag's
# correlated live-flight anomaly records (testId=0xff, fixed layout).


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
        self.setMinimumSize(1320, 550)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Live-flight anomaly status (PollDiag tag 74 records, testId=0xff).
        anomaly_bar = QHBoxLayout()
        anomaly_bar.addWidget(QLabel("Anomaly Status:"))
        self.test_status = QLabel("idle")
        anomaly_bar.addWidget(self.test_status, 1)
        layout.addLayout(anomaly_bar)

        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Time", "Tag", "Fields"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setSectionResizeMode(
            QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeToContents)
        layout.addWidget(self.table)

        self._tag_row: dict[int, int] = {}
        self._populate_placeholders()

    @staticmethod
    def _fields_label(text="", color="#000"):
        lbl = _WrappedLabel(text)
        lbl.setStyleSheet(f"color: {color};")
        return lbl

    def _populate_placeholders(self):
        for tag in sorted(_TAG_NAMES):
            if _TAG_NAMES[tag] == "Unused":
                continue
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem("—"))
            tag_name = _TAG_NAMES[tag]
            self.table.setItem(row, 1, QTableWidgetItem(f"R {tag_name}"))
            self.table.setCellWidget(row, 2,
                                     self._fields_label("(waiting)", "#999"))
            self._tag_row[tag] = row

    def add_packet(self, tag: int, data: Any, direction: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        tag_name = _TAG_NAMES.get(tag, f"Tag{tag}")
        header = f"{direction} {tag_name}"
        fields = _decode_fields(tag, data if isinstance(data, bytes) else b"", data)

        # Live decode of PollDiag anomaly responses (the only remaining tag 74
        # payload since the diagnostic test suite was stripped 2026-09-24).
        if tag == 74 and hasattr(data, 'test_id'):
            self._render_test(data)

        if tag in self._tag_row:
            row = self._tag_row[tag]
            self.table.item(row, 0).setText(ts)
            self.table.item(row, 1).setText(header)
            lbl = self.table.cellWidget(row, 2)
            if lbl:
                lbl.setText(fields)
                lbl.setStyleSheet("color: #000;")
        else:
            self.table.insertRow(0)
            if self.table.rowCount() > self.MAX_ROWS:
                self.table.removeRow(self.table.rowCount() - 1)
            self.table.setItem(0, 0, QTableWidgetItem(ts))
            self.table.setItem(0, 1, QTableWidgetItem(header))
            self.table.setCellWidget(0, 2, self._fields_label(fields))

    # ------------------------------------------------------------------
    # PollDiag anomaly display
    # ------------------------------------------------------------------
    def _render_test(self, data):
        tid = data.test_id

        # Live-flight I2C anomaly record (PollDiag). Decode the fixed layout.
        if tid == 0xff:
            d = [int(x) for x in (data.data or [0] * 16)][0:16]
            st = FlightState.get_name(d[2]) if d[2] >= 0 else f"a{d[2]}"
            ns = NavState.get_name(d[3]) if d[3] >= 0 else f"n{d[3]}"
            def _tran(v):
                return (f"{FlightState.get_name((v >> 8) & 0xff)}/"
                        f"{NavState.get_name(v & 0xff)}")
            pkg = int(d[15])
            imu_sel = (pkg >> 16) & 0xff
            bus_no = (pkg >> 8) & 0xff
            parts = ["🔺 ANOMALY",
                     f"FS={st}", f"NS={ns}", f"i2cErr={d[4]}",
                     f"imu#{imu_sel}", f"bus{bus_no}",
                     f"tick={d[1]}",
                     f"acc=({d[5]/1000:.2f},{d[6]/1000:.2f},"
                     f"{d[7]/1000:.2f})",
                     f"rate=({d[8]/1000:.2f},{d[9]/1000:.2f},"
                     f"{d[10]/1000:.2f})",
                     f"temp={d[11]/1000:.1f}",
                     f"trans={_tran(d[12])}({_tran(d[13])}({_tran(d[14])})"]
            self.test_status.setText("  ".join(parts))
            self.test_status.setStyleSheet("color: #ff7f00; font-weight: bold;")
        return None
