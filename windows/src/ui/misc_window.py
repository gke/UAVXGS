# ui/misc_window.py

from typing import Any, Optional
from datetime import datetime
from dataclasses import fields, is_dataclass

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from protocol_enums import FlightState, NavState


_TAG_NAMES = {
    15: "Stats", 21: "Mission", 50: "Request", 52: "Misc", 53: "Unused",
    54: "BB", 55: "Unused", 56: "Unused", 57: "Tuning", 58: "Unused",
    59: "Guidance", 60: "Unused", 61: "Unused", 64: "Wind", 65: "Unused",
    67: "ExecTime", 68: "Unused", 73: "TestReq", 74: "TestRsp",
}

# Test selector entries: (label, test_id). testId matches FC tests.h TEST_ID_*.
# TEST_ID_I2C_FAULT (fault injection) was removed from the FC 2026-08-09.
TEST_SELECTIONS = [
    ("I2C Bus / IMU Bus", 0),
    ("Magnetometer Bus", 1),
    ("IMU Stats (Madgwick)", 2),
]

# Per-test vector meaning. data[0] = phase id, data[0] = sensor lane or
# counters at [1..3] and, when present, a live sensor snapshot in the
# high lane [8..10] (mag) or [8..14] (IMU). Matches FC tests.c.
#
# I2C (test 0) emitted phases:
#   phase 3  hammer isx : str      vec[1]=iter vec[2]=i2cerrΔ vec[3]=glitch
#                         sensor   vec[8..14]=acc X/Y/Z rate X/Y/Z temp
#   phase 4  done
#   phase 255 final
# MAG (test 1) emitted phases:
#   phase 2  hammer isr running    vec[1]=iter vec[2]=i2cerrΔ vec[3]=glitch
#   phase 4/255 done
TEST_FIELD_TABLE = {
    0: {
        3: [("iter", 1), ("i2cerrΔ", 2), ("glitch", 3)],
        4: [("iter", 1), ("i2cerrΔ", 2), ("glitch", 3)],
        255: [("iter", 1), ("i2cerrΔ", 2), ("glitch", 3)],
    },
    1: {
        2: [("iter", 1), ("i2cerrΔ", 2), ("glitch", 3)],
        4: [("iter", 1), ("i2cerrΔ", 2), ("glitch", 3), ("good", 4)],
        255: [("iter", 1), ("i2cerrΔ", 2), ("glitch", 3), ("good", 4)],
    },
    # IMU Stats (test 2):
    #   phase 2 burst snapshot  v1=samples v2=run v3-5=Acc(BF,LR,UD)*1000
    #                 v6-8=Rate(Roll,Pitch,Yaw)*1000 v9-11=Angle(Roll,Pitch,Yaw)*100
    #                 v12=AccMag*1000 v13=temp*1000 v14=NaN count
    #   phase 4 burst summary v1=run v2=samples v3=NaN v4-6=Acc mean
    #                 v7-9=Acc std v10-12=Angle mean(deg*100) v13=Acc|mean|
    #                 v14=Angle std  (angle order Pitch,Roll,Yaw = mean[6..8])
    #   phase 255 final v1=runs v2=total samples v3=total NaN
    2: {
        2: [("samples", 1), ("run", 2), ("accBF", 3), ("accLR", 4),
            ("accUD", 5), ("rR", 6), ("rP", 7), ("rY", 8), ("angR", 9),
            ("angP", 10), ("angY", 11), ("accMag", 12), ("temp", 13),
            ("NaN", 14)],
        4: [("run", 1), ("samples", 2), ("NaN", 3), ("accBm", 4),
            ("accLm", 5), ("accUm", 6), ("accBs", 7), ("accLs", 8),
            ("accUs", 9), ("angPm", 10), ("angRm", 11), ("angYm", 12),
            ("accM", 13), ("angPs", 14)],
        255: [("runs", 1), ("samples", 2), ("NaNtot", 3)],
    },
    # I2C Fault / Recovery (test 3):
    #   phase 1 proof      v1=good reads v2=i2cerrΔ (must be 0) v3=bad
    #   phase 2 inject     v1=inject# v2=i2cerrΔ total v3=bad v4=recoveries
    #   phase 4/255 done/final  same as phase 2
    3: {
        1: [("good", 1), ("i2cerrΔ", 2), ("bad", 3)],
        2: [("inject", 1), ("i2cerrΔ", 2), ("bad", 3), ("recover", 4)],
        4: [("inject", 1), ("i2cerrΔ", 2), ("bad", 3), ("recover", 4)],
        255: [("inject", 1), ("i2cerrΔ", 2), ("bad", 3), ("recover", 4)],
    },
}

# Sensor snapshot lane base offsets per test id. IMU test packs 7 int16
# (accX,accY,accZ,rateX,rateY,rateZ,temp) at vec[8..]; mag packs axisX,
# axisY, axisZ at vec[8..10].
TEST_SENSOR_LABELS = {
    0: ["accX", "accY", "accZ", "rX", "rY", "rZ", "temp"],
    1: ["magX", "magY", "magZ"],
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
        self.setMinimumSize(1320, 550)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Test selector bar
        test_bar = QHBoxLayout()
        test_bar.addWidget(QLabel("Test:"))
        self.test_combo = QComboBox()
        for label, _tid in TEST_SELECTIONS:
            self.test_combo.addItem(label)
        test_bar.addWidget(self.test_combo)
        self.run_test_btn = QPushButton("Run")
        self.run_test_btn.clicked.connect(self._run_test)
        test_bar.addWidget(self.run_test_btn)
        self.abort_test_btn = QPushButton("Abort")
        self.abort_test_btn.setEnabled(False)
        self.abort_test_btn.clicked.connect(self._abort_test)
        test_bar.addWidget(self.abort_test_btn)
        self.free_run_cb = QCheckBox("auto-restart on done")
        self.free_run_cb.setChecked(True)
        test_bar.addWidget(self.free_run_cb)
        self.test_status = QLabel("idle")
        test_bar.addWidget(self.test_status, 1)
        layout.addLayout(test_bar)

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

        # Live decode of test responses for the active diagnostic.
        if tag == 74 and hasattr(data, 'test_id'):
            self._render_test(data)

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

    # ------------------------------------------------------------------
    # Diagnostic test control
    # ------------------------------------------------------------------
    def _run_test(self):
        idx = self.test_combo.currentIndex()
        test_id = TEST_SELECTIONS[idx][1]
        mw = self.parent_window
        if mw and hasattr(mw, 'send_test'):
            mw.send_test(test_id, 0)
            self.run_test_btn.setEnabled(False)
            self.abort_test_btn.setEnabled(True)
            self.test_status.setText(
                f"{TEST_SELECTIONS[idx][0]} — running (disarm required)")
        return None

    def _abort_test(self):
        mw = self.parent_window
        if mw and hasattr(mw, 'send_test'):
            mw.send_test(self._current_test_id(), 1)
        self._finish_test("aborted")

    def _current_test_id(self):
        idx = self.test_combo.currentIndex()
        return TEST_SELECTIONS[idx][1]

    def _render_test(self, data):
        tid = data.test_id
        phase = data.phase

        # Live-flight I2C anomaly record (PollDiag). Overrides the test
        # field-table path entirely: decode the fixed layout.
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

        table = TEST_FIELD_TABLE.get(tid)
        if table is None:
            self.test_status.setText(f"test {tid} phase {phase}")
            return
        fields = table.get(phase, [])
        phasename = {0: 'announced', 2: 'hammering', 3: 'hammering',
                     4: 'done'}.get(phase, str(phase))
        parts = [f"phase {phasename}"]
        if data.data and data.data[0] < 0:
            parts.append("REFUSED (armed)")
            self._finish_test()
        else:
            for label, idx in fields:
                if idx < len(data.data):
                    parts.append(f"{label}={int(data.data[idx])}")
            # Live sensor snapshot: high lane vec[8..] renders at each
            # checkpoint so real IMU/mag values can be watched as the
            # test proceeds (not just counters).
            slabels = TEST_SENSOR_LABELS.get(tid)
            if slabels and data.data:
                sn = [int(data.data[8 + i]) for i in range(len(slabels))
                      if 8 + i < len(data.data)]
                if any(sn):   # any nonzero — a real capture happened
                    for i, lbl in enumerate(slabels):
                        if 8 + i < len(data.data):
                            parts.append(f"{lbl}="
                                         + str(int(data.data[8 + i])))
            if phase == 255:
                # Re-arm is handled centrally in main_window (case 74) so it
                # works whether or not this panel is visible; do not send here.
                self.test_status.setText(
                    "auto-restart handled by main window")
        self.test_status.setText("  ".join(parts))
        return None

    def _finish_test(self, note="done"):
        self.run_test_btn.setEnabled(True)
        self.abort_test_btn.setEnabled(False)
        if note:
            self.test_status.setText(f"[{note}]")
