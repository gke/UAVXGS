# widgets/anomaly_box.py
"""
Anomalies box — IMU health + per-device SIO error counts.

IMU rejects and exec timings come from the execution-time packet (Tag=67);
per-device SIO error counts come from UAVXI2CErrorsPacket (Tag=76).
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QGroupBox, QGridLayout, QLabel


class AnomalyBox(QGroupBox):
    """Compact read-only box: IMU rejects, SIO error counts."""

    def __init__(self, parent=None):
        super().__init__("I2C/SPI Anomalies", parent)
        self.setStyleSheet(
            "QGroupBox { font-weight: bold; border: 1px solid black; "
            "border-radius: 4px; margin-top: 6px; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 6px; "
            "padding: 0 3px 0 3px; }")

        lay = QGridLayout()
        lay.setHorizontalSpacing(4)
        lay.setVerticalSpacing(3)
        lay.setColumnMinimumWidth(0, 55)
        lay.setColumnMinimumWidth(1, 50)
        lay.setColumnMinimumWidth(2, 50)
        lay.setColumnMinimumWidth(3, 50)

        # ── IMU rejects / Exec Time (Tag=67) ──
        lay.addWidget(QLabel("IMU Rej"), 0, 0)
        self.imu_rejects = QLabel("0")
        self.imu_rejects.setStyleSheet("font-weight: bold;")
        self.imu_rejects.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.imu_rejects, 0, 1)

        lay.addWidget(QLabel("Avg Exec µs"), 1, 0)
        self.exec_avg = QLabel("0µs")
        self.exec_avg.setStyleSheet("font-weight: bold;")
        self.exec_avg.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.exec_avg, 1, 1)
        lay.addWidget(QLabel("Peak Exec µs"), 1, 2)
        self.exec_peak = QLabel("0µs")
        self.exec_peak.setStyleSheet("font-weight: bold;")
        self.exec_peak.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.exec_peak, 1, 3)

# ── Per-device SIO error counts + avg/peak uS (Tag=76) ──
        for col, txt in ((1, "Errors"), (2, "Avg uS"), (3, "Peak uS")):
            hdr = QLabel(txt)
            hdr.setAlignment(Qt.AlignCenter)
            lay.addWidget(hdr, 2, col)
        dev_lines = [
            ("IMU", "sio_imu", "sio_imu_avg", "sio_imu_peak"),
            ("Mag", "sio_mag", "sio_mag_avg", "sio_mag_peak"),
            ("Baro", "sio_baro", "sio_baro_avg", "sio_baro_peak"),
        ]
        for row, (name, cnt_attr, avg_attr, peak_attr) in enumerate(dev_lines, start=3):
            name_lbl = QLabel(name)
            lay.addWidget(name_lbl, row, 0)
            if name == "Mag":
                self.sio_mag_name = name_lbl
            lbl = QLabel("0")
            lbl.setStyleSheet("font-weight: bold;")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setMinimumWidth(35)
            lay.addWidget(lbl, row, 1)
            setattr(self, cnt_attr, lbl)

            for col, val_attr in ((2, avg_attr), (3, peak_attr)):
                t_lbl = QLabel("-")
                t_lbl.setStyleSheet("font-weight: bold;")
                t_lbl.setAlignment(Qt.AlignCenter)
                t_lbl.setMinimumWidth(35)
                lay.addWidget(t_lbl, row, col)
                setattr(self, val_attr, t_lbl)

        self.mag_fault_name = QLabel("Mag fault")
        lay.addWidget(self.mag_fault_name, 6, 0)
        self.mag_fault = QLabel("-")
        self.mag_fault.setStyleSheet("font-weight: bold;")
        self.mag_fault.setAlignment(Qt.AlignLeft)
        lay.addWidget(self.mag_fault, 6, 1, 1, 3)

        self.setLayout(lay)
        self._alert = False
        self._mag_absent = False

    def set_mag_absent(self):
        """Retire the Mag row and Mag fault row once absence is confirmed.

        Called once, from the main window's latched absence path; the row
        would otherwise keep showing zero counts and zero timings for a
        device that is not fitted, and probe-NACK SIO errors logged at
        boot by a magless board would read as a live fault.
        """
        self._mag_absent = True
        for w in (self.sio_mag_name, self.sio_mag, self.sio_mag_avg,
                  self.sio_mag_peak, self.mag_fault_name, self.mag_fault):
            w.hide()

    def update_exec(self, imu_rejects: int, exec_avg_us: int = 0, exec_peak_us: int = 0):
        """Refresh the IMU-reject / exec time fields from the execution-time packet."""
        self.imu_rejects.setText(str(imu_rejects))
        self.exec_avg.setText(f"{exec_avg_us}µs")
        self.exec_peak.setText(f"{exec_peak_us}µs")

    def update_sio_counts(self, d):
        """Refresh the per-device error counters + avg/peak uS from Tag=76."""
        self.sio_imu.setText(str(d.imu_errors))
        self.sio_mag.setText(str(d.mag_errors))
        self.sio_baro.setText(str(d.baro_errors))
        self.sio_imu_avg.setText(str(d.imu_avg_us))
        self.sio_imu_peak.setText(str(d.imu_peak_us))
        self.sio_mag_avg.setText(str(d.mag_avg_us))
        self.sio_mag_peak.setText(str(d.mag_peak_us))
        self.sio_baro_avg.setText(str(d.baro_avg_us))
        self.sio_baro_peak.setText(str(d.baro_peak_us))
        reason = getattr(d, "mag_fail_reason", 0)
        if reason:
            self.mag_fault.setText(d.mag_fail_reason_name)
            self.mag_fault.setStyleSheet("font-weight: bold; color: #c00000;")
        else:
            self.mag_fault.setText("-")
            self.mag_fault.setStyleSheet("font-weight: bold;")
        mag_fault = (not self._mag_absent) and (d.mag_errors or reason)
        if d.imu_errors or d.baro_errors or mag_fault:
            self.set_alert(True)

    def set_alert(self, on: bool):
        """Sticky orange border once any I2C fault has been seen."""
        if on == self._alert:
            return
        self._alert = on
        if on:
            self.setStyleSheet(
                "QGroupBox { font-weight: bold; border: 3px solid #ff7f00; "
                "border-radius: 4px; margin-top: 6px; } "
                "QGroupBox::title { subcontrol-origin: margin; left: 6px; "
                "padding: 0 3px 0 3px; color: #ff7f00; }")
        else:
            self.setStyleSheet(
                "QGroupBox { font-weight: bold; border: 1px solid black; "
                "border-radius: 4px; margin-top: 6px; } "
                "QGroupBox::title { subcontrol-origin: margin; left: 6px; "
                "padding: 0 3px 0 3px; }")
