# widgets/anomaly_box.py
"""
Anomalies box — IMU/WDT health + per-device SIO error counts.

IMU rejects and the WDT trip mark come from the execution-time packet
(Tag=67); per-device SIO error counts come from UAVXI2CErrorsPacket (Tag=76).
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QGroupBox, QGridLayout, QLabel


class AnomalyBox(QGroupBox):
    """Compact read-only box: IMU rejects, WDT, SIO error counts."""

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

        # ── IMU rejects / WDT / Exec Time (Tag=67) ──
        lay.addWidget(QLabel("IMU Rej"), 0, 0)
        self.imu_rejects = QLabel("0")
        self.imu_rejects.setStyleSheet("font-weight: bold;")
        self.imu_rejects.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.imu_rejects, 0, 1)
        lay.addWidget(QLabel("WDT"), 0, 2)
        self.wdt_mark = QLabel("0")
        self.wdt_mark.setStyleSheet("font-weight: bold;")
        self.wdt_mark.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.wdt_mark, 0, 3)

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
            lay.addWidget(QLabel(name), row, 0)
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

        self.setLayout(lay)
        self._alert = False

    def update_exec(self, imu_rejects: int, wdt_mark: int, exec_avg_us: int = 0, exec_peak_us: int = 0):
        """Refresh the IMU-reject / WDT / exec time fields from the execution-time packet."""
        self.imu_rejects.setText(str(imu_rejects))
        self.wdt_mark.setText(str(wdt_mark))
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
        if d.imu_errors or d.mag_errors or d.baro_errors:
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
