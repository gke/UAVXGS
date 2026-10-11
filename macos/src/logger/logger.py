# logging/logger.py
"""
Anomaly and IMU diagnostics CSV logging.
The legacy CSV flight-log (Logger) and KML track (GpsKmlLogger) were removed:
the always-on raw telemetry log (logger.rawlog) supersedes both.
"""

import csv
import os
from datetime import datetime


class AnomalyLogger:
    """Appends live IMU I2C-anomaly records (tag 74, testId=0xff) to a CSV.

    Each fault gets one timestamped row so the flight-state sequence and
    IMU/rate/temperature values at the glitch can be analyzed afterward.
    """

    HEADERS = [
        "local_time", "fc_tick_uS", "FlightState", "NavState", "I2CErrors",
        "AccBF_mG", "AccIR_mG", "AccUD_mG", "RateR_mrads", "RateP_mrads",
        "RateY_mrads", "Temp_mdegC", "Tran0", "Tran1", "Tran2", "len", "head",
        "imuSel", "busNo",
    ]

    def __init__(self, base_dir: str = None):
        if base_dir is None:
            base_dir = os.path.expanduser("~/UAVXLogs")
        self.base_dir = base_dir
        os.makedirs(base_dir, exist_ok=True)
        self.file = None
        self.writer = None

    def _ensure_open(self):
        if self.writer is not None:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.base_dir, f"{ts}_anomalies.log")
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADERS)
        self.file.flush()

    def log_anomaly(self, tick, state, navstate, errors, acc_bf, acc_lr,
                    acc_ud, rate_r, rate_p, rate_y, temp, tr0, tr1, tr2,
                    packed_diag):
        self._ensure_open()
        packed = int(packed_diag or 0)
        length = (packed >> 24) & 0xff
        head = packed & 0xff
        imu_sel = (packed >> 16) & 0xff
        bus_no = (packed >> 8) & 0xff
        self.writer.writerow([
            datetime.now().isoformat(timespec="milliseconds"),
            tick, state, navstate, errors,
            acc_bf, acc_lr, acc_ud, rate_r, rate_p, rate_y, temp,
            tr0, tr1, tr2, length, head, imu_sel, bus_no,
        ])
        self.file.flush()

    def close(self):
        if self.file:
            self.file.close()
            self.file = None
            self.writer = None
