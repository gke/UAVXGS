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


class ImuStatsLogger:
    """Appends IMU Stats benchmark responses (tag 74, testId=2) to a CSV.

    Each burst boundary emits a live snapshot (phase 2) and a per-burst
    Welford summary (phase 4); the run ends with a final roll-up (phase
    255). One row per packet lets the burst sequence and NaN statistics
    be analyzed afterward. Intended for bench diagnosis: flat hold and
    fixed banks.
    """

    HEADERS = [
        "local_time", "phase", "run", "samples",
        "accBF_m", "accLR_m", "accUD_m", "rateR", "rateP", "rateY",
        "angR", "angP", "angY", "accMag", "temp", "nan_burst",
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
        path = os.path.join(self.base_dir, f"{ts}_imu_stats.log")
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADERS)
        self.file.flush()

    def log(self, phase, d):
        """One row for a testId=2 phase 2/4/255 packet (d = data[16]).

        Wire order differs by phase (see tests.c PollIMUStats):
          phase 2 live snapshot: v1=samples-in-run v2=run#
          phase 4 burst summary: v1=run# v2=samples
          phase 255 final:       v1=runs done v2=total samples
        """
        self._ensure_open()
        if phase == 2:
            samples, run = d[1], d[2]
        elif phase == 4:
            run, samples = d[1], d[2]
        else:  # 255 final
            run, samples = d[1], d[2]
        self.writer.writerow([
            datetime.now().isoformat(timespec="seconds"),
            phase,
            int(run) if len(d) > 1 else 0,
            int(samples) if len(d) > 2 else 0,
            int(d[3]) if len(d) > 3 else 0,   # accBF
            int(d[4]) if len(d) > 4 else 0,   # accLR
            int(d[5]) if len(d) > 5 else 0,   # accUD
            int(d[6]) if len(d) > 6 else 0,   # rateR
            int(d[7]) if len(d) > 7 else 0,   # rateP
            int(d[8]) if len(d) > 8 else 0,   # rateY
            int(d[9]) if len(d) > 9 else 0,   # angR
            int(d[10]) if len(d) > 10 else 0, # angP
            int(d[11]) if len(d) > 11 else 0, # angY
            int(d[12]) if len(d) > 12 else 0, # accMag
            int(d[13]) if len(d) > 13 else 0, # temp
            int(d[14]) if len(d) > 14 else 0, # nan this burst / total
        ])
        self.file.flush()

    def close(self):
        if self.file:
            self.file.close()
            self.file = None
            self.writer = None
