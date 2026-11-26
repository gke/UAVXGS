# ui/identify_window.py - Identify summaries (tag 57)
"""
Separate window showing the FC's identify data: the raw per-axis ARX rate
estimates and the v4 bounded flight summaries for acceleration confidence,
angle-I/rate-D, mag and baro/altitude-KF behaviour. The GCS reports only what
the FC computed; it never derives plant gain, time constant, proposed Kp or a
tuning recommendation.
"""

import math

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QGridLayout, QGroupBox, QLabel, QMainWindow, QScrollArea, QTabWidget,
    QVBoxLayout, QWidget
)

IDENTIFY_MIN_SAMPLES = 200
IDENTIFY_CONV_SIG = 0.02

_AXES = ("Roll", "Pitch", "Yaw")
_ROWS = (
    ("State", "_ident_state"),
    ("a", "_ident_a"),
    ("b", "_ident_b"),
    ("sigA", "_ident_sa"),
    ("sigB", "_ident_sb"),
    ("n", "_ident_n"),
)
_SUMMARY_GROUPS = (
    ("Rate innovation and low-confidence rate", (
        ("Innovation RMS", "innov_rms", True, "f"),
        ("Innovation max", "innov_max", True, "f"),
        ("Innovation lag-1", "innov_lag1", True, "f"),
        ("Rate at low conf RMS", "rate_low_rms", True, "f"),
        ("Rate at low conf max", "rate_low_max", True, "f"),
    )),
    ("Accelerometer confidence", (
        ("Confidence mean", "acc_conf_mean", False, "f"),
        ("Confidence min", "acc_conf_min", False, "f"),
        ("Confidence max", "acc_conf_max", False, "f"),
        ("Samples", "acc_conf_n", False, "n"),
        ("Low (<0.3)", "acc_conf_low_count", False, "n"),
        ("Mid (0.3-0.7)", "acc_conf_mid_count", False, "n"),
        ("High (>=0.7)", "acc_conf_high_count", False, "n"),
        ("Vertical <-0.7", "acc_dir_low_count", False, "n"),
        ("Vertical -0.7..-0.3", "acc_dir_mid_count", False, "n"),
        ("Vertical >=-0.3", "acc_dir_high_count", False, "n"),
        ("Acc mag mean", "acc_mag_mean", False, "f"),
        ("Acc mag min", "acc_mag_min", False, "f"),
        ("Acc mag max", "acc_mag_max", False, "f"),
        ("Acc mag std dev", "acc_mag_dev_rms", False, "f"),
    )),
    ("Angle integral and rate D", (
        ("IntE RMS", "int_e_rms", True, "f"),
        ("IntE max", "int_e_max", True, "f"),
        ("I-attributable D RMS", "d_int_e_rms", True, "f"),
        ("Total D RMS", "d_total_rms", True, "f"),
        ("Limit hits", "int_e_limit_hits", True, "n"),
        ("Resets", "int_e_resets", True, "n"),
    )),
    ("Magnetometer", (
        ("Filtered innovation RMS", "mag_innov_rms", False, "f"),
        ("Filtered innovation max", "mag_innov_max", False, "f"),
        ("Valid samples", "mag_valid_count", False, "n"),
        ("Yaw-disagree samples", "mag_yaw_disagree_count", False, "n"),
    )),
    ("Barometer and altitude KF", (
        ("Innovation RMS", "baro_innov_rms", False, "f"),
        ("Innovation max", "baro_innov_max", False, "f"),
        ("Innovation lag-1", "baro_innov_lag1", False, "f"),
        ("KF baro variance mean", "kf_baro_var_mean", False, "f"),
        ("KF baro variance max", "kf_baro_var_max", False, "f"),
        ("ROC RMS", "roc_rms", False, "f"),
        ("AltHold comp RMS", "alt_hold_comp_rms", False, "f"),
        ("AltHold comp max", "alt_hold_comp_max", False, "f"),
        ("Samples", "baro_n", False, "n"),
    )),
)
_VALUE_STYLE = "font-weight: bold; font-size: 14px; color: %s;"
_ST_UNKNOWN = _VALUE_STYLE % "#888888"
_ST_TRACK = _VALUE_STYLE % "#a15500"
_ST_CONV = _VALUE_STYLE % "#000000"
_SUM_STYLE = "font-weight: bold; font-size: 12px; color: %s;"
_SUM_UNKNOWN = _SUM_STYLE % "#888888"
_SUM_KNOWN = _SUM_STYLE % "#000000"
_LBL_STYLE = "font-weight: bold; font-size: 12px;"
_TITLE_STYLE = "font-size: 18px; font-weight: bold;"
_GROUP_STYLE = "QGroupBox { font-weight: bold; }"


def _format_value(value, kind):
    if value is None or not math.isfinite(value):
        return "--", False
    if kind == "n":
        if value < 0:
            return "--", False
        return str(int(value)), True
    return "%.6g" % value, True


class IdentifyWindow(QMainWindow):
    """FC-computed identify summaries, one column per axis where applicable."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.setWindowTitle("Identify - Flight Evidence")
        self.setMinimumSize(600, 420)

        central = QWidget()
        self.setCentralWidget(central)
        lay = QVBoxLayout(central)
        lay.setSpacing(8)
        lay.setContentsMargins(8, 8, 8, 8)

        title = QLabel("Identify - Flight Evidence")
        title.setStyleSheet(_TITLE_STYLE)
        title.setAlignment(Qt.AlignCenter)
        lay.addWidget(title)

        tabs = QTabWidget()
        tabs.addTab(self._rate_tab(), "Rate dynamics")
        tabs.addTab(self._summary_tab(), "Flight summaries")
        lay.addWidget(tabs)

        self.resize(720, 560)
        self._clear_all()
        self._clear_summary()

    def _rate_tab(self):
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setSpacing(8)
        lay.setContentsMargins(8, 8, 8, 8)

        group = QGroupBox("ARX rate model")
        group.setStyleSheet(_GROUP_STYLE)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        grid.addWidget(QLabel(""), 0, 0)
        for c, ax in enumerate(_AXES, start=1):
            hdr = QLabel("<b>%s</b>" % ax)
            hdr.setStyleSheet("font-weight: bold; font-size: 14px;")
            hdr.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            grid.addWidget(hdr, 0, c)

        for r, (label, attr) in enumerate(_ROWS, start=1):
            lbl = QLabel("<b>%s</b>" % label)
            lbl.setStyleSheet(_LBL_STYLE)
            grid.addWidget(lbl, r, 0)
            cells = []
            for c in range(1, 4):
                cell = QLabel("--")
                cell.setStyleSheet(_ST_UNKNOWN)
                cell.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                grid.addWidget(cell, r, c)
                cells.append(cell)
            setattr(self, attr, cells)

        group.setLayout(grid)
        lay.addWidget(group)

        self.status = QLabel("waiting for tag 57 (no FC identify data yet)")
        self.status.setStyleSheet("color: #888; font-size: 12px;")
        lay.addWidget(self.status)

        expl = QLabel(
            "<b>a</b> = per-tick rate retention; <b>b</b> = per-tick input "
            "coefficient (FC input is -Out). <b>sigA/sigB</b> are standard "
            "errors; <b>n</b> is sample count. Converged means n\u2265%d and "
            "both sig &lt; %.3f. Values are raw ARX estimates, not plant gain "
            "or time constant." % (IDENTIFY_MIN_SAMPLES, IDENTIFY_CONV_SIG)
        )
        expl.setWordWrap(True)
        expl.setStyleSheet("color: #555; font-size: 12px;")
        lay.addWidget(expl)
        lay.addStretch()
        return tab

    def _summary_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setSpacing(8)
        lay.setContentsMargins(8, 8, 8, 8)

        for title, rows in _SUMMARY_GROUPS:
            group = QGroupBox(title)
            group.setStyleSheet(_GROUP_STYLE)
            grid = QGridLayout()
            grid.setHorizontalSpacing(10)
            grid.setVerticalSpacing(6)
            grid.addWidget(QLabel(""), 0, 0)
            cols = len(_AXES) if rows[0][2] else 1
            labels = _AXES if cols == len(_AXES) else ("Value",)
            for c, label in enumerate(labels, start=1):
                hdr = QLabel("<b>%s</b>" % label)
                hdr.setStyleSheet("font-weight: bold; font-size: 12px;")
                hdr.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                grid.addWidget(hdr, 0, c)

            for r, (label, key, per_axis, _) in enumerate(rows, start=1):
                lbl = QLabel("<b>%s</b>" % label)
                lbl.setStyleSheet(_LBL_STYLE)
                grid.addWidget(lbl, r, 0)
                cells = []
                for c in range(1, cols + 1):
                    cell = QLabel("--")
                    cell.setStyleSheet(_SUM_UNKNOWN)
                    cell.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                    grid.addWidget(cell, r, c)
                    cells.append(cell)
                setattr(self, "_sum_" + key, cells)

            group.setLayout(grid)
            lay.addWidget(group)

        self.summary_status = QLabel(
            "waiting for the v4 summary on tag 57")
        self.summary_status.setStyleSheet("color: #888; font-size: 12px;")
        lay.addWidget(self.summary_status)

        expl = QLabel(
            "Innovation RMS is computed onboard as sqrt(SSE/(n-2)). sigA/sigB "
            "are parameter standard errors, not noise measurements. The mag "
            "innovation is the FC's filtered signal and reads lower than the "
            "unfiltered rate and baro figures; KF baro variance is the "
            "measurement variance the filter used. Values are bounded flight "
            "summaries; the GCS derives no gains or tuning recommendations."
        )
        expl.setWordWrap(True)
        expl.setStyleSheet("color: #555; font-size: 12px;")
        lay.addWidget(expl)
        lay.addStretch()

        scroll.setWidget(inner)
        return scroll

    def _clear_all(self):
        for _, attr in _ROWS:
            for cell in getattr(self, attr):
                cell.setText("--")
                cell.setStyleSheet(_ST_UNKNOWN)

    def _clear_summary(self):
        for _, rows in _SUMMARY_GROUPS:
            for _, key, _, _ in rows:
                for cell in getattr(self, "_sum_" + key):
                    cell.setText("--")
                    cell.setStyleSheet(_SUM_UNKNOWN)

    def _refresh(self, tune):
        try:
            if tune is None or not (
                    getattr(tune, "has_ident", False)
                    or getattr(tune, "has_summary", False)):
                self._clear_all()
                self._clear_summary()
                self.status.setText("waiting for the FC identify block on tag 57")
                self.status.setStyleSheet("color: #888; font-size: 12px;")
                self.summary_status.setText(
                    "waiting for the v4 summary on tag 57")
                self.summary_status.setStyleSheet(
                    "color: #888; font-size: 12px;")
                return
            if getattr(tune, "has_ident", False):
                self._do_refresh(tune)
            else:
                self._clear_all()
                self.status.setText("waiting for the FC identify block on tag 57")
                self.status.setStyleSheet("color: #888; font-size: 12px;")
            self._refresh_summary(tune)
        except Exception as exc:
            self.status.setText("error: %s" % exc)
            self.status.setStyleSheet("color: red; font-size: 12px;")

    def _do_refresh(self, tune):
        ident = (
            (tune.ident_a_roll, tune.ident_b_roll,
             tune.ident_sig_a_roll, tune.ident_sig_b_roll,
             tune.ident_n_roll),
            (tune.ident_a_pitch, tune.ident_b_pitch,
             tune.ident_sig_a_pitch, tune.ident_sig_b_pitch,
             tune.ident_n_pitch),
            (tune.ident_a_yaw, tune.ident_b_yaw,
             tune.ident_sig_a_yaw, tune.ident_sig_b_yaw,
             tune.ident_n_yaw),
        )
        converged = []
        for r, (a, b, sa, sb, n) in enumerate(ident):
            known = (n > 0 and all(math.isfinite(v) for v in (a, b, sa, sb)))
            ok = (known and n >= IDENTIFY_MIN_SAMPLES
                  and sa < IDENTIFY_CONV_SIG and sb < IDENTIFY_CONV_SIG)
            if ok:
                converged.append(_AXES[r])
            style = _ST_CONV if ok else (_ST_TRACK if known else _ST_UNKNOWN)
            values = (
                ("Converged" if ok else "Tracking") if known else "--",
                a if known else None,
                b if known else None,
                sa if known else None,
                sb if known else None,
                n if known else None,
            )
            for (_, attr), value in zip(_ROWS, values):
                cell = getattr(self, attr)[r]
                if value is None:
                    cell.setText("--")
                elif attr == "_ident_n":
                    cell.setText(str(int(value)))
                elif attr in ("_ident_sa", "_ident_sb"):
                    cell.setText("%.6f" % value)
                elif attr == "_ident_state":
                    cell.setText(value)
                else:
                    cell.setText("%.4f" % value)
                cell.setStyleSheet(style)

        if converged:
            self.status.setText("Converged: %s" % ", ".join(converged))
            self.status.setStyleSheet(
                "color: #000; font-weight: bold; font-size: 12px;")
        elif any(row[4] > 0 for row in ident):
            self.status.setText("Tracking (no axis converged yet)")
            self.status.setStyleSheet("color: #a15500; font-size: 12px;")
        else:
            self.status.setText("Idle (no samples yet)")
            self.status.setStyleSheet("color: #888; font-size: 12px;")

    def _refresh_summary(self, tune):
        if not getattr(tune, "has_summary", False):
            self._clear_summary()
            self.summary_status.setText("waiting for the v4 summary on tag 57")
            self.summary_status.setStyleSheet("color: #888; font-size: 12px;")
            return

        if not getattr(tune, "acc_conf_n", 0):
            self._clear_summary()
            self.summary_status.setText("waiting for a flight (no samples yet)")
            self.summary_status.setStyleSheet("color: #888; font-size: 12px;")
            return

        for _, rows in _SUMMARY_GROUPS:
            for _, key, per_axis, kind in rows:
                cells = getattr(self, "_sum_" + key)
                for r, cell in enumerate(cells):
                    attr = key
                    if per_axis:
                        attr += "_" + _AXES[r].lower()
                    text, known = _format_value(
                        getattr(tune, attr, None), kind)
                    cell.setText(text)
                    cell.setStyleSheet(_SUM_KNOWN if known else _SUM_UNKNOWN)

        n = getattr(tune, "acc_conf_n", 0)
        self.summary_status.setText(
            "summary from the most recent committed flight "
            "(%d accel samples)" % int(n))
        self.summary_status.setStyleSheet(
            "color: #000; font-weight: bold; font-size: 12px;")
