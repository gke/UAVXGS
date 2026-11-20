# ui/identify_window.py - Passive per-axis KF-RLS plant identification window
"""
Separate top-level window (matches the CalibrationWindow pattern) showing the
FC's live per-axis plant identification (tag 57 TUNING identify block + flashed
restore).

Layout: one grid, columns = axes (Roll/Pitch/Yaw), rows = the measure fields
(Current Rate Kp/Kd, Assumed tau, then the identified plant a/b/sigA/sigB/n,
derived tau/K, proposed Rate Kp and the step-response comparison).

The identify block is appended to tag 57 (telem.c SendTuningPacket). The FC
never divides/log-exports: everything derived is computed here. Mirrors the
FC constants from identify.h.
"""

import math

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

from critic.metrics import step_metrics_window

# Mirrors of identify.h (FC-side tuning constants — keep in sync).
IDENTIFY_MIN_SAMPLES = 200     # minimum n before "converged" is considered
IDENTIFY_CONV_SIG = 0.01       # sigA/sigB below this counts as converged
_IDENTIFY_DT = 0.002           # 500 Hz control period

# Assumed open-loop plant time constant for the "assumed step" column (tau_s).
# These are the emulator's MotorTau assumptions (emu.c), not FC constants.
_ASSUMED_TAU_MR = 0.10
_ASSUMED_TAU_FW = 0.08

_AXES = ("Roll", "Pitch", "Yaw")

# One colour vocabulary for every value cell, three states and no more:
#   unknown   -> "--" in grey   (no data for that axis yet)
#   tracking  -> orange         (values present, not converged)
#   converged -> black          (values present and trustworthy)
# The axis is identified by its column header, never by colour: the old
# per-axis blue/green/red collided with the convergence colours and made the
# grid unreadable (Greg, 2026-11-20).
_VAL_BASE = "font-weight: bold; font-size: 14px; color: %s;"
_ST_UNKNOWN = _VAL_BASE % "#888888"
_ST_TRACK = _VAL_BASE % "#a15500"
_ST_CONV = _VAL_BASE % "#000000"
_HL = " background-color: #fff9c4;"

# Styling constants (match calibration panel aesthetic)
_LBL_STYLE = "font-weight: bold; font-size: 12px;"
_TITLE_STYLE = "font-size: 18px; font-weight: bold;"
_GROUP_STYLE = "QGroupBox { font-weight: bold; }"


def _closed_loop_step(k, tau, dT, kp, duration_s=2.0):
    """Simulate a first-order plant (DC gain k, time constant tau) in a
    unity rate loop with proportional-only gain kp, driven by a step input
    of 1.0. Output is the closed-loop rate response.  Bounded to prevent
    numerical runaway."""
    times, values = [], []
    rate = 0.0
    steps = min(int(duration_s / dT), 5000)
    for i in range(steps):
        t = i * dT
        er = 1.0 - rate
        out = kp * er
        rate += (dT / tau) * (k * out - rate)
        if not math.isfinite(rate):
            rate = 0.0
        times.append(t)
        values.append(rate)
    return times, values


def _identify_state(sig_a, sig_b, n):
    if (n >= IDENTIFY_MIN_SAMPLES
            and sig_a < IDENTIFY_CONV_SIG
            and sig_b < IDENTIFY_CONV_SIG):
        return "Converged"
    if n > 0:
        return "Tracking"
    return "Idle"


def _fmt(val, fmt_s=".4f"):
    """Format a float, or return '--' for None / non-finite."""
    if val is None or not math.isfinite(val):
        return "--"
    return format(val, fmt_s)


class IdentifyWindow(QMainWindow):
    """Separate top-level window (matches CalibrationWindow pattern):
    per-axis identified plant vs current PID tuning, one grid with columns
    per axis."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.setWindowTitle("Identify - Rate Kp and Kd")
        self.setMinimumSize(700, 500)

        central = QWidget()
        self.setCentralWidget(central)
        main_lay = QVBoxLayout(central)
        main_lay.setSpacing(8)
        main_lay.setContentsMargins(8, 8, 8, 8)

        # --- Title (like CalibrationWindow) ---
        title = QLabel("Identify - Rate Kp and Kd")
        title.setStyleSheet(_TITLE_STYLE)
        title.setAlignment(Qt.AlignCenter)
        main_lay.addWidget(title)

        group = QGroupBox()
        group.setStyleSheet(_GROUP_STYLE)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)

        # --- Rows: the measure fields. Columns: axis (Roll/Pitch/Yaw). ---
        rows = [
            ("Current Kp", "_gains_kp"),
            ("Current Kd", "_gains_kd"),
            ("Revised Kp", "_ident_pk"),
            ("Assumed \u03c4 (s)", "_gains_tau"),
            ("a", "_ident_a"),
            ("b", "_ident_b"),
            ("sigA", "_ident_sa"),
            ("sigB", "_ident_sb"),
            ("n", "_ident_n"),
            ("\u03c4 ident (s)", "_ident_tau"),
            ("K (DC gain)", "_ident_k"),
            ("Step rise/settle", "_ident_step"),
        ]

        # Header row: label + one column per axis (centred). The axis is named
        # here and nowhere else; value colour carries state, not axis.
        grid.addWidget(QLabel(""), 0, 0)
        for c, ax in enumerate(_AXES, start=1):
            hdr = QLabel("<b>%s</b>" % ax)
            hdr.setStyleSheet("font-weight: bold; font-size: 14px;")
            hdr.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            grid.addWidget(hdr, 0, c)

        for r, (label, attr) in enumerate(rows, start=1):
            # The Revised Kp row is the key result: it is highlighted, not
            # coloured, so its state colour still reads the same way.
            is_revised_kp = (attr == "_ident_pk")
            lbl = QLabel("<b>%s</b>" % label)
            lbl.setStyleSheet(_LBL_STYLE + (_HL if is_revised_kp else ""))
            grid.addWidget(lbl, r, 0)
            for c in range(1, 4):
                val_lbl = QLabel("--")
                val_lbl.setStyleSheet(
                    _ST_UNKNOWN + (_HL if is_revised_kp else ""))
                val_lbl.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                grid.addWidget(val_lbl, r, c)
            setattr(self, attr, [None] * 3)
            arr = getattr(self, attr)
            for c in range(3):
                arr[c] = grid.itemAtPosition(r, c + 1).widget()

        group.setLayout(grid)
        main_lay.addWidget(group)

        # --- Convergence status line ---
        self.status = QLabel("waiting for tag 57 (no FC identified data yet)")
        self.status.setStyleSheet("color: #888; font-weight: normal; font-size: 12px;")
        main_lay.addWidget(self.status)

        # --- Explanation ---
        expl = QLabel(
            "<b>a</b> per-tick rate retention — higher = slower response.<br>"
            "<b>b</b> per-tick control authority — higher = faster.<br>"
            "<b>\u03c4</b> time constant = -dT/ln(a).<br>"
            "<b>K</b> DC gain = b/(1-a).<br>"
            "<b>Kp (revised)</b> Kp that makes the identified plant "
            "respond like the assumed model at your current Kp "
            "(matches current when plant is as assumed; slower plant "
            "needs more Kp). "
            "Black = converged, orange = still tracking, grey -- = no data "
            "(n\u2265200, sig < 0.01 to converge)."
        )
        expl.setWordWrap(True)
        expl.setStyleSheet("color: #555; font-size: 12px;")
        main_lay.addWidget(expl)

        main_lay.addStretch()
        self.resize(700, 360)
        self._clear_all()

    # ------------------------------------------------------------------
    def _param_window(self):
        """The parameter window, created on demand but never shown.

        MainWindow builds ParameterWindow lazily in show_parameter_window(),
        so opening Identify first used to find no widgets and blanked the
        current-gain rows. ensure_param_window() seeds any pending state the
        same way the normal show path does.
        """
        ensure = getattr(self.parent_window, "ensure_param_window", None)
        if ensure is not None:
            return ensure()
        return getattr(self.parent_window, "param_window", None)

    def _curve_axis_gains(self):
        """Current (Rate Kp, Rate Kd) per axis from the param window's live
        widgets, or (None, None) if the window/values are unavailable."""
        gains = {}
        pw = self._param_window()
        if pw is None:
            return gains
        from protocol_enums import ParamIndex
        idx = {0: (ParamIndex.ROLL_RATE_KP, ParamIndex.ROLL_RATE_KD),
               1: (ParamIndex.PITCH_RATE_KP, ParamIndex.PITCH_RATE_KD),
               2: (ParamIndex.YAW_RATE_KP, ParamIndex.YAW_RATE_KD)}

        def raw_value(tag):
            w = pw.param_widget(int(tag))
            if isinstance(w, QDoubleSpinBox):
                mult = pw._display_mult(int(tag)) if hasattr(pw, "_display_mult") else 1.0
                return w.value() / mult if mult else None
            if isinstance(w, QSpinBox):
                return float(w.value())
            return None

        for ax, (kpi, kdi) in idx.items():
            gains[ax] = (raw_value(kpi), raw_value(kdi))
        return gains

    def _assumed_tau(self):
        pw = self._param_window()
        if pw and hasattr(pw, "_current_af_category"):
            try:
                cat = pw._current_af_category()
            except Exception:
                cat = None
            if cat in ("MR", "VTOL", "MR/VTOL"):
                return _ASSUMED_TAU_MR
        return _ASSUMED_TAU_FW

    def _clear_all(self):
        """Set every value label to '--' (unknown state)."""
        for r in range(3):
            for attr in ("_gains_kp", "_gains_kd", "_gains_tau",
                         "_ident_a", "_ident_b", "_ident_sa", "_ident_sb",
                         "_ident_n", "_ident_tau", "_ident_k",
                         "_ident_pk", "_ident_step"):
                arr = getattr(self, attr)
                arr[r].setText("--")
                arr[r].setStyleSheet(_ST_UNKNOWN + (_HL if attr == "_ident_pk" else ""))

    # ------------------------------------------------------------------
    def _refresh(self, tune):
        """Rebuild every label from a TuningData (or None = clear).
        Wrapped in try/except so a bad packet never kills the GCS."""
        try:
            self._do_refresh(tune)
        except Exception as exc:
            self.status.setText("error: %s" % exc)
            self.status.setStyleSheet("color: red; font-weight: normal;")

    def _do_refresh(self, tune):
        gains = self._curve_axis_gains()
        assumed_tau = self._assumed_tau()

        if tune is None or not getattr(tune, "has_ident", False):
            self._clear_all()
            self.status.setText(
                "waiting for the FC identify block (reflash with "
                "identify.c first; streams at ~1 Hz on tag 57)")
            self.status.setStyleSheet("color: #888; font-weight: normal;")
            return

        ident = [
            (tune.ident_a_roll, tune.ident_b_roll,
             tune.ident_sig_a_roll, tune.ident_sig_b_roll,
             tune.ident_n_roll),
            (tune.ident_a_pitch, tune.ident_b_pitch,
             tune.ident_sig_a_pitch, tune.ident_sig_b_pitch,
             tune.ident_n_pitch),
            (tune.ident_a_yaw, tune.ident_b_yaw,
             tune.ident_sig_a_yaw, tune.ident_sig_b_yaw,
             tune.ident_n_yaw),
        ]

        n_converged = 0
        for r in range(3):
            a, b, sa, sb, n = ident[r]
            kp, kd = gains.get(r, (None, None))

            # The ident block is only meaningful once the axis has been
            # excited: n == 0 means no samples, and a/b are still their
            # initial zeros. Those cells read "--", not "0.0000".
            known = n > 0
            is_converged = known and _identify_state(sa, sb, n) == "Converged"
            if is_converged:
                n_converged += 1
            st = _ST_CONV if is_converged else (_ST_TRACK if known else _ST_UNKNOWN)

            # --- Current gains & assumed plant: known unless the parameter
            # window could not supply them, so they are never state-coloured.
            for attr, val in (("_gains_kp", kp), ("_gains_kd", kd)):
                arr = getattr(self, attr)
                arr[r].setText(_fmt(val))
                arr[r].setStyleSheet(_ST_CONV if val is not None else _ST_UNKNOWN)
            self._gains_tau[r].setText("%.3f" % assumed_tau)
            self._gains_tau[r].setStyleSheet(_ST_CONV)

            # --- Identified plant ---
            tau_f = k_f = prop_kp_f = None

            if known and 0.0 < a < 1.0:
                tau_f = -_IDENTIFY_DT / math.log(a)
                k_f = b / (1.0 - a)
                if math.isfinite(k_f) and k_f > 0:
                    # Proposed Kp: make the IDENTIFIED plant respond like the
                    # ASSUMED model does with the current Kp.
                    # Assumed closed-loop pole: a_as - b_as*Kp_current, where
                    # a_as = exp(-dT/tau_assumed), b_as = K*(1-a_as) (same
                    # authority as identified, assumed tau).
                    a_as = math.exp(-_IDENTIFY_DT / assumed_tau)
                    b_as = k_f * (1.0 - a_as)
                    pole_target = a_as - b_as * kp if kp is not None else None
                    if (pole_target is not None
                            and -1.0 < pole_target < 1.0):
                        prop_kp_f = (a - pole_target) / b

            for attr, val, fmt_s in (("_ident_a", a if known else None, ".4f"),
                                     ("_ident_b", b if known else None, ".4f"),
                                     ("_ident_sa", sa if known else None, ".6f"),
                                     ("_ident_sb", sb if known else None, ".6f"),
                                     ("_ident_tau", tau_f, ".3f"),
                                     ("_ident_k", k_f, ".3f"),
                                     ("_ident_pk", prop_kp_f, ".4f")):
                arr = getattr(self, attr)
                arr[r].setText(_fmt(val, fmt_s))
                arr[r].setStyleSheet(st + (_HL if attr == "_ident_pk" else ""))
            self._ident_n[r].setText(str(int(n)))
            self._ident_n[r].setStyleSheet(st)

            # Step response: current Kp through identified vs assumed plant.
            step_txt = "--"
            if (kp is not None and kp > 0
                    and tau_f is not None and k_f is not None
                    and math.isfinite(k_f) and k_f > 0):
                try:
                    ts_e, ys_e = _closed_loop_step(
                        k_f, tau_f, _IDENTIFY_DT, kp)
                    rise_e, _, _ = step_metrics_window(ts_e, ys_e, 1.0)
                    ts_a, ys_a = _closed_loop_step(
                        k_f, assumed_tau, _IDENTIFY_DT, kp)
                    rise_a, _, _ = step_metrics_window(ts_a, ys_a, 1.0)
                    step_txt = "%.2f/%.2fs" % (rise_a, rise_e)
                except Exception:
                    step_txt = "--"
            self._ident_step[r].setText(step_txt)
            self._ident_step[r].setStyleSheet(
                st if step_txt != "--" else _ST_UNKNOWN)

# Status line
        labels = []
        for r in range(3):
            a, b, sa, sb, n = ident[r]
            if _identify_state(sa, sb, n) == "Converged":
                labels.append(_AXES[r])
        if labels:
            self.status.setText(
                "Converged: %s  |  target \u03c4=%.2f s  |  "
                "black = n\u2265%d, sig < %.3f"
                % (", ".join(labels), assumed_tau,
                   IDENTIFY_MIN_SAMPLES, IDENTIFY_CONV_SIG))
            self.status.setStyleSheet(
                "color: #000000; font-weight: bold; font-size: 12px;")
        elif any(ident[r][4] > 0 for r in range(3)):
            self.status.setText(
                "Tracking (no axis converged yet)  |  "
                "target \u03c4=%.2f s" % assumed_tau)
            self.status.setStyleSheet(
                "color: #a15500; font-weight: normal; font-size: 12px;")
        else:
            self.status.setText(
                "Idle (no excitation data yet)  |  "
                "target \u03c4=%.2f s" % assumed_tau)
            self.status.setStyleSheet(
                "color: #888; font-weight: normal; font-size: 12px;")