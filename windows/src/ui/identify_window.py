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

# Per-axis colors (match CalibrationWindow: Roll/X=blue, Pitch/Y=green, Yaw/Z=red)
_AXIS_COLORS = ("#3498db", "#2ecc71", "#e74c3c")

# Styling constants (match calibration panel aesthetic)
_LBL_STYLE = "font-weight: bold; font-size: 12px;"
_VAL_STYLE = "font-weight: bold; font-size: 14px;"
_VAL_STYLE_GREEN = "font-weight: bold; font-size: 14px; color: #1f7a45;"
_VAL_STYLE_AMBER = "font-weight: bold; font-size: 14px; color: #a15500;"
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

        # Header row: label + one column per axis (centred, per-axis color).
        grid.addWidget(QLabel(""), 0, 0)
        for c, ax in enumerate(_AXES, start=1):
            hdr = QLabel("<b>%s</b>" % ax)
            hdr.setStyleSheet("font-weight: bold; font-size: 14px; color: %s;" % _AXIS_COLORS[c-1])
            hdr.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            grid.addWidget(hdr, 0, c)

        for r, (label, attr) in enumerate(rows, start=1):
            # Special highlight for the Revised Kp row (the key result)
            is_revised_kp = (attr == "_ident_pk")
            if is_revised_kp:
                lbl = QLabel("<b>%s</b>" % label)
                lbl.setStyleSheet(_LBL_STYLE + " background-color: #fff9c4;")
                grid.addWidget(lbl, r, 0)
            else:
                lbl = QLabel("<b>%s</b>" % label)
                lbl.setStyleSheet(_LBL_STYLE)
                grid.addWidget(lbl, r, 0)
            for c in range(1, 4):
                val_lbl = QLabel("--")
                if is_revised_kp:
                    val_lbl.setStyleSheet("font-weight: bold; font-size: 14px; color: %s; background-color: #fff9c4;" % _AXIS_COLORS[c-1])
                else:
                    val_lbl.setStyleSheet("font-weight: bold; font-size: 14px; color: %s;" % _AXIS_COLORS[c-1])
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
            "Green = converged (n\u2265200, sig < 0.01)."
        )
        expl.setWordWrap(True)
        expl.setStyleSheet("color: #555; font-size: 12px;")
        main_lay.addWidget(expl)

        main_lay.addStretch()
        self.resize(700, 360)
        self._clear_all()

    # ------------------------------------------------------------------
    def _curve_axis_gains(self):
        """Current (Rate Kp, Rate Kd) per axis from the param window's live
        widgets, or (None, None) if the window/values are unavailable."""
        gains = {}
        pw = getattr(self.parent_window, "param_window", None)
        if pw is None:
            return gains
        from protocol_enums import ParamIndex
        idx = {0: (ParamIndex.ROLL_RATE_KP, ParamIndex.ROLL_RATE_KD),
               1: (ParamIndex.PITCH_RATE_KP, ParamIndex.PITCH_RATE_KD),
               2: (ParamIndex.YAW_RATE_KP, ParamIndex.YAW_RATE_KD)}
        for ax, (kpi, kdi) in idx.items():
            kp = kd = None
            w = pw.params.get(int(kpi))
            if isinstance(w, QDoubleSpinBox):
                kp = (w.value() / pw._display_mult(int(kpi))
                      if hasattr(pw, "_display_mult") else w.value())
            elif isinstance(w, QSpinBox):
                kp = float(w.value())
            w = pw.params.get(int(kdi))
            if isinstance(w, QDoubleSpinBox):
                kd = (w.value() / pw._display_mult(int(kdi))
                      if hasattr(pw, "_display_mult") else w.value())
            elif isinstance(w, QSpinBox):
                kd = float(w.value())
            gains[ax] = (kp, kd)
        return gains

    def _assumed_tau(self):
        pw = getattr(self.parent_window, "param_window", None)
        if pw and hasattr(pw, "_current_af_category"):
            try:
                cat = pw._current_af_category()
            except Exception:
                cat = None
            if cat in ("MR", "VTOL", "MR/VTOL"):
                return _ASSUMED_TAU_MR
        return _ASSUMED_TAU_FW

    def _clear_all(self):
        """Set every value label to '--' with per-axis base colors."""
        for r in range(3):
            for attr in ("_gains_kp", "_gains_kd", "_gains_tau",
                         "_ident_a", "_ident_b", "_ident_sa", "_ident_sb",
                         "_ident_n", "_ident_tau", "_ident_k",
                         "_ident_pk", "_ident_step"):
                arr = getattr(self, attr)
                arr[r].setText("--")
                if attr == "_ident_pk":
                    arr[r].setStyleSheet("font-weight: bold; font-size: 14px; color: %s; background-color: #fff9c4;" % _AXIS_COLORS[r])
                else:
                    arr[r].setStyleSheet("font-weight: bold; font-size: 14px; color: %s;" % _AXIS_COLORS[r])

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

            # --- Current gains & assumed plant ---
            base_style = "font-weight: bold; font-size: 14px; color: %s;" % _AXIS_COLORS[r]
            self._gains_kp[r].setText(_fmt(kp))
            self._gains_kp[r].setStyleSheet(base_style)
            self._gains_kd[r].setText(_fmt(kd))
            self._gains_kd[r].setStyleSheet(base_style)
            self._gains_tau[r].setText("%.3f" % assumed_tau)
            self._gains_tau[r].setStyleSheet(base_style)

            # --- Identified plant ---
            tau_f = k_f = prop_kp_f = None
            is_converged = _identify_state(sa, sb, n) == "Converged"
            if is_converged:
                n_converged += 1

            if 0.0 < a < 1.0:
                tau_f = -_IDENTIFY_DT / math.log(a)
                k_f = b / (1.0 - a) if a != 1.0 else None
                if (k_f is not None and math.isfinite(k_f) and k_f > 0):
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

            # Raw identified parameters: per-axis color
            raw_style = "font-weight: bold; font-size: 14px; color: %s;" % _AXIS_COLORS[r]
            self._ident_a[r].setText(_fmt(a, ".4f"))
            self._ident_a[r].setStyleSheet(raw_style)
            self._ident_b[r].setText(_fmt(b, ".4f"))
            self._ident_b[r].setStyleSheet(raw_style)
            self._ident_sa[r].setText(_fmt(sa, ".6f"))
            self._ident_sa[r].setStyleSheet(raw_style)
            self._ident_sb[r].setText(_fmt(sb, ".6f"))
            self._ident_sb[r].setStyleSheet(raw_style)
            self._ident_n[r].setText(str(int(n)))
            self._ident_n[r].setStyleSheet(raw_style)

            # Derived/convergence-sensitive rows: green/amber
            conv_style = (_VAL_STYLE_GREEN if is_converged
                          else _VAL_STYLE_AMBER if n > 0
                          else raw_style)
            self._ident_tau[r].setText(_fmt(tau_f, ".3f"))
            self._ident_tau[r].setStyleSheet(conv_style)
            self._ident_k[r].setText(_fmt(k_f, ".3f"))
            self._ident_k[r].setStyleSheet(conv_style)
            self._ident_pk[r].setText(_fmt(prop_kp_f, ".4f"))
            self._ident_pk[r].setStyleSheet(conv_style)

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
            self._ident_step[r].setStyleSheet(conv_style)

# Status line
        labels = []
        for r in range(3):
            a, b, sa, sb, n = ident[r]
            if _identify_state(sa, sb, n) == "Converged":
                labels.append(_AXES[r])
        if labels:
            self.status.setText(
                "Converged: %s  |  target \u03c4=%.2f s  |  "
                "green = n\u2265%d, sig < %.3f"
                % (", ".join(labels), assumed_tau,
                   IDENTIFY_MIN_SAMPLES, IDENTIFY_CONV_SIG))
            self.status.setStyleSheet(
                "color: #1f7a45; font-weight: bold; font-size: 12px;")
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