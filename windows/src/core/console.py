# core/console.py
"""Terminal (run.sh console) diagnostics, opt-in only.

The always-on .rawlog (bidirectional, FC and GCS) is the diagnostic of
record. This flag revives the old per-packet console dump for a terminal
session when wanted. Mirrors UAVXGS_VERBOSE_SERIAL: default off.
"""
import os

VERBOSE_CONSOLE = os.environ.get("UAVXGS_VERBOSE_CONSOLE", "0") == "1"


def console(*args, **kwargs):
    if VERBOSE_CONSOLE:
        print(*args, **kwargs)
