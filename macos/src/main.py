#!/usr/bin/env python3
# main.py
"""
UAVX Groundstation - Python Port
Main entry point
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from ui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("UAVX Groundstation")
    app.setOrganizationName("UAVX")

    # Icon lives in the sibling UAVXGUI folder (XGS.ico).
    here = os.path.dirname(os.path.abspath(__file__))
    icon_candidates = [
        os.path.join(here, "..", "..", "..", "UAVXGUI", "XGS.ico"),
        os.path.join(here, "XGS.ico"),
    ]
    for icon_path in icon_candidates:
        if os.path.isfile(icon_path):
            app.setWindowIcon(QIcon(os.path.abspath(icon_path)))
            break

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
