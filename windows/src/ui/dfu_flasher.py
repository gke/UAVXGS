import os
import time
from typing import Optional
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QFileDialog, QMessageBox, QTextEdit, QProgressBar, QRadioButton,
    QLineEdit
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont
from protocol_enums import PacketTag, MiscCommand

from stm32_dfu import find_dfu_devices, flash_firmware as dfu_flash
from stm32_uart_bl import flash_firmware_uart


class DfuUsbWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(int, int, str)
    finished = pyqtSignal(bool, str)

    def __init__(self, bin_path: str, flash_base: int = 0x08000000):
        super().__init__()
        self.bin_path = bin_path
        self.flash_base = flash_base

    def run(self):
        def prog(pct, total, msg):
            self.progress.emit(pct, total, msg)
            self.log.emit(msg)

        self.log.emit(f"Starting DFU flash @ 0x{self.flash_base:08X}...")
        error = dfu_flash(self.bin_path, progress_cb=prog, flash_base=self.flash_base)
        if error:
            self.finished.emit(False, error)
        else:
            self.finished.emit(True, "Firmware flashed successfully")


class UartNativeWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(int, int, str)
    finished = pyqtSignal(bool, str)

    def __init__(self, bin_path: str, port: str, baud: int = 115200, flash_base: int = 0x08000000):
        super().__init__()
        self.bin_path = bin_path
        self.port = port
        self.baud = baud
        self.flash_base = flash_base

    def run(self):
        def prog(pct, total, msg):
            self.progress.emit(pct, total, msg)
            self.log.emit(msg)

        self.log.emit(f"Starting UART flash on {self.port}...")
        error = flash_firmware_uart(
            self.port, self.bin_path, self.baud,
            flash_base=self.flash_base, progress_cb=prog,
        )
        if error:
            self.finished.emit(False, error)
        else:
            self.finished.emit(True, "Firmware flashed successfully")


class DfuFlasherWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.bin_path: Optional[str] = None
        self.worker: Optional[QThread] = None
        self._uart_port: Optional[str] = None
        self.setWindowTitle("Firmware Flasher")
        self.setMinimumSize(660, 440)
        self._build_ui()

    def _build_ui(self):
        cw = QWidget()
        self.setCentralWidget(cw)
        layout = QVBoxLayout(cw)

        # Mode selection
        mode_row = QHBoxLayout()
        self.mode_usb = QRadioButton("USB DFU")
        self.mode_usb.setToolTip("STM32 in DFU mode over USB (SpeedyBee)")
        self.mode_uart = QRadioButton("UART Bootloader")
        self.mode_uart.setToolTip("STM32 system bootloader via UART (BOOT0 required)")
        self.mode_uart.setChecked(True)
        mode_row.addWidget(QLabel("Mode:"))
        mode_row.addWidget(self.mode_usb)
        mode_row.addWidget(self.mode_uart)
        mode_row.addStretch()
        layout.addLayout(mode_row)

        # File selection
        file_row = QHBoxLayout()
        self.file_label = QLabel("No file selected")
        self.file_label.setStyleSheet("padding: 4px; border: 1px solid #888; border-radius: 4px;")
        self.browse_btn = QPushButton("Browse...")
        self.browse_btn.clicked.connect(self._browse)
        file_row.addWidget(self.file_label, 1)
        file_row.addWidget(self.browse_btn)
        layout.addLayout(file_row)

        # Flash base address + backup/restore
        opt_row = QHBoxLayout()
        opt_row.addWidget(QLabel("Flash base:"))
        self.addr_edit = QLineEdit("0x08000000")
        self.addr_edit.setToolTip(
            "Address the firmware is linked for. UAVX = 0x08000000. "
            "Boards with a resident bootloader in sector 0 = 0x08004000."
        )
        self.addr_edit.setMaximumWidth(110)
        opt_row.addWidget(self.addr_edit)
        opt_row.addStretch(1)
        layout.addLayout(opt_row)

        # Action buttons
        btn_row = QHBoxLayout()
        self.bootloader_btn = QPushButton("Enter Bootloader")
        self.bootloader_btn.setToolTip("Tell FC to reboot into bootloader mode")
        self.bootloader_btn.clicked.connect(self._enter_bootloader)
        self.scan_btn = QPushButton("Scan for DFU")
        self.scan_btn.clicked.connect(self._scan)
        self.flash_btn = QPushButton("Flash Firmware")
        self.flash_btn.setStyleSheet("font-weight: bold; color: #9b59b6;")
        self.flash_btn.setEnabled(False)
        self.flash_btn.clicked.connect(self._flash)
        btn_row.addWidget(self.bootloader_btn)
        btn_row.addWidget(self.scan_btn)
        btn_row.addStretch()
        btn_row.addWidget(self.flash_btn)
        layout.addLayout(btn_row)

        # Status / device info
        self.device_label = QLabel("Ready")
        self.device_label.setStyleSheet("color: #888; padding: 2px;")
        layout.addWidget(self.device_label)

        # Progress
        self.progress = QProgressBar()
        self.progress.setVisible(False)
        layout.addWidget(self.progress)

        # Log
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setFont(QFont("monospace", 10))
        layout.addWidget(self.log, 1)

        # Connect signals after all widgets exist
        self.mode_usb.toggled.connect(self._mode_changed)
        self.mode_uart.toggled.connect(self._mode_changed)
        self._mode_changed()  # apply initial UART mode state

    # ---- mode ----

    def _mode_changed(self):
        is_uart = self.mode_uart.isChecked()
        self.bootloader_btn.setVisible(True)
        self.scan_btn.setVisible(not is_uart)
        # UAVX firmware is always linked for the default ARM location 0x08000000
        # (one base address — keeps the Makefile simple). The user can still
        # override the address manually if needed.
        self.device_label.setText("Ready")
        self.device_label.setStyleSheet("color: #888; padding: 2px;")
        self._check_ready()

    def _get_port(self) -> Optional[str]:
        if self.parent_window and hasattr(self.parent_window, 'telemetry'):
            tel = self.parent_window.telemetry
            if tel and hasattr(tel, 'port') and tel.port:
                return tel.port
        if self.parent_window and hasattr(self.parent_window, 'port_combo'):
            return self.parent_window.port_combo.currentText()
        return "/dev/ttyUSB0"

    # ---- file ----

    def _browse(self):
        default_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
            "..", "UAVXArmQ", "obj"
        )
        if not os.path.isdir(default_dir):
            default_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
                "..", "UAVXArm32F4", "obj"
            )
        if not os.path.isdir(default_dir):
            default_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
                "obj"
            )
        path, _ = QFileDialog.getOpenFileName(
            self, "Select firmware binary", default_dir,
            "Binary files (*.bin);;All files (*)"
        )
        if path:
            self.bin_path = path
            self.file_label.setText(os.path.basename(path))
            self.file_label.setToolTip(path)
            self._check_ready()

    def _flash_base(self) -> int:
        try:
            return int(self.addr_edit.text().strip(), 0)
        except ValueError:
            return 0x08000000

    def _run_worker(self, worker):
        """Start a worker, disabling controls until it finishes."""
        self.worker = worker
        self.progress.setVisible(True)
        self.progress.setValue(0)
        self.flash_btn.setEnabled(False)
        self.browse_btn.setEnabled(False)
        self.bootloader_btn.setEnabled(False)
        self.scan_btn.setEnabled(False)
        self.mode_usb.setEnabled(False)
        self.mode_uart.setEnabled(False)
        worker.progress.connect(self._on_progress)
        worker.log.connect(self.log.append)
        worker.finished.connect(self._on_flash_done)
        worker.start()

    def _enter_uart_bootloader(self, port: str) -> bool:
        """Prompt the user to put the FC in STM32 system-bootloader mode.
        Returns True if we should proceed, False if the user cancelled.
        """
        reply = QMessageBox.question(
            self, "UART Bootloader",
            f"Put the FC in bootloader mode:\n"
            f"  1. Set BOOT0 = 1\n"
            f"  2. Press RESET\n\n"
            f"Then click OK to continue on {port}",
            QMessageBox.Ok | QMessageBox.Cancel
        )
        if reply != QMessageBox.Ok:
            return False
        self.log.append(f"Disconnecting telemetry on {port}...")
        self._disconnect_telemetry()
        return True

    # ---- actions ----

    def _is_connected(self) -> bool:
        return (
            self.parent_window is not None
            and hasattr(self.parent_window, 'telemetry')
            and self.parent_window.telemetry is not None
            and hasattr(self.parent_window, 'connected')
            and self.parent_window.connected
        )

    def _send_bootloader_cmd(self):
        if self.parent_window and hasattr(self.parent_window, 'send_request'):
            self.parent_window.send_request(PacketTag.MISC, MiscCommand.BOOTLOADER, 0)
            self.log.append("Sent bootloader command to FC")

    def _enter_bootloader(self):
        self._dfu_device = None
        if self._is_connected():
            self._send_bootloader_cmd()
            self.log.append("FC should reset into bootloader mode")
        else:
            self.log.append("Not connected to FC — put FC in bootloader mode manually (BOOT0+RESET)")

        if self.mode_usb.isChecked():
            # The FC re-enumerates a couple of seconds after resetting into the
            # ROM bootloader; poll so the user does not have to hit Scan again.
            self.log.append("Auto-scanning for the DFU device (several seconds)...")
            self._scan(tries=6)
        else:
            self._check_ready()

    def _scan(self, tries=1):
        for attempt in range(tries):
            if tries > 1 and attempt > 0:
                from PyQt5.QtWidgets import QApplication
                QApplication.processEvents()
                time.sleep(1)
            if tries > 1:
                self.log.append(f"Scan attempt {attempt + 1}/{tries}...")
            diagnostics = []
            devices = find_dfu_devices(diagnostics=diagnostics)
            for line in diagnostics:
                self.log.append("  " + line)
            if not devices:
                self._dfu_device = None
                self.device_label.setText("No DFU device found")
                self.device_label.setStyleSheet("color: red; padding: 2px;")
                if not diagnostics:
                    self.log.append(
                        "  No USB DFU device visible. If the board uses a "
                        "USB-serial bridge (CH340/CP210x), the STM32 ROM "
                        "bootloader only answers over UART on the same port — "
                        "use UART Bootloader mode instead. To see the full "
                        "USB map, run: python3 stm32_dfu.py"
                    )
                else:
                    self.log.append(
                        "  DFU device present but blocked. Check that the GCS "
                        "can access /dev/bus/usb (udev rule, user in plugdev, "
                        "not a Flatpak sandbox). See: python3 stm32_dfu.py"
                    )
                continue
            d = devices[0]
            self._dfu_device = d
            self.device_label.setText(
                f"DFU: Bus {d.usb.bus:03d} Dev {d.usb.address:03d} "
                f"VID {d.usb.vid:04x}:{d.usb.pid:04x}"
            )
            self.device_label.setStyleSheet("color: green; padding: 2px; font-weight: bold;")
            self.log.append(f"  Found DFU device")
            break
        self._check_ready()

    def _disconnect_telemetry(self):
        if self.parent_window and hasattr(self.parent_window, 'disconnect'):
            self.parent_window.disconnect()
            self.log.append("Telemetry disconnected")

    def _check_ready(self):
        if self.mode_usb.isChecked():
            has_dev = hasattr(self, '_dfu_device') and self._dfu_device is not None
            self.flash_btn.setEnabled(has_dev and self.bin_path is not None)
        else:
            self.flash_btn.setEnabled(self.bin_path is not None)

    def _flash(self):
        if not self.bin_path or not os.path.exists(self.bin_path):
            QMessageBox.warning(self, "Error", "Please select a valid firmware file")
            return

        self.log.clear()
        flash_base = self._flash_base()
        if self.mode_usb.isChecked():
            self._run_worker(DfuUsbWorker(self.bin_path, flash_base=flash_base))
        else:
            port = self._get_port()
            self._uart_port = port
            if not self._enter_uart_bootloader(port):
                self._on_flash_done(False, "Cancelled")
                return
            self._run_worker(UartNativeWorker(self.bin_path, port, flash_base=flash_base))

    def _on_progress(self, pct: int, total: int, msg: str):
        self.progress.setValue(pct)

    def _on_flash_done(self, success: bool, message: str):
        self.browse_btn.setEnabled(True)
        self.mode_usb.setEnabled(True)
        self.mode_uart.setEnabled(True)
        if success:
            self.log.append("")
            self.log.append("SUCCESS: Firmware flashed!")
            self.device_label.setText("Done — device reset")
            self.device_label.setStyleSheet("color: blue; padding: 2px; font-weight: bold;")
            # DFU reset re-enumerates the FC as a fresh USB device; any
            # telemetry connection held across the flash is now stale.
            if self._is_connected():
                self._disconnect_telemetry()
            if self.mode_usb.isChecked():
                self.log.append("")
                self.log.append("Click 'Connect' in main window to resume telemetry on USB")
            elif self._uart_port:
                self.log.append("")
                self.log.append(f"Click 'Connect' in main window to resume telemetry on {self._uart_port}")
        else:
            self.log.append("")
            self.log.append(f"FAILED: {message}")
        self._check_ready()
        is_usb = self.mode_usb.isChecked()
        self.bootloader_btn.setVisible(True)
        self.scan_btn.setVisible(is_usb)
        self.progress.setVisible(False)
