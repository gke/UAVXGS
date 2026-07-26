"""
Native STM32 DFU protocol implementation via Linux usbfs.
Implements AN3156 (USB DFU protocol used in the STM32 bootloader).
No external dependencies — uses ctypes + fcntl.ioctl on /dev/bus/usb/.
"""

import ctypes
import fcntl
import os
import struct
import time

# ---- Linux USB device fs ioctl numbers (computed at load time) ----

_IOC_NONE = 0
_IOC_WRITE = 1
_IOC_READ = 2


def _IOC(dir_, typ, nr, size):
    return (dir_ << 30) | (typ << 8) | (nr << 0) | (size << 16)


def _IOR(typ, nr, size):
    return _IOC(_IOC_READ, typ, nr, size)


def _IOW(typ, nr, size):
    return _IOC(_IOC_WRITE, typ, nr, size)


def _IOWR(typ, nr, size):
    return _IOC(_IOC_READ | _IOC_WRITE, typ, nr, size)


class UsbDevFsCtrlTransfer(ctypes.Structure):
    _fields_ = [
        ("bRequestType", ctypes.c_uint8),
        ("bRequest", ctypes.c_uint8),
        ("wValue", ctypes.c_uint16),
        ("wIndex", ctypes.c_uint16),
        ("wLength", ctypes.c_uint16),
        ("timeout", ctypes.c_uint32),
        ("data", ctypes.c_void_p),
    ]


USBDEVFS_CONTROL = _IOWR(ord("U"), 0, ctypes.sizeof(UsbDevFsCtrlTransfer))
USBDEVFS_CLAIMINTERFACE = _IOR(ord("U"), 15, ctypes.sizeof(ctypes.c_uint))
USBDEVFS_RELEASEINTERFACE = _IOR(ord("U"), 16, ctypes.sizeof(ctypes.c_uint))

# ---- DFU constants ----

DFU_REQ_DETACH = 0x00
DFU_REQ_DNLOAD = 0x01
DFU_REQ_UPLOAD = 0x02
DFU_REQ_GETSTATUS = 0x03
DFU_REQ_CLRSTATUS = 0x04
DFU_REQ_GETSTATE = 0x05
DFU_REQ_ABORT = 0x06

DFU_BM_REQ_OUT = 0x21  # Host-to-device, Class, Interface
DFU_BM_REQ_IN = 0xA1   # Device-to-host, Class, Interface

# STM32 commands (first byte of DFU_DNLOAD when wValue=0)
STM_CMD_SET_ADDRESS = 0x21
STM_CMD_ERASE = 0x41
STM_CMD_READ_UNPROTECT = 0x92

# DFU states
DFU_STATE_IDLE = 0x00
DFU_STATE_DNLOAD_SYNC = 0x01
DFU_STATE_DNBUSY = 0x02
DFU_STATE_DNLOAD_IDLE = 0x03
DFU_STATE_MANIFEST_SYNC = 0x04
DFU_STATE_MANIFEST = 0x05
DFU_STATE_MANIFEST_WAIT_RESET = 0x06
DFU_STATE_UPLOAD_IDLE = 0x07
DFU_STATE_ERROR = 0x0A

# STM32 flash geometry (STM32F405/407)
FLASH_BASE = 0x08000000
SECTOR_SIZE = 16 * 1024  # 16 KB sectors 0-3
CONFIG_ADDR = 0x08004000  # Sector 1
CONFIG_SIZE = 16 * 1024

# DFU interface class/subclass/protocol
DFU_IFACE_CLASS = 0xFE
DFU_IFACE_SUBCLASS = 0x01

# Default STM32 DFU VID/PID
STM32_DFU_VID = 0x0483
STM32_DFU_PID = 0xDF11


def _compute_ioctl_size(s):
    return ctypes.sizeof(s)


# ---- Low-level usbfs transport ----

class UsbDevice:
    """Open and control a USB device via /dev/bus/usb/"""

    def __init__(self, bus: int, address: int, vid: int, pid: int):
        self.bus = bus
        self.address = address
        self.vid = vid
        self.pid = pid
        self.fd: int = -1
        self._path = f"/dev/bus/usb/{bus:03d}/{address:03d}"

    def open(self) -> bool:
        try:
            self.fd = os.open(self._path, os.O_RDWR)
            return True
        except OSError as e:
            print(f"  USB open failed: {e}")
            return False

    def close(self):
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def claim_interface(self, iface: int = 0) -> bool:
        try:
            buf = (ctypes.c_uint * 1)(iface)
            fcntl.ioctl(self.fd, USBDEVFS_CLAIMINTERFACE, buf)
            return True
        except OSError as e:
            print(f"  Claim interface failed: {e}")
            return False

    def release_interface(self, iface: int = 0):
        try:
            buf = (ctypes.c_uint * 1)(iface)
            fcntl.ioctl(self.fd, USBDEVFS_RELEASEINTERFACE, buf)
        except OSError:
            pass

    def control_transfer(
        self, bm_request_type: int, b_request: int,
        w_value: int, w_index: int, data_or_len, timeout: int = 5000
    ) -> bytes:
        if isinstance(data_or_len, int):
            data_len = data_or_len
            data_buf = bytearray(data_len)
        else:
            data_buf = bytearray(data_or_len)
            data_len = len(data_buf)

        data_char = ctypes.c_char.from_buffer(data_buf)
        data_addr = ctypes.addressof(data_char)

        req_buf = bytearray(ctypes.sizeof(UsbDevFsCtrlTransfer))
        req = UsbDevFsCtrlTransfer.from_buffer(req_buf)
        req.bRequestType = bm_request_type
        req.bRequest = b_request
        req.wValue = w_value
        req.wIndex = w_index
        req.wLength = data_len
        req.timeout = timeout
        req.data = data_addr

        fcntl.ioctl(self.fd, USBDEVFS_CONTROL, req_buf)

        if isinstance(data_or_len, int):
            return bytes(data_buf[:data_len])
        return bytes(data_buf[:data_len])

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()


# ---- DFU Protocol Layer ----

class DfuDevice:
    """DFU protocol over a USB device in DFU mode"""

    def __init__(self, usb: UsbDevice):
        self.usb = usb
        self.interface = 0
        self.transfer_size = 2048  # STM32 default, will be read from descriptor

    def _ctrl_out(self, req: int, value: int, data: bytes = b"") -> bytes:
        return self.usb.control_transfer(
            DFU_BM_REQ_OUT, req, value, self.interface, data
        )

    def _ctrl_in(self, req: int, value: int, length: int) -> bytes:
        return self.usb.control_transfer(
            DFU_BM_REQ_IN, req, value, self.interface, length
        )

    def get_status(self) -> tuple:
        """Return (status, poll_timeout, state, str_index)"""
        raw = self._ctrl_in(DFU_REQ_GETSTATUS, 0, 6)
        status, poll_timeout, state, str_idx = struct.unpack("<BiBb", raw)
        return status, poll_timeout, state, str_idx

    def get_state(self) -> int:
        raw = self._ctrl_in(DFU_REQ_GETSTATE, 0, 1)
        return raw[0]

    def clear_status(self):
        self._ctrl_out(DFU_REQ_CLRSTATUS, 0)

    def abort(self):
        self._ctrl_out(DFU_REQ_ABORT, 0)

    def _wait_idle(self, timeout_ms: int = 5000) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            try:
                _, poll_to, state, _ = self.get_status()
                if state == DFU_STATE_IDLE:
                    return True
                if state == DFU_STATE_ERROR:
                    self.clear_status()
                time.sleep(max(poll_to, 1) / 1000.0)
            except Exception:
                time.sleep(0.01)
        return False

    def _wait_busy(self, poll_timeout: int, timeout_ms: int = 30000) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            try:
                _, poll_to, state, _ = self.get_status()
                if state != DFU_STATE_DNBUSY:
                    return state == DFU_STATE_DNLOAD_IDLE or state == DFU_STATE_IDLE
                time.sleep(max(poll_timeout, 1) / 1000.0)
            except Exception:
                time.sleep(0.01)
        return False

    def set_address(self, addr: int) -> bool:
        """Set address pointer (STM32 Set Address Pointer command)"""
        cmd = struct.pack("<BI", STM_CMD_SET_ADDRESS, addr)
        self._ctrl_out(DFU_REQ_DNLOAD, 0, cmd)
        _, poll_to, state, _ = self.get_status()
        if state != DFU_STATE_DNBUSY:
            return False
        time.sleep(max(poll_to, 1) / 1000.0)
        _, _, state, _ = self.get_status()
        return state != DFU_STATE_ERROR

    def erase_page(self, addr: int) -> bool:
        """Erase one flash page (2KB erase granule on STM32F4)"""
        cmd = struct.pack("<BI", STM_CMD_ERASE, addr)
        self._ctrl_out(DFU_REQ_DNLOAD, 0, cmd)
        _, poll_to, state, _ = self.get_status()
        if state != DFU_STATE_DNBUSY:
            return False
        time.sleep(max(poll_to, 1) / 1000.0)
        _, _, state, _ = self.get_status()
        return state != DFU_STATE_ERROR

    def read_memory(self, addr: int, length: int) -> bytes:
        """Read memory at current address pointer. Returns data or raises."""
        if not self.set_address(addr):
            raise RuntimeError(f"Set address failed (read @ 0x{addr:08X})")

        result = bytearray()
        remaining = length
        block_num = 2
        while remaining > 0:
            chunk = min(remaining, self.transfer_size)
            raw = self._ctrl_in(DFU_REQ_UPLOAD, block_num, chunk)
            result.extend(raw)
            remaining -= len(raw)
            block_num += 1
            if len(raw) == 0:
                break
        return bytes(result)

    def write_block(self, addr: int, data: bytes) -> bool:
        """Write a single block (up to transfer_size bytes) at address"""
        if not self.set_address(addr):
            return False
        self._ctrl_out(DFU_REQ_DNLOAD, 2, data)
        _, poll_to, state, _ = self.get_status()
        if state != DFU_STATE_DNBUSY:
            return False
        time.sleep(max(poll_to, 1) / 1000.0)
        _, _, state, _ = self.get_status()
        return state != DFU_STATE_ERROR

    def write_memory(self, addr: int, data: bytes, progress_cb=None) -> bool:
        """Write data to flash, erasing pages as needed. Splits into transfer_size chunks."""
        pos = 0
        total = len(data)
        while pos < total:
            chunk = data[pos:pos + self.transfer_size]
            chunk_addr = addr + pos

            # Erase pages covered by this chunk
            page_start = chunk_addr & ~(SECTOR_SIZE - 1)
            page_end = (chunk_addr + len(chunk) + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1)
            for page in range(page_start, page_end, SECTOR_SIZE):
                if not self.erase_page(page):
                    raise RuntimeError(f"Erase failed @ 0x{page:08X}")

            if not self.write_block(chunk_addr, chunk):
                raise RuntimeError(f"Write failed @ 0x{chunk_addr:08X}")

            pos += len(chunk)
            if progress_cb:
                progress_cb(pos, total)
        return True

    def leave_dfu(self, addr: int = FLASH_BASE) -> bool:
        """Exit DFU mode and start the application at addr.
        AN3156: send DNLOAD(0) leave request after pointing at the jump address.
        """
        if not self.set_address(addr):
            return False
        self._ctrl_out(DFU_REQ_DNLOAD, 0, b"")
        _, poll_to, state, _ = self.get_status()
        if state == DFU_STATE_MANIFEST:
            time.sleep(1)
            return True
        return False


# ---- Device Discovery ----

def scan_usb_sysfs() -> list[dict]:
    """Scan /sys/bus/usb/devices for DFU-mode devices"""
    devices = []
    sysfs = "/sys/bus/usb/devices"
    if not os.path.isdir(sysfs):
        return devices

    for name in os.listdir(sysfs):
        devpath = os.path.join(sysfs, name)
        if not os.path.isdir(devpath):
            continue
        if ":" in name:
            continue  # interface, not device

        try:
            vid = int(open(os.path.join(devpath, "idVendor")).read().strip(), 16)
            pid = int(open(os.path.join(devpath, "idProduct")).read().strip(), 16)
            bus = int(open(os.path.join(devpath, "busnum")).read().strip())
            devnum = int(open(os.path.join(devpath, "devnum")).read().strip())
        except (ValueError, OSError):
            continue

        # Check for DFU interface
        for iface_name in os.listdir(devpath):
            if ":" not in iface_name:
                continue
            iface_path = os.path.join(devpath, iface_name)
            try:
                cls = int(open(os.path.join(iface_path, "bInterfaceClass")).read().strip())
                sub = int(open(os.path.join(iface_path, "bInterfaceSubClass")).read().strip())
                if cls == DFU_IFACE_CLASS and sub == DFU_IFACE_SUBCLASS:
                    devices.append({
                        "bus": bus, "devnum": devnum,
                        "vid": vid, "pid": pid,
                        "devpath": f"/dev/bus/usb/{bus:03d}/{devnum:03d}",
                    })
                    break
            except (ValueError, OSError):
                continue

    return devices


def find_dfu_devices() -> list[DfuDevice]:
    """Find all STM32 DFU devices on USB"""
    result = []
    for info in scan_usb_sysfs():
        usb = UsbDevice(info["bus"], info["devnum"], info["vid"], info["pid"])
        if not usb.open():
            continue
        if not usb.claim_interface(0):
            usb.close()
            continue
        dfu = DfuDevice(usb)
        dfu.interface = 0
        result.append(dfu)
    return result


# ---- High-level flashing ----

DFU_VID = 0x0483
DFU_PID = 0xDF11


def flash_firmware(bin_path: str, progress_cb=None, flash_base: int = 0x08000000) -> str:
    """Flash a .bin firmware file with config sector preservation.
    flash_base is the address the firmware is linked for. UAVX firmware owns the
    whole flash and starts at 0x08000000. Boards with a resident bootloader in
    sector 0 (e.g. SpeedyBee/HY3) link firmware at 0x08004000.
    Returns empty string on success, or error message on failure.
    """
    if not os.path.exists(bin_path):
        return f"File not found: {bin_path}"

    with open(bin_path, "rb") as f:
        firmware = f.read()

    if len(firmware) == 0:
        return "Empty firmware file"

    devices = find_dfu_devices()
    if not devices:
        return "No DFU device found. Enter bootloader mode first."

    dfu = devices[0]

    try:
        if progress_cb:
            progress_cb(0, 100, "Entering DFU idle...")
        if not dfu._wait_idle(2000):
            dfu.clear_status()
            if not dfu._wait_idle(2000):
                return "Failed to get DFU device into idle state"

        # Step 1: Save config sector (Sector 1, 0x08004000, 16 KB)
        if progress_cb:
            progress_cb(5, 100, "Saving config sector...")
        config_data = b""
        try:
            config_data = dfu.read_memory(CONFIG_ADDR, CONFIG_SIZE)
        except (OSError, RuntimeError):
            pass  # first flash or blank — ignore
        have_config = len(config_data) > 0 and any(b != 0xFF for b in config_data)
        # Only preserve the config sector when it lies OUTSIDE the firmware
        # region. For boards whose app starts at 0x08004000 (e.g. SpeedyBee/iNav,
        # which keep their bootloader in sector 0), the firmware overlaps the
        # config sector, so preserving/restoring it would corrupt the firmware.
        preserve_config = flash_base < CONFIG_ADDR
        if not preserve_config:
            have_config = False

        # Step 2: Flash firmware at flash_base (0x08004000 for this target)
        if progress_cb:
            progress_cb(10, 100, f"Flashing firmware @ 0x{flash_base:08X}...")

        def flash_prog(pos, total):
            pct = 10 + int(70 * pos / total)
            if progress_cb:
                progress_cb(pct, 100, f"Flashing... {pos}/{total} bytes")

        dfu.write_memory(flash_base, firmware, progress_cb=flash_prog)

        # Step 3: Restore config sector
        if have_config:
            if progress_cb:
                progress_cb(85, 100, "Restoring config sector...")
            # Erase sector 1 first
            dfu.erase_page(CONFIG_ADDR)
            # Write saved config
            dfu.write_block(CONFIG_ADDR, config_data)
        else:
            if progress_cb:
                progress_cb(85, 100, "No config to restore")

        # Step 4: Leave DFU mode — jump to firmware base
        if progress_cb:
            progress_cb(95, 100, "Starting firmware...")
        dfu.leave_dfu(flash_base)

    except Exception as e:
        return f"Flash failed: {e}"
    finally:
        try:
            dfu.usb.release_interface()
        except Exception:
            pass
        try:
            dfu.usb.close()
        except Exception:
            pass

    if progress_cb:
        progress_cb(100, 100, "Done!")
    return ""


# ---- Quick self-test ----

def main():
    """List DFU devices"""
    devices = find_dfu_devices()
    if not devices:
        print("No DFU devices found")
        return
    for d in devices:
        print(f"  Bus {d.usb.bus:03d} Device {d.usb.address:03d}: "
              f"ID {d.usb.vid:04x}:{d.usb.pid:04x}")
        try:
            status = d.get_status()
            print(f"    Status: {status}")
        except Exception as e:
            print(f"    Status error: {e}")
        finally:
            d.usb.release_interface()
            d.usb.close()


if __name__ == "__main__":
    main()
