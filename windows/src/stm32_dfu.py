"""
Native STM32 DFU protocol implementation via Linux usbfs.
Implements AN3156 (USB DFU protocol used in the STM32 bootloader).
No external dependencies — uses ctypes + fcntl.ioctl on /dev/bus/usb/.
"""

import ctypes
import fcntl
import os
import struct
import sys
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


class UsbDevFsGetDesc(ctypes.Structure):
    _fields_ = [
        ("bRequestType", ctypes.c_uint8),
        ("bDescType", ctypes.c_uint8),
        ("wIndex", ctypes.c_uint16),
        ("wLength", ctypes.c_uint16),
        ("pData", ctypes.c_void_p),
    ]


USBDEVFS_CONTROL = _IOWR(ord("U"), 0, ctypes.sizeof(UsbDevFsCtrlTransfer))
USBDEVFS_CLAIMINTERFACE = _IOR(ord("U"), 15, ctypes.sizeof(ctypes.c_uint))
USBDEVFS_RELEASEINTERFACE = _IOR(ord("U"), 16, ctypes.sizeof(ctypes.c_uint))
USBDEVFS_GETDESCRIPTOR = _IOR(ord("U"), 19, ctypes.sizeof(UsbDevFsGetDesc))

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

# DFU states (DFU 1.1 spec — dfuIDLE is 2, NOT 0!)
DFU_STATE_APP_IDLE = 0x00
DFU_STATE_APP_DETACH = 0x01
DFU_STATE_IDLE = 0x02
DFU_STATE_DNLOAD_SYNC = 0x03
DFU_STATE_DNBUSY = 0x04
DFU_STATE_DNLOAD_IDLE = 0x05
DFU_STATE_MANIFEST_SYNC = 0x06
DFU_STATE_MANIFEST = 0x07
DFU_STATE_MANIFEST_WAIT_RESET = 0x08
DFU_STATE_UPLOAD_IDLE = 0x09
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

    def set_configuration(self, configuration: int = 1) -> bool:
        """Standard SET_CONFIGURATION. A freshly enumerated STM32 ROM bootloader
        is commonly left unconfigured (no kernel driver binds it, so no
        interface dirs appear in sysfs) and STALLs DFU class requests on an
        unconfigured interface — configure it before the DFU handshake."""
        try:
            self.control_transfer(0x00, 0x09, configuration, 0, b"")
            return True
        except OSError as e:
            print(f"  Set configuration failed: {e}")
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
        else:
            data_or_len = bytes(data_or_len)
            data_len = len(data_or_len)
        if data_len == 0:
            # c_char.from_buffer needs at least 1 byte even when wLength=0
            # (clear_status/abort send empty zero-length OUT transfers).
            data_buf = bytearray(1)
        else:
            data_buf = bytearray(data_or_len)

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

    def _read_descriptor(self, bm_request_type: int, b_descriptor_type: int,
                         w_index: int, w_length: int) -> bytes:
        """Fetch a USB descriptor via USBDEVFS_GETDESCRIPTOR. Works even before
        the kernel has configured/bound the device — which is exactly the state
        of a freshly enumerated ROM bootloader (sysfs shows no interface dirs)."""
        buf = bytearray(w_length)
        data_ptr = ctypes.c_char.from_buffer(buf)
        desc_buf = bytearray(ctypes.sizeof(UsbDevFsGetDesc))
        desc = UsbDevFsGetDesc.from_buffer(desc_buf)
        desc.bRequestType = bm_request_type
        desc.bDescType = b_descriptor_type
        desc.wIndex = w_index
        desc.wLength = w_length
        desc.pData = ctypes.addressof(data_ptr)
        fcntl.ioctl(self.fd, USBDEVFS_GETDESCRIPTOR, desc_buf)
        return bytes(buf)

    def get_config_interfaces(self) -> list:
        """Parse the config descriptor for (bInterfaceClass, bInterfaceSubClass,
        bInterfaceProtocol) of every interface. Returns () when unavailable."""
        ifaces = []
        header = self._read_descriptor(0x80, 0x02, 0, 9)  # DT_CONFIG, index 0
        if len(header) < 4:
            return ifaces
        total = header[2] | (header[3] << 8)
        if total > 4096:
            total = 4096
        if total < 9:
            return ifaces
        data = self._read_descriptor(0x80, 0x02, 0, total)
        i, n = 0, len(data)
        while i + 2 <= n:
            blen, btype = data[i], data[i + 1]
            if blen < 2:
                break
            if btype == 0x04 and i + 9 <= n:  # DT_INTERFACE
                ifaces.append((data[i + 5], data[i + 6], data[i + 7]))
            i += blen
        return ifaces

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
        self.last_transfer_error = ""

    def _ctrl_out(self, req: int, value: int, data: bytes = b"") -> bytes:
        return self.usb.control_transfer(
            DFU_BM_REQ_OUT, req, value, self.interface, data
        )

    def _ctrl_in(self, req: int, value: int, length: int) -> bytes:
        return self.usb.control_transfer(
            DFU_BM_REQ_IN, req, value, self.interface, length
        )

    def get_status(self) -> tuple:
        """Return (status, poll_timeout, state, str_index).

        DFU_GETSTATUS response is exactly 6 bytes: bStatus(1) +
        bwPollTimeout(3, little-endian) + bState(1) + iString(1)."""
        raw = self._ctrl_in(DFU_REQ_GETSTATUS, 0, 6)
        status, tmo3, state, str_idx = struct.unpack("<B3sBb", raw)
        poll_to = int.from_bytes(tmo3, "little")
        return status, poll_to, state, str_idx

    def get_state(self) -> int:
        raw = self._ctrl_in(DFU_REQ_GETSTATE, 0, 1)
        return raw[0]

    def clear_status(self):
        self._ctrl_out(DFU_REQ_CLRSTATUS, 0)

    def abort(self):
        self._ctrl_out(DFU_REQ_ABORT, 0)

    def _wait_idle(self, timeout_ms: int = 5000) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000.0
        self.last_states = []
        stuck_ms = 0
        while time.monotonic() < deadline:
            try:
                status, poll_to, state, _ = self.get_status()
                self.last_states.append((status, poll_to, state))
                if state == DFU_STATE_IDLE:
                    return True
                if state == DFU_STATE_ERROR:
                    self.clear_status()
                    stuck_ms = 0
                elif state in (DFU_STATE_DNLOAD_SYNC, DFU_STATE_DNLOAD_IDLE,
                               DFU_STATE_UPLOAD_IDLE):
                    self.abort()
                    stuck_ms = 0
                elif state == DFU_STATE_DNBUSY:
                    # Some STM32 ROM bootloaders boot into dfuDNBUSY and only
                    # leave it after a CLR_STATUS; treat a long DNBUSY as stuck.
                    stuck_ms += max(poll_to, 1)
                    if stuck_ms > 2000:
                        self.clear_status()
                        stuck_ms = 0
                # Cap the per-iteration sleep: ST bootloaders may echo a huge
                # bwPollTimeout (e.g. 0xFFFFFF ~ 16.7 s) that would otherwise
                # starve this polling loop beyond its deadline.
                time.sleep(min(max(poll_to, 1), 100) / 1000.0)
            except OSError as e:
                self.last_transfer_error = f"{e.errno} {e.strerror}"
                time.sleep(0.01)
            except Exception as e:
                self.last_transfer_error = str(e)
                time.sleep(0.01)
        return False

    def _wait_busy(self, poll_timeout: int, timeout_ms: int = 30000) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            try:
                _, poll_to, state, _ = self.get_status()
                if state != DFU_STATE_DNBUSY:
                    return state == DFU_STATE_DNLOAD_IDLE or state == DFU_STATE_IDLE
                time.sleep(min(max(poll_timeout, 1), 1000) / 1000.0)
            except OSError as e:
                self.last_transfer_error = f"{e.errno} {e.strerror}"
                time.sleep(0.01)
            except Exception as e:
                self.last_transfer_error = str(e)
                time.sleep(0.01)
        return False

    def set_address(self, addr: int) -> bool:
        """Set address pointer (STM32 Set Address Pointer command)"""
        cmd = struct.pack("<BI", STM_CMD_SET_ADDRESS, addr)
        self._ctrl_out(DFU_REQ_DNLOAD, 0, cmd)
        _, poll_to, state, _ = self.get_status()
        if state != DFU_STATE_DNBUSY:
            return False
        time.sleep(min(max(poll_to, 1), 1000) / 1000.0)
        _, _, state, _ = self.get_status()
        return state != DFU_STATE_ERROR

    def erase_page(self, addr: int) -> bool:
        """Erase the flash sector containing addr (STM32 Erase command)."""
        cmd = struct.pack("<BI", STM_CMD_ERASE, addr)
        self._ctrl_out(DFU_REQ_DNLOAD, 0, cmd)
        _, poll_to, state, _ = self.get_status()
        if state != DFU_STATE_DNBUSY:
            return False
        time.sleep(min(max(poll_to, 1), 1000) / 1000.0)
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
        """Write data (any size) to already-erased flash at addr."""
        return self._write_blocks(addr, data)

    def _write_blocks(self, addr: int, data: bytes, progress_cb=None) -> bool:
        """Download `data` to already-erased flash at `addr`. Set Address
        Pointer once, then DNLOAD each <= transfer_size chunk with strictly
        incrementing block numbers (dfu-util semantics)."""
        if not self.set_address(addr):
            return False
        pos = 0
        block = 2
        total = len(data)
        while pos < total:
            chunk = data[pos:pos + self.transfer_size]
            try:
                self._ctrl_out(DFU_REQ_DNLOAD, block, chunk)
            except OSError as e:
                self.last_transfer_error = f"{e.errno} {e.strerror}"
                return False
            if not self._wait_txn():
                if not self.last_transfer_error:
                    self.last_transfer_error = "STALL/timeout after DNLOAD"
                return False
            pos += len(chunk)
            block += 1
            if progress_cb:
                progress_cb(pos, total, f"Flashing... {pos}/{total}")
        return True

    def _wait_txn(self, timeout_ms: int = 30000) -> bool:
        """After a DNLOAD the device is DNBUSY, then settles to
        DNLOAD_IDLE or IDLE. True when settled, False on ERROR/timeout."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            try:
                _, poll_to, state, _ = self.get_status()
                if state == DFU_STATE_ERROR:
                    return False
                if state in (DFU_STATE_DNLOAD_IDLE, DFU_STATE_IDLE):
                    return True
                time.sleep(min(max(poll_to, 1), 100) / 1000.0)
            except OSError as e:
                self.last_transfer_error = f"{e.errno} {e.strerror}"
                time.sleep(0.01)
            except Exception as e:
                self.last_transfer_error = str(e)
                time.sleep(0.01)
        return False

    def write_memory(self, addr: int, data: bytes, progress_cb=None) -> bool:
        """Write data to flash, erasing pages as needed. Splits into
        transfer_size chunks with incrementing block numbers."""
        total = len(data)
        if total == 0:
            return True
        page = addr & ~(SECTOR_SIZE - 1)
        end = (addr + total + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1)
        while page < end:
            if not self.erase_page(page):
                raise RuntimeError(f"Erase failed @ 0x{page:08X}")
            page += SECTOR_SIZE
        if not self._write_blocks(addr, data, progress_cb):
            raise RuntimeError("DFU download failed")
        if progress_cb:
            progress_cb(total, total, f"Flashing... {total}/{total}")
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

def scan_usb_sysfs() -> tuple:
    """Enumerate every USB device from sysfs with DFU flag, per-interface
    class info and a usbfs openability probe. Returns (devices, diagnostics).

    A device that is present in /sys but whose /dev/bus/usb node cannot be
    opened is still reported (with open_err) instead of being silently dropped
    — that is the classic 'no DFU devices found' when flags are right and flash
    should work (Flatpak sandbox, missing udev rule, not in plugdev)."""
    devices = []
    diagnostics = []
    sysfs = "/sys/bus/usb/devices"
    if not os.path.isdir(sysfs):
        return devices, ["/sys/bus/usb/devices missing: no USB visible to the GCS"]

    for name in sorted(os.listdir(sysfs)):
        devpath = os.path.join(sysfs, name)
        if ":" in name or not os.path.isdir(devpath):
            continue

        try:
            def _rsys(fn):
                with open(os.path.join(devpath, fn)) as f:
                    return f.read().strip()

            vid = int(_rsys("idVendor"), 16)
            pid = int(_rsys("idProduct"), 16)
            bus = int(_rsys("busnum"))
            devnum = int(_rsys("devnum"))

            interfaces = []
            dfu = False
            for ifname in os.listdir(devpath):
                if ":" not in ifname:
                    continue
                try:
                    ipath = os.path.join(devpath, ifname)
                    cls = int(open(os.path.join(ipath, "bInterfaceClass")).read().strip())
                    sub = int(open(os.path.join(ipath, "bInterfaceSubClass")).read().strip())
                    proto = int(open(os.path.join(ipath, "bInterfaceProtocol")).read().strip())
                    interfaces.append((cls, sub, proto))
                    dfu |= (cls == DFU_IFACE_CLASS and sub == DFU_IFACE_SUBCLASS)
                except (ValueError, OSError):
                    continue
        except (ValueError, OSError):
            continue  # device vanished mid-scan or unreadable

        node = f"/dev/bus/usb/{bus:03d}/{devnum:03d}"
        open_err = ""
        if os.path.exists(node):
            try:
                fd = os.open(node, os.O_RDWR)
                os.close(fd)
            except OSError as e:
                open_err = e.strerror or str(e)
        else:
            open_err = "no /dev/bus/usb node (sandbox / permissions)"

        devices.append({
            "bus": bus, "devnum": devnum, "vid": vid, "pid": pid,
            "dfu": dfu, "interfaces": interfaces, "open_err": open_err,
            "devpath": node,
        })

    return devices, diagnostics


def _probe_usb_dfu(usb, sysfs_interfaces) -> tuple:
    """Combine sysfs interface classes with a usbfs descriptor probe.

    A freshly enumerated STM32 ROM bootloader often has no interface dirs in
    sysfs yet (the kernel has not configured it), but GET_DESCRIPTOR still
    answers — so trust the descriptor walk, and finally fall back on the
    well-known ST DFU VID/PID."""
    interfaces = list(sysfs_interfaces)
    try:
        desc = usb.get_config_interfaces()
        seen = set(interfaces)
        for t in desc:
            if t not in seen:
                interfaces.append(t)
                seen.add(t)
    except OSError:
        pass
    dfu = any(c == DFU_IFACE_CLASS and s == DFU_IFACE_SUBCLASS
              for c, s, _ in interfaces)
    if not dfu:
        dfu = (usb.vid == STM32_DFU_VID and usb.pid == STM32_DFU_PID)
    return dfu, interfaces


def find_dfu_devices(diagnostics=None) -> list:
    """Find all STM32 DFU devices on USB. Discovery opens the device node but
    DEFERS the interface claim to the flash path (which retries on EBUSY) so a
    freshly enumerated bootloader is not missed. Populates `diagnostics` (if
    given) with everything the scan saw — used by the flasher UI."""
    result = []
    if diagnostics is None:
        collect = []
    else:
        collect = diagnostics
    all_devs, diag = scan_usb_sysfs()
    collect.extend(diag)

    for info in all_devs:
        if info["open_err"]:
            if info["dfu"]:
                collect.append(
                    f"DFU device {info['vid']:04X}:{info['pid']:04X} Bus {info['bus']:03d} "
                    f"Dev {info['devnum']:03d} visible in sysfs but not openable: "
                    f"{info['open_err']}")
            continue
        usb = UsbDevice(info["bus"], info["devnum"], info["vid"], info["pid"])
        if not usb.open():
            collect.append(f"Open failed for {info['devpath']}")
            continue
        is_dfu, interfaces = _probe_usb_dfu(usb, info["interfaces"])
        if not is_dfu:
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

    diagnostics = []
    devices = find_dfu_devices(diagnostics=diagnostics)
    if not devices:
        msg = "No DFU device found. Enter bootloader mode first."
        if diagnostics:
            msg += "\n" + "\n".join("  " + d for d in diagnostics)
        return msg

    dfu = devices[0]

    try:
        # A freshly enumerated ROM bootloader is often unconfigured (no kernel
        # driver, no sysfs interface dirs); DFU class requests STALL until the
        # device has a configuration, so set one first — then claim best-effort
        # (DFU download itself is pure control-transfer over EP0).
        if not dfu.usb.set_configuration(1) and progress_cb:
            progress_cb(0, 100, "Note: SET_CONFIGURATION failed (trying anyway)")
        for _ in range(3):
            if dfu.usb.claim_interface(0):
                break
            time.sleep(0.25)
        else:
            if progress_cb:
                progress_cb(0, 100, "Note: DFU interface not claimed (control-only flash)")

        if progress_cb:
            progress_cb(0, 100, "Entering DFU idle...")
        if not dfu._wait_idle(2000):
            dfu.clear_status()
            if not dfu._wait_idle(2000):
                detail = ""
                if dfu.last_transfer_error:
                    detail = f" (last transfer error: {dfu.last_transfer_error})"
                elif getattr(dfu, "last_states", None):
                    detail = f" (GET_STATUS never reached IDLE; saw " \
                             f"{', '.join('st=%d,pt=%d,state=%d' % s for s in dfu.last_states[-5:])})"
                return f"Failed to get DFU device into idle state{detail}"

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

        def flash_prog(pos, total, msg):
            pct = 10 + int(70 * pos / total)
            if progress_cb:
                progress_cb(pct, 100, msg)

        dfu.write_memory(flash_base, firmware, progress_cb=flash_prog)

        # Step 3: Restore config sector
        if have_config:
            if progress_cb:
                progress_cb(85, 100, "Restoring config sector...")
            # Erase sector 1 first, then write the saved config in chunks
            if not dfu.erase_page(CONFIG_ADDR):
                return f"Config restore failed: erase @ 0x{CONFIG_ADDR:08X}"
            if not dfu._write_blocks(CONFIG_ADDR, config_data):
                return (f"Config restore failed: "
                        f"{dfu.last_transfer_error or 'DFU download error'}")
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

def _raw_diag():
    """Probe a DFU device with NO pre-handshake: no SET_CONFIGURATION, no
    interface claim. Reveals the bootloader's native DFU state so we can tell a
    device/session problem from a bug in our own handshake."""
    all_devs, _ = scan_usb_sysfs()
    dfu = [d for d in all_devs if d["dfu"] and not d["open_err"]]
    if not dfu:
        print("No openable DFU device found.")
        return
    for d in dfu:
        print(f"\nRaw probe of {d['vid']:04X}:{d['pid']:04X} "
              f"Bus {d['bus']:03d} Dev {d['devnum']:03d}")
        usb = UsbDevice(d["bus"], d["devnum"], d["vid"], d["pid"])
        if not usb.open():
            print("  open failed")
            continue
        du = DfuDevice(usb)
        try:
            for i in range(3):
                raw = du._ctrl_in(DFU_REQ_GETSTATUS, 0, 6)
                print(f"  GET_STATUS[{i}] raw={raw.hex()} -> " + repr(du.get_status()))
                time.sleep(1.0)
            print("  sending CLRSTATUS (zero-length OUT as the very first command)...")
            du.clear_status()
            raw = du._ctrl_in(DFU_REQ_GETSTATUS, 0, 6)
            print(f"  after CLRSTATUS raw={raw.hex()} -> " + repr(du.get_status()))
            raw = du._ctrl_in(DFU_REQ_GETSTATE, 0, 1)
            print(f"  GET_STATE raw={raw.hex()} -> state={raw[0]}")
        except OSError as e:
            print(f"  USB error: {e}")
        finally:
            usb.close()


def main():
    """Diagnostic dump: every USB device seen, DFU flag, access state.

    Run `python3 stm32_dfu.py` while the FC is in bootloader mode. It reports
    exactly what find_dfu_devices() sees so a 'no DFU devices found' can be
    pinned to (a) the board not enumerating as DFU, (b) usbfs permissions /
    sandbox blocking /dev/bus/usb, or (c) a claim race.

    `python3 stm32_dfu.py raw` additionally probes the bootloader with zero
    pre-handshake (see _raw_diag)."""
    if len(sys.argv) > 1 and sys.argv[1] == "raw":
        _raw_diag()
        return
    all_devs, diag = scan_usb_sysfs()
    if not all_devs and not diag:
        print("No /sys/bus/usb/devices — no USB visible at all")
        return

    # Probe openable devices via usbfs descriptors (works pre-configuration).
    for d in all_devs:
        if d["open_err"]:
            continue
        usb = UsbDevice(d["bus"], d["devnum"], d["vid"], d["pid"])
        if not usb.open():
            continue
        is_dfu, ifaces = _probe_usb_dfu(usb, d["interfaces"])
        usb.close()
        d["dfu"] = is_dfu
        if ifaces:
            d["interfaces"] = ifaces

    print(f"{'Bus':>4}  {'Dev':>4}  {'VID:PID':>10}  DFU  {'Interfaces':<24} Access")
    for d in all_devs:
        ifaces = ",".join(f"{c:02X}/{s:02X}/{p:02X}" for c, s, p in d["interfaces"]) or "-"
        print(f"{d['bus']:4d}  {d['devnum']:4d}  "
              f"{d['vid']:04X}:{d['pid']:04X}  "
              f"{'YES' if d['dfu'] else ' - '}  {ifaces:<24} "
              f"{'OK' if not d['open_err'] else 'DENIED: ' + d['open_err']}")

    if diag:
        print("\nDiagnostics:")
        for line in diag:
            print("  " + line)

    dfu = [d for d in all_devs if d["dfu"]]
    if not dfu:
        print("\nNo DFU-mode device found.")
    else:
        found = [d for d in dfu if not d["open_err"]]
        print(f"\n{len(found)} openable DFU device(s) ready to flash.")
        for d in found:
            print(f"  Bus {d['bus']:03d} Dev {d['devnum']:03d} "
                  f"{d['vid']:04X}:{d['pid']:04X}")


if __name__ == "__main__":
    main()
