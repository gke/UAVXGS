"""
STM32 UART bootloader protocol (AN3155) for STM32F4.
Flashing with config sector preservation over a serial port.
"""

import os, struct, time
from typing import Optional, Callable

# ---- Protocol constants ----

ACK = 0x79
NACK = 0x1F

# Command codes from AN3155
CMD_GET = 0x00
CMD_GVR = 0x01
CMD_GID = 0x02
CMD_RM = 0x11
CMD_GO = 0x21
CMD_WM = 0x31
CMD_ER = 0x43         # Standard erase
CMD_EE = 0x44         # Extended erase (STM32F4+)
CMD_WP = 0x63
CMD_WU = 0x73
CMD_RP = 0x82
CMD_RU = 0x92
CMD_SP = 0x99

# Flash geometry
FLASH_BASE = 0x08000000
CONFIG_ADDR = 0x08004000
SECTOR_SIZE = 16 * 1024  # 16 KB for sectors 0-3
PAGE_SIZE = 1024          # Write chunk size for UART bootloader

# Parity setting
PARITY_EVEN = "E"
PARITY_NONE = "N"


# ---- Protocol helpers ----

def _xor(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c


# ---- UART Bootloader Protocol ----

class UartBootloader:
    """AN3155 STM32 UART bootloader protocol"""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0,
                 io_dump: bool = False):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self._supported_cmds: list[int] = []
        self.io_dump = io_dump
        self._io_log: list[str] = []

    # ---- low-level I/O ----

    def _open(self, timeout: float = None, parity: str = PARITY_EVEN) -> bool:
        import serial
        try:
            self.ser = serial.Serial(
                self.port, self.baud, bytesize=8, parity=parity,
                stopbits=1, timeout=timeout if timeout is not None else self.timeout,
            )
            # Re-apply settings explicitly after open — some adapters ignore the
            # constructor setting until the port is (re)configured.
            self.ser.baudrate = self.baud
            self.ser.parity = parity
            self.ser.bytesize = 8
            self.ser.stopbits = 1
            return True
        except serial.SerialException as e:
            if "Permission denied" in str(e):
                print(f"  Permission denied on {self.port} — add user to dialout group or install udev rule")
            else:
                print(f"  Open failed: {e}")
            return False
        except Exception as e:
            print(f"  Open failed: {e}")
            return False

    def _set_timeout(self, timeout: float):
        if self.ser:
            self.ser.timeout = timeout

    def _close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _flush_input(self):
        if self.ser:
            self.ser.reset_input_buffer()

    def _flush_output(self):
        if self.ser:
            self.ser.reset_output_buffer()

    def _send(self, data: bytes):
        self.ser.write(data)
        self.ser.flush()
        if self.io_dump:
            line = f"TX: {data.hex()}"
            self._io_log.append(line)
            print(f"  {line}")

    def _recv(self, length: int = 1) -> bytes:
        data = self.ser.read(length)
        if data and self.io_dump:
            line = f"RX: {data.hex()}"
            self._io_log.append(line)
            print(f"  {line}")
        return data

    def _send_cmd(self, cmd: int) -> bool:
        """Send command byte + XOR checksum, wait for ACK.
        Returns True only on ACK (0x79). NACK or timeout returns False.
        Handles bootloaders that echo transmitted bytes back."""
        payload = bytes([cmd, cmd ^ 0xFF])
        self._send(payload)
        deadline = time.time() + self.ser.timeout
        while time.time() < deadline:
            resp = self._recv(1)
            if not resp:
                continue
            if resp[0] == ACK:
                return True
            if resp[0] == NACK:
                self.sync_diag.append(f"cmd 0x{cmd:02X}: NACK")
                return False
        self.sync_diag.append(f"cmd 0x{cmd:02X}: timeout")
        return False

    def _send_data_with_xor(self, data: bytes) -> bool:
        """Send data bytes + plain XOR checksum, wait for ACK.
        Per AN3155, data frames use plain XOR (total=0x00);
        only command frames use XOR-to-0xFF (total=0xFF).
        Handles bootloaders that echo transmitted bytes back."""
        cksum = _xor(data)
        self._send(data + bytes([cksum]))
        deadline = time.time() + self.ser.timeout
        while time.time() < deadline:
            resp = self._recv(1)
            if not resp:
                continue
            if resp[0] == ACK:
                return True
            if resp[0] == NACK:
                return False
        return False

    def _recv_data_with_xor(self, length: int) -> Optional[bytes]:
        """Receive length data bytes + 1 XOR byte, return data if checksum OK."""
        raw = self._recv(length + 1)
        if len(raw) < length + 1:
            return None
        data = raw[:length]
        cksum = raw[length]
        if _xor(data) != cksum:
            return None
        return data

    def _recv_until_ack(self) -> bool:
        received = b""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            b = self._recv(1)
            if not b:
                continue
            if b[0] == ACK:
                return True
            if b[0] == NACK:
                return False
            received += b
        return False

    # ---- sync ----

    def _sync_once(self, parity: str, baud: int,
                   progress_cb: Optional[Callable] = None) -> bool:
        """Attempt sync on the already-open port at a given parity/baud.
        Modeled on stm32flash: send 0x7F once, wait for ACK or NACK.
        Returns True if bootloader is alive (ACK = fresh, NACK = already active).
        Returns False on timeout (no bootloader) or 0x7F echo (firmware running)."""
        self.ser.baudrate = baud
        self.ser.parity = parity
        self._flush_input()
        self._flush_output()
        # Send 0x7F sync byte. The bootloader auto-detects baud from this frame.
        # If bootloader just reset → ACK. If already in cmd mode → NACK.
        self._set_timeout(self.timeout)
        if progress_cb:
            progress_cb(1, 100, f"Sync ({baud} {parity})...")
        self._send(b"\x7f")
        resp = self._recv(1)
        if resp:
            byte = resp[0]
            if byte == ACK:
                self.sync_diag.append(
                    f"sync {baud} {parity}: ACK (0x79) — bootloader fresh")
                return True
            if byte == NACK:
                self.sync_diag.append(
                    f"sync {baud} {parity}: NACK (0x1F) — bootloader alive, already active")
                return True
            if byte == 0x7F:
                self.sync_diag.append(
                    f"sync {baud} {parity}: echo (0x7F) — firmware running, not bootloader")
                return False
            self.sync_diag.append(
                f"sync {baud} {parity}: got 0x{byte:02X} (expected 0x79 or 0x1F)")
        else:
            self.sync_diag.append(f"sync {baud} {parity}: no response (timeout)")
        return False

    # ---- commands ----

    def get_version(self) -> Optional[int]:
        """Get bootloader protocol version (GVR command, 0x01).
        Returns version byte on success. This is the simpler connectivity
        check used by stm32flash before GET."""
        if not self._send_cmd(CMD_GVR):
            return None
        # Read version + 2 option bytes, then final ACK
        raw = self._recv(3)
        if not raw or len(raw) < 1:
            return None
        ver = raw[0]
        ack = self._recv(1)
        if not ack or ack[0] != ACK:
            return None
        return ver

    def get_commands(self) -> Optional[list[int]]:
        """Get bootloader version and supported commands.
        AN3155 GET response format varies across BL versions: some include
        the trailing ACK in the N+1 byte count, some send it separately.
        Handle both: if raw[-1] is ACK, exclude it; otherwise read separately."""
        if not self._send_cmd(CMD_GET):
            return None
        b = self._recv(1)
        if not b:
            return None
        n = b[0]  # N = number of bytes to follow - 1
        raw = self._recv(n + 1)  # Read N+1 bytes
        if len(raw) != n + 1:
            return None
        if raw[-1] == ACK:
            # Trailing ACK is inside the N+1 data
            ver = raw[0]
            cmds = list(raw[1:-1])
        else:
            # Trailing ACK is separate — read it now
            ver = raw[0]
            cmds = list(raw[1:])
            ack = self._recv(1)
            if not ack or ack[0] != ACK:
                return None
        self._supported_cmds = cmds
        return cmds

    def get_id(self) -> Optional[int]:
        """Get chip ID."""
        if not self._send_cmd(CMD_GID):
            return None
        len_byte = self._recv(1)
        if not len_byte:
            return None
        n = len_byte[0]
        if n < 1 or n > 4:
            return None
        raw = self._recv(n + 1)
        if len(raw) != n + 1:
            return None
        if _xor(raw[:n]) != raw[n]:
            return None
        ack = self._recv(1)
        if not ack or ack[0] != ACK:
            return None
        val = 0
        for b in raw[:n]:
            val = (val << 8) | b
        return val

    def read_memory(self, addr: int, length: int, chunk: int = 256,
                    progress_cb: Optional[Callable] = None) -> Optional[bytes]:
        """Read memory from address. AN3155 Read Memory allows at most 256
        bytes per command, so longer reads are split into multiple full
        Read-Memory transactions (CMD + address + length each). Each data
        block is read in a loop to tolerate short reads under timeout."""
        out = bytearray()
        remaining = length
        pos = 0
        while remaining > 0:
            n = min(remaining, chunk)
            if not self._send_cmd(CMD_RM):
                return None
            addr_bytes = struct.pack(">I", addr + pos)
            if not self._send_data_with_xor(addr_bytes):
                return None
            # Send length-1 with XOR, wait for ACK
            nb = max(0, n - 1) & 0xFF
            self._send(bytes([nb, nb ^ 0xFF]))
            resp = self._recv(1)
            if not resp or resp[0] != ACK:
                return None
            # Read exactly n bytes (loop for short reads)
            buf = bytearray()
            deadline = time.monotonic() + self.timeout + 1.0
            while len(buf) < n and time.monotonic() < deadline:
                chunk_data = self._recv(n - len(buf))
                if chunk_data:
                    buf.extend(chunk_data)
                else:
                    time.sleep(0.01)
            if len(buf) != n:
                return None
            out.extend(buf)
            remaining -= n
            pos += n
            if progress_cb:
                progress_cb(pos, length, f"Read {pos}/{length} bytes")
        return bytes(out)

    def write_memory(self, addr: int, data: bytes) -> bool:
        """Write data bytes to address (must be in an erased sector).
        AN3155 §3.6: data must be padded to 4-byte alignment with 0xFF.
        Max 256 bytes per call (single-byte n on wire; n+1 = len).
        Use chunked calls for larger payloads."""
        if len(data) > 256:
            self.sync_diag.append(f"write: len>256 ({len(data)})")
            return False
        if not self._send_cmd(CMD_WM):
            self.sync_diag.append(f"write: CMD_WM NACK at 0x{addr:08X}")
            return False
        addr_bytes = struct.pack(">I", addr)
        if not self._send_data_with_xor(addr_bytes):
            self.sync_diag.append(f"write: addr NACK at 0x{addr:08X}")
            return False
        aligned_len = (len(data) + 3) & ~3
        padded = data + b"\xff" * (aligned_len - len(data))
        n = aligned_len - 1
        payload = bytes([n & 0xFF]) + padded
        if not self._send_data_with_xor(payload):
            self.sync_diag.append(f"write: data NACK at 0x{addr:08X} len={len(data)}")
            return False
        return True

    def _recover_bootloader(self) -> bool:
        """Recover bootloader from a stuck data-waiting state.
        Sends a break to reset UART, flushes buffers, an invalid command
        0xFF to force bootloader back to command-waiting."""
        try:
            self.ser.send_break(duration=0.1)
        except Exception:
            pass
        self._flush_input()
        self._flush_output()
        prev = self.ser.timeout
        self.ser.timeout = 1
        try:
            self._send(b"\xff\x00")  # Invalid command 0xFF + XOR checksum
            resp = self._recv(1)
            return len(resp) == 1
        finally:
            self.ser.timeout = prev

    def _erase_extended(self, sectors: list[int]) -> bool:
        """Extended erase (0x44) — per stm32flash: count and page numbers
        are 2-byte MSB-first (big-endian), plain XOR checksum.
        Erase can take up to 2 minutes for F40x. Echo-aware."""
        count = len(sectors)
        payload = struct.pack(">H", count - 1) + b"".join(
            struct.pack(">H", s) for s in sectors
        )
        for alt_cksum in (False, True):
            if alt_cksum:
                # Try XOR-to-0xFF checksum (some bootloaders use this for all frames)
                cksum = _xor(payload) ^ 0xFF
            else:
                cksum = _xor(payload)
            frame = payload + bytes([cksum])
            self.sync_diag.append(
                f"  extended erase data ({len(frame)} bytes: {frame.hex()})"
                f"  cksum={'XOR2FF' if alt_cksum else 'XOR'}"
            )
            self._flush_input()
            self._send(frame)
            deadline = time.time() + 120
            while time.time() < deadline:
                resp = self._recv(1)
                if not resp:
                    continue
                if resp[0] == ACK:
                    self.sync_diag.append("  extended erase ACK")
                    return True
                if resp[0] == NACK:
                    self.sync_diag.append("  extended erase NACK")
                    if alt_cksum:
                        return False
                    break
            self.sync_diag.append("  extended erase: no ACK, retrying alt checksum")
        self.sync_diag.append("  extended erase: timeout")
        return False

    def erase_sectors(self, sectors: list[int]) -> bool:
        """Erase sectors via Extended Erase (0x44) — this bootloader
        does NOT support standard erase (0x43). Uses 2-byte MSB-first
        count and page numbers per stm32flash. Retries with recovery."""
        self.sync_diag.append(f"erase: extended erase (0x44) pages {sectors}")
        for attempt in range(2):
            if not self._send_cmd(CMD_EE):
                self.sync_diag.append(f"  CMD_EE NACK (attempt {attempt+1})")
                self._recover_bootloader()
                continue
            if self._erase_extended(sectors):
                return True
            self.sync_diag.append(f"  extended erase: attempt {attempt+1} failed")
            self._recover_bootloader()
        self.sync_diag.append("erase: failed after 2 attempts")
        return False

    def go(self, addr: int = FLASH_BASE) -> bool:
        """Start execution at address (default 0x08000000)."""
        if not self._send_cmd(CMD_GO):
            return False
        addr_bytes = struct.pack(">I", addr)
        return self._send_data_with_xor(addr_bytes)

    def _probe_bootloader(self) -> bool:
        """After sync, probe the bootloader with GVR then GET.
        Returns True and populates _supported_cmds on success.
        Flushes input at end to leave clean serial state."""
        self._flush_input()
        time.sleep(0.05)
        gvr_ok = self.get_version() is not None
        self._flush_input()
        if self.get_commands() is None:
            if not gvr_ok:
                self.sync_diag.append("GET also failed")
                return False
        if self._supported_cmds:
            hex_cmds = " ".join(f"0x{c:02X}" for c in self._supported_cmds)
            self.sync_diag.append(f"  supported cmds: {hex_cmds}")
        self._flush_input()
        return True

    def enter(self, progress_cb: Optional[Callable] = None) -> bool:
        """Open serial port and sync with bootloader.
        Modeled on stm32flash + florisla/stm32loader:
        1. Send 0x7F sync byte — accept ACK or NACK (bootloader alive)
        2. Flush input + 50ms settle
        3. Probe with GVR, then GET
        Falls back to other baud/parity if primary fails.
        Primary 57600 8E1 matches what stm32flash auto-negotiates for this bootloader.
        """
        self.sync_diag = []

        for parity, baud in [
            (PARITY_EVEN, 57600),   # stm32flash default for STM32F4 bootloader
            (PARITY_NONE, 57600),
            (PARITY_EVEN, 115200),
            (PARITY_NONE, 115200),
            (PARITY_EVEN, 9600),
        ]:
            if self._open(parity=parity):
                if self._sync_once(parity, baud, progress_cb):
                    if self._probe_bootloader():
                        return True
                self._close()
        return False

    def exit(self):
        self._close()


# ---- High-level flash function ----

# STM32F405/407 sector layout
_SECTOR_LAYOUT = [
    (0x08000000, 16 * 1024),   # 0
    (0x08004000, 16 * 1024),   # 1
    (0x08008000, 16 * 1024),   # 2
    (0x0800C000, 16 * 1024),   # 3
    (0x08010000, 64 * 1024),   # 4
    (0x08020000, 128 * 1024),  # 5
    (0x08040000, 128 * 1024),  # 6
    (0x08060000, 128 * 1024),  # 7
    (0x08080000, 128 * 1024),  # 8
    (0x080A0000, 128 * 1024),  # 9
    (0x080C0000, 128 * 1024),  # 10
    (0x080E0000, 128 * 1024),  # 11
]

def wait_for_port(port: str, timeout: float = 12.0,
                  progress_cb: Optional[Callable] = None) -> bool:
    """Wait for serial port to disappear then reappear (MCU reset cycle).
    Returns True when port is ready, False on timeout.
    If the port is already stable (BOOT0 case), returns immediately.
    """
    poll = 0.25
    rounds = int(timeout / poll)
    absent_need = 8  # port absent for 8 polls ≈ 2 s = full reboot
    absent = 0

    # Phase 1 — wait for port to vanish (MCU resetting / CH340 re-enumerating)
    for _ in range(rounds):
        if not os.path.exists(port):
            absent += 1
            if absent >= absent_need:
                if progress_cb:
                    progress_cb(10, 100, "Port disappeared — MCU reset detected")
                break
        else:
            absent = 0
        time.sleep(poll)

    if absent < absent_need:
        # Port never vanished — FC may already be in bootloader (BOOT0 case)
        if progress_cb:
            progress_cb(10, 100, "Port stable — trying direct sync")
        return True

    # Phase 2 — wait for port to come back (USB re-enumeration)
    for _ in range(rounds):
        if os.path.exists(port):
            if progress_cb:
                progress_cb(15, 100, "Port reappeared — settling...")
            time.sleep(0.5)
            return True
        time.sleep(poll)

    return False


def _sectors_for_range(start: int, size: int) -> list[int]:
    """Return list of sector indices covered by address range [start, start+size)."""
    end = start + size
    indices = []
    for i, (saddr, ssize) in enumerate(_SECTOR_LAYOUT):
        s_end = saddr + ssize
        if saddr < end and s_end > start:
            indices.append(i)
    return indices


def flash_firmware_uart(
    port: str, bin_path: str, baud: int = 115200,
    progress_cb: Optional[Callable] = None,
    flash_base: int = FLASH_BASE,
    io_dump: bool = False,
) -> str:
    """Flash .bin to STM32F4 over UART bootloader (AN3155).
    flash_base is the address the firmware is linked for. UAVX firmware owns the
    whole flash and starts at 0x08000000. Boards with a resident bootloader in
    sector 0 (e.g. SpeedyBee/HY3) link firmware at 0x08004000.
    Preserves config sector (Sector 1, 0x08004000).
    Returns empty string on success, error message on failure.
    """
    if not os.path.exists(bin_path):
        return f"File not found: {bin_path}"

    with open(bin_path, "rb") as f:
        firmware = f.read()

    if not firmware:
        return "Empty firmware file"

    bl = UartBootloader(port, baud, io_dump=io_dump)
    synced = False
    for attempt in range(4):
        if bl.enter():
            synced = True
            break
        if progress_cb:
            progress_cb(0, 100, f"Sync attempt {attempt+1} failed, retrying...")
        bl._close()
        if attempt == 0:
            if progress_cb:
                progress_cb(0, 100, "Waiting for port to settle...")
            wait_for_port(port, progress_cb=progress_cb)
        time.sleep(1.0)
    if not synced:
        diag = ""
        if hasattr(bl, "sync_diag") and bl.sync_diag:
            diag = "\nDiagnostics:\n  " + "\n  ".join(bl.sync_diag)
        return (
            f"Failed to sync with bootloader on {port}.\n"
            f"Suggestions:\n"
            f"  - Ensure BOOT0 = 1 and press RESET\n"
            f"  - Power-cycle the FC (disconnect USB power completely)\n"
            f"  - Check serial adapter wiring (TX→RX, RX→TX, GND→GND)\n"
            f"  - Install udev rules for serial adapter permission (see obj/99-uavx-flasher.rules)"
            f"{diag}"
        )

    try:
        if progress_cb:
            progress_cb(2, 100, "Synced with bootloader")

        # Step 1: Save config sector (Sector 1)
        if progress_cb:
            progress_cb(5, 100, "Reading config sector...")
        config_data = None
        try:
            raw = bl.read_memory(CONFIG_ADDR, SECTOR_SIZE)
            if raw is not None:
                config_data = raw
        except Exception as e:
            if progress_cb:
                progress_cb(5, 100, f"Config read error: {e}")

        have_config = (
            config_data is not None
            and len(config_data) > 0
            and any(b != 0xFF for b in config_data)
        )
        # Only preserve the config sector when it lies OUTSIDE the firmware
        # region. For boards whose app starts at 0x08004000 (e.g. SpeedyBee/iNav,
        # which keep their bootloader in sector 0), the firmware overlaps the
        # config sector, so preserving/restoring it would corrupt the firmware.
        preserve_config = flash_base < CONFIG_ADDR
        if not preserve_config:
            have_config = False
        if have_config:
            magic = struct.unpack("<I", config_data[:4])[0]
            if progress_cb:
                progress_cb(8, 100,
                            f"Config saved ({len(config_data)} bytes, "
                            f"magic=0x{magic:08X})")
        else:
            if progress_cb:
                progress_cb(8, 100,
                            "No config to preserve"
                            if preserve_config else
                            "Config sector overlaps firmware — not preserved")

        # Step 2: Erase firmware sectors (everything except sector 1)
        fw_sectors = _sectors_for_range(flash_base, len(firmware))
        if progress_cb:
            progress_cb(10, 100, f"Erasing sectors {fw_sectors}...")
        if not bl.erase_sectors(fw_sectors):
            diag = ""
            if hasattr(bl, "sync_diag") and bl.sync_diag:
                diag = "\n  " + "\n  ".join(bl.sync_diag)
            bl.exit()
            return f"Failed to erase firmware sectors{diag}"

        # Step 3: Write firmware in chunks
        if progress_cb:
            progress_cb(15, 100, "Flashing firmware...")

        pos = 0
        total = len(firmware)
        write_size = 256

        while pos < total:
            chunk = firmware[pos:pos + write_size]
            chunk_addr = flash_base + pos
            if not bl.write_memory(chunk_addr, chunk):
                diag = ""
                if hasattr(bl, "sync_diag") and bl.sync_diag:
                    diag = "\n  " + "\n  ".join(bl.sync_diag)
                bl.exit()
                return f"Write failed at 0x{chunk_addr:08X}{diag}"
            pos += len(chunk)
            if progress_cb:
                pct = 15 + int(70 * pos / total)
                progress_cb(pct, 100, f"Flashing... {pos}/{total}")

        # Step 3.5: Verify firmware
        if progress_cb:
            progress_cb(85, 100, "Verifying firmware...")
        readback = bl.read_memory(flash_base, len(firmware))
        if readback is None:
            bl.exit()
            return "Verify failed: could not read back firmware"
        if readback != firmware:
            bl.exit()
            return "Verify failed: firmware mismatch"
        if progress_cb:
            progress_cb(87, 100, "Firmware verified")

        # Step 4: Restore config sector
        if have_config:
            if progress_cb:
                progress_cb(88, 100, "Restoring config sector...")
            # STM32 flash must be erased before re-writing.  The firmware write
            # filled sector 1 with default-config data, so erase it first.
            if not bl.erase_sectors([1]):
                bl.exit()
                return "Failed to erase sector 1 for config restore"
            # Write in chunks (AN3155 Write Memory max 256 bytes per cmd)
            cfg_pos = 0
            cfg_total = len(config_data)
            while cfg_pos < cfg_total:
                chunk = config_data[cfg_pos:cfg_pos + 256]
                if not bl.write_memory(CONFIG_ADDR + cfg_pos, chunk):
                    bl.exit()
                    return f"Failed to restore config sector at offset 0x{cfg_pos:04X}"
                cfg_pos += len(chunk)
            # Verify config
            if progress_cb:
                progress_cb(90, 100, "Verifying config...")
            readback = bl.read_memory(CONFIG_ADDR, len(config_data))
            if readback is None:
                bl.exit()
                return "Config restore verify failed: could not read back"
            if readback != config_data:
                bl.exit()
                return "Config restore verify failed: data mismatch"
            if progress_cb:
                progress_cb(92, 100, "Config restored and verified")
        else:
            if progress_cb:
                progress_cb(88, 100, "No config to restore")

        # Step 5: Start firmware
        if progress_cb:
            progress_cb(95, 100, "Starting firmware...")
        bl.go(flash_base)

    except Exception as e:
        bl.exit()
        return f"Flash failed: {e}"

    bl.exit()

    if progress_cb:
        progress_cb(100, 100, "Done!")
    return ""
