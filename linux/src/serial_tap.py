#!/usr/bin/env python3
"""
Serial tap — creates a PTY that bridges to a real serial port,
logging every byte in both directions.

Usage (as root):
    ./serial_tap.py [--port /dev/ttyUSB0] [--baud 115200]

Then connect your flashing tool to the PTY printed, e.g.:
    sudo stm32flash -w fw.bin -v /dev/pts/3

All bytes are logged to stdout and optionally a file.
"""

import argparse, os, pty, select, sys, threading, time
from binascii import hexlify

try:
    import serial
except ImportError:
    serial = None


def _hex(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


class SerialTap:
    def __init__(self, real_port: str, baud: int = 115200,
                 log_file: str = None):
        self.real_port = real_port
        self.baud = baud
        self.log_file = log_file
        self._stop = threading.Event()
        self._log_fh = None

    def _log(self, direction: str, data: bytes):
        if not data:
            return
        ts = time.monotonic()
        line = f"{ts:09.3f} {direction} ({len(data):3d})  {_hex(data)}"
        print(line, flush=True)
        if self._log_fh:
            self._log_fh.write(line + "\n")
            self._log_fh.flush()

    def run(self):
        if serial is None:
            print("ERROR: pyserial not available", file=sys.stderr)
            return 1

        if self.log_file:
            self._log_fh = open(self.log_file, "w")

        # Open the real serial port
        try:
            ser = serial.Serial(
                self.real_port, self.baud,
                bytesize=8, parity="E", stopbits=1,
                timeout=0.05, rtscts=False, dsrdtr=False,
            )
            ser.baudrate = self.baud
            ser.parity = "E"
        except Exception as e:
            print(f"ERROR opening serial {self.real_port}: {e}", file=sys.stderr)
            return 1

        # Create PTY pair
        master_fd, slave_fd = os.openpty()
        pty_name = os.ttyname(slave_fd)

        # Set PTY to raw mode so it passes bytes through unchanged
        import termios
        attrs = termios.tcgetattr(master_fd)
        attrs[0] = attrs[0] & ~(termios.IGNBRK | termios.BRKINT |
                                termios.PARMRK | termios.ISTRIP |
                                termios.INLCR | termios.IGNCR |
                                termios.ICRNL | termios.IXON)
        attrs[1] = attrs[1] & ~termios.OPOST
        attrs[2] = attrs[2] & ~(termios.CSIZE | termios.PARENB |
                                termios.CSTOPB | termios.CREAD |
                                termios.CLOCAL)
        attrs[3] = attrs[3] & ~(termios.ECHO | termios.ECHONL |
                                termios.ICANON | termios.ISIG |
                                termios.IEXTEN)
        termios.tcsetattr(master_fd, termios.TCSANOW, attrs)

        print(f"=== Serial Tap ===", file=sys.stderr)
        print(f"  Real port: {self.real_port} @ {self.baud} 8E1", file=sys.stderr)
        print(f"  Tap PTY:   {pty_name}", file=sys.stderr)
        print(f"  Connect your tool to {pty_name}", file=sys.stderr)
        if self.log_file:
            print(f"  Log file:  {self.log_file}", file=sys.stderr)
        print(file=sys.stderr)
        print("  Bytes logged to stdout; press Ctrl+C to stop", file=sys.stderr)
        print(file=sys.stderr)
        print("TX→ means bytes TO the device, RX← means bytes FROM the device", file=sys.stderr)
        print(f"{'TIME':>9s}  DIR  LEN  HEX", file=sys.stderr)
        print("-" * 60, file=sys.stderr)

        def pty_to_serial():
            """Read from PTY master → write to real serial port."""
            while not self._stop.is_set():
                r, _, _ = select.select([master_fd], [], [], 0.1)
                if r:
                    try:
                        data = os.read(master_fd, 1024)
                        if not data:
                            break
                        self._log("TX→", data)
                        ser.write(data)
                        ser.flush()
                    except OSError:
                        break

        def serial_to_pty():
            """Read from real serial port → write to PTY master."""
            while not self._stop.is_set():
                try:
                    data = ser.read(512)
                    if data:
                        self._log("RX←", data)
                        os.write(master_fd, data)
                except serial.SerialException:
                    break

        threads = [
            threading.Thread(target=pty_to_serial, daemon=True),
            threading.Thread(target=serial_to_pty, daemon=True),
        ]
        for t in threads:
            t.start()

        try:
            while all(t.is_alive() for t in threads):
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop.set()
            ser.close()
            os.close(master_fd)
            os.close(slave_fd)
            if self._log_fh:
                self._log_fh.close()

        return 0


def main():
    ap = argparse.ArgumentParser(description="Serial tap / log proxy")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    ap.add_argument("--log", default=None, help="Log file path (optional)")
    args = ap.parse_args()

    tap = SerialTap(args.port, args.baud, args.log)
    return tap.run()


if __name__ == "__main__":
    sys.exit(main())
