#!/usr/bin/env python3
"""
Run stm32flash through a serial tap, logging all bytes.
Usage:
    sudo ./capture_loader.py <bin_file> [stm32flash_args...]

Example:
    sudo ./capture_loader.py UAVXF4V3Q_r11.bin -v
"""

import argparse, os, pty, select, subprocess, sys, threading, time

try:
    import serial
except ImportError:
    serial = None


def _hex(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


def main():
    if serial is None:
        print("ERROR: pyserial not available", file=sys.stderr)
        return 1

    ap = argparse.ArgumentParser(description="Capture stm32flash serial traffic")
    ap.add_argument("bin_file", help="Firmware .bin file")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    ap.add_argument("--log", default=None, help="Log file path")
    args, extra = ap.parse_known_args()

    real_port = args.port
    log_fh = open(args.log, "w") if args.log else None

    # Open real serial port
    try:
        ser = serial.Serial(
            real_port, args.baud,
            bytesize=8, parity="E", stopbits=1,
            timeout=0.05, rtscts=False, dsrdtr=False,
        )
        ser.baudrate = args.baud
        ser.parity = "E"
    except Exception as e:
        print(f"ERROR opening serial {real_port}: {e}", file=sys.stderr)
        return 1

    # Create PTY
    master_fd, slave_fd = os.openpty()
    pty_name = os.ttyname(slave_fd)

    # Set PTY raw
    import termios
    attrs = termios.tcgetattr(master_fd)
    for fl in (0, 1, 2):
        attrs[fl] = attrs[fl] & ~(termios.IGNBRK | termios.BRKINT |
                                   termios.PARMRK | termios.ISTRIP |
                                   termios.INLCR | termios.IGNCR |
                                   termios.ICRNL | termios.IXON)
    attrs[1] = attrs[1] & ~termios.OPOST
    attrs[2] = attrs[2] & ~(termios.CSIZE | termios.PARENB |
                             termios.CSTOPB | termios.CREAD | termios.CLOCAL)
    attrs[3] = attrs[3] & ~(termios.ECHO | termios.ECHONL |
                             termios.ICANON | termios.ISIG | termios.IEXTEN)
    termios.tcsetattr(master_fd, termios.TCSANOW, attrs)

    stop = threading.Event()

    def log(direction: str, data: bytes):
        if not data:
            return
        ts = time.monotonic()
        line = f"{ts:09.3f} {direction} ({len(data):3d})  {_hex(data)}"
        print(line, flush=True)
        if log_fh:
            log_fh.write(line + "\n")
            log_fh.flush()

    # Bridge threads
    def pty_to_serial():
        while not stop.is_set():
            r, _, _ = select.select([master_fd], [], [], 0.1)
            if r:
                try:
                    data = os.read(master_fd, 1024)
                    if not data:
                        break
                    log("TX→", data)
                    ser.write(data)
                    ser.flush()
                except OSError:
                    break

    def serial_to_pty():
        while not stop.is_set():
            try:
                data = ser.read(512)
                if data:
                    log("RX←", data)
                    os.write(master_fd, data)
            except serial.SerialException:
                break

    threads = [
        threading.Thread(target=pty_to_serial, daemon=True),
        threading.Thread(target=serial_to_pty, daemon=True),
    ]
    for t in threads:
        t.start()

    # Build stm32flash command
    cmd = ["stm32flash"] + extra + ["-w", args.bin_file, "-v", pty_name]

    print(f"=== Serial Capture ===", file=sys.stderr)
    print(f"  Real port: {real_port} @ {args.baud} 8E1", file=sys.stderr)
    print(f"  PTY:       {pty_name}", file=sys.stderr)
    print(f"  Command:   {' '.join(cmd)}", file=sys.stderr)
    if log_fh:
        print(f"  Log:       {args.log}", file=sys.stderr)
    print(file=sys.stderr)

    try:
        rc = subprocess.call(cmd, timeout=120)
    except subprocess.TimeoutExpired:
        print("stm32flash timed out (120s)", file=sys.stderr)
        rc = 1
    except FileNotFoundError:
        print("ERROR: stm32flash not found. Install: sudo apt install stm32flash",
              file=sys.stderr)
        rc = 1
    finally:
        stop.set()
        ser.close()
        os.close(master_fd)
        os.close(slave_fd)
        if log_fh:
            log_fh.close()

    return rc


if __name__ == "__main__":
    sys.exit(main())
