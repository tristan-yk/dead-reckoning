"""Capture a few seconds of firmware serial output.

Finds the ST-LINK virtual COM port automatically, so it keeps working when
Windows reassigns the port number. Usage: capture.py [seconds] [baud]
"""
import sys
import time

import serial
import serial.tools.list_ports


def find_stlink_port():
    for p in serial.tools.list_ports.comports():
        if "STLink" in p.description or "ST-Link" in p.description:
            return p.device
    return None


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600

    port = find_stlink_port()
    if port is None:
        print("No ST-LINK virtual COM port found.", file=sys.stderr)
        return 1

    print(f"# {port} @ {baud} for {seconds}s")
    with serial.Serial(port, baud, timeout=0.2) as ser:
        deadline = time.time() + seconds
        while time.time() < deadline:
            line = ser.readline()
            if line:
                sys.stdout.write(line.decode("utf-8", errors="replace"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
