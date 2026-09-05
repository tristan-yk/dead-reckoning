import serial
import sys

PORT = "COM11"
BAUD = 921600

try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1.0,
    )
except serial.SerialException as e:
    print(f"Failed to open {PORT}: {e}")
    sys.exit(1)

print(f"Listening on {PORT} @ {BAUD} baud")
try:
    while True:
        line = ser.readline()
        if line:
            try:
                print(line.decode("utf-8", errors="replace").rstrip())
            except UnicodeDecodeError:
                print(line)
except KeyboardInterrupt:
    print("\nExiting.")
finally:
    ser.close()
