"""Live dashboard for the dead-reckoning board.

Reads the 100 Hz telemetry stream from the board's serial port and serves it to
a browser page over server-sent events. Run it and open the URL it prints.

A plain static file server cannot do this - it has no way to reach the serial
port - so this script is both the serial reader and the web server. The only
dependency beyond the standard library is pyserial, which the capture script
already uses.
"""
import json
import queue
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
import serial.tools.list_ports

PORT = 8000
BAUD = 921600
HERE = Path(__file__).resolve().parent

# Field layout of the "S," rows emitted by main.c, in order after the tag.
FIELDS = [
    "ms", "state", "flags",
    "q0", "q1", "q2", "q3",
    "bx", "by", "bz",
    "a_z", "v_z", "h", "P",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "mx", "my", "mz",
    "baro", "temp",
]

subscribers = []
subscribers_lock = threading.Lock()


def find_port():
    for p in serial.tools.list_ports.comports():
        if "STLink" in p.description or "ST-Link" in p.description:
            return p.device
    return None


def publish(sample):
    """Hand a sample to every connected browser, dropping it for any client
    that has fallen behind rather than blocking the serial reader."""
    with subscribers_lock:
        targets = list(subscribers)
    for q in targets:
        try:
            q.put_nowait(sample)
        except queue.Full:
            pass


def serial_reader(port):
    print("reading %s at %d baud" % (port, BAUD))
    with serial.Serial(port, BAUD, timeout=1.0) as ser:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line.startswith("S,"):
                # Status lines and boot messages are passed through for the log
                # pane rather than parsed.
                if line:
                    publish({"log": line})
                continue
            parts = line.split(",")[1:]
            if len(parts) != len(FIELDS):
                continue
            try:
                publish({k: float(v) for k, v in zip(FIELDS, parts)})
            except ValueError:
                pass


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # the default logger prints a line per request, which is noise here

    def do_GET(self):
        if self.path == "/":
            body = (HERE / "dashboard.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()

            q = queue.Queue(maxsize=500)
            with subscribers_lock:
                subscribers.append(q)
            try:
                while True:
                    # Batch whatever has arrived so the browser gets about 20
                    # updates a second rather than 100 separate events.
                    batch = [q.get()]
                    while len(batch) < 50:
                        try:
                            batch.append(q.get_nowait())
                        except queue.Empty:
                            break
                    payload = json.dumps(batch)
                    self.wfile.write(("data: %s\n\n" % payload).encode())
                    self.wfile.flush()
            except (ConnectionError, OSError):
                # Windows raises ConnectionAbortedError when a tab closes; the
                # base classes cover that and the posix equivalents.
                pass
            finally:
                with subscribers_lock:
                    if q in subscribers:
                        subscribers.remove(q)
            return

        self.send_error(404)


def main():
    port = find_port()
    if port is None:
        print("No ST-LINK virtual COM port found. Plug the board in.", file=sys.stderr)
        return 1

    threading.Thread(target=serial_reader, args=(port,), daemon=True).start()

    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print("dashboard on http://127.0.0.1:%d  (ctrl-c to stop)" % PORT)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
