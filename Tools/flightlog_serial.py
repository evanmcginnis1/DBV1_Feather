#!/usr/bin/env python3
"""
flightlog_serial.py

Host-side serial console for the DBV1 flight controller.

- Acts as a plain line-based serial terminal: whatever you type is sent as ONE packet when you
  press Enter (no newline is sent, since the firmware treats each USB packet as a full message).
- When the flight controller streams a log (download_logs), the log is saved to
  <repository root>/FlightLogs/FLIGHTLOG<log_number>.csv

Usage:
    python3 Tools/flightlog_serial.py [port]

Requires pyserial:
    python3 -m pip install --user pyserial

NOTE: this file was generated entirely by Claude Code; it is not key to the functionality of the flight controller, but 
      rather was created to be a quick tool for simplifying the serial interface. 
"""

import csv
import sys
import threading
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None

# Resolved from this file's location (<repo>/Tools/), so it works wherever the repository is cloned
REPO_ROOT = Path(__file__).resolve().parent.parent
OUTPUT_DIR = REPO_ROOT / "FlightLogs"

# Must match the printf strings in Core/Src/USB_Handler.c
METADATA_HEADER_PREFIX = "Log Counter,"
DATA_HEADER_PREFIX = "Current state,"
EMPTY_LOG_MARKER = "contains no datapoints"
END_OF_LOG_MARKER = "End of log"

PORT_SCAN_INTERVAL_S = 1.0
READ_TIMEOUT_S = 0.2
PROGRESS_EVERY_N_ROWS = 50


def split_fields(line):
    return [field.strip() for field in line.split(",")]


class LogCapture:
    """Watches incoming serial lines and writes streamed flight logs to CSV.

    Transitions are driven only by line content, never by time, so a slow or paused stream just
    leaves the capture open until more data arrives.
    """

    IDLE, META, HEADER, DATA = range(4)

    def __init__(self, output_dir=OUTPUT_DIR):
        self.output_dir = Path(output_dir)
        self.lock = threading.Lock()
        self.state = self.IDLE
        self.metadata_header = None
        self.metadata_row = None
        self.log_number = None
        self.file = None
        self.writer = None
        self.path = None
        self.num_columns = 0
        self.num_rows = 0

    def feed(self, line):
        """Process one received line. Returns True if the line should be echoed to the terminal."""
        with self.lock:
            return self._feed(line)

    def close(self, reason=""):
        """Close the CSV in progress, if any. Whatever was received so far is kept."""
        with self.lock:
            self._finish(reason)
            self.state = self.IDLE

    def _feed(self, line):
        stripped = line.strip()

        if self.state == self.DATA:
            if not stripped:
                return False
            if stripped.startswith(END_OF_LOG_MARKER):
                self._finish()
                self.state = self.IDLE
                return False
            fields = split_fields(stripped)
            if self.num_rows == 0 and len(fields) > 1 and len(fields) != self.num_columns:
                # trust the first row over the header so a header/row mismatch doesn't lose the log
                print(f"[flightlog] Warning: header has {self.num_columns} columns but rows have "
                      f"{len(fields)}; saving rows as received")
                self.num_columns = len(fields)
            if len(fields) == self.num_columns:
                self._write_row(fields)
                return False
            # anything else means the stream is over (full logs have no end marker)
            self._finish()
            self.state = self.IDLE
            return self._feed(line)

        if self.state == self.META:
            fields = split_fields(stripped)
            try:
                self.log_number = int(fields[0])
            except ValueError:
                self.state = self.IDLE
                return self._feed(line)
            self.metadata_row = fields
            self.state = self.HEADER
            return True

        if self.state == self.HEADER:
            if stripped.startswith(DATA_HEADER_PREFIX):
                self._start(split_fields(stripped))
                self.state = self.DATA
                return False
            self.state = self.IDLE
            if EMPTY_LOG_MARKER in stripped:
                return True
            return self._feed(line)

        # IDLE
        if stripped.startswith(METADATA_HEADER_PREFIX):
            self.metadata_header = split_fields(stripped)
            self.state = self.META
        return True

    def _unique_stem(self):
        # log numbers are reused after a chip erase, so never overwrite an existing file
        stem = f"FLIGHTLOG{self.log_number}"
        suffix = 0
        while (self.output_dir / f"{stem}.csv").exists():
            suffix += 1
            stem = f"FLIGHTLOG{self.log_number}_{suffix}"
        return stem

    def _start(self, header_fields):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stem = self._unique_stem()
        self.path = self.output_dir / f"{stem}.csv"

        with open(self.output_dir / f"{stem}_meta.csv", "w", newline="") as meta_file:
            meta_writer = csv.writer(meta_file)
            meta_writer.writerow(self.metadata_header)
            meta_writer.writerow(self.metadata_row)

        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(header_fields)
        self.num_columns = len(header_fields)
        self.num_rows = 0
        print(f"[flightlog] Capturing log {self.log_number} -> {self.path}")

    def _write_row(self, fields):
        self.writer.writerow(fields)
        self.num_rows += 1
        if self.num_rows % PROGRESS_EVERY_N_ROWS == 0:
            self.file.flush()
            print(f"\r[flightlog] {self.num_rows} rows", end="", flush=True)

    def _finish(self, reason=""):
        if self.file is None:
            return
        self.file.close()
        self.file = None
        self.writer = None
        note = f" ({reason})" if reason else ""
        print(f"\r[flightlog] Saved {self.num_rows} rows to {self.path}{note}")


class Connection:
    """Owns the serial port. Waits forever for the port to appear and re-opens it if it drops."""

    def __init__(self, capture, port=None):
        self.capture = capture
        self.requested_port = port
        self.ser = None
        self.warned_multiple = False

    def send(self, text):
        ser = self.ser
        if ser is None:
            print("[flightlog] Not connected; message not sent")
            return
        try:
            # one write = one USB packet = one message to the firmware
            ser.write(text.encode())
            ser.flush()
        except (serial.SerialException, OSError) as error:
            print(f"[flightlog] Send failed: {error}")

    def _find_port(self):
        if self.requested_port:
            return self.requested_port
        candidates = [p.device for p in list_ports.comports() if "usbmodem" in p.device]
        if len(candidates) == 1:
            return candidates[0]
        if len(candidates) > 1 and not self.warned_multiple:
            self.warned_multiple = True
            print("[flightlog] Multiple USB serial devices found: " + ", ".join(candidates))
            print("[flightlog] Re-run with the port as an argument, or unplug the others")
        return None

    def _open(self):
        print("[flightlog] Waiting for flight controller...")
        while True:
            port = self._find_port()
            if port:
                try:
                    ser = serial.Serial(port, timeout=READ_TIMEOUT_S)
                    print(f"[flightlog] Connected to {port}")
                    return ser
                except (serial.SerialException, OSError):
                    pass
            time.sleep(PORT_SCAN_INTERVAL_S)

    def run(self):
        while True:
            self.ser = self._open()
            pending = b""
            try:
                while True:
                    # a quiet port is normal; just keep waiting
                    chunk = self.ser.read(self.ser.in_waiting or 1)
                    if not chunk:
                        continue
                    pending += chunk
                    *lines, pending = pending.split(b"\n")
                    for raw_line in lines:
                        line = raw_line.decode(errors="replace").rstrip("\r")
                        if self.capture.feed(line):
                            print(line)
            except (serial.SerialException, OSError) as error:
                print(f"\n[flightlog] Connection lost: {error}")
            ser, self.ser = self.ser, None
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
            self.capture.close("connection lost, log may be incomplete")


def main():
    if serial is None:
        sys.exit("pyserial is not installed. Run: python3 -m pip install --user pyserial")

    port = sys.argv[1] if len(sys.argv) > 1 else None
    capture = LogCapture()
    connection = Connection(capture, port)
    threading.Thread(target=connection.run, daemon=True).start()

    print("[flightlog] Type a command and press Enter to send it. Ctrl-C to quit.")
    try:
        while True:
            # input() only returns on Enter, so the whole line goes out at once
            text = input().strip()
            if text:
                connection.send(text)
    except (KeyboardInterrupt, EOFError):
        pass
    capture.close("stopped by user, log may be incomplete")
    print("\n[flightlog] Exiting")


if __name__ == "__main__":
    main()
