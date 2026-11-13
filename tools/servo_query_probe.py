#!/usr/bin/env python3
"""Read-only diagnostic for the serial path to the YH-K32 bus-servo controller.

Only transmits three query forms: PID, PMOD, PRAD. Never sends motion,
torque-release, reset, or configuration commands. Opening a serial port
may still reset a controller: check stored startup actions and support
the mechanism before use.

Prerequisite: pip install pyserial
Example:
    python3 tools/servo_query_probe.py --port /dev/ttyUSB0 --ids 000 001

A valid reply over USB is *not* guaranteed, even when the servo itself
supports PRAD. Save raw output and distinguish timeout from echo.
"""
import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit("pyserial missing: install it with 'python3 -m pip install pyserial'")

QUERY_SUFFIXES = (("id", "PID"), ("mode", "PMOD"), ("position", "PRAD"))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial device (prefer /dev/serial/by-id/...)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--ids", nargs="+", default=["000"], help="Servo IDs 000..254")
    parser.add_argument("--timeout", type=float, default=0.8, help="Seconds per query")
    args = parser.parse_args()
    if args.timeout <= 0 or args.baud <= 0:
        parser.error("baud and timeout must be positive")
    for servo_id in args.ids:
        if not re.fullmatch(r"\d{3}", servo_id) or int(servo_id) > 254:
            parser.error(f"invalid ID {servo_id!r}; use 000..254, never broadcast 255")
    return args


def receive_response(port, timeout):
    deadline = time.monotonic() + timeout
    data = bytearray()
    last_rx = None
    while time.monotonic() < deadline and len(data) < 1024:
        waiting = port.in_waiting
        chunk = port.read(waiting if waiting else 1)
        if chunk:
            data.extend(chunk)
            last_rx = time.monotonic()
        elif last_rx is not None and (time.monotonic() - last_rx) >= 0.15:
            break
    return bytes(data)


def classify(servo_id, query, response, command):
    if not response:
        return "TIMEOUT (no USB reply; does not prove servo has no feedback)"
    payload = response.decode("ascii", errors="replace").strip()
    if payload == command:
        return "ECHO ONLY (not position feedback)"
    if query == "position" and re.search(r"#" + re.escape(servo_id) + r"P\d{4}!", payload):
        return "POSITION RESPONSE FORMAT FOUND (requires physical consistency test)"
    if query == "mode" and re.search(r"#" + re.escape(servo_id) + r"PMOD[1-8]!", payload):
        return "MODE RESPONSE FORMAT FOUND"
    if query == "id" and ("#" + servo_id + "P!") in payload:
        return "ID RESPONSE FORMAT FOUND"
    return "UNCLASSIFIED (review raw bytes, including board echo)"


def main():
    args = parse_args()
    print(f"Port: {args.port} | {args.baud} 8N1 | query-only; no motion commands")
    print("Warning: opening the serial port may reset the board.")
    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.5,
        ) as port:
            for servo_id in args.ids:
                for query, suffix in QUERY_SUFFIXES:
                    command = f"#{servo_id}{suffix}!"
                    port.reset_input_buffer()
                    port.write(command.encode("ascii"))
                    port.flush()
                    response = receive_response(port, args.timeout)
                    print(f"ID={servo_id} query={query} TX={command!r} RX={response!r}")
                    print(f"  => {classify(servo_id, query, response, command)}")
                    time.sleep(0.12)
    except (serial.SerialException, OSError) as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
