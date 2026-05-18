#!/usr/bin/env python3
import argparse
import json
import time

import serial


def open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.write_timeout = 1
    ser.dsrdtr = False
    ser.rtscts = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.setDTR(False)
    ser.setRTS(False)
    return ser


def send_json(ser: serial.Serial, payload: dict) -> None:
    msg = json.dumps(payload, separators=(",", ":")) + "\n"
    ser.write(msg.encode("utf-8"))
    ser.flush()


def read_json_lines(ser: serial.Serial, seconds: float) -> list[dict]:
    deadline = time.time() + seconds
    objects = []
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        text = raw.decode("utf-8", "replace").strip()
        try:
            objects.append(json.loads(text))
        except json.JSONDecodeError:
            pass
    return objects


def wait_until_ready(ser: serial.Serial, seconds: float) -> None:
    deadline = time.time() + seconds
    saw_boot_output = False
    quiet_deadline = time.time() + 1.0
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            if not saw_boot_output and time.time() >= quiet_deadline:
                return
            continue
        text = raw.decode("utf-8", "replace").strip()
        saw_boot_output = True
        if "UGV started." in text:
            return


def main() -> int:
    parser = argparse.ArgumentParser(description="Read WAVE ROVER status over serial.")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.25)
    parser.add_argument("--wait", type=float, default=20.0)
    args = parser.parse_args()

    with open_serial(args.port, args.baud, args.timeout) as ser:
        wait_until_ready(ser, args.wait)
        send_json(ser, {"T": 143, "cmd": 0})
        time.sleep(0.1)
        send_json(ser, {"T": 130})
        packets = read_json_lines(ser, 5.0)

    feedback = [p for p in packets if p.get("T") == 1001]
    if not feedback:
        print("No feedback packet received.")
        return 1

    latest = feedback[-1]
    print(json.dumps(latest, indent=2, sort_keys=True))
    if "v" in latest:
        print(f"voltage: {latest['v']:.3f} V")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
