#!/usr/bin/env python3
import argparse
import json
import time
from dataclasses import dataclass

import serial


def clamp_pwm(value: float) -> float:
    return max(-0.5, min(0.5, value))


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


def body_to_lr(x_pwm: float, z_pwm: float) -> tuple[float, float]:
    x_pwm = clamp_pwm(x_pwm)
    z_pwm = clamp_pwm(z_pwm)
    left = clamp_pwm(x_pwm - z_pwm)
    right = clamp_pwm(x_pwm + z_pwm)
    return left, right


@dataclass
class DriveCommand:
    x_pwm: float
    z_pwm: float
    left_pwm: float
    right_pwm: float
    duration_s: float


class RoverController:
    def __init__(self, port: str = "/dev/serial0", baud: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: serial.Serial | None = None

    def __enter__(self) -> "RoverController":
        self.ser = open_serial(self.port, self.baud, self.timeout)
        time.sleep(0.2)
        send_json(self.ser, {"T": 143, "cmd": 0})
        time.sleep(0.05)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            self.stop()
        finally:
            if self.ser is not None:
                self.ser.close()
                self.ser = None

    def send_lr(self, left_pwm: float, right_pwm: float) -> None:
        if self.ser is None:
            raise RuntimeError("Serial port is not open")
        send_json(
            self.ser,
            {
                "T": 1,
                "L": clamp_pwm(left_pwm),
                "R": clamp_pwm(right_pwm),
            },
        )

    def stop(self, repeats: int = 6, pause_s: float = 0.05) -> None:
        if self.ser is None:
            return
        for _ in range(repeats):
            self.send_lr(0.0, 0.0)
            time.sleep(pause_s)

    def drive_pulse(
        self,
        x_pwm: float,
        z_pwm: float,
        duration_s: float,
        command_period_s: float = 0.10,
        stop_repeats: int = 8,
    ) -> DriveCommand:
        left_pwm, right_pwm = body_to_lr(x_pwm, z_pwm)
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.send_lr(left_pwm, right_pwm)
            time.sleep(command_period_s)
        self.stop(repeats=stop_repeats)
        return DriveCommand(
            x_pwm=clamp_pwm(x_pwm),
            z_pwm=clamp_pwm(z_pwm),
            left_pwm=left_pwm,
            right_pwm=right_pwm,
            duration_s=duration_s,
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Basic WAVE ROVER Phase 3 control helper. "
            "Body frame: +X forward, +Z yaw is CCW/left turn."
        )
    )
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument(
        "--command-period",
        type=float,
        default=0.10,
        help="How often to repeat T=1 during the pulse.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    pulse = subparsers.add_parser(
        "pulse",
        help="Send a body-frame pulse with configurable +X forward PWM and +Z turn PWM.",
    )
    pulse.add_argument("--x-pwm", type=float, default=0.0, help="Forward command in [-0.5, 0.5].")
    pulse.add_argument("--z-pwm", type=float, default=0.0, help="Turn command in [-0.5, 0.5].")
    pulse.add_argument("--duration", type=float, required=True, help="Pulse duration in seconds.")

    forward = subparsers.add_parser(
        "forward-test",
        help="Convenience wrapper for quick forward deadband testing.",
    )
    forward.add_argument("--pwm", type=float, required=True, help="Forward PWM in [-0.5, 0.5].")
    forward.add_argument("--duration", type=float, required=True, help="Pulse duration in seconds.")

    stop = subparsers.add_parser("stop", help="Send repeated zero commands.")
    stop.add_argument("--repeats", type=int, default=8)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    with RoverController(port=args.port, baud=args.baud, timeout=args.timeout) as rover:
        if args.command == "pulse":
            cmd = rover.drive_pulse(
                x_pwm=args.x_pwm,
                z_pwm=args.z_pwm,
                duration_s=args.duration,
                command_period_s=args.command_period,
            )
        elif args.command == "forward-test":
            cmd = rover.drive_pulse(
                x_pwm=args.pwm,
                z_pwm=0.0,
                duration_s=args.duration,
                command_period_s=args.command_period,
            )
        elif args.command == "stop":
            rover.stop(repeats=args.repeats)
            print("stop sent")
            return 0
        else:
            parser.error(f"Unsupported command: {args.command}")

    print(
        "pulse sent: "
        f"x_pwm={cmd.x_pwm:.3f} "
        f"z_pwm={cmd.z_pwm:.3f} "
        f"L={cmd.left_pwm:.3f} "
        f"R={cmd.right_pwm:.3f} "
        f"duration={cmd.duration_s:.3f}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
