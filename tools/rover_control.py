#!/usr/bin/env python3
import argparse
import time
from dataclasses import dataclass

from minimal_rover_serial import CMD_PWM, CMD_STOP, PWM, open_port, write_command


def clamp_pwm(value: float) -> float:
    return max(-0.5, min(0.5, value))


def pwm_to_milli(value: float) -> int:
    return int(round(clamp_pwm(value) * 1000.0))


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
    def __init__(
        self,
        port: str = "/dev/serial0",
        baud: int = 460800,
        timeout: float = 0.05,
        pulse_duration_ms: int = 120,
    ):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.pulse_duration_ms = max(20, int(pulse_duration_ms))
        self.ser = None
        self.seq = 1

    def __enter__(self) -> "RoverController":
        args = argparse.Namespace(port=self.port, baud=self.baud)
        self.ser = open_port(args)
        self.ser.timeout = self.timeout
        self.ser.reset_input_buffer()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            self.stop()
        finally:
            if self.ser is not None:
                self.ser.close()
                self.ser = None

    def next_seq(self) -> int:
        seq = self.seq
        self.seq = 1 if self.seq >= 0xFFFF else self.seq + 1
        return seq

    def send_lr(self, left_pwm: float, right_pwm: float) -> None:
        if self.ser is None:
            raise RuntimeError("Serial port is not open")
        payload = PWM.pack(
            pwm_to_milli(left_pwm),
            pwm_to_milli(right_pwm),
            self.pulse_duration_ms,
            0,
        )
        write_command(self.ser, CMD_PWM, self.next_seq(), payload)

    def stop(self, repeats: int = 3, pause_s: float = 0.03) -> None:
        if self.ser is None:
            return
        for _ in range(max(1, repeats)):
            write_command(self.ser, CMD_STOP, self.next_seq())
            time.sleep(pause_s)

    def drive_pulse(
        self,
        x_pwm: float,
        z_pwm: float,
        duration_s: float,
        command_period_s: float = 0.10,
        stop_repeats: int = 3,
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
            "Basic WAVE ROVER control helper over the current minimal ESP32 binary serial protocol. "
            "Body frame: +X forward, +Z yaw is CCW/left turn."
        )
    )
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument(
        "--command-period",
        type=float,
        default=0.10,
        help="How often to refresh the bounded raw-PWM command during the pulse.",
    )
    parser.add_argument(
        "--pulse-duration-ms",
        type=int,
        default=120,
        help="Duration sent in each bounded raw-PWM command frame.",
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

    stop = subparsers.add_parser("stop", help="Send repeated STOP frames.")
    stop.add_argument("--repeats", type=int, default=3)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    with RoverController(
        port=args.port,
        baud=args.baud,
        timeout=args.timeout,
        pulse_duration_ms=args.pulse_duration_ms,
    ) as rover:
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
