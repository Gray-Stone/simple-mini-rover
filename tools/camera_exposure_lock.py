#!/usr/bin/env python3
import argparse
import subprocess
import sys


CONTROL_NAMES = [
    "auto_exposure",
    "exposure_time_absolute",
    "exposure_dynamic_framerate",
    "gain",
    "white_balance_automatic",
    "white_balance_temperature",
    "backlight_compensation",
    "contrast",
    "focus_absolute",
    "focus_automatic_continuous",
]


def v4l2_device_arg(device: str) -> str:
    if device.isdigit():
        return f"/dev/video{device}"
    return device


def run_v4l2(device: str, args: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["v4l2-ctl", "-d", v4l2_device_arg(device), *args],
        check=True,
        capture_output=True,
        text=True,
        timeout=2,
    )


def parse_controls(output: str) -> dict[str, int]:
    values: dict[str, int] = {}
    for line in output.splitlines():
        if ":" not in line:
            continue
        name, rest = line.split(":", 1)
        token = rest.strip().split(maxsplit=1)[0] if rest.strip() else ""
        try:
            values[name.strip()] = int(token)
        except ValueError:
            continue
    return values


def read_controls(device: str) -> dict[str, int]:
    result = run_v4l2(device, ["-C", ",".join(CONTROL_NAMES)])
    return parse_controls(result.stdout)


def print_controls(values: dict[str, int]) -> None:
    for name in CONTROL_NAMES:
        if name in values:
            print(f"{name}={values[name]}")


def lock_current_exposure(device: str, scale: float) -> tuple[dict[str, int], int]:
    before = read_controls(device)
    current = int(before["exposure_time_absolute"])
    locked = max(1, int(round(current * scale)))

    run_v4l2(device, ["-c", "auto_exposure=1"])
    run_v4l2(device, ["-c", "exposure_dynamic_framerate=0"])
    run_v4l2(device, ["-c", f"exposure_time_absolute={locked}"])
    after = read_controls(device)
    return after, locked


def set_auto_exposure(device: str) -> dict[str, int]:
    run_v4l2(device, ["-c", "auto_exposure=3"])
    run_v4l2(device, ["-c", "exposure_dynamic_framerate=0"])
    return read_controls(device)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read or lock the current camera exposure using v4l2-ctl."
    )
    parser.add_argument("--camera", default="/dev/video0")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("status", help="Print the current camera control values.")
    lock_parser = subparsers.add_parser(
        "lock-current",
        help="Read the current exposure value and switch to manual mode at that value.",
    )
    lock_parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Multiply the current auto exposure by this factor before locking it.",
    )
    subparsers.add_parser("auto", help="Return exposure mode to auto.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.command == "status":
            print_controls(read_controls(args.camera))
            return 0

        if args.command == "lock-current":
            controls, locked = lock_current_exposure(args.camera, args.scale)
            print(f"locked_exposure_time_absolute={locked}")
            print_controls(controls)
            return 0

        if args.command == "auto":
            print_controls(set_auto_exposure(args.camera))
            return 0
    except (subprocess.SubprocessError, KeyError, ValueError) as exc:
        print(f"camera_exposure_lock_failed: {exc}", file=sys.stderr)
        return 1

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
