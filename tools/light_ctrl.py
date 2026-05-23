#!/usr/bin/env python3
import argparse
import subprocess
import sys


GPIO_BCM = 18
HEADER_PIN = 12
GROUND_PIN = 39


def run_command(args: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        args,
        check=True,
        capture_output=True,
        text=True,
        timeout=2,
    )


def pinctrl_set(level_high: bool) -> None:
    run_command(["pinctrl", "set", str(GPIO_BCM), "op", "dh" if level_high else "dl"])


def pin_state() -> str:
    result = run_command(["raspi-gpio", "get", str(GPIO_BCM)])
    return result.stdout.strip()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control the custom Pi-header docking light on GPIO18."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("on", help="Drive GPIO18 high to turn the light on.")
    subparsers.add_parser("off", help="Drive GPIO18 low to turn the light off.")
    subparsers.add_parser("status", help="Report the current GPIO18 state.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.command == "on":
        pinctrl_set(True)
        print(
            f"light=on bcm={GPIO_BCM} pin=P{HEADER_PIN} ground=P{GROUND_PIN}",
            flush=True,
        )
    elif args.command == "off":
        pinctrl_set(False)
        print(
            f"light=off bcm={GPIO_BCM} pin=P{HEADER_PIN} ground=P{GROUND_PIN}",
            flush=True,
        )
    elif args.command == "status":
        print(pin_state(), flush=True)
    else:
        print(f"unknown command: {args.command}", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
