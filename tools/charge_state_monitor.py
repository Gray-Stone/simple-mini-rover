#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

from charge_detection import (
    ChargeDetector,
    ChargeObservation,
    format_optional,
    summarize_numeric_fields,
)
from rover_motion_probe import (
    drain_serial,
    open_serial,
    score_chassis_packet,
    select_best_packet,
    send_json,
    wait_until_ready,
)


def read_feedback_packet(ser, read_window_s: float) -> dict | None:
    send_json(ser, {"T": 130})
    packets = drain_serial(ser, read_window_s)
    return select_best_packet(packets, score_chassis_packet)


def print_observation(
    observation: ChargeObservation,
    *,
    changed: bool,
    verbose: bool,
    show_fields: bool,
) -> None:
    if not verbose and not changed:
        return

    prefix = "STATE" if changed else "sample"
    print(
        f"{prefix} {observation.timestamp_utc} "
        f"state={observation.state} "
        f"method={observation.method} "
        f"voltage={format_optional(observation.voltage_v)}V "
        f"baseline={format_optional(observation.voltage_baseline_v)}V "
        f"delta={format_optional(observation.voltage_delta_v)}V "
        f"current={format_optional(observation.current_a)}A"
        + (f"({observation.current_key})" if observation.current_key else "")
        + " "
        f"power={format_optional(observation.power_w)}W"
        + (f"({observation.power_key})" if observation.power_key else "")
        + (
            f" explicit={observation.explicit_state}({observation.explicit_key})"
            if observation.explicit_key
            else ""
        )
    )
    if show_fields:
        print(json.dumps(summarize_numeric_fields(observation.packet), sort_keys=True))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Monitor WAVE ROVER chassis feedback and detect transitions from not charging "
            "to charging using an explicit status/current field when available, otherwise "
            "a conservative voltage-rise heuristic."
        )
    )
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.10)
    parser.add_argument("--wait", type=float, default=3.0, help="Boot/quiet wait after opening.")
    parser.add_argument("--poll-period", type=float, default=0.5)
    parser.add_argument("--read-window", type=float, default=0.20)
    parser.add_argument("--duration", type=float, help="Optional finite run time in seconds.")
    parser.add_argument(
        "--log-jsonl",
        type=Path,
        help="Optional JSONL file for all observations.",
    )
    parser.add_argument(
        "--show-fields",
        action="store_true",
        help="Print the numeric feedback fields seen in each selected packet.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print every sample instead of only state transitions.",
    )
    parser.add_argument(
        "--current-threshold-a",
        type=float,
        default=0.05,
        help="Minimum absolute current to trust a current-based charging decision.",
    )
    parser.add_argument(
        "--power-threshold-w",
        type=float,
        default=0.5,
        help="Minimum absolute power to trust a power-based charging decision.",
    )
    parser.add_argument(
        "--negative-current-means-charging",
        action="store_true",
        help="Interpret negative current as charging instead of positive current.",
    )
    parser.add_argument(
        "--negative-power-means-charging",
        action="store_true",
        help="Interpret negative power as charging instead of positive power.",
    )
    parser.add_argument(
        "--voltage-rise-v",
        type=float,
        default=0.20,
        help="Required rise above recent baseline before voltage-only charging is declared.",
    )
    parser.add_argument(
        "--voltage-release-v",
        type=float,
        default=0.08,
        help="Charging clears once voltage falls back near the latched baseline by this margin.",
    )
    parser.add_argument(
        "--confirm-samples",
        type=int,
        default=3,
        help="Consecutive voltage-rise hits required to declare charging in voltage-only mode.",
    )
    parser.add_argument(
        "--release-samples",
        type=int,
        default=2,
        help="Consecutive near-baseline samples required to clear charging in voltage-only mode.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    detector = ChargeDetector(
        current_threshold_a=args.current_threshold_a,
        power_threshold_w=args.power_threshold_w,
        voltage_rise_v=args.voltage_rise_v,
        voltage_release_v=args.voltage_release_v,
        confirm_samples=args.confirm_samples,
        release_samples=args.release_samples,
        negative_current_means_charging=args.negative_current_means_charging,
        negative_power_means_charging=args.negative_power_means_charging,
    )

    if args.log_jsonl:
        args.log_jsonl.parent.mkdir(parents=True, exist_ok=True)

    deadline = time.monotonic() + args.duration if args.duration is not None else None
    previous_state = None

    with open_serial(args.port, args.baud, args.timeout) as ser:
        wait_until_ready(ser, args.wait)
        send_json(ser, {"T": 143, "cmd": 0})
        time.sleep(0.05)

        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break

            packet = read_feedback_packet(ser, args.read_window)
            if packet is None:
                time.sleep(args.poll_period)
                continue

            observation = detector.update(packet)
            changed = previous_state is not None and observation.state != previous_state
            print_observation(
                observation,
                changed=changed or previous_state is None,
                verbose=args.verbose,
                show_fields=args.show_fields,
            )
            previous_state = observation.state

            if args.log_jsonl:
                with args.log_jsonl.open("a", encoding="utf-8") as fh:
                    fh.write(json.dumps(observation.as_dict(), sort_keys=True) + "\n")

            time.sleep(args.poll_period)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
