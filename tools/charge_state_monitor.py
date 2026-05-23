#!/usr/bin/env python3
import argparse
import json
import math
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from rover_motion_probe import (
    drain_serial,
    extract_voltage,
    lower_numeric_map,
    open_serial,
    score_chassis_packet,
    select_best_packet,
    send_json,
    wait_until_ready,
)


EXPLICIT_STATE_KEYS = (
    "charging",
    "is_charging",
    "charge_state",
    "charger_state",
    "dock_state",
    "docked",
)

CURRENT_KEYS_AMPS = (
    "current",
    "current_a",
    "charge_current",
    "charge_current_a",
    "battery_current",
    "battery_current_a",
    "input_current",
    "input_current_a",
    "amps",
    "amp",
)

CURRENT_KEYS_MILLIAMPS = (
    "current_ma",
    "charge_current_ma",
    "battery_current_ma",
    "input_current_ma",
    "milliamps",
    "ma",
)

POWER_KEYS_WATTS = (
    "power",
    "power_w",
    "charge_power",
    "charge_power_w",
    "input_power",
    "input_power_w",
    "watts",
)

POWER_KEYS_MILLIWATTS = (
    "power_mw",
    "charge_power_mw",
    "input_power_mw",
    "milliwatts",
    "mw",
)

TRUTHY = {"1", "true", "charging", "docked", "on", "yes"}
FALSY = {"0", "false", "not_charging", "off", "no"}


def now_utc() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def parse_boolish(value) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        if value in (0, 1):
            return bool(value)
        return None
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in TRUTHY:
            return True
        if normalized in FALSY:
            return False
    return None


def extract_explicit_charge_state(packet: dict | None) -> tuple[bool | None, str | None]:
    if not packet:
        return None, None
    for key, value in packet.items():
        state = parse_boolish(value)
        if state is None:
            continue
        if key.lower() in EXPLICIT_STATE_KEYS:
            return state, key
    return None, None


def extract_named_numeric(
    packet: dict | None,
    unit_keys: tuple[str, ...],
    scale: float = 1.0,
) -> tuple[float | None, str | None]:
    if not packet:
        return None, None
    lower = lower_numeric_map(packet)
    for key in unit_keys:
        if key in lower:
            return lower[key] * scale, key
    return None, None


def extract_current_a(packet: dict | None) -> tuple[float | None, str | None]:
    current_a, key = extract_named_numeric(packet, CURRENT_KEYS_AMPS)
    if key is not None:
        return current_a, key
    current_ma, key = extract_named_numeric(packet, CURRENT_KEYS_MILLIAMPS, scale=0.001)
    return current_ma, key


def extract_power_w(packet: dict | None) -> tuple[float | None, str | None]:
    power_w, key = extract_named_numeric(packet, POWER_KEYS_WATTS)
    if key is not None:
        return power_w, key
    power_mw, key = extract_named_numeric(packet, POWER_KEYS_MILLIWATTS, scale=0.001)
    return power_mw, key


def summarize_numeric_fields(packet: dict | None) -> dict[str, float]:
    return dict(sorted(lower_numeric_map(packet).items())) if packet else {}


def format_optional(value: float | None, digits: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "-"
    return f"{value:.{digits}f}"


@dataclass
class ChargeObservation:
    timestamp_utc: str
    state: str
    method: str
    voltage_v: float | None
    current_a: float | None
    current_key: str | None
    power_w: float | None
    power_key: str | None
    explicit_state: bool | None
    explicit_key: str | None
    voltage_baseline_v: float | None
    voltage_delta_v: float | None
    packet: dict

    def as_dict(self) -> dict:
        return {
            "timestamp_utc": self.timestamp_utc,
            "state": self.state,
            "method": self.method,
            "voltage_v": self.voltage_v,
            "current_a": self.current_a,
            "current_key": self.current_key,
            "power_w": self.power_w,
            "power_key": self.power_key,
            "explicit_state": self.explicit_state,
            "explicit_key": self.explicit_key,
            "voltage_baseline_v": self.voltage_baseline_v,
            "voltage_delta_v": self.voltage_delta_v,
            "packet": self.packet,
        }


class ChargeDetector:
    def __init__(
        self,
        current_threshold_a: float,
        power_threshold_w: float,
        voltage_rise_v: float,
        voltage_release_v: float,
        confirm_samples: int,
        release_samples: int,
        negative_current_means_charging: bool,
        negative_power_means_charging: bool,
    ):
        self.current_threshold_a = abs(current_threshold_a)
        self.power_threshold_w = abs(power_threshold_w)
        self.voltage_rise_v = abs(voltage_rise_v)
        self.voltage_release_v = abs(voltage_release_v)
        self.confirm_samples = max(1, confirm_samples)
        self.release_samples = max(1, release_samples)
        self.negative_current_means_charging = negative_current_means_charging
        self.negative_power_means_charging = negative_power_means_charging

        self.state = "unknown"
        self.voltage_samples: deque[float] = deque(maxlen=max(confirm_samples * 8, 20))
        self.rise_hits = 0
        self.release_hits = 0
        self.latched_baseline_v: float | None = None

    def update(self, packet: dict) -> ChargeObservation:
        timestamp_utc = now_utc()
        explicit_state, explicit_key = extract_explicit_charge_state(packet)
        current_a, current_key = extract_current_a(packet)
        power_w, power_key = extract_power_w(packet)
        voltage_v = extract_voltage(packet)

        voltage_baseline_v = None
        voltage_delta_v = None
        method = "unknown"

        if voltage_v is not None:
            self.voltage_samples.append(voltage_v)
            voltage_baseline_v = min(self.voltage_samples)
            voltage_delta_v = voltage_v - voltage_baseline_v

        if explicit_state is not None:
            self.state = "charging" if explicit_state else "not_charging"
            self.latched_baseline_v = voltage_baseline_v
            self.rise_hits = 0
            self.release_hits = 0
            method = f"explicit:{explicit_key}"
        elif current_a is not None and abs(current_a) >= self.current_threshold_a:
            charging = current_a < 0 if self.negative_current_means_charging else current_a > 0
            self.state = "charging" if charging else "not_charging"
            self.latched_baseline_v = voltage_baseline_v
            self.rise_hits = 0
            self.release_hits = 0
            method = f"current:{current_key}"
        elif power_w is not None and abs(power_w) >= self.power_threshold_w:
            charging = power_w < 0 if self.negative_power_means_charging else power_w > 0
            self.state = "charging" if charging else "not_charging"
            self.latched_baseline_v = voltage_baseline_v
            self.rise_hits = 0
            self.release_hits = 0
            method = f"power:{power_key}"
        elif voltage_v is not None and voltage_baseline_v is not None:
            if self.state != "charging":
                if voltage_delta_v is not None and voltage_delta_v >= self.voltage_rise_v:
                    self.rise_hits += 1
                else:
                    self.rise_hits = 0
                if self.rise_hits >= self.confirm_samples:
                    self.state = "charging"
                    self.latched_baseline_v = voltage_baseline_v
                    self.release_hits = 0
                elif self.state == "unknown":
                    self.state = "not_charging"
                method = "voltage-rise"
            else:
                baseline = (
                    self.latched_baseline_v if self.latched_baseline_v is not None else voltage_baseline_v
                )
                if voltage_v <= baseline + self.voltage_release_v:
                    self.release_hits += 1
                else:
                    self.release_hits = 0
                if self.release_hits >= self.release_samples:
                    self.state = "not_charging"
                    self.latched_baseline_v = min(self.voltage_samples)
                    self.rise_hits = 0
                method = "voltage-latched"

        return ChargeObservation(
            timestamp_utc=timestamp_utc,
            state=self.state,
            method=method,
            voltage_v=voltage_v,
            current_a=current_a,
            current_key=current_key,
            power_w=power_w,
            power_key=power_key,
            explicit_state=explicit_state,
            explicit_key=explicit_key,
            voltage_baseline_v=voltage_baseline_v,
            voltage_delta_v=voltage_delta_v,
            packet=packet,
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
