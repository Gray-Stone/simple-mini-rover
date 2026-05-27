#!/usr/bin/env python3
"""Shared charging-state heuristics for rover feedback consumers."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone

from rover_motion_probe import extract_voltage, lower_numeric_map


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
