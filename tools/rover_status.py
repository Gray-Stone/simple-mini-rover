#!/usr/bin/env python3
import argparse
import json

from minimal_rover_serial import PACKET_TELEMETRY, format_telemetry, open_port, read_packets, unpack_telemetry_packet


def telemetry_dict(sample) -> dict[str, float | int | bool | str | list[str]]:
    return {
        "uptime_ms": sample.uptime_ms,
        "active_seq": sample.active_seq,
        "phase": sample.phase_name,
        "flags": sample.flags,
        "flag_names": sample.flag_names,
        "x_target_mm": sample.x_target_mm,
        "z_target_deg": sample.z_target_cdeg / 100.0,
        "x_est_mm": sample.x_est_mm,
        "z_est_deg": sample.z_est_cdeg / 100.0,
        "left_milli": sample.left_milli,
        "right_milli": sample.right_milli,
        "gyro_z_dps": sample.gyro_z_cdeg_s / 100.0,
        "power_ready": sample.power_ready,
        "bus_mv": sample.bus_mv,
        "voltage_v": sample.bus_mv / 1000.0,
        "current_ma": sample.current_ma,
        "current_a": sample.current_ma / 1000.0,
        "shunt_uv": sample.shunt_uv,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Read one telemetry snapshot from the current minimal WAVE ROVER serial protocol.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--read-seconds", type=float, default=1.0)
    parser.add_argument("--json", action="store_true", help="Print the latest telemetry as formatted JSON.")
    args = parser.parse_args()

    with open_port(args) as ser:
        ser.timeout = args.timeout
        ser.reset_input_buffer()
        packets = read_packets(ser, args.read_seconds)

    telemetry_packets = [
        packet for packet in packets if packet.packet_type == PACKET_TELEMETRY
    ]
    if not telemetry_packets:
        print("No telemetry packet received.")
        return 1

    sample = unpack_telemetry_packet(telemetry_packets[-1])
    if sample is None:
        print("Latest telemetry packet could not be decoded.")
        return 1

    if args.json:
        print(json.dumps(telemetry_dict(sample), indent=2, sort_keys=True))
    else:
        print(format_telemetry(telemetry_packets[-1]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
