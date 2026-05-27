#!/usr/bin/env python3
import argparse
import struct
import time
from dataclasses import dataclass

import serial


MAGIC = 0x5752
VERSION = 1
MAX_PAYLOAD = 96
HEADER = struct.Struct("<HBBHH")
CRC = struct.Struct("<H")
MOVE_REL = struct.Struct("<iiHHhh")
PWM = struct.Struct("<hhHH")
ACK = struct.Struct("<BBH")
TELEMETRY = struct.Struct("<IHBBiiiihhiiii")

CMD_STOP = 1
CMD_MOVE_REL = 2
CMD_PWM = 3
PACKET_ACK = 0x80
PACKET_TELEMETRY = 0x81

PHASE_NAMES = {
    0: "idle",
    1: "turn",
    2: "drive",
    3: "done",
    4: "fault",
    5: "pwm",
}

FLAG_NAMES = (
    (0x01, "active"),
    (0x02, "timeout"),
    (0x04, "imu_ready"),
    (0x08, "power_ready"),
)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(packet_type: int, seq: int, payload: bytes = b"") -> bytes:
    header = HEADER.pack(MAGIC, VERSION, packet_type, len(payload), seq)
    body = header + payload
    return body + CRC.pack(crc16_ccitt(body))


@dataclass
class Packet:
    packet_type: int
    seq: int
    payload: bytes


@dataclass
class TelemetrySample:
    uptime_ms: int
    active_seq: int
    phase: int
    flags: int
    x_target_mm: int
    z_target_cdeg: int
    x_est_mm: int
    z_est_cdeg: int
    left_milli: int
    right_milli: int
    gyro_z_cdeg_s: int
    bus_mv: int
    current_ma: int
    shunt_uv: int

    @property
    def phase_name(self) -> str:
        return PHASE_NAMES.get(self.phase, f"unknown({self.phase})")

    @property
    def flag_names(self) -> list[str]:
        return [name for bit, name in FLAG_NAMES if self.flags & bit]

    @property
    def power_ready(self) -> bool:
        return bool(self.flags & 0x08)

    def as_charge_packet(self) -> dict[str, float | int | bool]:
        packet: dict[str, float | int | bool] = {
            "power_ready": self.power_ready,
        }
        if not self.power_ready:
            return packet
        packet.update(
            {
                "voltage": self.bus_mv / 1000.0,
                "voltage_v": self.bus_mv / 1000.0,
                "bus_mv": self.bus_mv,
                "current_ma": self.current_ma,
                "current_a": self.current_ma / 1000.0,
                "shunt_uv": self.shunt_uv,
                "power_w": (self.bus_mv * self.current_ma) / 1_000_000.0,
            }
        )
        return packet


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[Packet]:
        self.buffer.extend(data)
        packets = []
        magic = struct.pack("<H", MAGIC)
        while True:
            start = self.buffer.find(magic)
            if start < 0:
                del self.buffer[:-1]
                return packets
            if start:
                del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                return packets

            parsed_magic, version, packet_type, length, seq = HEADER.unpack_from(self.buffer)
            if parsed_magic != MAGIC or length > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            wire_len = HEADER.size + length + CRC.size
            if len(self.buffer) < wire_len:
                return packets

            raw = bytes(self.buffer[:wire_len])
            del self.buffer[:wire_len]
            expected_crc = CRC.unpack_from(raw, HEADER.size + length)[0]
            if expected_crc != crc16_ccitt(raw[:-CRC.size]) or version != VERSION:
                continue
            packets.append(Packet(packet_type, seq, raw[HEADER.size : HEADER.size + length]))


def open_port(args) -> serial.Serial:
    return serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1)


def unpack_telemetry_payload(payload: bytes) -> TelemetrySample | None:
    if len(payload) != TELEMETRY.size:
        return None
    return TelemetrySample(*TELEMETRY.unpack(payload))


def unpack_telemetry_packet(packet: Packet) -> TelemetrySample | None:
    if packet.packet_type != PACKET_TELEMETRY:
        return None
    return unpack_telemetry_payload(packet.payload)


def format_packet(packet: Packet) -> str:
    if packet.packet_type == PACKET_ACK and len(packet.payload) == ACK.size:
        status, command_type, detail = ACK.unpack(packet.payload)
        return f"ACK seq={packet.seq} status={status} command={command_type} detail={detail}"
    telemetry = unpack_telemetry_packet(packet)
    if telemetry is not None:
        return (
            f"TEL ms={telemetry.uptime_ms} active_seq={telemetry.active_seq} "
            f"phase={telemetry.phase} flags=0x{telemetry.flags:02x} "
            f"target=({telemetry.x_target_mm}mm,{telemetry.z_target_cdeg / 100:.2f}deg) "
            f"est=({telemetry.x_est_mm}mm,{telemetry.z_est_cdeg / 100:.2f}deg) "
            f"pwm=({telemetry.left_milli},{telemetry.right_milli}) "
            f"gyro_z={telemetry.gyro_z_cdeg_s / 100:.2f}dps "
            f"power=({telemetry.bus_mv}mV,{telemetry.current_ma}mA,{telemetry.shunt_uv}uV)"
        )
    return f"PACKET type=0x{packet.packet_type:02x} seq={packet.seq} payload={packet.payload.hex()}"


def format_telemetry(packet: Packet) -> str:
    telemetry = unpack_telemetry_packet(packet)
    if telemetry is None:
        raise ValueError("packet is not a telemetry frame")

    flags_text = ",".join(telemetry.flag_names) if telemetry.flag_names else "none"
    return (
        f"uptime={telemetry.uptime_ms}ms seq={telemetry.active_seq} "
        f"phase={telemetry.phase_name} flags={flags_text} "
        f"target=({telemetry.x_target_mm}mm,{telemetry.z_target_cdeg / 100:.2f}deg) "
        f"est=({telemetry.x_est_mm}mm,{telemetry.z_est_cdeg / 100:.2f}deg) "
        f"pwm=({telemetry.left_milli},{telemetry.right_milli}) "
        f"gyro_z={telemetry.gyro_z_cdeg_s / 100:.2f}dps "
        f"bus={telemetry.bus_mv}mV current={telemetry.current_ma}mA shunt={telemetry.shunt_uv}uV"
    )


def write_command(ser: serial.Serial, packet_type: int, seq: int, payload: bytes = b"") -> None:
    ser.write(frame(packet_type, seq, payload))
    ser.flush()


def read_for(ser: serial.Serial, seconds: float) -> None:
    parser = Parser()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        for packet in parser.feed(chunk):
            print(format_packet(packet))


def read_packets(ser: serial.Serial, seconds: float) -> list[Packet]:
    parser = Parser()
    packets = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        packets.extend(parser.feed(chunk))
    return packets


def run_stop(args) -> None:
    with open_port(args) as ser:
        ser.reset_input_buffer()
        write_command(ser, CMD_STOP, args.seq)
        read_for(ser, args.read_seconds)


def run_move(args) -> None:
    payload = MOVE_REL.pack(
        args.x_mm,
        int(round(args.z_deg * 100)),
        args.max_time_ms,
        0,
        args.drive_milli,
        args.turn_milli,
    )
    with open_port(args) as ser:
        ser.reset_input_buffer()
        write_command(ser, CMD_MOVE_REL, args.seq, payload)
        read_for(ser, args.read_seconds)


def run_pwm(args) -> None:
    if args.milli is None and (args.left_milli is None or args.right_milli is None):
        raise SystemExit("pwm requires --milli or both --left-milli and --right-milli")

    left_milli = args.left_milli if args.left_milli is not None else args.milli
    right_milli = args.right_milli if args.right_milli is not None else args.milli
    payload = PWM.pack(left_milli, right_milli, args.duration_ms, 0)
    with open_port(args) as ser:
        ser.reset_input_buffer()
        write_command(ser, CMD_PWM, args.seq, payload)
        read_for(ser, args.read_seconds)


def run_monitor(args) -> None:
    with open_port(args) as ser:
        read_for(ser, args.read_seconds)


def run_status(args) -> None:
    with open_port(args) as ser:
        ser.reset_input_buffer()
        packets = read_packets(ser, args.read_seconds)

    telemetry = [packet for packet in packets if packet.packet_type == PACKET_TELEMETRY and len(packet.payload) == TELEMETRY.size]
    if not telemetry:
        print("No telemetry packet received.")
        raise SystemExit(1)

    print(format_telemetry(telemetry[-1]))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Speak the minimal ESP-IDF WAVE ROVER binary UART protocol.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--read-seconds", type=float, default=1.0)

    subparsers = parser.add_subparsers(dest="command", required=True)
    stop = subparsers.add_parser("stop", help="Send immediate stop and print replies.")
    stop.set_defaults(func=run_stop)

    move = subparsers.add_parser("move", help="Send one bounded relative move.")
    move.add_argument("--x-mm", type=int, default=0)
    move.add_argument("--z-deg", type=float, default=0.0)
    move.add_argument("--max-time-ms", type=int, default=0)
    move.add_argument("--drive-milli", type=int, default=0, help="Mapped drive PWM magnitude, 250..700; 0 uses firmware default.")
    move.add_argument("--turn-milli", type=int, default=0, help="Turn PWM magnitude, 450..900; 0 uses firmware default.")
    move.set_defaults(func=run_move)

    pwm = subparsers.add_parser("pwm", help="Run raw motor PWM for a bounded duration.")
    pwm.add_argument("--milli", type=int, help="Set both motors to this PWM milli value.")
    pwm.add_argument("--left-milli", type=int, help="Left motor PWM milli value, -1000..1000.")
    pwm.add_argument("--right-milli", type=int, help="Right motor PWM milli value, -1000..1000.")
    pwm.add_argument("--duration-ms", type=int, required=True)
    pwm.set_defaults(func=run_pwm)

    monitor = subparsers.add_parser("monitor", help="Print telemetry already streaming from the ESP32.")
    monitor.set_defaults(func=run_monitor)

    status = subparsers.add_parser("status", help="Print one compact telemetry snapshot from the ESP32.")
    status.set_defaults(func=run_status)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
