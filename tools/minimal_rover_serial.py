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
MOVE_REL = struct.Struct("<iiHH")
ACK = struct.Struct("<BBH")
TELEMETRY = struct.Struct("<IHBBiiiihhi")

CMD_STOP = 1
CMD_MOVE_REL = 2
PACKET_ACK = 0x80
PACKET_TELEMETRY = 0x81


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


def format_packet(packet: Packet) -> str:
    if packet.packet_type == PACKET_ACK and len(packet.payload) == ACK.size:
        status, command_type, detail = ACK.unpack(packet.payload)
        return f"ACK seq={packet.seq} status={status} command={command_type} detail={detail}"
    if packet.packet_type == PACKET_TELEMETRY and len(packet.payload) == TELEMETRY.size:
        (
            uptime_ms,
            active_seq,
            phase,
            flags,
            x_target_mm,
            z_target_cdeg,
            x_est_mm,
            z_est_cdeg,
            left_milli,
            right_milli,
            gyro_z_cdeg_s,
        ) = TELEMETRY.unpack(packet.payload)
        return (
            f"TEL ms={uptime_ms} active_seq={active_seq} phase={phase} flags=0x{flags:02x} "
            f"target=({x_target_mm}mm,{z_target_cdeg / 100:.2f}deg) "
            f"est=({x_est_mm}mm,{z_est_cdeg / 100:.2f}deg) "
            f"pwm=({left_milli},{right_milli}) gyro_z={gyro_z_cdeg_s / 100:.2f}dps"
        )
    return f"PACKET type=0x{packet.packet_type:02x} seq={packet.seq} payload={packet.payload.hex()}"


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
    )
    with open_port(args) as ser:
        ser.reset_input_buffer()
        write_command(ser, CMD_MOVE_REL, args.seq, payload)
        read_for(ser, args.read_seconds)


def run_monitor(args) -> None:
    with open_port(args) as ser:
        read_for(ser, args.read_seconds)


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
    move.set_defaults(func=run_move)

    monitor = subparsers.add_parser("monitor", help="Print telemetry already streaming from the ESP32.")
    monitor.set_defaults(func=run_monitor)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
