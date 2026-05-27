#!/usr/bin/env python3
"""
Continuously log AprilTag pose while sending a timed raw-PWM command program.

Edit COMMAND_PROGRAM below for the calibration sequence you want to run. Each
command can use either {"milli": ...} for both motors or explicit
{"left_milli": ..., "right_milli": ...}.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import serial

from minimal_rover_serial import (
    ACK,
    CMD_PWM,
    CMD_STOP,
    PACKET_ACK,
    PACKET_TELEMETRY,
    PWM,
    TELEMETRY,
    Parser,
    write_command,
)


# Edit this list for calibration runs.
# Positive milli drives forward. Negative milli drives backward.
COMMAND_PROGRAM: list[dict[str, Any]] = [

    {"milli":  650, "duration_ms": 50, "settle_ms": 1000},
    {"milli": -650, "duration_ms": 50, "settle_ms": 1000},
    {"milli":  650, "duration_ms": 100, "settle_ms": 1000},
    {"milli": -650, "duration_ms": 100, "settle_ms": 1000},
    {"milli":  650, "duration_ms": 150, "settle_ms": 1000},
    {"milli": -650, "duration_ms": 150, "settle_ms": 1000},
    {"milli":  650, "duration_ms": 200, "settle_ms": 1000},
    {"milli": -650, "duration_ms": 200, "settle_ms": 1000},
    {"milli":  650, "duration_ms": 300, "settle_ms": 1000},
    {"milli": -650, "duration_ms": 300, "settle_ms": 1000},
    {"milli":  650, "duration_ms": 400, "settle_ms": 1000},
    {"milli": -650, "duration_ms": 400, "settle_ms": 1000},
    {"milli":  650, "duration_ms": 500, "settle_ms": 1000},
    {"milli": -650, "duration_ms": 500, "settle_ms": 1000},

    {"milli":  700, "duration_ms": 50, "settle_ms": 1000},
    {"milli": -700, "duration_ms": 50, "settle_ms": 1000},
    {"milli":  700, "duration_ms": 100, "settle_ms": 1000},
    {"milli": -700, "duration_ms": 100, "settle_ms": 1000},
    {"milli":  700, "duration_ms": 150, "settle_ms": 1000},
    {"milli": -700, "duration_ms": 150, "settle_ms": 1000},
    {"milli":  700, "duration_ms": 200, "settle_ms": 1000},
    {"milli": -700, "duration_ms": 200, "settle_ms": 1000},
    {"milli":  700, "duration_ms": 300, "settle_ms": 1000},
    {"milli": -700, "duration_ms": 300, "settle_ms": 1000},
    {"milli":  700, "duration_ms": 400, "settle_ms": 1000},
    {"milli": -700, "duration_ms": 400, "settle_ms": 1000},
    {"milli":  700, "duration_ms": 500, "settle_ms": 1000},
    {"milli": -700, "duration_ms": 500, "settle_ms": 1000},

]


@dataclass(frozen=True)
class TimelineClock:
    start_mono_ns: int

    @classmethod
    def start(cls) -> "TimelineClock":
        return cls(start_mono_ns=time.monotonic_ns())

    def stamp(self) -> dict[str, Any]:
        now_mono_ns = time.monotonic_ns()
        return {
            "mono_ns": now_mono_ns,
            "t_s": round((now_mono_ns - self.start_mono_ns) / 1_000_000_000, 6),
            "wall_ns": time.time_ns(),
        }


class TimelineWriter:
    def __init__(self, path: Path, clock: TimelineClock):
        self.path = path
        self.clock = clock
        self.lock = threading.Lock()
        self.file = path.open("w", buffering=1)

    def close(self) -> None:
        with self.lock:
            self.file.close()

    def write(self, kind: str, **fields: Any) -> None:
        record = {"kind": kind, **self.clock.stamp(), **fields}
        line = json.dumps(record, separators=(",", ":"))
        with self.lock:
            self.file.write(line + "\n")


class ConsoleStatus:
    def __init__(self, quiet: bool):
        self.quiet = quiet
        self.lock = threading.Lock()
        self.tag_frames = 0
        self.tag_matches = 0
        self.serial_packets = 0
        self.latest_tag: dict[str, Any] | None = None

    def print(self, message: str) -> None:
        if self.quiet:
            return
        with self.lock:
            print(message, flush=True)

    def tag_seen(self, matched: bool, payload: dict[str, Any] | None = None) -> None:
        with self.lock:
            self.tag_frames += 1
            if matched:
                self.tag_matches += 1
                self.latest_tag = {
                    "seen_mono_ns": time.monotonic_ns(),
                    "frame": payload.get("frame") if payload else None,
                    "pose": payload.get("pose") if payload else None,
                }

    def serial_seen(self) -> None:
        with self.lock:
            self.serial_packets += 1

    def snapshot(self) -> tuple[int, int, int]:
        with self.lock:
            return self.tag_frames, self.tag_matches, self.serial_packets

    def latest_tag_snapshot(self) -> dict[str, Any] | None:
        with self.lock:
            if self.latest_tag is None:
                return None
            snapshot = dict(self.latest_tag)
        snapshot["age_s"] = round((time.monotonic_ns() - snapshot["seen_mono_ns"]) / 1_000_000_000, 3)
        return snapshot

    def print_latest_tag(self, prefix: str) -> None:
        snapshot = self.latest_tag_snapshot()
        if snapshot is None:
            self.print(f"{prefix}: tag none")
            return

        pose = snapshot.get("pose") or {}
        euler_xyz_deg = pose.get("euler_xyz_deg")
        euler_text = ""
        if isinstance(euler_xyz_deg, list) and len(euler_xyz_deg) >= 3:
            euler_text = (
                f" euler_xyz=({euler_xyz_deg[0]:.1f},"
                f"{euler_xyz_deg[1]:.1f},{euler_xyz_deg[2]:.1f})deg"
            )
        self.print(
            f"{prefix}: tag frame={snapshot.get('frame')} age={snapshot['age_s']:.3f}s "
            f"x={pose.get('lateral_m', 0.0):.3f}m "
            f"y={pose.get('vertical_m', 0.0):.3f}m "
            f"z={pose.get('range_m', 0.0):.3f}m{euler_text}"
        )


def normalize_command(command: dict[str, Any], index: int, default_settle_ms: int) -> dict[str, Any]:
    if "duration_ms" not in command:
        raise ValueError(f"command {index} is missing duration_ms")
    duration_ms = int(command["duration_ms"])
    if duration_ms <= 0:
        raise ValueError(f"command {index} duration_ms must be positive")

    if "milli" in command:
        left_milli = int(command["milli"])
        right_milli = int(command["milli"])
    else:
        left_milli = int(command["left_milli"])
        right_milli = int(command["right_milli"])

    if not -1000 <= left_milli <= 1000 or not -1000 <= right_milli <= 1000:
        raise ValueError(f"command {index} PWM is outside [-1000, 1000]")

    return {
        "index": index,
        "left_milli": left_milli,
        "right_milli": right_milli,
        "duration_ms": duration_ms,
        "settle_ms": int(command.get("settle_ms", default_settle_ms)),
    }


def load_program(args: argparse.Namespace) -> list[dict[str, Any]]:
    if args.program_json is None:
        source = COMMAND_PROGRAM
    else:
        source = json.loads(args.program_json.read_text())
        if not isinstance(source, list):
            raise SystemExit("--program-json must contain a list of command objects")

    limit = None if args.max_commands == 0 else args.max_commands
    program = [
        normalize_command(command, index, args.default_settle_ms)
        for index, command in enumerate(source[:limit], start=1)
    ]
    if not program:
        raise SystemExit("command program is empty")
    return program


def decode_packet(packet) -> dict[str, Any]:
    if packet.packet_type == PACKET_ACK and len(packet.payload) == ACK.size:
        status, command_type, detail = ACK.unpack(packet.payload)
        return {
            "packet": "ack",
            "seq": packet.seq,
            "status": status,
            "command_type": command_type,
            "detail": detail,
        }

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
            bus_mv,
            current_ma,
            shunt_uv,
        ) = TELEMETRY.unpack(packet.payload)
        return {
            "packet": "telemetry",
            "uptime_ms": uptime_ms,
            "active_seq": active_seq,
            "phase": phase,
            "flags": flags,
            "x_target_mm": x_target_mm,
            "z_target_cdeg": z_target_cdeg,
            "x_est_mm": x_est_mm,
            "z_est_cdeg": z_est_cdeg,
            "left_milli": left_milli,
            "right_milli": right_milli,
            "gyro_z_cdeg_s": gyro_z_cdeg_s,
            "bus_mv": bus_mv,
            "current_ma": current_ma,
            "shunt_uv": shunt_uv,
        }

    return {
        "packet": "unknown",
        "packet_type": packet.packet_type,
        "seq": packet.seq,
        "payload_hex": packet.payload.hex(),
    }


def serial_reader(
    ser: serial.Serial,
    writer: TimelineWriter,
    stop_event: threading.Event,
    console: ConsoleStatus,
) -> None:
    parser = Parser()
    while not stop_event.is_set():
        try:
            chunk = ser.read(256)
        except (OSError, TypeError, serial.SerialException) as exc:
            if not stop_event.is_set():
                writer.write("serial_error", message=str(exc))
            return
        if not chunk:
            continue
        for packet in parser.feed(chunk):
            decoded = decode_packet(packet)
            writer.write("serial", **decoded)
            console.serial_seen()
            if decoded["packet"] == "ack":
                console.print(
                    f"ACK seq={decoded['seq']} status={decoded['status']} "
                    f"command={decoded['command_type']} detail={decoded['detail']}"
                )


def camera_reader(
    proc: subprocess.Popen[str],
    writer: TimelineWriter,
    stop_event: threading.Event,
    first_tag_event: threading.Event,
    console: ConsoleStatus,
) -> None:
    assert proc.stdout is not None
    while not stop_event.is_set():
        line = proc.stdout.readline()
        if not line:
            if proc.poll() is not None:
                writer.write("camera_exit", returncode=proc.returncode)
                return
            continue
        line = line.strip()
        if not line:
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            writer.write("camera_stdout", line=line)
            continue
        matched = bool(payload.get("matched_expected") and payload.get("pose"))
        console.tag_seen(matched, payload)
        if matched:
            if not first_tag_event.is_set():
                pose = payload["pose"]
                console.print(
                    f"tag ready frame={payload.get('frame')} "
                    f"z={pose.get('range_m', 0.0):.3f}m x={pose.get('lateral_m', 0.0):.3f}m"
                )
            first_tag_event.set()
        writer.write("tag", record=payload)


def camera_stderr_reader(
    proc: subprocess.Popen[str],
    writer: TimelineWriter,
    stop_event: threading.Event,
    console: ConsoleStatus,
) -> None:
    assert proc.stderr is not None
    for line in proc.stderr:
        if stop_event.is_set():
            break
        stripped = line.rstrip()
        writer.write("camera_stderr", line=stripped)
        if stripped.startswith("camera="):
            console.print(stripped)


def start_camera(args: argparse.Namespace, debug_image: Path) -> subprocess.Popen[str]:
    command = [
        sys.executable,
        "tools/apriltag_pose.py",
        "--camera",
        args.camera,
        "--frames",
        "0",
        "--width",
        str(args.width),
        "--height",
        str(args.height),
        "--focus-absolute",
        str(args.focus_absolute),
        "--jsonl",
        "--save-debug",
        str(debug_image),
    ]
    if args.low_light_preset:
        command.append("--low-light-preset")
    if args.exposure_time_absolute is not None:
        command.extend(["--exposure-time-absolute", str(args.exposure_time_absolute)])

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    return subprocess.Popen(
        command,
        cwd=Path.cwd(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=1,
        env=env,
    )


def send_pwm_command(ser: serial.Serial, seq: int, command: dict[str, Any], writer: TimelineWriter) -> None:
    payload = PWM.pack(command["left_milli"], command["right_milli"], command["duration_ms"], 0)
    writer.write("command", event="start", seq=seq, **command)
    write_command(ser, CMD_PWM, seq, payload)
    writer.write("command", event="write_done", seq=seq, **command)
    time.sleep(command["duration_ms"] / 1000.0)
    writer.write("command", event="duration_elapsed", seq=seq, **command)


def stop_rover(ser: serial.Serial, seq: int, writer: TimelineWriter) -> None:
    writer.write("command", event="stop_start", seq=seq)
    write_command(ser, CMD_STOP, seq)
    writer.write("command", event="stop_write_done", seq=seq)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Continuous AprilTag + timed raw-PWM timeline collector.")
    parser.add_argument(
        "--out-dir",
        type=Path,
        help="Directory for the timestamped timeline file and sidecars.",
    )
    parser.add_argument("--out-file", type=Path, help="Explicit timeline JSONL output file.")
    parser.add_argument("--program-json", type=Path, help="Optional JSON list of command objects.")
    parser.add_argument("--max-commands", type=int, default=0, help="Limit commands; 0 means all.")
    parser.add_argument("--seq-start", type=int, default=700)
    parser.add_argument("--default-settle-ms", type=int, default=1000)
    parser.add_argument("--pre-roll-s", type=float, default=2.0)
    parser.add_argument("--post-roll-s", type=float, default=2.0)
    parser.add_argument("--tag-ready-timeout-s", type=float, default=8.0)
    parser.add_argument("--no-require-tag", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Log command timing without opening serial.")
    parser.add_argument("--quiet", action="store_true", help="Only print final output paths.")

    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=460800)

    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--focus-absolute", type=int, default=350)
    parser.add_argument(
        "--exposure-time-absolute",
        type=int,
        help="Manual UVC exposure_time_absolute value; overrides the low-light preset exposure.",
    )
    parser.add_argument("--low-light-preset", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.max_commands < 0:
        raise SystemExit("--max-commands must be >= 0")

    program = load_program(args)
    if args.out_file is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_dir = args.out_dir or Path("data/minimal_motion_specs/pwm_timelines")
        timeline_path = out_dir / f"{stamp}_timeline.jsonl"
    else:
        timeline_path = args.out_file
        out_dir = args.out_dir or timeline_path.parent
        if not timeline_path.is_absolute():
            timeline_path = out_dir / timeline_path.name

    out_dir.mkdir(parents=True, exist_ok=True)
    timeline_stem = timeline_path.stem
    manifest_path = timeline_path.with_name(f"{timeline_stem}_manifest.json")
    debug_image_path = timeline_path.with_name(f"{timeline_stem}_apriltag_last.jpg")

    clock = TimelineClock.start()
    writer = TimelineWriter(timeline_path, clock)
    console = ConsoleStatus(args.quiet)
    stop_event = threading.Event()
    first_tag_event = threading.Event()
    camera_proc: subprocess.Popen[str] | None = None
    ser: serial.Serial | None = None
    need_final_stop = False
    threads: list[threading.Thread] = []

    def request_stop(signum, _frame):
        writer.write("signal", signum=signum)
        stop_event.set()

    old_int = signal.signal(signal.SIGINT, request_stop)
    old_term = signal.signal(signal.SIGTERM, request_stop)

    try:
        manifest = {
            "program": program,
            "args": vars(args)
            | {
                "out_dir": str(out_dir),
                "out_file": str(timeline_path),
                "program_json": str(args.program_json) if args.program_json else None,
            },
        }
        manifest_path.write_text(json.dumps(manifest, indent=2))
        console.print(f"timeline={timeline_path}")
        console.print(f"manifest={manifest_path}")
        console.print(f"debug_image={debug_image_path}")
        console.print(
            f"program commands={len(program)} pre_roll={args.pre_roll_s:.2f}s "
            f"post_roll={args.post_roll_s:.2f}s dry_run={args.dry_run}"
        )
        writer.write(
            "run",
            event="start",
            out_dir=str(out_dir),
            timeline=str(timeline_path),
            manifest=str(manifest_path),
            debug_image=str(debug_image_path),
            program=program,
            args=manifest["args"],
            program_len=len(program),
        )

        console.print(
            f"starting camera {args.camera} {args.width}x{args.height} "
            f"focus={args.focus_absolute} exposure={args.exposure_time_absolute} "
            f"low_light={args.low_light_preset}"
        )
        camera_proc = start_camera(args, debug_image_path)
        threads.extend(
            [
                threading.Thread(
                    target=camera_reader,
                    args=(camera_proc, writer, stop_event, first_tag_event, console),
                    daemon=True,
                ),
                threading.Thread(
                    target=camera_stderr_reader,
                    args=(camera_proc, writer, stop_event, console),
                    daemon=True,
                ),
            ]
        )
        for thread in threads:
            thread.start()

        if not args.no_require_tag:
            console.print(f"waiting for first tag match, timeout={args.tag_ready_timeout_s:.1f}s")
            if not first_tag_event.wait(args.tag_ready_timeout_s):
                writer.write("run", event="aborted_no_tag", timeout_s=args.tag_ready_timeout_s)
                console.print("aborted: no tag match before timeout")
                return 2

        writer.write("run", event="pre_roll_begin", seconds=args.pre_roll_s)
        console.print(f"pre-roll {args.pre_roll_s:.2f}s")
        time.sleep(args.pre_roll_s)
        writer.write("run", event="pre_roll_end")

        if not args.dry_run:
            console.print(f"opening serial {args.port} @ {args.baud}")
            ser = serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1)
            ser.reset_input_buffer()
            need_final_stop = True
            serial_thread = threading.Thread(
                target=serial_reader,
                args=(ser, writer, stop_event, console),
                daemon=True,
            )
            serial_thread.start()
            threads.append(serial_thread)

        seq = args.seq_start
        for command in program:
            if stop_event.is_set():
                break
            tag_snapshot = console.latest_tag_snapshot()
            writer.write(
                "tag_snapshot",
                event="before_command",
                seq=seq,
                command_index=command["index"],
                tag=tag_snapshot,
            )
            console.print_latest_tag(f"before command {command['index']}/{len(program)}")
            console.print(
                f"command {command['index']}/{len(program)} seq={seq} "
                f"L={command['left_milli']} R={command['right_milli']} "
                f"duration={command['duration_ms']}ms"
            )
            if args.dry_run:
                writer.write("command", event="dry_start", seq=seq, **command)
                time.sleep(command["duration_ms"] / 1000.0)
                writer.write("command", event="dry_duration_elapsed", seq=seq, **command)
            else:
                assert ser is not None
                send_pwm_command(ser, seq, command, writer)
            console.print(f"command {command['index']} duration elapsed")
            seq += 1

            if command["settle_ms"] > 0:
                writer.write("settle", event="begin", duration_ms=command["settle_ms"], seq=seq - 1)
                console.print(f"settle {command['settle_ms']}ms")
                time.sleep(command["settle_ms"] / 1000.0)
                writer.write("settle", event="end", seq=seq - 1)

        if ser is not None:
            console.print(f"sending final stop seq={seq}")
            stop_rover(ser, seq, writer)
            need_final_stop = False
            time.sleep(0.2)

        writer.write("run", event="post_roll_begin", seconds=args.post_roll_s)
        console.print(f"post-roll {args.post_roll_s:.2f}s")
        time.sleep(args.post_roll_s)
        writer.write("run", event="post_roll_end")
        writer.write("run", event="complete")
        tag_frames, tag_matches, serial_packets = console.snapshot()
        console.print(
            f"complete tag_frames={tag_frames} tag_matches={tag_matches} "
            f"serial_packets={serial_packets}"
        )
        print(f"timeline={timeline_path}")
        print(f"manifest={manifest_path}")
        print(f"debug_image={debug_image_path}")
        return 0
    finally:
        if ser is not None and need_final_stop:
            try:
                stop_rover(ser, args.seq_start + len(program) + 1000, writer)
            except Exception as exc:
                writer.write("serial_error", message=f"final stop failed: {exc}")
        stop_event.set()
        if ser is not None:
            ser.close()
        if camera_proc is not None and camera_proc.poll() is None:
            camera_proc.terminate()
            try:
                camera_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                camera_proc.kill()
        for thread in threads:
            thread.join(timeout=0.5)
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        writer.close()


if __name__ == "__main__":
    raise SystemExit(main())
