#!/usr/bin/env python3
"""Experimental live AprilTag visual servo loop with bounded PWM leases.

This is a sidecar bring-up tool, not the final auto-docking planner. It measures
camera/detection/control timing and can optionally drive using short raw-PWM
commands that expire shortly after the next expected host cycle.
"""

import argparse
import json
import math
import signal
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import serial

from apriltag_pose import (
    apply_camera_control_overrides,
    build_tag_object_points,
    detect_pose,
    load_camera_calibration,
    make_detector,
    open_camera,
    overlay_pose,
    read_camera_controls,
    scaled_camera_matrix,
)
from minimal_rover_serial import (
    ACK,
    CMD_PWM,
    CMD_STOP,
    PACKET_ACK,
    PWM,
    Parser,
    format_packet,
    unpack_telemetry_packet,
    write_command,
)


stop_requested = False


def request_stop(_signum, _frame) -> None:
    global stop_requested
    stop_requested = True


def now_utc() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def median(values: list[float]) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    return float(statistics.median(finite)) if finite else None


def percentile(values: list[float], pct: float) -> float | None:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    if len(finite) == 1:
        return float(finite[0])
    index = (len(finite) - 1) * pct / 100.0
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return float(finite[lo])
    frac = index - lo
    return float(finite[lo] * (1.0 - frac) + finite[hi] * frac)


def read_target_translation(path: Path) -> tuple[float, float] | None:
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError:
        return None
    translation = data.get("translation_m", {}).get("median") or []
    if len(translation) < 3:
        return None
    return float(translation[0]), float(translation[2])


def clamp_milli(value: float, limit: int) -> int:
    return int(round(clamp(value, -float(limit), float(limit))))


def jsonable_args(args: argparse.Namespace) -> dict[str, Any]:
    values = {}
    for key, value in vars(args).items():
        values[key] = str(value) if isinstance(value, Path) else value
    return values


class LeaseRover:
    def __init__(self, port: str, baud: int, timeout_s: float, seq_start: int):
        self.ser = serial.Serial(port, baud, timeout=timeout_s, write_timeout=1)
        self.parser = Parser()
        self.seq = seq_start
        self.telemetry_tail: list[str] = []
        self.ack_tail: list[dict[str, Any]] = []
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def next_seq(self) -> int:
        seq = self.seq
        self.seq = 1 if self.seq >= 0xFFFF else self.seq + 1
        return seq

    def drain(self, read_s: float = 0.0) -> list[str]:
        deadline = time.monotonic() + max(0.0, read_s)
        lines: list[str] = []
        while True:
            chunk = self.ser.read(256)
            if chunk:
                for packet in self.parser.feed(chunk):
                    text = format_packet(packet)
                    lines.append(text)
                    self.telemetry_tail.append(text)
                    self.telemetry_tail = self.telemetry_tail[-20:]
                    if packet.packet_type == PACKET_ACK and len(packet.payload) == ACK.size:
                        status, command_type, detail = ACK.unpack(packet.payload)
                        self.ack_tail.append(
                            {
                                "seq": packet.seq,
                                "status": int(status),
                                "command_type": int(command_type),
                                "detail": int(detail),
                            }
                        )
                        self.ack_tail = self.ack_tail[-20:]
                    telemetry = unpack_telemetry_packet(packet)
                    if telemetry is not None:
                        continue
                continue
            if time.monotonic() >= deadline:
                return lines

    def stop(self) -> int:
        seq = self.next_seq()
        write_command(self.ser, CMD_STOP, seq)
        self.drain(0.02)
        return seq

    def pwm_lease(
        self,
        left_milli: int,
        right_milli: int,
        duration_ms: int,
        preempt_with_stop: bool,
    ) -> dict[str, Any]:
        stop_seq = None
        if preempt_with_stop:
            stop_seq = self.stop()
        seq = self.next_seq()
        payload = PWM.pack(left_milli, right_milli, duration_ms, 0)
        write_command(self.ser, CMD_PWM, seq, payload)
        self.drain(0.02)
        return {
            "seq": seq,
            "stop_seq": stop_seq,
            "left_milli": left_milli,
            "right_milli": right_milli,
            "duration_ms": duration_ms,
            "ack_tail": self.ack_tail[-6:],
        }


def choose_command(args: argparse.Namespace, pose: dict[str, Any] | None) -> dict[str, Any]:
    if pose is None:
        return {
            "reason": "no_tag",
            "done": False,
            "x_milli": 0,
            "z_milli": 0,
            "left_milli": 0,
            "right_milli": 0,
        }

    lateral_error_m = float(pose["lateral_m"]) - args.target_lateral_m
    range_error_m = float(pose["range_m"]) - args.target_range_m
    lateral_error_mm = lateral_error_m * 1000.0
    range_error_mm = range_error_m * 1000.0
    bearing_error_deg = math.degrees(math.atan2(lateral_error_m, max(float(pose["range_m"]), 1e-6)))

    lateral_good = abs(lateral_error_mm) <= args.lateral_deadband_mm
    range_good = abs(range_error_mm) <= args.range_deadband_mm
    bearing_good = abs(bearing_error_deg) <= args.bearing_deadband_deg
    if lateral_good and range_good and bearing_good:
        return {
            "reason": "inside_deadband",
            "done": True,
            "lateral_error_mm": lateral_error_mm,
            "range_error_mm": range_error_mm,
            "bearing_error_deg": bearing_error_deg,
            "x_milli": 0,
            "z_milli": 0,
            "left_milli": 0,
            "right_milli": 0,
        }

    x_milli = args.forward_milli if range_error_mm > args.range_deadband_mm else 0
    if range_error_mm <= 0.0:
        x_milli = 0
    if range_error_mm < args.near_range_mm:
        x_milli = min(x_milli, args.near_forward_milli)
    if abs(bearing_error_deg) > args.bearing_slowdown_deg:
        x_milli = int(round(x_milli * args.bearing_slowdown_scale))

    z_milli = 0
    if not bearing_good or not lateral_good:
        z_milli = clamp_milli(
            args.turn_sign * bearing_error_deg * args.turn_milli_per_deg,
            args.max_turn_milli,
        )

    left_milli = clamp_milli(x_milli - z_milli, args.max_wheel_milli)
    right_milli = clamp_milli(x_milli + z_milli, args.max_wheel_milli)
    return {
        "reason": "visual_servo",
        "done": False,
        "lateral_error_mm": lateral_error_mm,
        "range_error_mm": range_error_mm,
        "bearing_error_deg": bearing_error_deg,
        "x_milli": int(x_milli),
        "z_milli": int(z_milli),
        "left_milli": left_milli,
        "right_milli": right_milli,
    }


def summarize(records: list[dict[str, Any]], elapsed_s: float) -> dict[str, Any]:
    cycle_s = [record["cycle_s"] for record in records]
    capture_s = [record["capture_s"] for record in records]
    detect_s = [record["detect_s"] for record in records]
    sleep_s = [record["sleep_s"] for record in records]
    matched = [record for record in records if record.get("matched_expected")]
    detections = [record for record in records if record.get("detections_count", 0) > 0]
    read_ok = [record for record in records if record.get("camera_read_ok")]
    intervals = [
        records[i]["frame_monotonic_s"] - records[i - 1]["frame_monotonic_s"]
        for i in range(1, len(records))
    ]
    return {
        "ok": bool(read_ok),
        "elapsed_s": elapsed_s,
        "cycles": len(records),
        "overall_cycle_hz": len(records) / elapsed_s if elapsed_s > 0 else None,
        "camera_read_ok": len(read_ok),
        "camera_read_ok_rate": len(read_ok) / len(records) if records else 0.0,
        "camera_read_ok_per_s": len(read_ok) / elapsed_s if elapsed_s > 0 else None,
        "matched_expected": len(matched),
        "matched_expected_rate": len(matched) / len(records) if records else 0.0,
        "any_tag_detection": len(detections),
        "any_tag_detection_rate": len(detections) / len(records) if records else 0.0,
        "cycle_s_median": median(cycle_s),
        "cycle_s_p90": percentile(cycle_s, 90),
        "capture_s_median": median(capture_s),
        "detect_s_median": median(detect_s),
        "detect_s_p90": percentile(detect_s, 90),
        "sleep_s_median": median(sleep_s),
        "frame_interval_s_median": median(intervals),
        "frame_interval_s_p90": percentile(intervals, 90),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Experimental live AprilTag visual servo loop with bounded raw-PWM leases."
    )
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--model", type=Path)
    parser.add_argument("--family", default="tag16h5")
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.034)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--autofocus", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--focus-absolute", type=int)
    parser.add_argument("--auto-exposure", choices=("leave", "auto", "manual"), default="leave")
    parser.add_argument("--exposure-time", "--exposure-time-absolute", dest="exposure_time", type=int)
    parser.add_argument("--gain", type=int)
    parser.add_argument("--white-balance-auto", choices=("leave", "on", "off"), default="leave")
    parser.add_argument("--white-balance-temperature", type=int)
    parser.add_argument("--backlight-compensation", type=int)
    parser.add_argument("--contrast", type=int)
    parser.add_argument("--low-light-preset", "--low-light", action="store_true")
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--max-cycles", type=int, default=0)
    parser.add_argument("--command-period-s", type=float, default=0.35)
    parser.add_argument(
        "--lease-margin-s",
        type=float,
        default=0.10,
        help="Extra firmware PWM duration after the next expected host command time.",
    )
    parser.add_argument("--target-pose", type=Path, default=Path("config/auto_docking/dock_edge_tag_pose.json"))
    parser.add_argument("--target-range-m", type=float)
    parser.add_argument("--target-lateral-m", type=float)
    parser.add_argument("--range-deadband-mm", type=float, default=25.0)
    parser.add_argument("--lateral-deadband-mm", type=float, default=20.0)
    parser.add_argument("--bearing-deadband-deg", type=float, default=2.0)
    parser.add_argument("--near-range-mm", type=float, default=90.0)
    parser.add_argument("--forward-milli", type=int, default=140)
    parser.add_argument("--near-forward-milli", type=int, default=120)
    parser.add_argument("--turn-milli-per-deg", type=float, default=8.0)
    parser.add_argument("--max-turn-milli", type=int, default=70)
    parser.add_argument("--max-wheel-milli", type=int, default=220)
    parser.add_argument("--bearing-slowdown-deg", type=float, default=7.0)
    parser.add_argument("--bearing-slowdown-scale", type=float, default=0.55)
    parser.add_argument("--turn-sign", type=float, default=-1.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--preempt-with-stop", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--serial-timeout-s", type=float, default=0.01)
    parser.add_argument("--seq-start", type=int, default=5000)
    parser.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--output-root", type=Path, default=Path("data/live_visual_servo_runs"))
    parser.add_argument("--name", default="live_visual_servo")
    parser.add_argument("--save-debug", action=argparse.BooleanOptionalAction, default=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    target = read_target_translation(args.target_pose)
    if args.target_lateral_m is None:
        args.target_lateral_m = target[0] if target is not None else 0.0
    if args.target_range_m is None:
        args.target_range_m = target[1] if target is not None else 0.336

    lease_duration_ms = int(math.ceil((args.command_period_s + args.lease_margin_s) * 1000.0))
    run_dir = args.output_root / (datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{args.name}")
    run_dir.mkdir(parents=True, exist_ok=True)
    cycles_path = run_dir / "cycles.jsonl"
    summary_path = run_dir / "summary.json"

    calibration = load_camera_calibration(
        args.model,
        focus_absolute=args.focus_absolute,
        autofocus=args.autofocus,
    )
    detector = make_detector(args.family)
    startup_controls = apply_camera_control_overrides(args)
    cap = open_camera(args)
    actual_size = (
        int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))),
        int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))),
    )
    actual = {
        "width": actual_size[0],
        "height": actual_size[1],
        "fps": cap.get(cv2.CAP_PROP_FPS),
        "focus": cap.get(cv2.CAP_PROP_FOCUS),
        "autofocus": cap.get(cv2.CAP_PROP_AUTOFOCUS),
    }
    camera_matrix = scaled_camera_matrix(calibration.camera_matrix, calibration.image_size, actual_size)
    dist_coeffs = calibration.dist_coeffs.copy()
    object_points = build_tag_object_points(args.tag_size)
    camera_controls = read_camera_controls(args.camera)
    rover = (
        LeaseRover(args.port, args.baud, args.serial_timeout_s, args.seq_start)
        if args.execute
        else None
    )

    metadata = {
        "started_at": now_utc(),
        "execute": bool(args.execute),
        "camera": args.camera,
        "actual_camera": actual,
        "camera_controls_startup": startup_controls,
        "camera_controls_after_open": camera_controls,
        "model": str(calibration.model_path),
        "target_pose": str(args.target_pose),
        "target_lateral_m": args.target_lateral_m,
        "target_range_m": args.target_range_m,
        "command_period_s": args.command_period_s,
        "lease_margin_s": args.lease_margin_s,
        "lease_duration_ms": lease_duration_ms,
        "preempt_with_stop": bool(args.preempt_with_stop),
        "argv": jsonable_args(args),
    }
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))

    print(
        f"run_dir={run_dir} execute={int(args.execute)} "
        f"camera={actual['width']}x{actual['height']}@{actual['fps']:.1f} "
        f"autofocus={int(actual['autofocus'])} focus={actual['focus']:.0f} "
        f"period={args.command_period_s:.3f}s lease={lease_duration_ms}ms",
        flush=True,
    )

    records: list[dict[str, Any]] = []
    last_debug = None
    started = time.monotonic()
    next_cycle_time = started
    exit_code = 0
    try:
        for _ in range(max(args.warmup_frames, 0)):
            if stop_requested:
                break
            cap.read()

        with cycles_path.open("a", encoding="utf-8") as log_file:
            cycle_index = 0
            while not stop_requested:
                if args.max_cycles and cycle_index >= args.max_cycles:
                    break
                now = time.monotonic()
                if args.duration_s > 0 and now - started >= args.duration_s:
                    break
                cycle_index += 1
                cycle_start = time.monotonic()
                lag_s = max(0.0, cycle_start - next_cycle_time)

                capture_start = time.monotonic()
                ok, frame = cap.read()
                capture_done = time.monotonic()
                result = {"detections": [], "matched": None, "pose": None, "rejected_count": 0}
                if ok and frame is not None:
                    detect_start = time.monotonic()
                    result = detect_pose(
                        frame=frame,
                        detector=detector,
                        expected_id=args.id,
                        object_points=object_points,
                        camera_matrix=camera_matrix,
                        dist_coeffs=dist_coeffs,
                    )
                    detect_done = time.monotonic()
                else:
                    detect_start = capture_done
                    detect_done = capture_done

                pose = result["pose"] if result.get("matched") and result.get("pose") else None
                command = choose_command(args, pose)
                lease = None
                if rover is not None:
                    if command["left_milli"] == 0 and command["right_milli"] == 0:
                        if args.preempt_with_stop:
                            stop_seq = rover.stop()
                            lease = {"stop_seq": stop_seq, "left_milli": 0, "right_milli": 0}
                    else:
                        lease = rover.pwm_lease(
                            int(command["left_milli"]),
                            int(command["right_milli"]),
                            lease_duration_ms,
                            preempt_with_stop=args.preempt_with_stop,
                        )

                cycle_done = time.monotonic()
                next_cycle_time = max(next_cycle_time + args.command_period_s, cycle_start + args.command_period_s)
                sleep_s = max(0.0, next_cycle_time - time.monotonic())
                record = {
                    "cycle": cycle_index,
                    "timestamp_utc": now_utc(),
                    "elapsed_s": cycle_start - started,
                    "frame_monotonic_s": cycle_start,
                    "camera_read_ok": bool(ok and frame is not None),
                    "matched_expected": bool(pose),
                    "detections_count": len(result.get("detections") or []),
                    "detections": result.get("detections") or [],
                    "rejected_count": int(result.get("rejected_count") or 0),
                    "pose": pose,
                    "command": command,
                    "lease": lease,
                    "lag_s": lag_s,
                    "capture_s": capture_done - capture_start,
                    "detect_s": detect_done - detect_start,
                    "cycle_s": cycle_done - cycle_start,
                    "sleep_s": sleep_s,
                }
                records.append(record)
                log_file.write(json.dumps(record, separators=(",", ":")) + "\n")
                log_file.flush()

                if pose is not None and frame is not None:
                    last_debug = overlay_pose(
                        frame,
                        result.get("matched_corners"),
                        pose,
                        camera_matrix,
                        dist_coeffs,
                        args.tag_size,
                    )

                print(
                    f"cycle={cycle_index:03d} matched={int(bool(pose))} "
                    f"cycle={record['cycle_s']:.3f}s detect={record['detect_s']:.3f}s "
                    f"lag={lag_s:.3f}s L={command['left_milli']:+4d} R={command['right_milli']:+4d} "
                    f"reason={command['reason']}",
                    flush=True,
                )
                if command.get("done"):
                    break
                if sleep_s > 0:
                    time.sleep(sleep_s)
    finally:
        if rover is not None:
            if args.stop_on_exit:
                rover.stop()
            rover.close()
        cap.release()

    elapsed_s = time.monotonic() - started
    summary = summarize(records, elapsed_s) if records else {"ok": False, "elapsed_s": elapsed_s, "cycles": 0}
    summary.update(metadata)
    summary["ended_at"] = now_utc()
    summary_path.write_text(json.dumps(summary, indent=2))
    if args.save_debug and last_debug is not None:
        cv2.imwrite(str(run_dir / "last_match.jpg"), last_debug)

    print(
        f"summary_path={summary_path} cycles={summary.get('cycles')} "
        f"hz={summary.get('overall_cycle_hz')} matched_rate={summary.get('matched_expected_rate')}",
        flush=True,
    )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
