#!/usr/bin/env python3
"""AprilTag visual-servo auto docking using the minimal ESP32 move protocol.

The intended flow is two-stage:

1. Use the camera AprilTag pose to drive into a known edge-aligned pre-contact pose.
2. Once visually aligned, use INA219 telemetry to decide whether charging contact
   already happened. If not, apply a slow bounded forward push and stop as soon
   as charging is detected.
"""

import argparse
from collections import deque
import json
import math
import signal
import statistics
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

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
    pose_matrix,
    read_camera_controls,
    scaled_camera_matrix,
)
from charge_detection import ChargeDetector, ChargeObservation, format_optional
from minimal_rover_serial import (
    ACK,
    CMD_MOVE_REL,
    CMD_PWM,
    CMD_STOP,
    MOVE_REL,
    PWM,
    PACKET_ACK,
    TelemetrySample,
    Parser,
    format_packet,
    unpack_telemetry_packet,
    write_command,
)


PHASE_IDLE = 0
PHASE_DONE = 3
PHASE_FAULT = 4
TELEM_ACTIVE = 1 << 0

stop_requested = False


def request_stop(_signum, _frame) -> None:
    global stop_requested
    stop_requested = True


def now_utc() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def median(values: list[float | None]) -> float | None:
    finite = [value for value in values if value is not None and math.isfinite(value)]
    if not finite:
        return None
    return float(statistics.median(finite))


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def wrap_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def horizontal_half_fov_deg(camera_matrix: np.ndarray, image_width_px: int) -> float:
    fx = float(camera_matrix[0, 0])
    if fx <= 0.0 or image_width_px <= 0:
        return 25.0
    return math.degrees(math.atan(image_width_px / (2.0 * fx)))


def read_target_pose(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return None


def target_camera_in_tag_frame(target_pose: dict[str, Any] | None) -> dict[str, Any] | None:
    if not target_pose:
        return None
    rotation = np.array(
        target_pose.get("rotation", {}).get("rotation_matrix_mean"),
        dtype=np.float64,
    )
    translation = np.array(
        target_pose.get("translation_m", {}).get("median"),
        dtype=np.float64,
    )
    if rotation.shape != (3, 3) or translation.shape != (3,):
        return None
    tag_from_camera = np.linalg.inv(pose_matrix(rotation, translation))
    return {
        "position_m": [float(value) for value in tag_from_camera[:3, 3].tolist()],
        "forward_axis": [float(value) for value in tag_from_camera[:3, 2].tolist()],
    }


def pose_metrics(pose: dict[str, Any], target_camera_in_tag: dict[str, Any] | None) -> dict[str, Any]:
    tag_from_camera = np.array(pose["tag_from_camera"], dtype=np.float64)
    camera_pos_tag = tag_from_camera[:3, 3]
    camera_rot_tag = tag_from_camera[:3, :3]
    camera_forward_tag = camera_rot_tag[:, 2]
    tag_bearing_deg = math.degrees(math.atan2(pose["lateral_m"], pose["range_m"]))
    heading_vs_tag_normal_deg = math.degrees(
        math.atan2(camera_forward_tag[0], max(camera_forward_tag[2], 1e-9))
    )

    target_lateral_error_m = None
    target_range_error_m = None
    if target_camera_in_tag is not None:
        target_pos = target_camera_in_tag["position_m"]
        target_lateral_error_m = float(camera_pos_tag[0] - target_pos[0])
        target_range_error_m = float(camera_pos_tag[2] - target_pos[2])

    return {
        "tag_lateral_m": float(pose["lateral_m"]),
        "tag_range_m": float(pose["range_m"]),
        "tag_vertical_m": float(pose["vertical_m"]),
        "tag_euler_xyz_deg": [float(value) for value in pose["euler_xyz_deg"]],
        "tag_bearing_deg": float(tag_bearing_deg),
        "camera_in_tag_m": [float(value) for value in camera_pos_tag.tolist()],
        "heading_vs_tag_normal_deg": float(heading_vs_tag_normal_deg),
        "target_lateral_error_m": target_lateral_error_m,
        "target_range_error_m": target_range_error_m,
        "reprojection_rmse_px": float(pose["reprojection_rmse_px"]),
    }


def summarize_metrics(samples: list[dict[str, Any]]) -> dict[str, Any]:
    if not samples:
        return {"ok": False, "matched_samples": 0}

    camera_in_tag = [
        median([sample["camera_in_tag_m"][axis] for sample in samples])
        for axis in range(3)
    ]
    return {
        "ok": True,
        "matched_samples": len(samples),
        "tag_lateral_m": median([sample["tag_lateral_m"] for sample in samples]),
        "tag_range_m": median([sample["tag_range_m"] for sample in samples]),
        "tag_vertical_m": median([sample["tag_vertical_m"] for sample in samples]),
        "tag_bearing_deg": median([sample["tag_bearing_deg"] for sample in samples]),
        "tag_yaw_deg": median([sample["tag_euler_xyz_deg"][2] for sample in samples]),
        "camera_in_tag_m": camera_in_tag,
        "heading_vs_tag_normal_deg": median([sample["heading_vs_tag_normal_deg"] for sample in samples]),
        "target_lateral_error_m": median([sample["target_lateral_error_m"] for sample in samples]),
        "target_range_error_m": median([sample["target_range_error_m"] for sample in samples]),
        "reprojection_rmse_px": median([sample["reprojection_rmse_px"] for sample in samples]),
        "samples": samples,
    }


class ContinuousTagSensor:
    def __init__(
        self,
        cap: cv2.VideoCapture,
        detector,
        expected_id: int,
        object_points: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        target_camera_in_tag: dict[str, Any] | None,
        log_path: Path,
        history_limit: int = 256,
    ):
        self.cap = cap
        self.detector = detector
        self.expected_id = expected_id
        self.object_points = object_points
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.target_camera_in_tag = target_camera_in_tag
        self.frame_seq = 0
        self.read_failures = 0
        self.recent_frames: deque[dict[str, Any]] = deque(maxlen=history_limit)
        self.log_file = log_path.open("a", encoding="utf-8")

    def close(self) -> None:
        self.log_file.close()

    def poll_once(self, context: str = "background") -> dict[str, Any]:
        ok, frame = self.cap.read()
        event: dict[str, Any] = {
            "timestamp_utc": now_utc(),
            "context": context,
            "camera_read_ok": bool(ok and frame is not None),
        }
        if not ok or frame is None:
            self.read_failures += 1
            event["read_failures"] = self.read_failures
            self.log_file.write(json.dumps(event) + "\n")
            self.log_file.flush()
            return event

        self.frame_seq += 1
        result = detect_pose(
            frame=frame,
            detector=self.detector,
            expected_id=self.expected_id,
            object_points=self.object_points,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
        )
        event["frame"] = self.frame_seq
        event["read_failures"] = self.read_failures
        event["detections"] = [record["id"] for record in result["detections"]]
        event["rejected_count"] = int(result["rejected_count"])
        event["matched_expected"] = bool(result["matched"] and result["pose"])

        record: dict[str, Any] = {"frame": self.frame_seq, "matched": False}
        if result["matched"] and result["pose"]:
            metrics = pose_metrics(result["pose"], self.target_camera_in_tag)
            metrics["frame"] = self.frame_seq
            metrics["matched_center_px"] = result["matched"]["center_px"]
            metrics["matched_edge_px"] = result["matched"]["mean_edge_px"]
            record = metrics
            record["matched"] = True
            event["metrics"] = {
                "tag_range_m": round(float(metrics["tag_range_m"]), 6),
                "tag_lateral_m": round(float(metrics["tag_lateral_m"]), 6),
                "tag_bearing_deg": round(float(metrics["tag_bearing_deg"]), 4),
                "heading_vs_tag_normal_deg": round(float(metrics["heading_vs_tag_normal_deg"]), 4),
                "target_lateral_error_m": (
                    round(float(metrics["target_lateral_error_m"]), 6)
                    if metrics["target_lateral_error_m"] is not None
                    else None
                ),
                "target_range_error_m": (
                    round(float(metrics["target_range_error_m"]), 6)
                    if metrics["target_range_error_m"] is not None
                    else None
                ),
                "reprojection_rmse_px": round(float(metrics["reprojection_rmse_px"]), 4),
            }
            if metrics["target_lateral_error_m"] is not None and metrics["target_range_error_m"] is not None:
                event["metrics"]["dock_lateral_error_mm"] = round(
                    float(metrics["target_lateral_error_m"]) * 1000.0,
                    2,
                )
                event["metrics"]["dock_range_error_mm"] = round(
                    -float(metrics["target_range_error_m"]) * 1000.0,
                    2,
                )
            event["matched_center_px"] = result["matched"]["center_px"]
            event["matched_edge_px"] = result["matched"]["mean_edge_px"]
        self.recent_frames.append(record)
        self.log_file.write(json.dumps(event) + "\n")
        self.log_file.flush()
        return event

    def poll_for_duration(self, duration_s: float, context: str) -> None:
        if duration_s <= 0:
            return
        deadline = time.monotonic() + duration_s
        while not stop_requested and time.monotonic() < deadline:
            self.poll_once(context)


def capture_sense(
    sensor: ContinuousTagSensor,
    samples_needed: int,
    max_frames: int,
    max_read_failures: int,
    min_frame_seq: int = 0,
    progress_prefix: str | None = None,
    progress_every_frames: int = 0,
) -> dict[str, Any]:
    samples = [
        dict(frame_record)
        for frame_record in sensor.recent_frames
        if frame_record.get("matched") and int(frame_record["frame"]) > min_frame_seq
    ]
    matched_frames = {int(sample["frame"]) for sample in samples}
    frames = len(
        [frame_record for frame_record in sensor.recent_frames if int(frame_record["frame"]) > min_frame_seq]
    )
    read_failures = 0
    frames = 0
    if progress_prefix:
        print(f"{progress_prefix}: sensing tag, need {samples_needed} samples within {max_frames} frames", flush=True)
    cached_frames = [
        frame_record for frame_record in sensor.recent_frames if int(frame_record["frame"]) > min_frame_seq
    ]
    frames = len(cached_frames)
    if progress_prefix and samples:
        print(
            f"{progress_prefix}: using {len(samples)} cached matches from {frames} recent frames",
            flush=True,
        )
    while (
        not stop_requested
        and frames < max_frames
        and len(samples) < samples_needed
        and read_failures < max_read_failures
    ):
        event = sensor.poll_once(progress_prefix or "sense")
        if not event["camera_read_ok"]:
            read_failures += 1
            if progress_prefix and read_failures % max(10, progress_every_frames or 10) == 0:
                print(
                    f"{progress_prefix}: camera read failed {read_failures}/{max_read_failures}",
                    flush=True,
                )
            time.sleep(0.02)
            continue
        frames += 1
        if event.get("matched_expected"):
            latest = sensor.recent_frames[-1]
            latest_frame = int(latest["frame"])
            if latest_frame > min_frame_seq and latest_frame not in matched_frames:
                samples.append(dict(latest))
                matched_frames.add(latest_frame)
        if progress_prefix and progress_every_frames > 0 and frames % progress_every_frames == 0:
            print(f"{progress_prefix}: scanned {frames}/{max_frames} frames, matches={len(samples)}", flush=True)

    summary = summarize_metrics(samples)
    summary["frames_processed"] = frames
    summary["read_failures"] = read_failures
    if stop_requested:
        summary["failure_reason"] = "stop_requested"
    elif read_failures >= max_read_failures and not samples:
        summary["failure_reason"] = "camera_read_failed"
    summary["ok"] = bool(samples) and len(samples) >= samples_needed
    return summary


def print_sense(prefix: str, sense: dict[str, Any], target_lateral_m: float, target_range_m: float) -> None:
    if not sense.get("matched_samples"):
        failure_reason = sense.get("failure_reason")
        read_failures = int(sense.get("read_failures", 0) or 0)
        if failure_reason == "camera_read_failed":
            print(f"{prefix}: camera read failed ({read_failures} consecutive empty reads)", flush=True)
            return
        if failure_reason == "stop_requested":
            print(f"{prefix}: interrupted", flush=True)
            return
        print(f"{prefix}: no tag match", flush=True)
        return
    if sense.get("target_lateral_error_m") is not None and sense.get("target_range_error_m") is not None:
        lateral_error_mm = float(sense["target_lateral_error_m"]) * 1000.0
        range_error_mm = -float(sense["target_range_error_m"]) * 1000.0
        error_frame = "dock"
    else:
        lateral_error_mm = (sense["tag_lateral_m"] - target_lateral_m) * 1000.0
        range_error_mm = (sense["tag_range_m"] - target_range_m) * 1000.0
        error_frame = "camera"
    print(
        f"{prefix}: matched={sense['matched_samples']}/{sense['frames_processed']} "
        f"range={sense['tag_range_m']:.3f}m bearing={sense['tag_bearing_deg']:+.2f}deg "
        f"{error_frame}_lateral_error={lateral_error_mm:+.0f}mm "
        f"{error_frame}_range_error={range_error_mm:+.0f}mm "
        f"heading={sense['heading_vs_tag_normal_deg']:+.2f}deg "
        f"rmse={sense['reprojection_rmse_px']:.3f}px",
        flush=True,
    )


@dataclass
class MoveResult:
    ack_ok: bool
    fault: bool
    timed_out: bool
    telemetry: list[str]
    telemetry_samples: list[TelemetrySample]


@dataclass
class ChargeCheckResult:
    samples_seen: int
    power_ready: bool
    observation: ChargeObservation | None
    telemetry_tail: list[str]


@dataclass
class ContactPushResult:
    ack_ok: bool
    fault: bool
    timed_out: bool
    charge_detected: bool
    stop_sent: bool
    telemetry_tail: list[str]
    observation: ChargeObservation | None


class MinimalRover:
    def __init__(self, port: str, baud: int, charge_detector: ChargeDetector | None = None):
        self.ser = serial.Serial(port, baud, timeout=0.02, write_timeout=1)
        self.parser = Parser()
        self.charge_detector = charge_detector

    def close(self) -> None:
        self.ser.close()

    def stop(
        self,
        seq: int,
        read_s: float = 0.2,
        background_step: Callable[[], None] | None = None,
    ) -> MoveResult:
        write_command(self.ser, CMD_STOP, seq)
        return self._read_until(seq, read_s, wait_done=False, background_step=background_step)

    def move(
        self,
        seq: int,
        x_mm: int,
        z_deg: float,
        drive_milli: int,
        turn_milli: int,
        timeout_s: float,
        background_step: Callable[[], None] | None = None,
    ) -> MoveResult:
        payload = MOVE_REL.pack(x_mm, int(round(z_deg * 100.0)), 0, 0, drive_milli, turn_milli)
        write_command(self.ser, CMD_MOVE_REL, seq, payload)
        return self._read_until(seq, timeout_s, wait_done=True, background_step=background_step)

    def pwm(
        self,
        seq: int,
        left_milli: int,
        right_milli: int,
        duration_ms: int,
        timeout_s: float,
        background_step: Callable[[], None] | None = None,
    ) -> MoveResult:
        payload = PWM.pack(left_milli, right_milli, duration_ms, 0)
        write_command(self.ser, CMD_PWM, seq, payload)
        return self._read_until(seq, timeout_s, wait_done=True, background_step=background_step)

    def push_until_charge(
        self,
        push_seq: int,
        stop_seq: int,
        left_milli: int,
        right_milli: int,
        duration_ms: int,
        timeout_s: float,
        background_step: Callable[[], None] | None = None,
    ) -> ContactPushResult:
        payload = PWM.pack(left_milli, right_milli, duration_ms, 0)
        write_command(self.ser, CMD_PWM, push_seq, payload)

        deadline = time.monotonic() + timeout_s
        ack_ok = False
        fault = False
        telemetry_lines: list[str] = []
        last_observation: ChargeObservation | None = None
        stop_sent = False

        while time.monotonic() < deadline:
            chunk = self.ser.read(256)
            if not chunk:
                if background_step is not None:
                    background_step()
                continue
            for packet in self.parser.feed(chunk):
                text = format_packet(packet)
                telemetry_lines.append(text)
                if packet.packet_type == PACKET_ACK and len(packet.payload) == ACK.size:
                    status, _command_type, detail = ACK.unpack(packet.payload)
                    if packet.seq == push_seq:
                        ack_ok = status == 0
                        if status != 0:
                            telemetry_lines.append(f"ack_reject_status={status} detail={detail}")
                            return ContactPushResult(
                                ack_ok=False,
                                fault=False,
                                timed_out=False,
                                charge_detected=False,
                                stop_sent=False,
                                telemetry_tail=telemetry_lines[-12:],
                                observation=last_observation,
                            )

                telemetry = unpack_telemetry_packet(packet)
                if telemetry is None:
                    continue
                if self.charge_detector is not None and telemetry.power_ready:
                    last_observation = self.charge_detector.update(telemetry.as_charge_packet())
                    if last_observation.state == "charging":
                        stop_sent = True
                        stop_result = self.stop(stop_seq, read_s=0.6, background_step=background_step)
                        telemetry_lines.extend(stop_result.telemetry[-12:])
                        return ContactPushResult(
                            ack_ok=ack_ok and stop_result.ack_ok,
                            fault=fault or stop_result.fault,
                            timed_out=stop_result.timed_out,
                            charge_detected=True,
                            stop_sent=True,
                            telemetry_tail=telemetry_lines[-12:],
                            observation=last_observation,
                        )
                if telemetry.active_seq == push_seq and telemetry.phase == PHASE_FAULT:
                    fault = True
                    return ContactPushResult(
                        ack_ok=ack_ok,
                        fault=True,
                        timed_out=False,
                        charge_detected=False,
                        stop_sent=stop_sent,
                        telemetry_tail=telemetry_lines[-12:],
                        observation=last_observation,
                    )
                if (
                    ack_ok
                    and telemetry.active_seq == push_seq
                    and telemetry.phase in (PHASE_DONE, PHASE_IDLE)
                    and not (telemetry.flags & TELEM_ACTIVE)
                ):
                    return ContactPushResult(
                        ack_ok=True,
                        fault=fault,
                        timed_out=False,
                        charge_detected=False,
                        stop_sent=stop_sent,
                        telemetry_tail=telemetry_lines[-12:],
                        observation=last_observation,
                    )

        return ContactPushResult(
            ack_ok=ack_ok,
            fault=fault,
            timed_out=True,
            charge_detected=False,
            stop_sent=stop_sent,
            telemetry_tail=telemetry_lines[-12:],
            observation=last_observation,
        )

    def read_telemetry_window(
        self,
        timeout_s: float,
        background_step: Callable[[], None] | None = None,
    ) -> ChargeCheckResult:
        deadline = time.monotonic() + timeout_s
        telemetry_lines: list[str] = []
        observations: list[ChargeObservation] = []
        power_ready = False
        samples_seen = 0
        while time.monotonic() < deadline:
            chunk = self.ser.read(256)
            if not chunk:
                if background_step is not None:
                    background_step()
                continue
            for packet in self.parser.feed(chunk):
                text = format_packet(packet)
                telemetry_lines.append(text)
                telemetry = unpack_telemetry_packet(packet)
                if telemetry is None:
                    continue
                samples_seen += 1
                power_ready = power_ready or telemetry.power_ready
                if self.charge_detector is None or not telemetry.power_ready:
                    continue
                observations.append(self.charge_detector.update(telemetry.as_charge_packet()))
        return ChargeCheckResult(
            samples_seen=samples_seen,
            power_ready=power_ready,
            observation=observations[-1] if observations else None,
            telemetry_tail=telemetry_lines[-12:],
        )

    def _read_until(
        self,
        seq: int,
        timeout_s: float,
        wait_done: bool,
        background_step: Callable[[], None] | None = None,
    ) -> MoveResult:
        deadline = time.monotonic() + timeout_s
        ack_ok = False
        fault = False
        telemetry_lines: list[str] = []
        telemetry_samples: list[TelemetrySample] = []
        while time.monotonic() < deadline:
            chunk = self.ser.read(256)
            if not chunk:
                if background_step is not None:
                    background_step()
                continue
            for packet in self.parser.feed(chunk):
                text = format_packet(packet)
                telemetry_lines.append(text)
                if packet.packet_type == PACKET_ACK and len(packet.payload) == ACK.size:
                    status, _command_type, detail = ACK.unpack(packet.payload)
                    ack_ok = status == 0
                    if status != 0:
                        telemetry_lines.append(f"ack_reject_status={status} detail={detail}")
                        return MoveResult(False, False, False, telemetry_lines, telemetry_samples)
                telemetry = unpack_telemetry_packet(packet)
                if telemetry is not None:
                    telemetry_samples.append(telemetry)
                    if telemetry.active_seq == seq and telemetry.phase == PHASE_FAULT:
                        fault = True
                        return MoveResult(ack_ok, True, False, telemetry_lines, telemetry_samples)
                    if (
                        wait_done
                        and ack_ok
                        and telemetry.active_seq == seq
                        and telemetry.phase in (PHASE_DONE, PHASE_IDLE)
                        and not (telemetry.flags & TELEM_ACTIVE)
                    ):
                        return MoveResult(True, False, False, telemetry_lines, telemetry_samples)
                if not wait_done and ack_ok:
                    return MoveResult(True, False, False, telemetry_lines, telemetry_samples)
        return MoveResult(ack_ok, fault, True, telemetry_lines, telemetry_samples)


def action_segments(action: dict[str, Any]) -> list[dict[str, Any]]:
    if action["type"] == "waypoint":
        return list(action["segments"])
    return [{"x_mm": int(action["x_mm"]), "z_deg": float(action["z_deg"])}]


def execute_action(
    rover: MinimalRover,
    sensor: ContinuousTagSensor,
    seq: int,
    action: dict[str, Any],
    drive_milli: int,
    turn_milli: int,
    timeout_s: float,
    settle_s: float,
) -> tuple[int, list[dict[str, Any]], bool]:
    results = []
    segments = action_segments(action)
    for segment_index, segment in enumerate(segments, start=1):
        print(
            f"execute segment {segment_index}/{len(segments)}: "
            f"x={int(segment['x_mm'])}mm z={float(segment['z_deg']):+.2f}deg "
            f"drive_milli={drive_milli} turn_milli={turn_milli}",
            flush=True,
        )
        result = rover.move(
            seq=seq,
            x_mm=int(segment["x_mm"]),
            z_deg=float(segment["z_deg"]),
            drive_milli=drive_milli,
            turn_milli=turn_milli,
            timeout_s=timeout_s,
            background_step=lambda: sensor.poll_once(f"move seq={seq}"),
        )
        results.append(
            {
                "segment": segment_index,
                "seq": seq,
                "command": segment,
                "ack_ok": result.ack_ok,
                "fault": result.fault,
                "timed_out": result.timed_out,
                "telemetry_tail": result.telemetry[-12:],
            }
        )
        seq += 1
        if not result.ack_ok or result.fault or result.timed_out:
            return seq, results, False
        if settle_s > 0 and segment_index < len(segments):
            time.sleep(settle_s)
    return seq, results, True


def charge_result_dict(result: ChargeCheckResult) -> dict[str, Any]:
    observation = result.observation
    return {
        "samples_seen": result.samples_seen,
        "power_ready": result.power_ready,
        "observation": observation.as_dict() if observation else None,
        "telemetry_tail": result.telemetry_tail,
    }


def print_charge_result(prefix: str, result: ChargeCheckResult) -> None:
    if result.samples_seen <= 0:
        print(f"{prefix}: no telemetry samples", flush=True)
        return
    if not result.power_ready:
        print(f"{prefix}: telemetry received but INA219 not ready", flush=True)
        return
    observation = result.observation
    if observation is None:
        print(f"{prefix}: telemetry received but no charge observation", flush=True)
        return
    print(
        f"{prefix}: state={observation.state} method={observation.method} "
        f"voltage={format_optional(observation.voltage_v)}V "
        f"delta={format_optional(observation.voltage_delta_v)}V "
        f"current={format_optional(observation.current_a)}A "
        f"power={format_optional(observation.power_w)}W",
        flush=True,
    )


def contact_push_result_dict(result: ContactPushResult) -> dict[str, Any]:
    return {
        "ack_ok": result.ack_ok,
        "fault": result.fault,
        "timed_out": result.timed_out,
        "charge_detected": result.charge_detected,
        "stop_sent": result.stop_sent,
        "telemetry_tail": result.telemetry_tail,
        "observation": result.observation.as_dict() if result.observation else None,
    }


def print_contact_push_result(prefix: str, result: ContactPushResult) -> None:
    observation = result.observation
    print(
        f"{prefix}: ack_ok={result.ack_ok} fault={result.fault} "
        f"timed_out={result.timed_out} charge_detected={result.charge_detected} "
        f"stop_sent={result.stop_sent}",
        flush=True,
    )
    if observation is not None:
        print(
            f"{prefix}: method={observation.method} "
            f"voltage={format_optional(observation.voltage_v)}V "
            f"delta={format_optional(observation.voltage_delta_v)}V "
            f"current={format_optional(observation.current_a)}A "
            f"power={format_optional(observation.power_w)}W",
            flush=True,
        )


def funnel_half_width_mm(args: argparse.Namespace, range_error_mm: float) -> float:
    if range_error_mm <= args.approach_clearance_mm:
        return float(args.lateral_deadband_mm)
    extra_forward_mm = range_error_mm - args.approach_clearance_mm
    return float(args.lateral_deadband_mm) + math.tan(
        math.radians(args.funnel_half_angle_deg)
    ) * extra_forward_mm


def build_backoff_action(
    args: argparse.Namespace,
    amount_mm: float,
    range_error_mm: float,
    lateral_error_mm: float,
    bearing_error_deg: float,
    heading_error_deg: float,
    reason: str,
) -> dict[str, Any]:
    x_mm = -int(
        round(
            clamp(
                max(amount_mm, float(args.min_move_mm)),
                float(args.min_move_mm),
                float(args.max_reverse_step_mm),
            )
        )
    )
    return {
        "type": "backoff",
        "x_mm": x_mm,
        "z_deg": 0.0,
        "drive_milli": args.drive_milli,
        "turn_milli": args.turn_milli,
        "range_error_mm": range_error_mm,
        "lateral_error_mm": lateral_error_mm,
        "bearing_error_deg": bearing_error_deg,
        "heading_error_deg": heading_error_deg,
        "reason": reason,
    }


def build_waypoint_action(
    args: argparse.Namespace,
    range_error_mm: float,
    lateral_error_mm: float,
    bearing_error_deg: float,
    heading_error_deg: float,
    funnel_width_mm: float,
    inside_funnel: bool,
    hold_heading_after_drive: bool = False,
    reason: str = "slant_to_centerline_approach_waypoint",
) -> dict[str, Any]:
    forward_to_approach_mm = max(range_error_mm - args.approach_clearance_mm, 0.0)
    planning_forward_mm = max(forward_to_approach_mm, float(args.min_waypoint_forward_mm))
    lateral_left_to_waypoint_mm = -lateral_error_mm
    stage_distance_mm = math.hypot(forward_to_approach_mm, lateral_left_to_waypoint_mm)
    turn_to_waypoint_deg = clamp(
        math.degrees(
            math.atan2(
                lateral_left_to_waypoint_mm,
                max(planning_forward_mm, 1e-6),
            )
        ),
        -args.max_waypoint_turn_deg,
        args.max_waypoint_turn_deg,
    )
    drive_to_waypoint_mm = int(
        round(
            clamp(
                math.hypot(planning_forward_mm, lateral_left_to_waypoint_mm),
                float(args.min_move_mm),
                float(args.max_waypoint_drive_mm),
            )
        )
    )

    segments = [
        {"x_mm": 0, "z_deg": float(turn_to_waypoint_deg)},
        {"x_mm": drive_to_waypoint_mm, "z_deg": 0.0},
    ]
    action: dict[str, Any] = {
        "type": "waypoint",
        "segments": segments,
        "drive_milli": args.drive_milli,
        "turn_milli": args.turn_milli,
        "range_error_mm": range_error_mm,
        "lateral_error_mm": lateral_error_mm,
        "bearing_error_deg": bearing_error_deg,
        "heading_error_deg": heading_error_deg,
        "forward_to_approach_mm": forward_to_approach_mm,
        "planning_forward_mm": planning_forward_mm,
        "lateral_left_to_waypoint_mm": lateral_left_to_waypoint_mm,
        "stage_distance_mm": stage_distance_mm,
        "drive_to_waypoint_mm": drive_to_waypoint_mm,
        "funnel_half_width_mm": funnel_width_mm,
        "inside_funnel": inside_funnel,
        "reason": reason,
    }
    if hold_heading_after_drive:
        action["hold_heading_after_drive"] = True
        return action

    stage_reached_after_drive = drive_to_waypoint_mm >= max(stage_distance_mm - float(args.min_move_mm), 0.0)
    action["stage_reached_after_drive"] = stage_reached_after_drive
    if not stage_reached_after_drive:
        return action

    final_turn_deg = clamp(
        -(turn_to_waypoint_deg + heading_error_deg),
        -args.max_waypoint_final_turn_deg,
        args.max_waypoint_final_turn_deg,
    )
    if abs(final_turn_deg) >= 0.25:
        segments.append({"x_mm": 0, "z_deg": float(final_turn_deg)})
        action["final_turn_deg"] = float(final_turn_deg)
    return action


def planner_error_terms(args: argparse.Namespace, sense: dict[str, Any]) -> dict[str, Any]:
    target_position = getattr(args, "target_camera_in_tag_position_m", None)
    camera_position = sense.get("camera_in_tag_m")
    if target_position and camera_position and len(target_position) >= 3 and len(camera_position) >= 3:
        camera_x_m = float(camera_position[0])
        camera_z_m = float(camera_position[2])
        target_x_m = float(target_position[0])
        target_z_m = float(target_position[2])
        lateral_error_mm = (camera_x_m - target_x_m) * 1000.0
        range_error_mm = (target_z_m - camera_z_m) * 1000.0
        bearing_error_deg = math.degrees(
            math.atan2(lateral_error_mm / 1000.0, max(abs(camera_z_m), 1e-6))
        )
        return {
            "planner_frame": "dock_tag",
            "range_error_mm": range_error_mm,
            "lateral_error_mm": lateral_error_mm,
            "bearing_error_deg": bearing_error_deg,
            "camera_in_tag_m": [float(value) for value in camera_position],
            "target_camera_in_tag_m": [float(value) for value in target_position],
        }

    target_lateral_m = sense["tag_lateral_m"] - args.target_lateral_m
    target_range_m = sense["tag_range_m"] - args.target_range_m
    bearing_error_deg = math.degrees(
        math.atan2(target_lateral_m, max(sense["tag_range_m"], 1e-6))
    )
    return {
        "planner_frame": "camera_tag_translation",
        "range_error_mm": target_range_m * 1000.0,
        "lateral_error_mm": target_lateral_m * 1000.0,
        "bearing_error_deg": bearing_error_deg,
    }


def choose_action(args: argparse.Namespace, sense: dict[str, Any]) -> dict[str, Any]:
    if not sense.get("matched_samples"):
        return {"type": "abort", "reason": "no_tag"}

    errors = planner_error_terms(args, sense)
    planner_frame = errors["planner_frame"]
    range_error_mm = float(errors["range_error_mm"])
    lateral_error_mm = float(errors["lateral_error_mm"])
    bearing_error_deg = float(errors["bearing_error_deg"])
    heading_error_deg = wrap_deg(sense["heading_vs_tag_normal_deg"] - args.target_heading_deg)
    lateral_bad = abs(lateral_error_mm) > args.lateral_deadband_mm
    bearing_bad = abs(bearing_error_deg) > args.bearing_deadband_deg
    heading_bad_for_final = (
        bool(args.use_heading)
        and abs(heading_error_deg) > args.final_approach_heading_deg
    )
    funnel_width_mm = funnel_half_width_mm(args, range_error_mm)
    inside_funnel = abs(lateral_error_mm) <= funnel_width_mm
    forward_to_approach_mm = max(range_error_mm - args.approach_clearance_mm, 0.0)
    needs_lateral_reentry = abs(lateral_error_mm) > args.lateral_deadband_mm
    close_misaligned = lateral_bad or bearing_bad or heading_bad_for_final

    done = (
        abs(range_error_mm) <= args.range_deadband_mm
        and not lateral_bad
        and not bearing_bad
        and not heading_bad_for_final
    )
    if done:
        return {
            "type": "done",
            "range_error_mm": range_error_mm,
            "lateral_error_mm": lateral_error_mm,
            "bearing_error_deg": bearing_error_deg,
            "heading_error_deg": heading_error_deg,
            "planner_frame": planner_frame,
            "final_heading_limit_deg": args.final_approach_heading_deg,
            "funnel_half_width_mm": funnel_width_mm,
            "inside_funnel": inside_funnel,
        }

    if range_error_mm <= args.alignment_clearance_mm and close_misaligned:
        action = build_backoff_action(
            args=args,
            amount_mm=args.approach_clearance_mm - range_error_mm,
            range_error_mm=range_error_mm,
            lateral_error_mm=lateral_error_mm,
            bearing_error_deg=bearing_error_deg,
            heading_error_deg=heading_error_deg,
            reason="too_close_for_alignment",
        )
        action["planner_frame"] = planner_frame
        return action

    if range_error_mm > args.approach_clearance_mm and needs_lateral_reentry:
        if forward_to_approach_mm < args.min_waypoint_forward_mm:
            action = build_backoff_action(
                args=args,
                amount_mm=args.min_waypoint_forward_mm - forward_to_approach_mm,
                range_error_mm=range_error_mm,
                lateral_error_mm=lateral_error_mm,
                bearing_error_deg=bearing_error_deg,
                heading_error_deg=heading_error_deg,
                reason="too_close_for_alignment_waypoint",
            )
            action["planner_frame"] = planner_frame
            return action
        action = build_waypoint_action(
            args=args,
            range_error_mm=range_error_mm,
            lateral_error_mm=lateral_error_mm,
            bearing_error_deg=bearing_error_deg,
            heading_error_deg=heading_error_deg,
            funnel_width_mm=funnel_width_mm,
            inside_funnel=inside_funnel,
            hold_heading_after_drive=inside_funnel and abs(heading_error_deg) > args.funnel_max_heading_deg,
            reason=(
                "slant_to_funnel_approach_hold_heading"
                if inside_funnel and abs(heading_error_deg) > args.funnel_max_heading_deg
                else (
                    "slant_to_funnel_approach"
                    if not inside_funnel
                    else "slant_to_centerline_approach_waypoint"
                )
            ),
        )
        action["planner_frame"] = planner_frame
        return action

    if range_error_mm > args.approach_clearance_mm:
        if inside_funnel and abs(heading_error_deg) > args.funnel_max_heading_deg:
            action = build_waypoint_action(
                args=args,
                range_error_mm=range_error_mm,
                lateral_error_mm=lateral_error_mm,
                bearing_error_deg=bearing_error_deg,
                heading_error_deg=heading_error_deg,
                funnel_width_mm=funnel_width_mm,
                inside_funnel=True,
                hold_heading_after_drive=True,
                reason="slant_to_funnel_approach_hold_heading",
            )
            action["planner_frame"] = planner_frame
            return action
        if inside_funnel and abs(bearing_error_deg) <= args.funnel_drive_bearing_deg:
            x_mm = int(
                round(
                    clamp(
                        args.funnel_drive_gain * forward_to_approach_mm,
                        args.min_move_mm,
                        args.max_forward_step_mm,
                    )
                )
            )
            return {
                "type": "drive",
                "x_mm": x_mm,
                "z_deg": 0.0,
                "drive_milli": args.drive_milli,
                "turn_milli": args.turn_milli,
                "range_error_mm": range_error_mm,
                "lateral_error_mm": lateral_error_mm,
                "bearing_error_deg": bearing_error_deg,
                "heading_error_deg": heading_error_deg,
                "funnel_half_width_mm": funnel_width_mm,
                "inside_funnel": inside_funnel,
                "forward_to_approach_mm": forward_to_approach_mm,
                "planner_frame": planner_frame,
                "reason": "inside_funnel_drive_to_approach",
            }
        action = build_waypoint_action(
            args=args,
            range_error_mm=range_error_mm,
            lateral_error_mm=lateral_error_mm,
            bearing_error_deg=bearing_error_deg,
            heading_error_deg=heading_error_deg,
            funnel_width_mm=funnel_width_mm,
            inside_funnel=inside_funnel,
            hold_heading_after_drive=True,
            reason="slant_to_funnel_approach_hold_heading",
        )
        action["planner_frame"] = planner_frame
        return action

    if abs(bearing_error_deg) > args.bearing_deadband_deg:
        # Positive bearing means the tag is to camera-right; firmware positive yaw is left,
        # so the default sign is negative.
        z_deg = args.turn_sign * clamp(
            args.turn_gain * bearing_error_deg,
            -args.max_turn_deg,
            args.max_turn_deg,
        )
        return {
            "type": "turn",
            "z_deg": z_deg,
            "x_mm": 0,
            "drive_milli": args.drive_milli,
            "turn_milli": args.turn_milli,
            "range_error_mm": range_error_mm,
            "lateral_error_mm": lateral_error_mm,
            "bearing_error_deg": bearing_error_deg,
            "heading_error_deg": heading_error_deg,
            "funnel_half_width_mm": funnel_width_mm,
            "inside_funnel": inside_funnel,
            "planner_frame": planner_frame,
            "reason": "bearing_turn",
        }

    if (
        bool(args.use_heading)
        and abs(range_error_mm) <= args.heading_priority_range_mm
        and heading_bad_for_final
    ):
        z_deg = args.turn_sign * clamp(
            args.heading_turn_gain * heading_error_deg,
            -args.max_heading_turn_deg,
            args.max_heading_turn_deg,
        )
        return {
            "type": "turn",
            "z_deg": z_deg,
            "x_mm": 0,
            "drive_milli": args.drive_milli,
            "turn_milli": args.turn_milli,
            "range_error_mm": range_error_mm,
            "lateral_error_mm": lateral_error_mm,
            "bearing_error_deg": bearing_error_deg,
            "heading_error_deg": heading_error_deg,
            "funnel_half_width_mm": funnel_width_mm,
            "inside_funnel": inside_funnel,
            "planner_frame": planner_frame,
            "reason": "heading_turn",
        }

    x_mm = int(
        round(
            clamp(
                args.range_gain * range_error_mm,
                -args.max_reverse_step_mm,
                args.max_forward_step_mm,
            )
        )
    )
    if abs(x_mm) < args.min_move_mm:
        x_mm = args.min_move_mm if x_mm >= 0 else -args.min_move_mm
    return {
        "type": "drive",
        "x_mm": x_mm,
        "z_deg": 0.0,
        "drive_milli": args.drive_milli,
        "turn_milli": args.turn_milli,
        "range_error_mm": range_error_mm,
        "lateral_error_mm": lateral_error_mm,
        "bearing_error_deg": bearing_error_deg,
        "heading_error_deg": heading_error_deg,
        "funnel_half_width_mm": funnel_width_mm,
        "inside_funnel": inside_funnel,
        "planner_frame": planner_frame,
        "reason": "range_drive",
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Closed-loop AprilTag auto docking over minimal ESP32 serial.")
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--model", type=Path)
    parser.add_argument("--family", default="tag16h5")
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.034)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument(
        "--autofocus",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Continuous autofocus. Leave disabled for calibrated docking geometry unless deliberately doing a non-geometric bring-up check.",
    )
    parser.add_argument(
        "--focus-absolute",
        type=int,
        default=350,
        help="Manual focus value. Use 350 with the current saved calibration unless explicitly selecting a separately calibrated focus.",
    )
    parser.add_argument("--auto-exposure", choices=("leave", "auto", "manual"), default="leave")
    parser.add_argument("--exposure-time", "--exposure-time-absolute", dest="exposure_time", type=int)
    parser.add_argument("--gain", type=int)
    parser.add_argument("--white-balance-auto", choices=("leave", "on", "off"), default="leave")
    parser.add_argument("--white-balance-temperature", type=int)
    parser.add_argument("--backlight-compensation", type=int, default=1)
    parser.add_argument("--contrast", type=int, default=32)
    parser.add_argument("--low-light-preset", "--low-light", action="store_true")
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--max-frames", type=int, default=80)
    parser.add_argument(
        "--max-read-failures",
        type=int,
        default=80,
        help="Abort one sensing burst after this many empty camera reads instead of waiting forever.",
    )
    parser.add_argument("--sense-progress-frames", type=int, default=15)

    parser.add_argument("--target-pose", type=Path, default=Path("config/auto_docking/dock_edge_tag_pose.json"))
    parser.add_argument(
        "--target-range-m",
        type=float,
        help="Desired camera-frame tag range. Defaults to the selected target-pose median z.",
    )
    parser.add_argument(
        "--target-lateral-m",
        type=float,
        help="Desired camera-frame tag lateral offset. Defaults to the selected target-pose median x.",
    )
    parser.add_argument("--range-deadband-mm", type=float, default=25.0)
    parser.add_argument("--lateral-deadband-mm", type=float, default=25.0)
    parser.add_argument("--bearing-deadband-deg", type=float, default=3.0)
    parser.add_argument("--heading-deadband-deg", type=float, default=3.0)
    parser.add_argument(
        "--use-heading",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Use solvePnP heading in the control loop. Leave off when autofocus or other intrinsics changes make absolute pose orientation less trustworthy.",
    )
    parser.add_argument(
        "--final-approach-heading-deg",
        type=float,
        default=8.0,
        help="Allow final drive/done when lateral and bearing are good and heading error is within this angle.",
    )
    parser.add_argument(
        "--alignment-clearance-mm",
        type=float,
        default=140.0,
        help="If closer than this and not aligned, back away before turning.",
    )
    parser.add_argument(
        "--heading-priority-range-mm",
        type=float,
        default=300.0,
        help="Prioritize heading alignment once range error is within this staging band.",
    )
    parser.add_argument("--range-gain", type=float, default=0.85)
    parser.add_argument("--turn-gain", type=float, default=0.9)
    parser.add_argument("--heading-turn-gain", type=float, default=0.9)
    parser.add_argument("--turn-sign", type=float, default=-1.0)
    parser.add_argument(
        "--target-heading-deg",
        type=float,
        help="Desired heading-vs-tag-normal; defaults to the selected target-pose orientation.",
    )
    parser.add_argument("--max-turn-deg", type=float, default=8.0)
    parser.add_argument("--max-heading-turn-deg", type=float, default=3.0)
    parser.add_argument(
        "--approach-clearance-mm",
        type=float,
        default=260.0,
        help="Intermediate centerline range error to aim for before final docking.",
    )
    parser.add_argument("--max-waypoint-turn-deg", type=float, default=25.0)
    parser.add_argument("--max-waypoint-final-turn-deg", type=float, default=25.0)
    parser.add_argument("--min-waypoint-forward-mm", type=float, default=120.0)
    parser.add_argument("--max-waypoint-drive-mm", type=float, default=220.0)
    parser.add_argument(
        "--funnel-half-angle-deg",
        type=float,
        help="Half-angle of the widening capture funnel beyond the centerline approach point. Defaults from camera horizontal FOV minus a visibility margin.",
    )
    parser.add_argument(
        "--funnel-visibility-margin-deg",
        type=float,
        default=3.0,
        help="Reserve this many degrees inside the camera full horizontal FOV when deriving the default funnel half-angle.",
    )
    parser.add_argument(
        "--funnel-drive-gain",
        type=float,
        default=1.0,
        help="Drive gain while already inside the funnel and approaching the centerline approach point.",
    )
    parser.add_argument(
        "--funnel-max-heading-deg",
        type=float,
        default=12.0,
        help="Maximum heading error that still allows a straight funnel-approach drive instead of a correction waypoint.",
    )
    parser.add_argument(
        "--funnel-drive-bearing-deg",
        type=float,
        default=2.0,
        help="If the tag bearing is within this angle while inside the funnel, prefer a straight drive step over extra turning.",
    )
    parser.add_argument("--scan-step-deg", type=float, default=6.0)
    parser.add_argument("--max-forward-step-mm", type=float, default=100.0)
    parser.add_argument("--max-reverse-step-mm", type=float, default=80.0)
    parser.add_argument("--min-move-mm", type=int, default=20)
    parser.add_argument("--drive-milli", type=int, default=400)
    parser.add_argument("--turn-milli", type=int, default=650)
    parser.add_argument("--settle-s", type=float, default=0.45)
    parser.add_argument("--max-steps", type=int, default=12)
    parser.add_argument("--move-timeout-s", type=float, default=4.0)
    parser.add_argument(
        "--contact-ram-milli",
        "--contact-drive-milli",
        dest="contact_ram_milli",
        type=int,
        default=180,
        help="Raw PWM milli value for the final slow forward push into the dock.",
    )
    parser.add_argument(
        "--contact-ram-duration-ms",
        type=int,
        default=2500,
        help="Bounded duration for one final slow forward push. This should be longer than the expected contact time.",
    )
    parser.add_argument(
        "--contact-ram-timeout-s",
        type=float,
        default=4.0,
        help="Host-side timeout while monitoring one final slow forward push.",
    )
    parser.add_argument(
        "--contact-ram-settle-s",
        "--contact-settle-s",
        dest="contact_ram_settle_s",
        type=float,
        default=0.7,
        help="Idle settle time after a push that ended without detected charging.",
    )
    parser.add_argument(
        "--max-contact-ram-attempts",
        "--max-contact-nudges",
        dest="max_contact_ram_attempts",
        type=int,
        default=2,
        help="Maximum number of final slow push attempts after visual alignment.",
    )
    parser.add_argument(
        "--charge-check-read-s",
        type=float,
        default=0.6,
        help="How long to observe idle telemetry while checking for docking contact.",
    )
    parser.add_argument(
        "--charge-current-threshold-a",
        type=float,
        default=0.05,
        help="Minimum absolute INA219 current to trust as a charging indicator.",
    )
    parser.add_argument(
        "--charge-power-threshold-w",
        type=float,
        default=0.5,
        help="Minimum absolute power to trust as a charging indicator.",
    )
    parser.add_argument(
        "--charge-voltage-rise-v",
        type=float,
        default=0.20,
        help="Voltage-only docking guess threshold above recent baseline.",
    )
    parser.add_argument(
        "--charge-voltage-release-v",
        type=float,
        default=0.08,
        help="Voltage fall needed to clear a previous charging latch.",
    )
    parser.add_argument(
        "--charge-confirm-samples",
        type=int,
        default=3,
        help="Consecutive voltage-rise hits required before voltage-only charging is declared.",
    )
    parser.add_argument(
        "--charge-release-samples",
        type=int,
        default=2,
        help="Consecutive near-baseline samples required to clear a charging latch.",
    )
    parser.add_argument(
        "--charge-negative-current-means-charging",
        action="store_true",
        help="Interpret negative INA219 current as charging instead of positive current.",
    )
    parser.add_argument(
        "--charge-negative-power-means-charging",
        action="store_true",
        help="Interpret negative inferred power as charging instead of positive power.",
    )

    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--seq-start", type=int, default=1000)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output-root", type=Path, default=Path("data/auto_docking_runs"))
    parser.add_argument("--name", default="auto_dock")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    run_dir = args.output_root / (datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{args.name}")
    run_dir.mkdir(parents=True, exist_ok=True)

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
    camera_matrix = scaled_camera_matrix(calibration.camera_matrix, calibration.image_size, actual_size)
    dist_coeffs = calibration.dist_coeffs.copy()
    args.camera_half_fov_deg = horizontal_half_fov_deg(camera_matrix, actual_size[0])
    if args.funnel_half_angle_deg is None:
        args.funnel_half_angle_deg = clamp(
            (2.0 * args.camera_half_fov_deg) - args.funnel_visibility_margin_deg,
            8.0,
            35.0,
        )
    object_points = build_tag_object_points(args.tag_size)
    target_pose = read_target_pose(args.target_pose)
    target_camera_in_tag = target_camera_in_tag_frame(target_pose)
    args.target_camera_in_tag_position_m = (
        target_camera_in_tag["position_m"] if target_camera_in_tag is not None else None
    )
    target_translation = (target_pose or {}).get("translation_m", {}).get("median") or []
    if args.target_range_m is None:
        args.target_range_m = float(target_translation[2]) if len(target_translation) >= 3 else 0.344
    if args.target_lateral_m is None:
        args.target_lateral_m = float(target_translation[0]) if len(target_translation) >= 1 else 0.0
    if args.target_heading_deg is None:
        forward_axis = (target_camera_in_tag or {}).get("forward_axis") or []
        args.target_heading_deg = (
            math.degrees(math.atan2(float(forward_axis[0]), max(float(forward_axis[2]), 1e-9)))
            if len(forward_axis) >= 3
            else 0.0
        )
    camera_controls = read_camera_controls(args.camera)
    charge_detector = ChargeDetector(
        current_threshold_a=args.charge_current_threshold_a,
        power_threshold_w=args.charge_power_threshold_w,
        voltage_rise_v=args.charge_voltage_rise_v,
        voltage_release_v=args.charge_voltage_release_v,
        confirm_samples=args.charge_confirm_samples,
        release_samples=args.charge_release_samples,
        negative_current_means_charging=args.charge_negative_current_means_charging,
        negative_power_means_charging=args.charge_negative_power_means_charging,
    )
    rover = MinimalRover(args.port, args.baud, charge_detector=charge_detector) if args.execute else None
    tag_log_path = run_dir / "tag_observations.jsonl"
    sensor = ContinuousTagSensor(
        cap=cap,
        detector=detector,
        expected_id=args.id,
        object_points=object_points,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        target_camera_in_tag=target_camera_in_tag,
        log_path=tag_log_path,
    )

    records: list[dict[str, Any]] = []
    exit_code = 1
    seq = args.seq_start
    contact_push_attempts_used = 0
    next_sense_min_frame_seq = 0
    try:
        if args.warmup_frames > 0:
            print(f"camera warmup: reading {args.warmup_frames} frames", flush=True)
        for _ in range(max(args.warmup_frames, 0)):
            sensor.poll_once("warmup")

        if rover is not None:
            initial_charge = rover.read_telemetry_window(
                args.charge_check_read_s,
                background_step=lambda: sensor.poll_once("startup charge check"),
            )
            print_charge_result("startup charge check", initial_charge)
            records.append(
                {
                    "step": 0,
                    "captured_at": now_utc(),
                    "action": {"type": "startup_charge_check"},
                    "executed": False,
                    "charge_check": charge_result_dict(initial_charge),
                }
            )

        for step in range(1, args.max_steps + 1):
            if stop_requested:
                break
            print(f"step {step}/{args.max_steps}: acquiring tag pose", flush=True)
            sense = capture_sense(
                sensor=sensor,
                samples_needed=args.samples,
                max_frames=args.max_frames,
                max_read_failures=args.max_read_failures,
                min_frame_seq=next_sense_min_frame_seq,
                progress_prefix=f"step {step} sense",
                progress_every_frames=args.sense_progress_frames,
            )
            next_sense_min_frame_seq = sensor.frame_seq
            if stop_requested:
                record = {
                    "step": step,
                    "captured_at": now_utc(),
                    "sense": sense,
                    "action": {"type": "abort", "reason": "stop_requested"},
                    "executed": False,
                }
                print_sense(f"step {step} sense", sense, args.target_lateral_m, args.target_range_m)
                records.append(record)
                exit_code = 130
                break
            if (
                not sense.get("matched_samples")
                and args.execute
                and sense.get("failure_reason") != "camera_read_failed"
            ):
                assert rover is not None
                scan_records = []
                for z_deg in (args.scan_step_deg, -2.0 * args.scan_step_deg, args.scan_step_deg):
                    if stop_requested:
                        break
                    print(f"step {step} reacquire: no tag, scan turn {z_deg:+.2f}deg", flush=True)
                    result = rover.move(
                        seq,
                        0,
                        z_deg,
                        args.drive_milli,
                        args.turn_milli,
                        args.move_timeout_s,
                        background_step=lambda: sensor.poll_once(f"reacquire move seq={seq}"),
                    )
                    scan_records.append(
                        {
                            "seq": seq,
                            "z_deg": z_deg,
                            "ack_ok": result.ack_ok,
                            "fault": result.fault,
                            "timed_out": result.timed_out,
                            "telemetry_tail": result.telemetry[-8:],
                        }
                    )
                    seq += 1
                    if not result.ack_ok or result.fault or result.timed_out:
                        break
                    settle_start_frame_seq = sensor.frame_seq
                    sensor.poll_for_duration(args.settle_s, f"step {step} reacquire settle")
                    sense = capture_sense(
                        sensor=sensor,
                        samples_needed=args.samples,
                        max_frames=args.max_frames,
                        max_read_failures=args.max_read_failures,
                        min_frame_seq=settle_start_frame_seq,
                        progress_prefix=f"step {step} reacquire sense",
                        progress_every_frames=args.sense_progress_frames,
                    )
                    if stop_requested:
                        break
                    if sense.get("matched_samples"):
                        break
                if scan_records:
                    records.append(
                        {
                            "step": step,
                            "captured_at": now_utc(),
                            "sense": {"ok": False, "matched_samples": 0},
                            "action": {"type": "reacquire_scan"},
                            "executed": True,
                            "scan_records": scan_records,
                        }
                    )
            print_sense(f"step {step} sense", sense, args.target_lateral_m, args.target_range_m)
            action = choose_action(args, sense)
            print(f"step {step} action: {action}", flush=True)
            record = {
                "step": step,
                "captured_at": now_utc(),
                "sense": sense,
                "action": action,
                "executed": False,
            }

            if action["type"] == "done":
                if not args.execute:
                    records.append(record)
                    exit_code = 0
                    break

                assert rover is not None
                charge_before = rover.read_telemetry_window(
                    args.charge_check_read_s,
                    background_step=lambda: sensor.poll_once(f"step {step} contact check"),
                )
                print_charge_result(f"step {step} contact check", charge_before)
                record["executed"] = True
                record["contact_checks"] = [charge_result_dict(charge_before)]
                if charge_before.observation is not None and charge_before.observation.state == "charging":
                    record["final_state"] = "charging_detected"
                    records.append(record)
                    exit_code = 0
                    break
                if contact_push_attempts_used >= args.max_contact_ram_attempts or args.contact_ram_milli == 0:
                    record["final_state"] = "visual_done_without_charge"
                    records.append(record)
                    exit_code = 5
                    break

                print(
                    f"step {step} contact push: pwm=({args.contact_ram_milli},{args.contact_ram_milli}) "
                    f"duration_ms={args.contact_ram_duration_ms}",
                    flush=True,
                )
                push_result = rover.push_until_charge(
                    push_seq=seq,
                    stop_seq=seq + 1,
                    left_milli=args.contact_ram_milli,
                    right_milli=args.contact_ram_milli,
                    duration_ms=args.contact_ram_duration_ms,
                    timeout_s=args.contact_ram_timeout_s,
                    background_step=lambda: sensor.poll_once(f"step {step} contact push"),
                )
                push_seq = seq
                seq += 2 if push_result.stop_sent else 1
                contact_push_attempts_used += 1
                print_contact_push_result(f"step {step} contact push", push_result)
                record["contact_push"] = {
                    "index": contact_push_attempts_used,
                    "push_seq": push_seq,
                    "stop_seq": push_seq + 1 if push_result.stop_sent else None,
                    "left_milli": args.contact_ram_milli,
                    "right_milli": args.contact_ram_milli,
                    "duration_ms": args.contact_ram_duration_ms,
                    **contact_push_result_dict(push_result),
                }
                if not push_result.ack_ok or push_result.fault or push_result.timed_out:
                    record["final_state"] = "contact_push_failed"
                    records.append(record)
                    exit_code = 3
                    break
                if push_result.charge_detected:
                    record["final_state"] = "charging_detected"
                    records.append(record)
                    exit_code = 0
                    break

                sensor.poll_for_duration(args.contact_ram_settle_s, f"step {step} contact settle")
                charge_after = rover.read_telemetry_window(
                    args.charge_check_read_s,
                    background_step=lambda: sensor.poll_once(f"step {step} post-push charge check"),
                )
                print_charge_result(f"step {step} post-push charge check", charge_after)
                record["contact_checks"].append(charge_result_dict(charge_after))
                if charge_after.observation is not None and charge_after.observation.state == "charging":
                    record["final_state"] = "charging_detected"
                    records.append(record)
                    exit_code = 0
                    break

                record["final_state"] = "push_complete_no_charge"
                records.append(record)
                continue
            if action["type"] == "abort":
                records.append(record)
                exit_code = 2
                break

            if args.execute:
                assert rover is not None
                seq, segment_results, action_ok = execute_action(
                    rover=rover,
                    sensor=sensor,
                    seq=seq,
                    action=action,
                    drive_milli=int(action["drive_milli"]),
                    turn_milli=int(action["turn_milli"]),
                    timeout_s=args.move_timeout_s,
                    settle_s=args.settle_s,
                )
                print(
                    f"step {step} move_result: ok={action_ok} "
                    f"segments={len(segment_results)}",
                    flush=True,
                )
                record["executed"] = True
                record["segment_results"] = segment_results
                if not action_ok:
                    records.append(record)
                    exit_code = 3
                    break
            else:
                print("dry-run: not executing move; pass --execute to drive", flush=True)
                records.append(record)
                exit_code = 0
                break

            records.append(record)
            settle_start_frame_seq = sensor.frame_seq
            sensor.poll_for_duration(args.settle_s, f"step {step} settle")
            next_sense_min_frame_seq = settle_start_frame_seq
        else:
            exit_code = 4
    finally:
        if rover is not None:
            try:
                rover.stop(seq, background_step=lambda: sensor.poll_once("shutdown stop"))
            finally:
                rover.close()
        sensor.close()
        cap.release()

    payload = {
        "schema_version": "wave_rover.auto_dock/v1",
        "captured_at": now_utc(),
        "args": {
            **vars(args),
            "model": str(args.model) if args.model else None,
            "target_pose": str(args.target_pose),
            "output_root": str(args.output_root),
        },
        "camera": {
            "device": args.camera,
            "actual_size": actual_size,
            "model": str(calibration.model_path),
            "startup_controls": startup_controls,
            "controls": camera_controls,
        },
        "target_camera_in_tag": target_camera_in_tag,
        "contact_push_attempts_used": contact_push_attempts_used,
        "tag_observations_path": str(tag_log_path),
        "records": records,
        "exit_code": exit_code,
    }
    (run_dir / "session.json").write_text(json.dumps(payload, indent=2) + "\n")
    print(f"saved={run_dir}", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
