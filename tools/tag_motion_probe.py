#!/usr/bin/env python3
import argparse
import json
import math
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from apriltag_pose import (
    build_tag_object_points,
    detect_pose,
    load_camera_calibration,
    make_detector,
    open_camera,
    read_camera_controls,
    scaled_camera_matrix,
)
from rover_control import RoverController, body_to_lr, clamp_pwm
from rover_motion_probe import (
    extract_gyro_z_dps,
    extract_voltage,
    extract_yaw_deg,
    integrate_trapezoid,
    read_cycle_packets,
)


def now_utc() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Probe tiny rover motion while logging AprilTag pose and ESP32 IMU feedback "
            "in one run."
        )
    )
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--model", type=Path, help="Optional mrcal model path.")
    parser.add_argument("--family", default="tag16h5")
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.034)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--autofocus", action="store_true")
    parser.add_argument("--focus-absolute", type=int, default=350)
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument(
        "--mode",
        choices=("forward", "backward", "turn-ccw", "turn-cw"),
        default="forward",
    )
    parser.add_argument(
        "--x-pwm",
        type=float,
        default=0.0,
        help="Optional explicit +X PWM override. If set, overrides --mode defaults.",
    )
    parser.add_argument(
        "--z-pwm",
        type=float,
        default=0.0,
        help="Optional explicit +Z PWM override. If set, overrides --mode defaults.",
    )
    parser.add_argument("--pwm", type=float, default=0.10, help="Default pulse magnitude.")
    parser.add_argument("--duration", type=float, default=0.15, help="Motion pulse duration.")
    parser.add_argument("--pre-seconds", type=float, default=1.0)
    parser.add_argument("--post-seconds", type=float, default=1.0)
    parser.add_argument("--sample-period", type=float, default=0.08)
    parser.add_argument("--command-period", type=float, default=0.10)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("data/tag_motion_probes"),
    )
    parser.add_argument(
        "--target-pose",
        type=Path,
        default=Path("config/auto_docking/docked_tag_pose.json"),
        help="Saved docked target pose JSON for reference comparisons.",
    )
    parser.add_argument(
        "--name",
        default="probe",
        help="Short run label appended to the output directory name.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Optional exact run output directory. Overrides the timestamp-based default.",
    )
    parser.add_argument("--batch-id", help="Optional batch identifier for grouped collection runs.")
    parser.add_argument(
        "--sequence-name",
        help="Optional higher-level sequence label for grouped collection runs.",
    )
    parser.add_argument(
        "--run-index",
        type=int,
        help="Optional 1-based run index within a grouped collection run.",
    )
    return parser.parse_args()


def default_body_command(mode: str, pwm: float) -> tuple[float, float]:
    pwm = clamp_pwm(pwm)
    if mode == "forward":
        return pwm, 0.0
    if mode == "backward":
        return -pwm, 0.0
    if mode == "turn-ccw":
        return 0.0, pwm
    if mode == "turn-cw":
        return 0.0, -pwm
    raise ValueError(f"unsupported mode: {mode}")


def choose_body_command(args: argparse.Namespace) -> tuple[float, float]:
    if abs(args.x_pwm) > 1e-9 or abs(args.z_pwm) > 1e-9:
        return clamp_pwm(args.x_pwm), clamp_pwm(args.z_pwm)
    return default_body_command(args.mode, args.pwm)


def read_target_pose(path: Path) -> dict | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return None


def translation_delta(a: list[float] | None, b: list[float] | None) -> list[float] | None:
    if a is None or b is None:
        return None
    return [float(b[i] - a[i]) for i in range(3)]


def median_vector(vectors: list[list[float]]) -> list[float] | None:
    if not vectors:
        return None
    arr = np.array(vectors, dtype=np.float64)
    return [float(v) for v in np.median(arr, axis=0).tolist()]


def mean_or_none(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def collect_phase(
    phase_name: str,
    phase_duration_s: float,
    cap: cv2.VideoCapture,
    detector,
    expected_id: int,
    object_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rover: RoverController,
    left_pwm: float,
    right_pwm: float,
    moving: bool,
    command_period_s: float,
    sample_period_s: float,
) -> list[dict]:
    samples = []
    phase_start = time.monotonic()
    next_command_time = phase_start

    while True:
        now = time.monotonic()
        elapsed = now - phase_start
        if elapsed >= phase_duration_s:
            break

        if moving and now >= next_command_time:
            rover.send_lr(left_pwm, right_pwm)
            next_command_time += command_period_s

        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.02)
            continue

        pose_result = detect_pose(
            frame=frame,
            detector=detector,
            expected_id=expected_id,
            object_points=object_points,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )
        packets, imu_packet, chassis_packet = read_cycle_packets(
            rover.ser,
            read_window_s=min(sample_period_s * 0.45, 0.03),
            request_chassis=True,
        )
        yaw_deg, yaw_key = extract_yaw_deg(imu_packet or chassis_packet)
        gyro_z_dps, gyro_key = extract_gyro_z_dps(imu_packet)
        voltage_v = extract_voltage(chassis_packet)

        sample = {
            "phase": phase_name,
            "elapsed_s": elapsed,
            "tag_detected": bool(pose_result["matched"] and pose_result["pose"]),
            "tag_pose": pose_result["pose"],
            "tag_detection": pose_result["matched"],
            "rejected_count": pose_result["rejected_count"],
            "imu_packet": imu_packet,
            "chassis_packet": chassis_packet,
            "yaw_deg": yaw_deg,
            "yaw_key": yaw_key,
            "gyro_z_dps": gyro_z_dps,
            "gyro_key": gyro_key,
            "voltage_v": voltage_v,
            "raw_packets": packets,
        }
        samples.append(sample)
        time.sleep(sample_period_s)

    return samples


def summarize_phase(samples: list[dict]) -> dict:
    tag_vectors = [sample["tag_pose"]["tvec_m"] for sample in samples if sample.get("tag_pose")]
    tag_eulers = [
        sample["tag_pose"]["euler_xyz_deg"] for sample in samples if sample.get("tag_pose")
    ]
    yaw_values = [sample["yaw_deg"] for sample in samples if sample.get("yaw_deg") is not None]
    gyro_points = [
        (sample["elapsed_s"], sample["gyro_z_dps"])
        for sample in samples
        if sample.get("gyro_z_dps") is not None
    ]
    rmse_values = [
        sample["tag_pose"]["reprojection_rmse_px"]
        for sample in samples
        if sample.get("tag_pose")
    ]
    return {
        "sample_count": len(samples),
        "tag_detected_count": sum(1 for sample in samples if sample.get("tag_pose")),
        "tag_translation_median_m": median_vector(tag_vectors),
        "tag_euler_xyz_deg_median": median_vector(tag_eulers),
        "yaw_deg_mean": mean_or_none(yaw_values),
        "gyro_z_dps_mean": mean_or_none([v for _, v in gyro_points]),
        "gyro_integrated_deg": integrate_trapezoid(gyro_points),
        "tag_reprojection_rmse_px_mean": mean_or_none(rmse_values),
    }


def save_run(output_dir: Path, payload: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "session.json").write_text(json.dumps(payload, indent=2) + "\n")


def main() -> int:
    args = parse_args()

    x_pwm, z_pwm = choose_body_command(args)
    left_pwm, right_pwm = body_to_lr(x_pwm, z_pwm)

    calibration = load_camera_calibration(args.model)
    detector = make_detector(args.family)
    cap = open_camera(args)
    actual_size = (
        int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))),
        int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))),
    )
    camera_matrix = scaled_camera_matrix(
        calibration.camera_matrix,
        calibration.image_size,
        actual_size,
    )
    dist_coeffs = calibration.dist_coeffs.copy()
    object_points = build_tag_object_points(args.tag_size)
    target_pose = read_target_pose(args.target_pose)
    camera_controls = read_camera_controls(args.camera)

    if args.output_dir is not None:
        output_dir = args.output_dir
    else:
        run_name = datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{args.mode}_{args.name}"
        output_dir = args.output_root / run_name

    try:
        with RoverController(port=args.port, baud=args.baud, timeout=args.timeout) as rover:
            pre_samples = collect_phase(
                "pre",
                args.pre_seconds,
                cap,
                detector,
                args.id,
                object_points,
                camera_matrix,
                dist_coeffs,
                rover,
                0.0,
                0.0,
                moving=False,
                command_period_s=args.command_period,
                sample_period_s=args.sample_period,
            )
            motion_samples = collect_phase(
                "motion",
                args.duration,
                cap,
                detector,
                args.id,
                object_points,
                camera_matrix,
                dist_coeffs,
                rover,
                left_pwm,
                right_pwm,
                moving=True,
                command_period_s=args.command_period,
                sample_period_s=args.sample_period,
            )
            rover.stop()
            post_samples = collect_phase(
                "post",
                args.post_seconds,
                cap,
                detector,
                args.id,
                object_points,
                camera_matrix,
                dist_coeffs,
                rover,
                0.0,
                0.0,
                moving=False,
                command_period_s=args.command_period,
                sample_period_s=args.sample_period,
            )
    finally:
        cap.release()

    pre_summary = summarize_phase(pre_samples)
    motion_summary = summarize_phase(motion_samples)
    post_summary = summarize_phase(post_samples)

    target_translation = None
    if target_pose:
        target_translation = target_pose.get("translation_m", {}).get("median")

    payload = {
        "schema_version": "wave_rover.tag_motion_probe/v2",
        "run_id": output_dir.name,
        "captured_at": now_utc(),
        "batch": {
            "batch_id": args.batch_id,
            "sequence_name": args.sequence_name,
            "run_index": args.run_index,
        },
        "camera": {
            "device": args.camera,
            "actual_size": list(actual_size),
            "focus_absolute": args.focus_absolute,
            "model": str(calibration.model_path),
            "controls": camera_controls,
        },
        "command": {
            "mode": args.mode,
            "x_pwm": x_pwm,
            "z_pwm": z_pwm,
            "left_pwm": left_pwm,
            "right_pwm": right_pwm,
            "duration_s": args.duration,
        },
        "target_pose_reference": {
            "path": str(args.target_pose),
            "translation_median_m": target_translation,
        },
        "pre_summary": pre_summary,
        "motion_summary": motion_summary,
        "post_summary": post_summary,
        "derived": {
            "post_minus_pre_translation_m": translation_delta(
                pre_summary["tag_translation_median_m"],
                post_summary["tag_translation_median_m"],
            ),
            "pre_minus_target_translation_m": translation_delta(
                target_translation,
                pre_summary["tag_translation_median_m"],
            ),
            "post_minus_target_translation_m": translation_delta(
                target_translation,
                post_summary["tag_translation_median_m"],
            ),
        },
        "samples": {
            "pre": pre_samples,
            "motion": motion_samples,
            "post": post_samples,
        },
    }
    save_run(output_dir, payload)

    print(f"saved={output_dir}")
    print(
        "pre_tag="
        f"{pre_summary['tag_translation_median_m']} "
        "post_tag="
        f"{post_summary['tag_translation_median_m']} "
        "post_minus_pre="
        f"{payload['derived']['post_minus_pre_translation_m']}"
    )
    print(
        "imu: motion_yaw_mean="
        f"{motion_summary['yaw_deg_mean']} "
        "gyro_integrated_deg="
        f"{motion_summary['gyro_integrated_deg']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
