#!/usr/bin/env python3
import argparse
import json
import signal
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

import apriltag_pose as poselib
from apriltag_pose import (
    build_tag_object_points,
    detect_pose,
    load_camera_calibration,
    make_detector,
    open_camera,
    request_stop,
    rotation_matrix_to_euler_xyz_deg,
    scaled_camera_matrix,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture and save a docked AprilTag target pose from live camera detections."
    )
    parser.add_argument("--camera", default="/dev/video0", help="Camera device or index.")
    parser.add_argument(
        "--model",
        type=Path,
        help="Path to an mrcal camera model. Defaults to the newest saved model.",
    )
    parser.add_argument("--family", default="tag16h5", choices=["tag16h5", "tag25h9", "tag36h10", "tag36h11"])
    parser.add_argument("--id", type=int, default=0, help="Expected tag ID.")
    parser.add_argument("--tag-size", type=float, default=0.034, help="Tag black-square size in meters.")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument(
        "--autofocus",
        action="store_true",
        help="Enable continuous autofocus. Default is off for repeatable pose geometry.",
    )
    parser.add_argument(
        "--focus-absolute",
        type=int,
        default=350,
        help="Manual focus value for UVC cameras. Use 350 with the current saved calibration unless explicitly using a separately calibrated focus.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=25,
        help="Number of matched pose samples to collect before saving the target.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=300,
        help="Maximum camera frames to inspect before giving up.",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=15,
        help="Read and discard this many frames before calibration capture.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("config/auto_docking/docked_tag_pose.json"),
        help="Where to save the docked target pose JSON.",
    )
    return parser.parse_args()


def average_rotation_matrix(rotations: list[np.ndarray]) -> np.ndarray:
    stacked = np.zeros((3, 3), dtype=np.float64)
    for rotation in rotations:
        stacked += rotation
    u, _, vt = np.linalg.svd(stacked)
    averaged = u @ vt
    if np.linalg.det(averaged) < 0:
        u[:, -1] *= -1.0
        averaged = u @ vt
    return averaged


def build_output_record(
    args: argparse.Namespace,
    calibration_path: Path,
    actual: dict,
    frame_count: int,
    pose_samples: list[dict],
) -> dict:
    translations = np.array([sample["tvec_m"] for sample in pose_samples], dtype=np.float64)
    rotations = [
        np.array(sample["rotation_matrix"], dtype=np.float64) for sample in pose_samples
    ]
    average_rotation = average_rotation_matrix(rotations)
    avg_rvec, _ = cv2.Rodrigues(average_rotation)
    rmse_values = np.array(
        [sample["reprojection_rmse_px"] for sample in pose_samples],
        dtype=np.float64,
    )
    edge_values = np.array(
        [sample["mean_edge_px"] for sample in pose_samples if sample["mean_edge_px"] is not None],
        dtype=np.float64,
    )

    return {
        "version": 1,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "system": {
            "hostname": socket.gethostname(),
            "machine_id": Path("/etc/machine-id").read_text().strip() if Path("/etc/machine-id").exists() else None,
        },
        "pose_frame": {
            "name": "tag_in_camera_opencv",
            "camera_axes": {
                "x": "right_in_image",
                "y": "down_in_image",
                "z": "forward_from_camera",
            },
            "pose_definition": "solvePnP tag pose expressed in the camera frame",
        },
        "camera": {
            "device": args.camera,
            "camera_id": args.camera,
            "model": str(calibration_path),
            "requested": {
                "width": args.width,
                "height": args.height,
                "fps": args.fps,
                "fourcc": args.fourcc,
                "autofocus": bool(args.autofocus),
                "focus_absolute": args.focus_absolute,
            },
            "actual": actual,
        },
        "tag": {
            "family": args.family,
            "id": args.id,
            "size_m": args.tag_size,
        },
        "capture": {
            "frames_processed": frame_count,
            "matched_samples": len(pose_samples),
        },
        "translation_m": {
            "axis_order": ["x_right", "y_down", "z_forward"],
            "mean": [float(v) for v in translations.mean(axis=0).tolist()],
            "median": [float(v) for v in np.median(translations, axis=0).tolist()],
            "std": [float(v) for v in translations.std(axis=0).tolist()],
        },
        "rotation": {
            "rvec_mean_rad": [float(v) for v in avg_rvec.reshape(3).tolist()],
            "rotation_matrix_mean": [[float(v) for v in row] for row in average_rotation.tolist()],
            "euler_xyz_deg_mean": [
                float(v) for v in rotation_matrix_to_euler_xyz_deg(average_rotation)
            ],
        },
        "quality": {
            "reprojection_rmse_px_mean": float(rmse_values.mean()),
            "reprojection_rmse_px_max": float(rmse_values.max()),
            "mean_edge_px_mean": float(edge_values.mean()) if edge_values.size else None,
            "mean_edge_px_min": float(edge_values.min()) if edge_values.size else None,
        },
    }


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    calibration = load_camera_calibration(args.model)
    detector = make_detector(args.family)
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
    camera_matrix = scaled_camera_matrix(
        calibration.camera_matrix,
        calibration.image_size,
        actual_size,
    )
    dist_coeffs = calibration.dist_coeffs.copy()
    object_points = build_tag_object_points(args.tag_size)

    print(
        f"collecting docked tag pose samples={args.samples} model={calibration.model_path} "
        f"camera={args.camera} actual={actual_size[0]}x{actual_size[1]} focus={actual['focus']:.0f}"
    )

    pose_samples = []
    frame_count = 0

    try:
        for _ in range(max(args.warmup_frames, 0)):
            if poselib.stop_requested:
                break
            cap.read()

        while (
            not poselib.stop_requested
            and frame_count < args.max_frames
            and len(pose_samples) < args.samples
        ):
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            result = detect_pose(
                frame=frame,
                detector=detector,
                expected_id=args.id,
                object_points=object_points,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
            )
            frame_count += 1

            if result["pose"] and result["matched"]:
                sample = dict(result["pose"])
                sample["mean_edge_px"] = result["matched"]["mean_edge_px"]
                pose_samples.append(sample)
                tvec = sample["tvec_m"]
                print(
                    f"sample {len(pose_samples):02d}/{args.samples}: "
                    f"x={tvec[0]:+.3f} y={tvec[1]:+.3f} z={tvec[2]:+.3f} m "
                    f"rmse={sample['reprojection_rmse_px']:.3f}px"
                )
    finally:
        cap.release()

    if len(pose_samples) < args.samples:
        print(
            f"only collected {len(pose_samples)} matched samples in {frame_count} frames; not writing target"
        )
        return 2

    record = build_output_record(
        args=args,
        calibration_path=calibration.model_path,
        actual=actual,
        frame_count=frame_count,
        pose_samples=pose_samples,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, indent=2) + "\n")
    print(f"saved {args.output}")
    print(
        "target median translation_m="
        f"{record['translation_m']['median']} "
        f"euler_xyz_deg_mean={record['rotation']['euler_xyz_deg_mean']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
