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
from rover_control import RoverController


def now_utc() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture a short AprilTag pose burst, report heading/off-axis metrics, "
            "and optionally execute one one-sided arc step to reduce horizontal bearing error."
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
    parser.add_argument(
        "--auto-exposure",
        choices=("leave", "auto", "manual"),
        default="leave",
        help="Optional exposure mode override.",
    )
    parser.add_argument(
        "--exposure-time",
        type=int,
        help="Optional manual exposure_time_absolute value.",
    )
    parser.add_argument("--gain", type=int, help="Optional gain override.")
    parser.add_argument(
        "--white-balance-auto",
        choices=("leave", "on", "off"),
        default="leave",
        help="Optional auto white balance override.",
    )
    parser.add_argument(
        "--white-balance-temperature",
        type=int,
        help="Optional white balance temperature override.",
    )
    parser.add_argument(
        "--backlight-compensation",
        type=int,
        help="Optional backlight compensation override.",
    )
    parser.add_argument("--contrast", type=int, help="Optional contrast override.")
    parser.add_argument(
        "--low-light-preset",
        action="store_true",
        help="Apply the shared indoor-night pose preset from apriltag_pose.",
    )
    parser.add_argument("--warmup-frames", type=int, default=12)
    parser.add_argument("--samples", type=int, default=7, help="Matched samples per sense burst.")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=80,
        help="Maximum camera frames to inspect while collecting one burst.",
    )
    parser.add_argument(
        "--target-pose",
        type=Path,
        default=Path("config/auto_docking/docked_tag_pose.json"),
        help="Saved dock reference for comparison metrics.",
    )
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument(
        "--arc-pwm",
        type=float,
        default=0.16,
        help="PWM magnitude for the one-sided heading arc. Only one side is driven.",
    )
    parser.add_argument(
        "--arc-duration",
        type=float,
        default=0.25,
        help="Duration of the one-sided heading arc pulse in seconds.",
    )
    parser.add_argument(
        "--command-period",
        type=float,
        default=0.10,
        help="How often to repeat the T=1 command during the pulse.",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=0.45,
        help="Pause after the motion pulse before the post-step sense burst.",
    )
    parser.add_argument(
        "--bearing-deadband-deg",
        type=float,
        default=2.5,
        help="Do not step when the horizontal tag-center bearing is already within this band.",
    )
    parser.add_argument(
        "--heading-deadband-deg",
        type=float,
        default=2.0,
        help="Preferred deadband for heading_vs_normal when that metric is available.",
    )
    parser.add_argument(
        "--control-metric",
        choices=("auto", "bearing", "heading"),
        default="auto",
        help=(
            "Which visual metric to servo on. "
            "'bearing' is better for step-1 off-center aiming; "
            "'heading' is better when centering is already handled."
        ),
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually drive the rover. Default is sense/report only.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("data/docking_heading_steps"),
    )
    parser.add_argument(
        "--name",
        default="step",
        help="Short run label appended to the output directory name.",
    )
    return parser.parse_args()


def read_target_pose(path: Path) -> dict | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return None


def target_camera_in_tag_frame(target_pose: dict | None) -> dict | None:
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
        "position_m": [float(v) for v in tag_from_camera[:3, 3].tolist()],
        "forward_axis": [float(v) for v in tag_from_camera[:3, 2].tolist()],
    }


def median_or_none(values: list[float]) -> float | None:
    if not values:
        return None
    return float(statistics.median(values))


def derive_sample_metrics(pose: dict, target_camera_in_tag: dict | None) -> dict:
    tag_from_camera = np.array(pose["tag_from_camera"], dtype=np.float64)
    camera_pos_tag = tag_from_camera[:3, 3]
    camera_rot_tag = tag_from_camera[:3, :3]
    camera_forward_tag = camera_rot_tag[:, 2]

    tag_bearing_deg = math.degrees(math.atan2(pose["lateral_m"], pose["range_m"]))
    centerline_skew_deg = math.degrees(
        math.atan2(camera_pos_tag[0], max(-camera_pos_tag[2], 1e-9))
    )
    heading_vs_tag_normal_deg = math.degrees(
        math.atan2(camera_forward_tag[0], max(camera_forward_tag[2], 1e-9))
    )

    target_lateral_error = None
    target_range_error = None
    if target_camera_in_tag is not None:
        target_pos = target_camera_in_tag["position_m"]
        target_lateral_error = float(camera_pos_tag[0] - target_pos[0])
        target_range_error = float(camera_pos_tag[2] - target_pos[2])

    return {
        "tag_tvec_m": [float(v) for v in pose["tvec_m"]],
        "tag_euler_xyz_deg": [float(v) for v in pose["euler_xyz_deg"]],
        "camera_in_tag_m": [float(v) for v in camera_pos_tag.tolist()],
        "camera_forward_in_tag": [float(v) for v in camera_forward_tag.tolist()],
        "range_m": float(pose["range_m"]),
        "tag_bearing_deg": float(tag_bearing_deg),
        "off_axis_m": float(camera_pos_tag[0]),
        "centerline_skew_deg": float(centerline_skew_deg),
        "heading_vs_tag_normal_deg": float(heading_vs_tag_normal_deg),
        "vertical_offset_in_tag_m": float(camera_pos_tag[1]),
        "reprojection_rmse_px": float(pose["reprojection_rmse_px"]),
        "target_lateral_error_m": target_lateral_error,
        "target_range_error_m": target_range_error,
    }


def capture_burst(
    cap: cv2.VideoCapture,
    detector,
    expected_id: int,
    object_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    samples_needed: int,
    max_frames: int,
    target_camera_in_tag: dict | None,
) -> dict:
    matched = []
    frames_processed = 0
    last_detection = None

    while len(matched) < samples_needed and frames_processed < max_frames:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.03)
            continue

        result = detect_pose(
            frame=frame,
            detector=detector,
            expected_id=expected_id,
            object_points=object_points,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )
        frames_processed += 1
        if result["matched"] and result["pose"]:
            metrics = derive_sample_metrics(result["pose"], target_camera_in_tag)
            metrics["matched_center_px"] = result["matched"]["center_px"]
            metrics["matched_edge_px"] = result["matched"]["mean_edge_px"]
            matched.append(metrics)
            last_detection = result["matched"]

    if not matched:
        return {
            "ok": False,
            "frames_processed": frames_processed,
            "matched_samples": 0,
            "last_detection": last_detection,
        }

    summary = {
        "ok": len(matched) >= samples_needed,
        "frames_processed": frames_processed,
        "matched_samples": len(matched),
        "tag_tvec_m_median": [median_or_none([m["tag_tvec_m"][i] for m in matched]) for i in range(3)],
        "camera_in_tag_m_median": [
            median_or_none([m["camera_in_tag_m"][i] for m in matched]) for i in range(3)
        ],
        "camera_forward_in_tag_median": [
            median_or_none([m["camera_forward_in_tag"][i] for m in matched]) for i in range(3)
        ],
        "range_m_median": median_or_none([m["range_m"] for m in matched]),
        "tag_bearing_deg_median": median_or_none([m["tag_bearing_deg"] for m in matched]),
        "off_axis_m_median": median_or_none([m["off_axis_m"] for m in matched]),
        "centerline_skew_deg_median": median_or_none(
            [m["centerline_skew_deg"] for m in matched]
        ),
        "heading_vs_tag_normal_deg_median": median_or_none(
            [m["heading_vs_tag_normal_deg"] for m in matched]
        ),
        "vertical_offset_in_tag_m_median": median_or_none(
            [m["vertical_offset_in_tag_m"] for m in matched]
        ),
        "target_lateral_error_m_median": median_or_none(
            [m["target_lateral_error_m"] for m in matched if m["target_lateral_error_m"] is not None]
        ),
        "target_range_error_m_median": median_or_none(
            [m["target_range_error_m"] for m in matched if m["target_range_error_m"] is not None]
        ),
        "reprojection_rmse_px_median": median_or_none(
            [m["reprojection_rmse_px"] for m in matched]
        ),
        "matched_edge_px_median": median_or_none([m["matched_edge_px"] for m in matched]),
        "samples": matched,
    }
    return summary


def choose_arc_step(
    sense: dict,
    bearing_deadband_deg: float,
    heading_deadband_deg: float,
    control_metric: str,
    arc_pwm: float,
    arc_duration_s: float,
) -> dict:
    heading_deg = sense.get("heading_vs_tag_normal_deg_median")
    bearing_deg = sense.get("tag_bearing_deg_median")

    if control_metric == "heading":
        if heading_deg is not None and abs(heading_deg) > heading_deadband_deg:
            decision_metric = "heading_vs_normal"
            decision_value = heading_deg
        else:
            decision_metric = None
            decision_value = None
    elif control_metric == "bearing":
        if bearing_deg is not None and abs(bearing_deg) > bearing_deadband_deg:
            decision_metric = "bearing"
            decision_value = bearing_deg
        else:
            decision_metric = None
            decision_value = None
    elif heading_deg is not None and abs(heading_deg) > heading_deadband_deg:
        decision_metric = "heading_vs_normal"
        decision_value = heading_deg
    elif bearing_deg is not None and abs(bearing_deg) > bearing_deadband_deg:
        decision_metric = "bearing"
        decision_value = bearing_deg
    else:
        return {
            "should_move": False,
            "reason": "within_deadband"
            if (heading_deg is not None or bearing_deg is not None)
            else "no_heading_or_bearing",
            "left_pwm": 0.0,
            "right_pwm": 0.0,
            "duration_s": 0.0,
            "arc_mode": "none",
            "decision_metric": None,
            "decision_value_deg": None,
            "control_metric": control_metric,
        }

    if decision_value > 0.0:
        return {
            "should_move": True,
            "reason": f"{decision_metric}_positive_turn_right_arc",
            "left_pwm": arc_pwm,
            "right_pwm": 0.0,
            "duration_s": arc_duration_s,
            "arc_mode": "right_arc_left_side_only",
            "decision_metric": decision_metric,
            "decision_value_deg": float(decision_value),
            "control_metric": control_metric,
        }
    return {
        "should_move": True,
        "reason": f"{decision_metric}_negative_turn_left_arc",
        "left_pwm": 0.0,
        "right_pwm": arc_pwm,
        "duration_s": arc_duration_s,
        "arc_mode": "left_arc_right_side_only",
        "decision_metric": decision_metric,
        "decision_value_deg": float(decision_value),
        "control_metric": control_metric,
    }


def print_sense(prefix: str, sense: dict) -> None:
    if not sense.get("matched_samples"):
        print(f"{prefix}: no matched tag samples")
        return

    off_axis = sense.get("off_axis_m_median")
    target_lateral_error = sense.get("target_lateral_error_m_median")
    target_range_error = sense.get("target_range_error_m_median")
    print(
        f"{prefix}: matched={sense['matched_samples']}/{sense['frames_processed']} "
        f"bearing={sense['tag_bearing_deg_median']:+.2f} deg "
        f"heading_vs_normal={sense['heading_vs_tag_normal_deg_median']:+.2f} deg "
        f"off_axis={off_axis:+.3f} m "
        f"centerline_skew={sense['centerline_skew_deg_median']:+.2f} deg "
        f"range={sense['range_m_median']:.3f} m "
        f"rmse={sense['reprojection_rmse_px_median']:.3f}px"
    )
    if target_lateral_error is not None or target_range_error is not None:
        lateral_text = (
            f"{target_lateral_error:+.3f} m"
            if target_lateral_error is not None
            else "n/a"
        )
        range_text = (
            f"{target_range_error:+.3f} m"
            if target_range_error is not None
            else "n/a"
        )
        print(
            f"{prefix}_vs_target: lateral={lateral_text} "
            f"range={range_text}"
        )


def translation_delta(a: list[float] | None, b: list[float] | None) -> list[float] | None:
    if a is None or b is None:
        return None
    return [float(b[i] - a[i]) for i in range(3)]


def run_arc_step(
    rover: RoverController,
    left_pwm: float,
    right_pwm: float,
    duration_s: float,
    command_period_s: float,
) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        rover.send_lr(left_pwm, right_pwm)
        time.sleep(command_period_s)
    rover.stop()


def main() -> int:
    args = parse_args()

    calibration = load_camera_calibration(args.model)
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
    camera_matrix = scaled_camera_matrix(
        calibration.camera_matrix,
        calibration.image_size,
        actual_size,
    )
    dist_coeffs = calibration.dist_coeffs.copy()
    object_points = build_tag_object_points(args.tag_size)
    target_pose = read_target_pose(args.target_pose)
    target_camera_in_tag = target_camera_in_tag_frame(target_pose)
    camera_controls = read_camera_controls(args.camera)

    run_dir = args.output_root / (
        datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{args.name}"
    )
    run_dir.mkdir(parents=True, exist_ok=True)

    try:
        for _ in range(max(args.warmup_frames, 0)):
            cap.read()

        pre = capture_burst(
            cap=cap,
            detector=detector,
            expected_id=args.id,
            object_points=object_points,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            samples_needed=args.samples,
            max_frames=args.max_frames,
            target_camera_in_tag=target_camera_in_tag,
        )
        print_sense("pre", pre)

        motion = choose_arc_step(
            sense=pre,
            bearing_deadband_deg=args.bearing_deadband_deg,
            heading_deadband_deg=args.heading_deadband_deg,
            control_metric=args.control_metric,
            arc_pwm=args.arc_pwm,
            arc_duration_s=args.arc_duration,
        )
        print(
            "step: "
            f"mode={motion['arc_mode']} "
            f"execute={int(args.execute and motion['should_move'])} "
            f"reason={motion['reason']} "
            f"metric={motion.get('decision_metric')} "
            f"value={motion.get('decision_value_deg')} "
            f"L={motion['left_pwm']:+.3f} "
            f"R={motion['right_pwm']:+.3f} "
            f"duration={motion['duration_s']:.3f}s"
        )

        if args.execute and motion["should_move"]:
            with RoverController(port=args.port, baud=args.baud, timeout=args.timeout) as rover:
                run_arc_step(
                    rover=rover,
                    left_pwm=motion["left_pwm"],
                    right_pwm=motion["right_pwm"],
                    duration_s=motion["duration_s"],
                    command_period_s=args.command_period,
                )
        if args.execute and motion["should_move"] and args.settle_seconds > 0:
            time.sleep(args.settle_seconds)

        post = capture_burst(
            cap=cap,
            detector=detector,
            expected_id=args.id,
            object_points=object_points,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            samples_needed=args.samples,
            max_frames=args.max_frames,
            target_camera_in_tag=target_camera_in_tag,
        )
        print_sense("post", post)
    finally:
        cap.release()

    delta = {
        "tag_tvec_m": translation_delta(pre.get("tag_tvec_m_median"), post.get("tag_tvec_m_median")),
        "camera_in_tag_m": translation_delta(
            pre.get("camera_in_tag_m_median"),
            post.get("camera_in_tag_m_median"),
        ),
        "off_axis_m": (
            None
            if pre.get("off_axis_m_median") is None or post.get("off_axis_m_median") is None
            else float(post["off_axis_m_median"] - pre["off_axis_m_median"])
        ),
        "tag_bearing_deg": (
            None
            if pre.get("tag_bearing_deg_median") is None or post.get("tag_bearing_deg_median") is None
            else float(post["tag_bearing_deg_median"] - pre["tag_bearing_deg_median"])
        ),
        "heading_vs_tag_normal_deg": (
            None
            if pre.get("heading_vs_tag_normal_deg_median") is None
            or post.get("heading_vs_tag_normal_deg_median") is None
            else float(
                post["heading_vs_tag_normal_deg_median"]
                - pre["heading_vs_tag_normal_deg_median"]
            )
        ),
    }

    payload = {
        "schema_version": "wave_rover.dock_heading_step/v1",
        "captured_at": now_utc(),
        "camera": {
            "device": args.camera,
            "actual": actual,
            "startup_controls": startup_controls,
            "controls": camera_controls,
            "model": str(calibration.model_path),
        },
        "tag": {
            "family": args.family,
            "id": args.id,
            "size_m": args.tag_size,
        },
        "target_pose_reference": {
            "path": str(args.target_pose),
            "camera_in_tag": target_camera_in_tag,
        },
        "step_config": {
            "execute": bool(args.execute),
            "arc_pwm": args.arc_pwm,
            "arc_duration_s": args.arc_duration,
            "command_period_s": args.command_period,
            "bearing_deadband_deg": args.bearing_deadband_deg,
            "heading_deadband_deg": args.heading_deadband_deg,
            "control_metric": args.control_metric,
            "settle_seconds": args.settle_seconds,
            "samples": args.samples,
            "max_frames": args.max_frames,
        },
        "motion_decision": motion,
        "pre": pre,
        "post": post,
        "delta": delta,
    }
    (run_dir / "session.json").write_text(json.dumps(payload, indent=2) + "\n")
    print(f"saved={run_dir}")
    if delta["tag_bearing_deg"] is not None or delta["off_axis_m"] is not None:
        bearing_text = (
            f"{delta['tag_bearing_deg']:+.2f} deg"
            if delta["tag_bearing_deg"] is not None
            else "n/a"
        )
        off_axis_text = (
            f"{delta['off_axis_m']:+.3f} m"
            if delta["off_axis_m"] is not None
            else "n/a"
        )
        heading_text = (
            f"{delta['heading_vs_tag_normal_deg']:+.2f} deg"
            if delta["heading_vs_tag_normal_deg"] is not None
            else "n/a"
        )
        print(
            "delta: "
            f"bearing={bearing_text} "
            f"off_axis={off_axis_text} "
            f"heading_vs_normal={heading_text}"
        )
    return 0 if pre.get("matched_samples") else 2


if __name__ == "__main__":
    raise SystemExit(main())
