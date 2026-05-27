#!/usr/bin/env python3
import argparse
import json
import signal
import sys
import time
from pathlib import Path

import cv2


APRILTAG_FAMILIES = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


stop_requested = False


def request_stop(signum, frame):
    global stop_requested
    stop_requested = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Probe camera AprilTag detection for the auto-docking dock tag."
    )
    parser.add_argument("--camera", default="/dev/video0", help="Camera device or index.")
    parser.add_argument("--family", default="tag16h5", choices=sorted(APRILTAG_FAMILIES))
    parser.add_argument("--id", type=int, default=0, help="Expected tag ID.")
    parser.add_argument(
        "--tag-size",
        type=float,
        default=0.034,
        help="Actual black outer tag square size in meters. Used for reporting only here.",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument(
        "--autofocus",
        action="store_true",
        help="Enable continuous autofocus. Default is to disable it for repeatable geometry.",
    )
    parser.add_argument(
        "--focus-absolute",
        type=int,
        default=350,
        help="Manual focus value for UVC cameras. Use 350 with the current saved calibration unless explicitly using a separately calibrated focus.",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=120,
        help="Number of frames to inspect. Use 0 to run until interrupted.",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=15,
        help="Read and discard this many camera frames before detection.",
    )
    parser.add_argument(
        "--save-debug",
        type=Path,
        help="Save one debug image with detected corners drawn.",
    )
    parser.add_argument(
        "--jsonl",
        action="store_true",
        help="Print one machine-readable JSON object per processed frame.",
    )
    return parser.parse_args()


def camera_source(value: str):
    try:
        return int(value)
    except ValueError:
        return value


def open_camera(args: argparse.Namespace) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(camera_source(args.camera), cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {args.camera!r}")

    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if args.autofocus else 0)
    if args.focus_absolute is not None:
        cap.set(cv2.CAP_PROP_FOCUS, args.focus_absolute)
    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    return cap


def make_detector(family: str) -> cv2.aruco.ArucoDetector:
    dictionary = cv2.aruco.getPredefinedDictionary(APRILTAG_FAMILIES[family])
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return cv2.aruco.ArucoDetector(dictionary, params)


def summarize_detection(tag_id: int, corners) -> dict:
    pts = corners.reshape(4, 2)
    cx = float(pts[:, 0].mean())
    cy = float(pts[:, 1].mean())
    edge_lengths = []
    for i in range(4):
        p0 = pts[i]
        p1 = pts[(i + 1) % 4]
        edge_lengths.append(float(((p1 - p0) ** 2).sum() ** 0.5))
    return {
        "id": int(tag_id),
        "center_px": [round(cx, 2), round(cy, 2)],
        "corners_px": [[round(float(x), 2), round(float(y), 2)] for x, y in pts],
        "mean_edge_px": round(sum(edge_lengths) / len(edge_lengths), 2),
    }


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    detector = make_detector(args.family)
    cap = open_camera(args)
    actual = {
        "width": cap.get(cv2.CAP_PROP_FRAME_WIDTH),
        "height": cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
        "fps": cap.get(cv2.CAP_PROP_FPS),
        "fourcc": int(cap.get(cv2.CAP_PROP_FOURCC)),
    }

    print(
        f"camera={args.camera} actual={actual['width']:.0f}x{actual['height']:.0f}"
        f" fps={actual['fps']:.1f} family={args.family} expected_id={args.id}"
        f" tag_size_m={args.tag_size} autofocus={args.autofocus}"
        f" focus={cap.get(cv2.CAP_PROP_FOCUS):.0f}",
        file=sys.stderr,
    )

    processed = 0
    detected_frames = 0
    last_matched_state = None
    last_debug_frame = None
    started = time.monotonic()

    try:
        for _ in range(max(args.warmup_frames, 0)):
            if stop_requested:
                break
            cap.read()

        while not stop_requested:
            if args.frames and processed >= args.frames:
                break

            ok, frame = cap.read()
            now = time.monotonic()
            if not ok or frame is None:
                print("frame_read_failed", file=sys.stderr)
                time.sleep(0.05)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)
            detections = []
            if ids is not None:
                for tag_id, tag_corners in zip(ids.flatten(), corners):
                    detections.append(summarize_detection(int(tag_id), tag_corners))

            matched = [d for d in detections if d["id"] == args.id]
            if matched:
                detected_frames += 1
                last_debug_frame = frame.copy()
                cv2.aruco.drawDetectedMarkers(last_debug_frame, corners, ids)

            elapsed = max(now - started, 1e-6)
            record = {
                "frame": processed,
                "elapsed_s": round(elapsed, 3),
                "fps_avg": round((processed + 1) / elapsed, 2),
                "detections": detections,
                "matched_expected": bool(matched),
                "rejected_count": len(rejected),
            }

            if args.jsonl:
                print(json.dumps(record, separators=(",", ":")), flush=True)
            elif processed % 30 == 0 or bool(matched) != last_matched_state:
                print(json.dumps(record, indent=2), flush=True)
            last_matched_state = bool(matched)

            processed += 1
    finally:
        cap.release()

    if args.save_debug and last_debug_frame is not None:
        args.save_debug.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.save_debug), last_debug_frame)
        print(f"saved_debug={args.save_debug}", file=sys.stderr)

    print(
        f"summary frames={processed} detected_expected_frames={detected_frames}",
        file=sys.stderr,
    )
    return 0 if detected_frames else 2


if __name__ == "__main__":
    raise SystemExit(main())
