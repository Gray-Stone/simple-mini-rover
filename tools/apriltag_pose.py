#!/usr/bin/env python3
import argparse
import json
import math
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from http import server
from pathlib import Path
from urllib.parse import urlparse

import cv2
import mrcal
import numpy as np


APRILTAG_FAMILIES = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

DEFAULT_MODEL_ROOT = Path("data/camera_calibration/captures")
stop_requested = False


@dataclass
class CameraCalibration:
    model_path: Path
    lensmodel: str
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    image_size: tuple[int, int]


class PreviewState:
    def __init__(self):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.jpeg_bytes = None
        self.status = {
            "ok": True,
            "updated_at": None,
            "frame": None,
            "matched_expected": False,
            "pose": None,
            "detections": [],
            "camera_controls": {},
            "message": "waiting for frames",
        }
        self.frame_seq = 0

    def update(self, jpeg_bytes: bytes | None, status: dict):
        with self.condition:
            self.jpeg_bytes = jpeg_bytes
            self.status = status
            self.frame_seq += 1
            self.condition.notify_all()

    def snapshot(self):
        with self.lock:
            return self.frame_seq, self.jpeg_bytes, dict(self.status)

    def wait_for_frame(self, last_seq: int, timeout_s: float = 5.0):
        with self.condition:
            if self.frame_seq == last_seq:
                self.condition.wait(timeout=timeout_s)
            return self.frame_seq, self.jpeg_bytes, dict(self.status)


def request_stop(signum, frame):
    global stop_requested
    stop_requested = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate AprilTag pose from the live camera using a saved mrcal model."
    )
    parser.add_argument("--camera", default="/dev/video0", help="Camera device or index.")
    parser.add_argument(
        "--model",
        type=Path,
        help="Path to an mrcal camera model. Defaults to the newest saved model under data/camera_calibration/captures/.",
    )
    parser.add_argument("--family", default="tag16h5", choices=sorted(APRILTAG_FAMILIES))
    parser.add_argument("--id", type=int, default=0, help="Expected tag ID.")
    parser.add_argument("--tag-size", type=float, default=0.034, help="Tag black-square size in meters.")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
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
        "--frames",
        type=int,
        default=120,
        help="Number of frames to inspect. Use 0 to run until interrupted.",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=15,
        help="Read and discard this many frames before pose estimation.",
    )
    parser.add_argument(
        "--jsonl",
        action="store_true",
        help="Print one compact JSON record per processed frame.",
    )
    parser.add_argument(
        "--save-debug",
        type=Path,
        help="Optional output image path for the last matched frame with overlays.",
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Serve a simple web preview with overlay frames and live pose status.",
    )
    parser.add_argument("--host", default="0.0.0.0", help="Bind host for --serve.")
    parser.add_argument("--port", type=int, default=8090, help="Bind port for --serve.")
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=80,
        help="JPEG quality for the web preview stream.",
    )
    parser.add_argument(
        "--auto-exposure",
        choices=("leave", "auto", "manual"),
        default="leave",
        help="Optional exposure mode override.",
    )
    parser.add_argument(
        "--exposure-time",
        "--exposure-time-absolute",
        dest="exposure_time",
        type=int,
        help="Optional manual exposure_time_absolute value.",
    )
    parser.add_argument(
        "--gain",
        type=int,
        help="Optional gain override.",
    )
    parser.add_argument(
        "--white-balance-auto",
        choices=("leave", "on", "off"),
        default="leave",
        help="Optional auto white balance override.",
    )
    parser.add_argument(
        "--white-balance-temperature",
        type=int,
        help="Optional white balance temperature override; forces auto white balance off.",
    )
    parser.add_argument(
        "--backlight-compensation",
        type=int,
        help="Optional backlight compensation override.",
    )
    parser.add_argument(
        "--contrast",
        type=int,
        help="Optional contrast override.",
    )
    parser.add_argument(
        "--low-light-preset",
        action="store_true",
        help="Apply a conservative indoor-night detection preset: manual exposure, modest gain, locked white balance, and backlight compensation off.",
    )
    return parser.parse_args()


def camera_source(value: str):
    video_path = re.fullmatch(r"/dev/video(\d+)", str(value))
    if video_path:
        return int(video_path.group(1))
    try:
        return int(value)
    except ValueError:
        return value


def resolve_default_model_path() -> Path:
    summary_paths = sorted(DEFAULT_MODEL_ROOT.glob("*/calibration/mrcal/summary.json"))
    if summary_paths:
        newest = max(summary_paths, key=lambda path: path.stat().st_mtime)
        data = json.loads(newest.read_text())
        model_path = Path(data["model"])
        if model_path.is_file():
            return model_path

    model_paths = sorted(DEFAULT_MODEL_ROOT.glob("*/calibration/mrcal/camera-0.cameramodel"))
    if model_paths:
        return max(model_paths, key=lambda path: path.stat().st_mtime)

    raise FileNotFoundError(
        "no saved mrcal camera model found under data/camera_calibration/captures/"
    )


def load_camera_calibration(model_path: Path | None) -> CameraCalibration:
    resolved = model_path if model_path is not None else resolve_default_model_path()
    resolved = resolved.resolve()
    model = mrcal.cameramodel(str(resolved))
    lensmodel, intrinsics = model.intrinsics()
    if lensmodel != "LENSMODEL_OPENCV5":
        raise RuntimeError(
            f"unsupported lens model {lensmodel!r}; expected LENSMODEL_OPENCV5 from current calibration flow"
        )

    fx, fy, cx, cy, k1, k2, p1, p2, k3 = [float(v) for v in intrinsics.tolist()]
    camera_matrix = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
    imagersize = model.imagersize()
    return CameraCalibration(
        model_path=resolved,
        lensmodel=lensmodel,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=(int(imagersize[0]), int(imagersize[1])),
    )


def open_camera(args: argparse.Namespace) -> cv2.VideoCapture:
    args.camera = resolve_camera_device(str(args.camera))
    cap = cv2.VideoCapture(camera_source(args.camera), cv2.CAP_V4L2)
    if not cap.isOpened():
        available = ", ".join(list_capture_devices())
        if available:
            raise RuntimeError(
                f"could not open camera {args.camera!r}; available capture devices: {available}"
            )
        raise RuntimeError(f"could not open camera {args.camera!r}; no capture devices detected")

    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if args.autofocus else 0)
    if args.focus_absolute is not None:
        cap.set(cv2.CAP_PROP_FOCUS, args.focus_absolute)
    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    return cap


def v4l2_device_arg(device: str) -> str:
    if device.isdigit():
        return f"/dev/video{device}"
    return device


def device_supports_video_capture(device: str) -> bool:
    device_path = v4l2_device_arg(str(device))
    if not Path(device_path).exists():
        return False
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device_path, "--all"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )
    except (subprocess.SubprocessError, FileNotFoundError):
        return True
    output = result.stdout
    return "Video input" in output and "Video Capture" in output


def list_capture_devices() -> list[str]:
    candidates: list[str] = []
    seen: set[str] = set()
    by_id_root = Path("/dev/v4l/by-id")
    if by_id_root.exists():
        for entry in sorted(by_id_root.iterdir()):
            try:
                resolved = str(entry.resolve())
            except FileNotFoundError:
                continue
            if resolved not in seen:
                seen.add(resolved)
                candidates.append(resolved)
    for entry in sorted(Path("/dev").glob("video*")):
        resolved = str(entry)
        if resolved not in seen:
            seen.add(resolved)
            candidates.append(resolved)
    return [device for device in candidates if device_supports_video_capture(device)]


def resolve_camera_device(device: str) -> str:
    requested = v4l2_device_arg(str(device))
    if device_supports_video_capture(requested):
        return requested

    candidates = list_capture_devices()
    if candidates:
        resolved = candidates[0]
        if resolved != requested:
            print(f"camera_device_resolved={requested}->{resolved}", file=sys.stderr)
        return resolved
    return requested


def run_v4l2(device: str, args: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["v4l2-ctl", "-d", v4l2_device_arg(str(device)), *args],
        check=True,
        capture_output=True,
        text=True,
        timeout=2,
    )


def parse_v4l2_control_values(output: str) -> dict:
    values = {}
    for line in output.splitlines():
        if ":" not in line:
            continue
        name, rest = line.split(":", 1)
        token = rest.strip().split(maxsplit=1)[0] if rest.strip() else ""
        try:
            values[name.strip()] = int(token)
        except ValueError:
            continue
    return values


def read_camera_controls(device: str) -> dict:
    names = ",".join(
        [
            "auto_exposure",
            "exposure_time_absolute",
            "exposure_dynamic_framerate",
            "gain",
            "white_balance_automatic",
            "white_balance_temperature",
            "backlight_compensation",
            "contrast",
            "focus_absolute",
            "focus_automatic_continuous",
        ]
    )
    try:
        result = run_v4l2(device, ["-C", names])
    except (subprocess.SubprocessError, FileNotFoundError):
        return {}
    return parse_v4l2_control_values(result.stdout)


def set_camera_control(device: str, name: str, value: int) -> dict:
    run_v4l2(device, ["-c", f"{name}={int(value)}"])
    return read_camera_controls(device)


def apply_camera_control_overrides(args: argparse.Namespace) -> dict:
    args.camera = resolve_camera_device(str(args.camera))
    desired = {}
    if args.low_light_preset:
        desired.update(
            {
                "auto_exposure": 1,
                "exposure_dynamic_framerate": 0,
                "exposure_time_absolute": 280,
                "gain": 8,
                "white_balance_automatic": 0,
                "white_balance_temperature": 4200,
                "backlight_compensation": 0,
                "contrast": 40,
            }
        )

    if args.auto_exposure == "auto":
        desired["auto_exposure"] = 3
    elif args.auto_exposure == "manual":
        desired["auto_exposure"] = 1
        desired.setdefault("exposure_dynamic_framerate", 0)

    if args.exposure_time is not None:
        desired["auto_exposure"] = 1
        desired["exposure_dynamic_framerate"] = 0
        desired["exposure_time_absolute"] = int(args.exposure_time)

    if args.gain is not None:
        desired["gain"] = int(args.gain)

    if args.white_balance_auto == "on":
        desired["white_balance_automatic"] = 1
    elif args.white_balance_auto == "off":
        desired["white_balance_automatic"] = 0

    if args.white_balance_temperature is not None:
        desired["white_balance_automatic"] = 0
        desired["white_balance_temperature"] = int(args.white_balance_temperature)

    if args.backlight_compensation is not None:
        desired["backlight_compensation"] = int(args.backlight_compensation)

    if args.contrast is not None:
        desired["contrast"] = int(args.contrast)

    if not desired:
        return read_camera_controls(args.camera)

    try:
        for name, value in desired.items():
            run_v4l2(args.camera, ["-c", f"{name}={int(value)}"])
    except (subprocess.SubprocessError, FileNotFoundError) as exc:
        print(f"camera_control_apply_failed={exc}", file=sys.stderr)

    return read_camera_controls(args.camera)


def make_detector(family: str) -> cv2.aruco.ArucoDetector:
    dictionary = cv2.aruco.getPredefinedDictionary(APRILTAG_FAMILIES[family])
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return cv2.aruco.ArucoDetector(dictionary, params)


def scaled_camera_matrix(
    camera_matrix: np.ndarray,
    from_size: tuple[int, int],
    to_size: tuple[int, int],
) -> np.ndarray:
    sx = to_size[0] / from_size[0]
    sy = to_size[1] / from_size[1]
    scaled = camera_matrix.copy()
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled


def build_tag_object_points(tag_size_m: float) -> np.ndarray:
    half = tag_size_m / 2.0
    return np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float64,
    )


def rotation_matrix_to_euler_xyz_deg(rotation: np.ndarray) -> list[float]:
    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(rotation[2, 1], rotation[2, 2])
        y = math.atan2(-rotation[2, 0], sy)
        z = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        x = math.atan2(-rotation[1, 2], rotation[1, 1])
        y = math.atan2(-rotation[2, 0], sy)
        z = 0.0

    return [math.degrees(x), math.degrees(y), math.degrees(z)]


def pose_matrix(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.reshape(3)
    return transform


def solve_best_pose(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float] | None:
    candidates = []
    flags_to_try = [
        cv2.SOLVEPNP_ITERATIVE,
        cv2.SOLVEPNP_SQPNP,
        cv2.SOLVEPNP_IPPE_SQUARE,
    ]

    for flag in flags_to_try:
        try:
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                dist_coeffs,
                flags=flag,
            )
        except cv2.error:
            continue
        if not ok:
            continue

        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs,
        )
        reprojection = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
        reprojection_rmse_px = float(np.sqrt(np.mean(np.sum(reprojection**2, axis=1))))
        if not np.isfinite(reprojection_rmse_px):
            continue

        translation = tvec.reshape(3)
        if translation[2] <= 0.0:
            continue

        rotation, _ = cv2.Rodrigues(rvec)
        candidates.append((rotation, rvec, tvec, reprojection_rmse_px))

    if not candidates:
        return None
    return min(candidates, key=lambda item: item[3])


def detect_pose(
    frame: np.ndarray,
    detector: cv2.aruco.ArucoDetector,
    expected_id: int,
    object_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> dict:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    detections = []
    matched = None
    matched_corners = None

    if ids is not None:
        for tag_id, tag_corners in zip(ids.flatten(), corners):
            pts = tag_corners.reshape(4, 2)
            center = pts.mean(axis=0)
            edge_lengths = []
            for i in range(4):
                p0 = pts[i]
                p1 = pts[(i + 1) % 4]
                edge_lengths.append(float(np.linalg.norm(p1 - p0)))
            record = {
                "id": int(tag_id),
                "center_px": [round(float(center[0]), 2), round(float(center[1]), 2)],
                "corners_px": [
                    [round(float(x), 2), round(float(y), 2)] for x, y in pts.tolist()
                ],
                "mean_edge_px": round(sum(edge_lengths) / len(edge_lengths), 2),
            }
            detections.append(record)
            if int(tag_id) == expected_id:
                matched = record
                matched_corners = pts.astype(np.float64)

    pose = None
    if matched_corners is not None:
        solved = solve_best_pose(
            object_points,
            matched_corners,
            camera_matrix,
            dist_coeffs,
        )
        if solved is not None:
            rotation, rvec, tvec, reprojection_rmse_px = solved
            translation = tvec.reshape(3)
            pose = {
                "tvec_m": [float(v) for v in translation.tolist()],
                "rvec_rad": [float(v) for v in rvec.reshape(3).tolist()],
                "rotation_matrix": [[float(v) for v in row] for row in rotation.tolist()],
                "camera_from_tag": [
                    [float(v) for v in row]
                    for row in pose_matrix(rotation, translation).tolist()
                ],
                "tag_from_camera": [
                    [float(v) for v in row]
                    for row in np.linalg.inv(pose_matrix(rotation, translation)).tolist()
                ],
                "euler_xyz_deg": [round(v, 3) for v in rotation_matrix_to_euler_xyz_deg(rotation)],
                "range_m": float(translation[2]),
                "lateral_m": float(translation[0]),
                "vertical_m": float(translation[1]),
                "normal_in_camera": [float(v) for v in rotation[:, 2].tolist()],
                "reprojection_rmse_px": reprojection_rmse_px,
            }

    return {
        "detections": detections,
        "matched": matched,
        "pose": pose,
        "rejected_count": len(rejected),
        "matched_corners": matched_corners,
        "all_corners": corners,
        "ids": ids,
    }


def overlay_pose(
    frame: np.ndarray,
    matched_corners: np.ndarray | None,
    pose: dict | None,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    tag_size_m: float,
) -> np.ndarray:
    debug = frame.copy()
    if matched_corners is not None:
        pts = matched_corners.reshape((-1, 1, 2)).astype(np.int32)
        cv2.polylines(debug, [pts], True, (0, 255, 0), 2, lineType=cv2.LINE_AA)

    if pose is not None:
        rvec = np.array(pose["rvec_rad"], dtype=np.float64).reshape(3, 1)
        tvec = np.array(pose["tvec_m"], dtype=np.float64).reshape(3, 1)
        axis = tag_size_m * 0.5
        cv2.drawFrameAxes(debug, camera_matrix, dist_coeffs, rvec, tvec, axis, 2)
        label = (
            f"x={pose['lateral_m']:.3f} y={pose['vertical_m']:.3f} "
            f"z={pose['range_m']:.3f} m"
        )
        cv2.putText(
            debug,
            label,
            (24, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (40, 220, 40),
            2,
            cv2.LINE_AA,
        )
    return debug


def encode_jpeg(frame: np.ndarray, quality: int) -> bytes | None:
    ok, buf = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(max(10, min(quality, 100)))],
    )
    if not ok:
        return None
    return bytes(buf)


def frame_record(
    frame_index: int,
    elapsed_s: float,
    matched: dict | None,
    pose: dict | None,
    detections: list[dict],
    rejected_count: int,
) -> dict:
    return {
        "frame": frame_index,
        "elapsed_s": round(elapsed_s, 3),
        "matched_expected": bool(matched),
        "detections": detections,
        "rejected_count": rejected_count,
        "pose": pose,
    }


def preview_html(port: int) -> bytes:
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AprilTag Pose Preview</title>
  <style>
    body {{ font-family: monospace; margin: 16px; background: #111; color: #eee; }}
    img {{ max-width: 100%; height: auto; border: 1px solid #444; }}
    pre {{ background: #1b1b1b; padding: 12px; overflow: auto; border: 1px solid #333; }}
    .wrap {{ display: grid; gap: 16px; }}
    .controls {{ background: #1b1b1b; border: 1px solid #333; padding: 12px; }}
    .row {{ display: flex; gap: 12px; align-items: center; flex-wrap: wrap; }}
    input[type="range"] {{ width: min(560px, 100%); }}
    .muted {{ color: #aaa; font-size: 13px; }}
  </style>
</head>
<body>
  <div class="wrap">
    <div>
      <img src="/stream.mjpg" alt="AprilTag preview">
    </div>
    <div class="controls">
      <div class="row">
        <label for="exposure">Exposure</label>
        <input id="exposure" type="range" min="1" max="5000" step="1" value="280">
        <span id="exposureValue">280</span>
      </div>
      <div class="muted" id="controlStatus">manual exposure slider</div>
    </div>
    <div>
      <pre id="status">loading</pre>
    </div>
  </div>
  <script>
    let pendingExposure = null;
    let exposureTimer = null;
    let sliderDirty = false;

    function syncExposureSlider(data) {{
      const slider = document.getElementById('exposure');
      const valueNode = document.getElementById('exposureValue');
      const controls = data.camera_controls || {{}};
      const exposure = controls.exposure_time_absolute;
      if (typeof exposure === 'number') {{
        if (!sliderDirty) {{
          slider.value = String(exposure);
        }}
        valueNode.textContent = String(exposure);
      }}
    }}

    async function pushExposure(value) {{
      const statusNode = document.getElementById('controlStatus');
      pendingExposure = value;
      statusNode.textContent = `setting exposure to ${{value}}`;
      try {{
        const r = await fetch('/control/exposure', {{
          method: 'POST',
          headers: {{'Content-Type': 'application/json'}},
          body: JSON.stringify({{exposure_time_absolute: value}}),
        }});
        const data = await r.json();
        if (!r.ok) {{
          throw new Error(data.error || `HTTP ${{r.status}}`);
        }}
        statusNode.textContent = `exposure set to ${{data.camera_controls?.exposure_time_absolute ?? value}}`;
      }} catch (err) {{
        statusNode.textContent = `exposure update failed: ${{String(err)}}`;
      }} finally {{
        pendingExposure = null;
        sliderDirty = false;
      }}
    }}

    function scheduleExposure(value) {{
      clearTimeout(exposureTimer);
      exposureTimer = setTimeout(() => pushExposure(value), 140);
    }}

    async function refresh() {{
      try {{
        const r = await fetch('/status.json', {{cache: 'no-store'}});
        const data = await r.json();
        syncExposureSlider(data);
        document.getElementById('status').textContent = JSON.stringify(data, null, 2);
      }} catch (err) {{
        document.getElementById('status').textContent = String(err);
      }}
    }}

    const slider = document.getElementById('exposure');
    slider.addEventListener('input', (event) => {{
      sliderDirty = true;
      const value = Number(event.target.value);
      document.getElementById('exposureValue').textContent = String(value);
      scheduleExposure(value);
    }});

    refresh();
    setInterval(refresh, 500);
  </script>
</body>
</html>
"""
    return html.encode("utf-8")


def make_preview_handler(state: PreviewState, camera_device: str):
    class PreviewHandler(server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self):
            self.connection.settimeout(0.5)
            parsed = urlparse(self.path)
            if parsed.path == "/":
                body = preview_html(self.server.server_port)
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if parsed.path == "/status.json":
                _, _, status = state.snapshot()
                body = (json.dumps(status, indent=2) + "\n").encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if parsed.path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-store, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                seq = -1
                try:
                    while not stop_requested:
                        seq, jpeg_bytes, _ = state.wait_for_frame(seq, timeout_s=5.0)
                        if not jpeg_bytes:
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg_bytes)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpeg_bytes)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass
                return

            self.send_error(404)

        def do_POST(self):
            self.connection.settimeout(0.5)
            parsed = urlparse(self.path)
            if parsed.path != "/control/exposure":
                self.send_error(404)
                return

            try:
                content_length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                content_length = 0
            raw = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                payload = json.loads(raw.decode("utf-8"))
                exposure = int(payload["exposure_time_absolute"])
                controls = set_camera_control(camera_device, "auto_exposure", 1)
                controls = set_camera_control(camera_device, "exposure_dynamic_framerate", 0)
                controls = set_camera_control(camera_device, "exposure_time_absolute", exposure)
                body = json.dumps(
                    {
                        "ok": True,
                        "camera_controls": controls,
                    },
                    indent=2,
                ).encode("utf-8")
                self.send_response(200)
            except (KeyError, ValueError, json.JSONDecodeError, subprocess.SubprocessError, FileNotFoundError) as exc:
                body = json.dumps(
                    {
                        "ok": False,
                        "error": str(exc),
                    },
                    indent=2,
                ).encode("utf-8")
                self.send_response(400)

            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args):
            return

    return PreviewHandler


def start_preview_server(host: str, port: int, state: PreviewState, camera_device: str):
    class PreviewHTTPServer(server.ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    httpd = PreviewHTTPServer((host, port), make_preview_handler(state, camera_device))
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

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
    preview_state = PreviewState() if args.serve else None
    preview_server = None
    if preview_state is not None:
        preview_server = start_preview_server(args.host, args.port, preview_state, args.camera)
        print(
            f"preview=http://{args.host if args.host != '0.0.0.0' else '127.0.0.1'}:{args.port}/",
            file=sys.stderr,
        )

    print(
        f"camera={args.camera} actual={actual['width']}x{actual['height']} "
        f"fps={actual['fps']:.1f} family={args.family} expected_id={args.id} "
        f"tag_size_m={args.tag_size} model={calibration.model_path} "
        f"model_size={calibration.image_size[0]}x{calibration.image_size[1]} "
        f"focus={actual['focus']:.0f} autofocus={int(actual['autofocus'])} "
        f"controls={startup_controls}",
        file=sys.stderr,
    )

    processed = 0
    matched_frames = 0
    last_match_state = None
    last_debug = None
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
            if not ok or frame is None:
                print("frame_read_failed", file=sys.stderr)
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
            elapsed = max(time.monotonic() - started, 1e-6)
            record = frame_record(
                frame_index=processed,
                elapsed_s=elapsed,
                matched=result["matched"],
                pose=result["pose"],
                detections=result["detections"],
                rejected_count=result["rejected_count"],
            )
            record["fps_avg"] = round((processed + 1) / elapsed, 2)

            if result["matched"] and result["pose"]:
                matched_frames += 1
                display_frame = overlay_pose(
                    frame,
                    result["matched_corners"],
                    result["pose"],
                    camera_matrix,
                    dist_coeffs,
                    args.tag_size,
                )
                last_debug = display_frame
            else:
                display_frame = frame.copy()

            if preview_state is not None:
                preview_status = dict(record)
                preview_status["updated_at"] = time.time()
                preview_status["camera_controls"] = read_camera_controls(args.camera)
                preview_status["message"] = (
                    "matched expected tag" if result["matched"] else "expected tag not matched"
                )
                preview_state.update(
                    encode_jpeg(display_frame, args.jpeg_quality),
                    preview_status,
                )

            if args.jsonl:
                print(json.dumps(record, separators=(",", ":")), flush=True)
            elif processed % 30 == 0 or bool(result["matched"]) != last_match_state:
                print(json.dumps(record, indent=2), flush=True)

            last_match_state = bool(result["matched"])
            processed += 1
    finally:
        cap.release()
        if preview_server is not None:
            preview_server.shutdown()
            preview_server.server_close()

    if args.save_debug and last_debug is not None:
        args.save_debug.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.save_debug), last_debug)
        print(f"saved_debug={args.save_debug}", file=sys.stderr)

    print(
        f"summary frames={processed} matched_expected_frames={matched_frames}",
        file=sys.stderr,
    )
    return 0 if matched_frames else 2


if __name__ == "__main__":
    raise SystemExit(main())
