#!/usr/bin/env python3
import argparse
import json
import math
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template_string, request as flask_request, send_from_directory
from werkzeug.serving import make_server


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Browser preview and high-resolution image capture for camera calibration."
    )
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--output-root", type=Path, default=Path("data/camera_calibration/captures"))
    parser.add_argument("--session", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--preview-width", type=int, default=640)
    parser.add_argument("--preview-height", type=int, default=360)
    parser.add_argument("--stream-width", type=int, default=640)
    parser.add_argument("--stream-height", type=int, default=360)
    parser.add_argument("--capture-width", type=int, default=1920)
    parser.add_argument("--capture-height", type=int, default=1080)
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Requested camera FPS. Default matches the camera's advertised 1920x1080 MJPG mode.",
    )
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument(
        "--autofocus",
        action="store_true",
        help="Enable continuous autofocus. Default is to disable it for repeatable calibration geometry.",
    )
    parser.add_argument(
        "--focus-absolute",
        type=int,
        help="Optional manual focus value for UVC cameras, e.g. 432 on the current Arducam.",
    )
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--checkerboard-cols", type=int, default=8)
    parser.add_argument("--checkerboard-rows", type=int, default=6)
    parser.add_argument(
        "--score-interval",
        type=float,
        default=0.20,
        help="Seconds between preview quality-score updates. Default is 5 Hz while the MJPEG stream can run at preview FPS.",
    )
    parser.add_argument("--focus-min", type=int, default=0)
    parser.add_argument("--focus-max", type=int, default=1023)
    parser.add_argument("--focus-step", type=int, default=1)
    parser.add_argument(
        "--analysis-max-edge",
        type=int,
        default=1280,
        help="Maximum image edge used for scoring/detection. Saved captures remain full resolution.",
    )
    return parser.parse_args()


CAMERA_CONTROL_SPECS = [
    {"name": "focus_automatic_continuous", "label": "Autofocus", "type": "bool"},
    {"name": "focus_absolute", "label": "Focus", "type": "range", "min": 1, "max": 1023, "step": 1},
    {"name": "auto_exposure", "label": "Exposure mode", "type": "select", "options": [
        {"value": 3, "label": "Auto"},
        {"value": 1, "label": "Manual"},
    ]},
    {"name": "exposure_time_absolute", "label": "Exposure time", "type": "range", "min": 1, "max": 5000, "step": 1},
    {"name": "exposure_dynamic_framerate", "label": "Dynamic framerate", "type": "bool"},
    {"name": "gain", "label": "Gain", "type": "range", "min": 0, "max": 100, "step": 1},
    {"name": "brightness", "label": "Brightness", "type": "range", "min": -64, "max": 64, "step": 1},
    {"name": "contrast", "label": "Contrast", "type": "range", "min": 0, "max": 64, "step": 1},
    {"name": "saturation", "label": "Saturation", "type": "range", "min": 0, "max": 128, "step": 1},
    {"name": "gamma", "label": "Gamma", "type": "range", "min": 72, "max": 500, "step": 1},
    {"name": "sharpness", "label": "Sharpness", "type": "range", "min": 0, "max": 6, "step": 1},
    {"name": "backlight_compensation", "label": "Backlight comp", "type": "range", "min": 0, "max": 2, "step": 1},
    {"name": "white_balance_automatic", "label": "Auto white balance", "type": "bool"},
    {"name": "white_balance_temperature", "label": "White balance temp", "type": "range", "min": 2800, "max": 6500, "step": 1},
    {"name": "power_line_frequency", "label": "Power line", "type": "select", "options": [
        {"value": 0, "label": "Disabled"},
        {"value": 1, "label": "50 Hz"},
        {"value": 2, "label": "60 Hz"},
    ]},
]

CAMERA_CONTROL_BY_NAME = {spec["name"]: spec for spec in CAMERA_CONTROL_SPECS}


def camera_source(value: str):
    try:
        return int(value)
    except ValueError:
        return value


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


def v4l2_device_arg(device: str) -> str:
    if device.isdigit():
        return f"/dev/video{device}"
    return device


def bool_control_value(value) -> int:
    if isinstance(value, str):
        return 0 if value.strip().lower() in {"0", "false", "off", "no", ""} else 1
    return 1 if bool(value) else 0


def open_camera(
    device: str,
    width: int,
    height: int,
    fps: float,
    fourcc: str,
    autofocus: bool,
    focus_absolute: int | None,
) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(camera_source(device), cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {device!r}")
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofocus else 0)
    if focus_absolute is not None:
        cap.set(cv2.CAP_PROP_FOCUS, focus_absolute)
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def focus_category(focus_absolute: int | None) -> str:
    if focus_absolute is None:
        return "focus_unknown"
    return f"focus_{int(focus_absolute):04d}"


def detect_checkerboard(gray, pattern):
    if hasattr(cv2, "findChessboardCornersSB"):
        return cv2.findChessboardCornersSB(gray, pattern)

    found, corners = cv2.findChessboardCorners(gray, pattern)
    if found:
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return found, corners


def resize_for_analysis(frame, max_edge: int | None):
    if not max_edge or max_edge <= 0:
        return frame, 1.0

    height, width = frame.shape[:2]
    longest = max(width, height)
    if longest <= max_edge:
        return frame, 1.0

    scale = max_edge / float(longest)
    resized = cv2.resize(frame, (int(width * scale), int(height * scale)), interpolation=cv2.INTER_AREA)
    return resized, scale


def checkerboard_geometry(corners, pattern: tuple[int, int], width: int, height: int) -> dict:
    cols, _rows = pattern
    pts = corners.reshape(-1, 2).astype(np.float32)
    quad = np.array([pts[0], pts[cols - 1], pts[-1], pts[-cols]], dtype=np.float32)
    area = float(abs(cv2.contourArea(quad)))
    area_ratio = area / float(width * height)
    center = quad.mean(axis=0)

    top = float(np.linalg.norm(quad[1] - quad[0]))
    right = float(np.linalg.norm(quad[2] - quad[1]))
    bottom = float(np.linalg.norm(quad[2] - quad[3]))
    left = float(np.linalg.norm(quad[3] - quad[0]))
    top_bottom_ratio = min(top, bottom) / max(top, bottom) if max(top, bottom) else 0.0
    left_right_ratio = min(left, right) / max(left, right) if max(left, right) else 0.0
    tilt_strength = 1.0 - min(top_bottom_ratio, left_right_ratio)
    angle_degrees = math.degrees(math.atan2(float(quad[1][1] - quad[0][1]), float(quad[1][0] - quad[0][0])))

    min_x = float(np.min(quad[:, 0]))
    max_x = float(np.max(quad[:, 0]))
    min_y = float(np.min(quad[:, 1]))
    max_y = float(np.max(quad[:, 1]))
    margin = min(min_x, min_y, width - max_x, height - max_y) / float(min(width, height))

    return {
        "board_area_ratio": area_ratio,
        "board_center": [float(center[0] / width), float(center[1] / height)],
        "board_quad": [[float(x / width), float(y / height)] for x, y in quad],
        "board_margin_ratio": margin,
        "board_angle_degrees": angle_degrees,
        "board_tilt_strength": tilt_strength,
    }


def board_location_label(metrics: dict) -> str:
    center = metrics.get("board_center")
    if not center:
        return "not_detected"

    x, y = center
    horizontal = "left" if x < 0.42 else "right" if x > 0.58 else "center_x"
    vertical = "up" if y < 0.42 else "down" if y > 0.58 else "center_y"
    if horizontal == "center_x" and vertical == "center_y":
        return "center"
    if horizontal == "center_x":
        return vertical
    if vertical == "center_y":
        return horizontal
    return f"{vertical}_{horizontal}"


def board_tilt_label(metrics: dict) -> str:
    if not metrics.get("checkerboard_found"):
        return "not_detected"

    tilt = float(metrics.get("board_tilt_strength", 0.0))
    if tilt < 0.08:
        return "low_tilt"
    if tilt < 0.25:
        return "medium_tilt"
    return "high_tilt"


def board_rotation_label(metrics: dict) -> str:
    if not metrics.get("checkerboard_found"):
        return "not_detected"

    angle = abs(float(metrics.get("board_angle_degrees", 0.0)))
    angle = min(angle, abs(180.0 - angle))
    if angle < 8.0:
        return "level"
    if angle < 25.0:
        return "rotated"
    return "steep_rotation"


def novelty_score(metrics: dict, records: list[dict], category: str) -> tuple[float, str | None]:
    if not metrics.get("checkerboard_found"):
        return 0.0, None

    current = metrics.get("view_feature")
    if current is None:
        return 0.0, None

    nearest_distance = None
    nearest_image = None
    for record in records:
        if record.get("category") != category:
            continue
        saved_metrics = record.get("metrics") or {}
        if not saved_metrics.get("checkerboard_found"):
            continue
        saved = saved_metrics.get("view_feature")
        if saved is None:
            continue
        distance = math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(current, saved)))
        if nearest_distance is None or distance < nearest_distance:
            nearest_distance = distance
            nearest_image = record.get("image")

    if nearest_distance is None:
        return 1.0, None
    return clamp(nearest_distance / 0.35), nearest_image


class CameraState:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.session = args.output_root / args.session
        self.images_dir = self.session / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.session / "manifest.jsonl"
        self.session_info_path = self.session / "session.json"
        self.lock = threading.RLock()
        self.capture_lock = threading.Lock()
        self.running = True
        self.cap = None
        self.latest_frame = None
        self.latest_jpeg = None
        self.latest_metrics = {}
        self.latest_status = "starting"
        self.camera_actual = {}
        self.camera_controls = {}
        self.camera_controls_error = None
        self.last_score_at = 0.0
        self.current_focus_absolute = args.focus_absolute
        self.autofocus_enabled = args.autofocus
        self.records = self.load_manifest()
        self.capture_count = max((int(record.get("index", 0)) for record in self.records), default=0)
        self.thread = threading.Thread(target=self.preview_loop, daemon=True)
        self.apply_startup_controls()
        self.write_session_info()

    def load_manifest(self) -> list[dict]:
        if not self.manifest_path.exists():
            return []

        records = []
        with self.manifest_path.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
        return records

    def write_session_info(self) -> None:
        now = datetime.now().isoformat(timespec="seconds")
        existing = {}
        if self.session_info_path.exists():
            try:
                existing = json.loads(self.session_info_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                existing = {}
        data = {
            "camera": self.args.camera,
            "preview_size": [self.args.preview_width, self.args.preview_height],
            "stream_size": [self.args.stream_width, self.args.stream_height],
            "capture_size": [self.args.capture_width, self.args.capture_height],
            "fps": self.args.fps,
            "analysis_max_edge": self.args.analysis_max_edge,
            "checkerboard_inner_corners": [self.args.checkerboard_cols, self.args.checkerboard_rows],
            "score_interval": self.args.score_interval,
            "autofocus": self.autofocus_enabled,
            "initial_focus_absolute": self.args.focus_absolute,
            "current_focus_absolute": self.current_focus_absolute,
            "focus_range": [self.args.focus_min, self.args.focus_max, self.args.focus_step],
            "camera_controls": self.read_camera_control_values(),
            "created": existing.get("created", now),
            "last_opened": now,
            "records_loaded": len(self.records),
        }
        self.session_info_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        self.thread.join(timeout=2)
        with self.lock:
            if self.cap is not None:
                self.cap.release()
                self.cap = None

    def open_preview(self) -> None:
        self.apply_startup_controls()
        self.cap = open_camera(
            self.args.camera,
            self.args.capture_width,
            self.args.capture_height,
            self.args.fps,
            self.args.fourcc,
            self.autofocus_enabled,
            self.current_focus_absolute,
        )
        self.apply_startup_controls()
        self.camera_actual = self.read_camera_actual()
        for _ in range(max(0, self.args.warmup_frames)):
            self.cap.read()

    def read_camera_actual(self) -> dict:
        if self.cap is None:
            return {}
        return {
            "width": self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),
            "height": self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
            "fps": self.cap.get(cv2.CAP_PROP_FPS),
            "focus": self.cap.get(cv2.CAP_PROP_FOCUS),
        }

    def run_v4l2(self, args: list[str]) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["v4l2-ctl", "-d", v4l2_device_arg(str(self.args.camera)), *args],
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )

    def apply_startup_controls(self) -> None:
        try:
            self.run_v4l2(["-c", f"focus_automatic_continuous={1 if self.autofocus_enabled else 0}"])
            if not self.autofocus_enabled and self.current_focus_absolute is not None:
                self.run_v4l2(["-c", f"focus_absolute={int(self.current_focus_absolute)}"])
            values = self.read_camera_control_values()
        except (subprocess.SubprocessError, FileNotFoundError):
            return

        if "focus_absolute" in values and self.current_focus_absolute is None:
            self.current_focus_absolute = int(values["focus_absolute"])
        if "focus_automatic_continuous" in values:
            self.autofocus_enabled = bool(values["focus_automatic_continuous"])

    def read_camera_control_values(self) -> dict:
        names = ",".join(spec["name"] for spec in CAMERA_CONTROL_SPECS)
        try:
            result = self.run_v4l2(["-C", names])
            values = parse_v4l2_control_values(result.stdout)
        except (subprocess.SubprocessError, FileNotFoundError) as exc:
            self.camera_controls_error = str(exc)
            return dict(self.camera_controls)

        self.camera_controls_error = None
        self.camera_controls = values
        if "focus_absolute" in values:
            self.current_focus_absolute = int(values["focus_absolute"])
        if "focus_automatic_continuous" in values:
            self.autofocus_enabled = bool(values["focus_automatic_continuous"])
        return dict(values)

    def camera_control_state(self) -> dict:
        values = self.read_camera_control_values()
        controls = []
        for spec in CAMERA_CONTROL_SPECS:
            item = dict(spec)
            item["value"] = values.get(spec["name"])
            item["available"] = spec["name"] in values
            controls.append(item)
        return {
            "controls": controls,
            "values": values,
            "error": self.camera_controls_error,
        }

    def set_camera_control(self, name: str, value) -> dict:
        spec = CAMERA_CONTROL_BY_NAME.get(name)
        if spec is None:
            raise ValueError(f"unknown camera control {name!r}")

        if spec["type"] == "bool":
            normalized = bool_control_value(value)
        else:
            normalized = int(value)

        if spec["type"] == "range":
            normalized = int(max(spec["min"], min(spec["max"], normalized)))
        elif spec["type"] == "select":
            allowed = {int(option["value"]) for option in spec["options"]}
            if normalized not in allowed:
                raise ValueError(f"{name} must be one of {sorted(allowed)}")

        if name == "focus_absolute":
            self.run_v4l2(["-c", "focus_automatic_continuous=0"])
        elif name == "exposure_time_absolute":
            self.run_v4l2(["-c", "auto_exposure=1"])
            self.run_v4l2(["-c", "exposure_dynamic_framerate=0"])
        elif name == "auto_exposure" and normalized == 1:
            self.run_v4l2(["-c", "exposure_dynamic_framerate=0"])
        elif name == "white_balance_temperature":
            self.run_v4l2(["-c", "white_balance_automatic=0"])

        self.run_v4l2(["-c", f"{name}={normalized}"])
        with self.lock:
            if name == "focus_absolute":
                self.autofocus_enabled = False
                self.current_focus_absolute = normalized
                self.latest_metrics = {}
                self.last_score_at = 0.0
                if self.cap is not None:
                    self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                    self.cap.set(cv2.CAP_PROP_FOCUS, normalized)
            elif name == "focus_automatic_continuous":
                self.autofocus_enabled = bool(normalized)
                if self.cap is not None:
                    self.cap.set(cv2.CAP_PROP_AUTOFOCUS, normalized)
            self.camera_actual = self.read_camera_actual()
        values = self.read_camera_control_values()
        return {
            "name": name,
            "value": values.get(name, normalized),
            "controls": self.camera_control_state(),
        }

    def set_focus(self, value: int) -> dict:
        value = int(max(self.args.focus_min, min(self.args.focus_max, value)))
        with self.lock:
            self.autofocus_enabled = False
            self.current_focus_absolute = value
            self.latest_metrics = {}
            self.last_score_at = 0.0
            try:
                self.run_v4l2(["-c", "focus_automatic_continuous=0"])
                self.run_v4l2(["-c", f"focus_absolute={value}"])
            except (subprocess.SubprocessError, FileNotFoundError):
                pass
            if self.cap is not None:
                self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                self.cap.set(cv2.CAP_PROP_FOCUS, value)
                self.camera_actual = self.read_camera_actual()
            actual = self.actual_focus_locked()
        self.write_session_info()
        return actual

    def actual_focus_locked(self) -> dict:
        actual = None
        if self.cap is not None:
            actual_value = self.cap.get(cv2.CAP_PROP_FOCUS)
            if actual_value >= 0:
                actual = actual_value
        return {
            "autofocus": self.autofocus_enabled,
            "focus_absolute": self.current_focus_absolute,
            "camera_reported_focus": actual,
            "min": self.args.focus_min,
            "max": self.args.focus_max,
            "step": self.args.focus_step,
        }

    def focus_state(self) -> dict:
        with self.lock:
            return self.actual_focus_locked()

    def preview_loop(self) -> None:
        while self.running:
            try:
                with self.lock:
                    if self.cap is None:
                        self.open_preview()
                    ok, frame = self.cap.read()
                if not ok or frame is None:
                    self.latest_status = "preview read failed"
                    time.sleep(0.1)
                    continue

                preview_frame = cv2.resize(
                    frame,
                    (self.args.preview_width, self.args.preview_height),
                    interpolation=cv2.INTER_AREA,
                )
                debug, metrics = self.draw_feedback(preview_frame)
                preview = cv2.resize(
                    debug,
                    (self.args.stream_width, self.args.stream_height),
                    interpolation=cv2.INTER_AREA,
                )
                ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    score = metrics.get("score", 0)
                    found = metrics.get("checkerboard_found", False)
                    status = f"score={score:.0f}; checkerboard {'found' if found else 'not found'}"
                    with self.lock:
                        self.latest_frame = frame
                        self.latest_jpeg = encoded.tobytes()
                        self.latest_metrics = metrics
                        self.latest_status = status
            except Exception as exc:
                self.latest_status = f"preview error: {exc}"
                time.sleep(0.5)

    def compute_metrics(
        self,
        frame,
        category: str,
        records: list[dict],
        analysis_max_edge: int | None = None,
    ) -> tuple[dict, object | None]:
        analysis_frame, analysis_scale = resize_for_analysis(frame, analysis_max_edge)
        gray = cv2.cvtColor(analysis_frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape[:2]
        source_height, source_width = frame.shape[:2]
        pattern = (self.args.checkerboard_cols, self.args.checkerboard_rows)
        found = False
        corners = None
        if self.args.checkerboard_cols and self.args.checkerboard_rows:
            found, corners = detect_checkerboard(gray, pattern)

        laplacian_variance = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        mean_brightness = float(np.mean(gray))
        dark_clip_ratio = float(np.mean(gray <= 15))
        bright_clip_ratio = float(np.mean(gray >= 240))

        sharpness_score = clamp(laplacian_variance / 250.0)
        exposure_score = clamp(1.0 - abs(mean_brightness - 128.0) / 105.0 - max(dark_clip_ratio, bright_clip_ratio) * 2.0)
        geometry = {}
        if found and corners is not None:
            geometry = checkerboard_geometry(corners, pattern, width, height)
            area_ratio = geometry["board_area_ratio"]
            if area_ratio < 0.20:
                size_score = clamp(area_ratio / 0.20)
            else:
                size_score = clamp((0.80 - area_ratio) / 0.45)
            location_score = clamp(geometry["board_margin_ratio"] / 0.06)
            tilt_strength = geometry["board_tilt_strength"]
            tilt_score = 0.65 + 0.35 * clamp(tilt_strength / 0.25)
            if tilt_strength > 0.60:
                tilt_score *= clamp((0.85 - tilt_strength) / 0.25)
            view_feature = [
                geometry["board_center"][0],
                geometry["board_center"][1],
                geometry["board_area_ratio"] * 2.0,
                geometry["board_tilt_strength"],
                geometry["board_angle_degrees"] / 180.0,
            ]
        else:
            size_score = 0.0
            location_score = 0.0
            tilt_score = 0.0
            view_feature = None

        metrics = {
            "checkerboard_found": bool(found),
            "source_size": [int(source_width), int(source_height)],
            "analysis_size": [int(width), int(height)],
            "analysis_scale": float(analysis_scale),
            "laplacian_variance": laplacian_variance,
            "mean_brightness": mean_brightness,
            "dark_clip_ratio": dark_clip_ratio,
            "bright_clip_ratio": bright_clip_ratio,
            "sharpness_score": sharpness_score,
            "exposure_score": exposure_score,
            "size_score": size_score,
            "location_score": location_score,
            "tilt_score": tilt_score,
            "view_feature": view_feature,
            **geometry,
        }
        novelty, nearest_image = novelty_score(metrics, records, category)
        metrics["novelty_score"] = novelty
        metrics["nearest_image"] = nearest_image

        detection_score = 1.0 if found else 0.0
        score = (
            detection_score * 25.0
            + sharpness_score * 15.0
            + exposure_score * 15.0
            + size_score * 15.0
            + location_score * 10.0
            + tilt_score * 10.0
            + novelty * 10.0
        )
        metrics["score"] = round(score, 1)
        metrics["components"] = {
            "detection": round(detection_score * 25.0, 1),
            "sharpness": round(sharpness_score * 15.0, 1),
            "exposure": round(exposure_score * 15.0, 1),
            "size": round(size_score * 15.0, 1),
            "location": round(location_score * 10.0, 1),
            "tilt": round(tilt_score * 10.0, 1),
            "novelty": round(novelty * 10.0, 1),
        }
        metrics["pose_category"] = {
            "location": board_location_label(metrics),
            "tilt": board_tilt_label(metrics),
            "rotation": board_rotation_label(metrics),
        }
        return metrics, corners

    def draw_feedback(self, frame):
        debug = frame
        category = focus_category(self.current_focus_absolute)
        now = time.monotonic()
        should_score = (
            not self.latest_metrics
            or self.args.score_interval <= 0
            or now - self.last_score_at >= self.args.score_interval
        )
        if should_score:
            metrics, corners = self.compute_metrics(frame, category, self.records)
            self.last_score_at = now
        else:
            metrics = self.latest_metrics
            corners = None
        found = metrics["checkerboard_found"]
        pattern = (self.args.checkerboard_cols, self.args.checkerboard_rows)
        if found and corners is not None:
            debug = frame.copy()
            cv2.drawChessboardCorners(debug, pattern, corners, found)
        return debug, metrics

    def capture_high_res(self) -> dict:
        with self.capture_lock:
            with self.lock:
                if self.latest_frame is None:
                    raise RuntimeError("no full-resolution frame is available yet")
                frame = self.latest_frame.copy()
                capture_focus = self.current_focus_absolute
                capture_autofocus = self.autofocus_enabled
                actual = dict(self.camera_actual)
                controls = self.read_camera_control_values()

            self.capture_count += 1
            category = focus_category(capture_focus)
            category_dir = self.images_dir / category
            category_dir.mkdir(parents=True, exist_ok=True)
            image_name = f"cal_{self.capture_count:03d}.jpg"
            sidecar_name = f"cal_{self.capture_count:03d}.json"
            image_path = category_dir / image_name
            sidecar_path = category_dir / sidecar_name
            cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality])

            analysis_frame = cv2.resize(
                frame,
                (self.args.preview_width, self.args.preview_height),
                interpolation=cv2.INTER_AREA,
            )
            metrics, _corners = self.compute_metrics(
                analysis_frame,
                category,
                self.records,
                analysis_max_edge=self.args.analysis_max_edge,
            )
            metrics["capture_source_size"] = [int(frame.shape[1]), int(frame.shape[0])]
            metrics["metrics_frame_size"] = [int(analysis_frame.shape[1]), int(analysis_frame.shape[0])]
            metrics["metrics_source"] = "resized_latest_full_resolution_frame"

            record = {
                "index": self.capture_count,
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "image": f"images/{category}/{image_name}",
                "sidecar": f"images/{category}/{sidecar_name}",
                "category": category,
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
                "camera_actual": actual,
                "camera_controls": controls,
                "camera": {
                    "device": self.args.camera,
                    "autofocus": capture_autofocus,
                    "focus_absolute": capture_focus,
                    "fourcc": self.args.fourcc,
                },
                "metrics": metrics,
                "checkerboard_found": bool(metrics["checkerboard_found"]),
                "score": metrics["score"],
            }
            sidecar_path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            with self.manifest_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
            self.records.append(record)
            self.latest_status = f"saved {category}/{image_name}; score={metrics['score']:.0f}"
            return record

    def category_summary(self) -> dict:
        summary = {}
        for record in self.records:
            category = record.get("category", "uncategorized")
            item = summary.setdefault(
                category,
                {
                    "count": 0,
                    "checkerboard_count": 0,
                    "average_score": 0.0,
                    "focus_absolute": record.get("camera", {}).get("focus_absolute"),
                    "location_counts": {},
                    "tilt_counts": {},
                    "rotation_counts": {},
                },
            )
            item["count"] += 1
            item["checkerboard_count"] += 1 if record.get("checkerboard_found") else 0
            item["average_score"] += float(record.get("score", 0.0))
            metrics = record.get("metrics") or {}
            pose = metrics.get("pose_category") or {
                "location": board_location_label(metrics),
                "tilt": board_tilt_label(metrics),
                "rotation": board_rotation_label(metrics),
            }
            for source_key, target_key in (
                ("location", "location_counts"),
                ("tilt", "tilt_counts"),
                ("rotation", "rotation_counts"),
            ):
                label = pose.get(source_key, "unknown")
                item[target_key][label] = item[target_key].get(label, 0) + 1
        for item in summary.values():
            if item["count"]:
                item["average_score"] = round(item["average_score"] / item["count"], 1)
        return summary


PAGE = """<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Camera Calibration Capture</title>
  <style>
    :root {
      --ink: #172018;
      --muted: #5e6a5e;
      --paper: #f6f1e6;
      --panel: #fffaf0;
      --line: #d9ceb9;
      --good: #2f8f54;
      --warn: #c38321;
      --bad: #b84b3f;
    }
    body {
      background:
        radial-gradient(circle at 15% 0%, rgba(214, 151, 73, 0.20), transparent 28rem),
        linear-gradient(135deg, #f8f2e4, #e9efe1);
      color: var(--ink);
      font-family: ui-serif, Georgia, Cambria, "Times New Roman", serif;
      margin: 0;
    }
    main { margin: 0 auto; max-width: 1920px; padding: 18px; }
    h1 { font-size: clamp(30px, 4vw, 54px); line-height: 0.95; margin: 8px 0 12px; }
    h2 { margin: 0 0 10px; }
    code { background: rgba(255, 255, 255, 0.65); border: 1px solid var(--line); padding: 2px 5px; }
    .layout {
      align-items: start;
      display: grid;
      gap: 18px;
      grid-template-columns: minmax(640px, 1fr) minmax(780px, 1.05fr);
    }
    .panel {
      background: rgba(255, 250, 240, 0.88);
      border: 1px solid var(--line);
      border-radius: 18px;
      box-shadow: 0 16px 40px rgba(43, 35, 20, 0.12);
      padding: 14px;
    }
    .controls-panel { position: sticky; top: 12px; }
    .score-panel { margin-top: 18px; }
    .preview { width: 100%; border: 1px solid #9f927d; border-radius: 14px; display: block; background: #111; }
    .actions { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 12px; }
    button {
      background: #203327;
      border: 0;
      border-radius: 999px;
      color: white;
      cursor: pointer;
      font: 700 17px ui-sans-serif, system-ui, sans-serif;
      padding: 11px 17px;
    }
    button.secondary { background: #77644d; }
    button:disabled { cursor: not-allowed; opacity: 0.45; }
    .status { color: var(--muted); font-family: ui-sans-serif, system-ui, sans-serif; margin: 10px 0 0; }
    .score-strip {
      align-items: center;
      display: grid;
      gap: 14px;
      grid-template-columns: auto minmax(190px, 0.8fr) minmax(0, 1.5fr);
      margin-top: 12px;
    }
    .score-card { align-items: end; display: grid; gap: 12px; grid-template-columns: auto 1fr; }
    .score {
      font: 800 clamp(48px, 6vw, 82px) ui-sans-serif, system-ui, sans-serif;
      letter-spacing: -0.08em;
      line-height: 0.85;
    }
    .score.good { color: var(--good); }
    .score.warn { color: var(--warn); }
    .score.bad { color: var(--bad); }
    .meter { background: #eadfcb; border-radius: 999px; height: 10px; overflow: hidden; }
    .meter span { background: currentColor; display: block; height: 100%; width: 0%; }
    .metrics { display: grid; gap: 8px; grid-template-columns: repeat(3, minmax(0, 1fr)); margin-top: 0; }
    .metric { border-top: 1px solid var(--line); padding-top: 8px; }
    .metric b { display: block; font-family: ui-sans-serif, system-ui, sans-serif; font-size: 13px; text-transform: uppercase; }
    .metric span { color: var(--muted); font-family: ui-sans-serif, system-ui, sans-serif; }
    .control-grid {
      display: grid;
      gap: 16px;
      grid-template-columns: repeat(2, minmax(360px, 1fr));
      margin-top: 10px;
    }
    .control-column { min-width: 0; }
    .camera-control {
      border-top: 1px solid var(--line);
      display: grid;
      gap: 8px;
      grid-template-columns: minmax(96px, 0.58fr) minmax(150px, 1fr) 92px;
      padding-top: 8px;
    }
    .camera-control > span {
      color: var(--muted);
      font-family: ui-sans-serif, system-ui, sans-serif;
      font-size: 13px;
    }
    .control-inputs { align-items: center; display: grid; gap: 8px; grid-template-columns: 1fr; }
    .tuning-note {
      background: rgba(47, 143, 84, 0.10);
      border: 1px solid rgba(47, 143, 84, 0.28);
      border-radius: 12px;
      color: #24472d;
      font-family: ui-sans-serif, system-ui, sans-serif;
      font-size: 13px;
      line-height: 1.35;
      margin: 10px 0 0;
      padding: 10px;
    }
    input[type="range"] { width: 100%; }
    input[type="number"], select {
      background: #fffaf0;
      border: 1px solid var(--line);
      border-radius: 10px;
      color: var(--ink);
      box-sizing: border-box;
      font: 700 16px ui-sans-serif, system-ui, sans-serif;
      padding: 9px;
      width: 100%;
    }
    input[type="checkbox"] { transform: scale(1.2); transform-origin: left center; }
    label { font-family: ui-sans-serif, system-ui, sans-serif; }
    .categories { display: grid; gap: 14px; margin-top: 14px; }
    .category { background: rgba(255, 250, 240, 0.88); border: 1px solid var(--line); border-radius: 18px; padding: 14px; }
    .category h3 { display: flex; justify-content: space-between; margin: 0 0 10px; }
    .count-groups { display: grid; gap: 10px; grid-template-columns: repeat(3, minmax(0, 1fr)); }
    .count-group { border-top: 1px solid var(--line); padding-top: 8px; }
    .count-group b { display: block; font-family: ui-sans-serif, system-ui, sans-serif; font-size: 13px; margin-bottom: 5px; text-transform: uppercase; }
    .count-row { display: flex; font-family: ui-sans-serif, system-ui, sans-serif; justify-content: space-between; margin: 3px 0; }
    .empty { color: var(--muted); font-family: ui-sans-serif, system-ui, sans-serif; }
    @media (max-width: 1500px) {
      main { max-width: 1280px; }
      .layout { grid-template-columns: minmax(0, 1fr) minmax(520px, 0.85fr); }
      .controls-panel { grid-column: 2; position: static; }
      .score-strip { grid-template-columns: auto 1fr; }
      .metrics { grid-column: 1 / -1; grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .control-grid { grid-template-columns: 1fr; }
      .camera-control { grid-template-columns: 130px minmax(220px, 1fr) 96px; }
    }
    @media (max-width: 880px) {
      .layout { grid-template-columns: 1fr; }
      .controls-panel { grid-column: auto; }
      .metrics { grid-template-columns: 1fr; }
      .score-strip { grid-template-columns: 1fr; }
      .count-groups { grid-template-columns: 1fr; }
      .control-grid { grid-template-columns: 1fr; }
      .camera-control { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
<main>
  <h1>Camera Calibration Capture</h1>
  <p>Session: <code>{{ session }}</code></p>
  <div class="layout">
    <section class="panel preview-panel">
      <img class="preview" src="/stream" alt="preview">
      <div class="actions">
        <button onclick="capture()">Capture</button>
        <button class="secondary" onclick="refreshStatus()">Refresh</button>
      </div>
      <p class="status" id="status">loading</p>
      <div class="score-strip">
        <div id="score" class="score bad">0</div>
        <div>
          <div class="meter" id="scoreMeter"><span></span></div>
          <p class="status" id="scoreText">waiting for preview</p>
        </div>
        <div class="metrics" id="metrics"></div>
      </div>
    </section>
    <aside class="panel controls-panel">
      <h2>Camera Controls</h2>
      <p class="tuning-note">Tune first: set Exposure mode to Manual, turn Dynamic framerate off, keep Gain at 0, then adjust Exposure time until white clipping drops. Lock white balance next. Leave brightness/contrast/gamma/sharpness near defaults unless the image is still hard to detect.</p>
      <div class="control-grid" id="cameraControls"></div>
      <p class="status" id="cameraControlsStatus">loading controls</p>
    </aside>
  </div>
  <section class="panel score-panel">
    <h2>Capture Counts</h2>
    <section class="categories" id="categories"></section>
  </section>
</main>
<script>
let controlEditing = false;
let activeControlName = null;
let statusInFlight = false;

function isControlFocused() {
  const active = document.activeElement;
  return Boolean(active && active.id && active.id.startsWith('cameraControl'));
}

function markControlEditing(name) {
  activeControlName = name;
  controlEditing = true;
}

function releaseControlEditing(name) {
  window.setTimeout(() => {
    if (activeControlName === name && !isControlFocused()) {
      activeControlName = null;
      controlEditing = false;
    }
  }, 120);
}

function scoreClass(score) {
  if (score >= 75) return 'good';
  if (score >= 50) return 'warn';
  return 'bad';
}

function formatNumber(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return '?';
  return Number(value).toFixed(digits);
}

function renderMetrics(metrics) {
  const container = document.getElementById('metrics');
  const rows = [
    ['Checkerboard', metrics.checkerboard_found ? 'detected' : 'not found'],
    ['Sharpness', formatNumber(metrics.laplacian_variance, 0)],
    ['Exposure', 'mean ' + formatNumber(metrics.mean_brightness, 0)],
    ['Board size', formatNumber((metrics.board_area_ratio || 0) * 100, 1) + '%'],
    ['Location', (metrics.pose_category?.location || 'not_detected') + ', margin ' + formatNumber((metrics.board_margin_ratio || 0) * 100, 1) + '%'],
    ['Tilt', (metrics.pose_category?.tilt || 'not_detected') + ', ' + formatNumber(metrics.board_tilt_strength || 0, 2)],
    ['Rotation', (metrics.pose_category?.rotation || 'not_detected') + ', ' + formatNumber(metrics.board_angle_degrees || 0, 0) + ' deg'],
    ['Novelty', formatNumber((metrics.novelty_score || 0) * 100, 0) + '%'],
    ['Nearest', metrics.nearest_image || 'none']
  ];
  container.innerHTML = '';
  for (const [name, value] of rows) {
    const item = document.createElement('div');
    item.className = 'metric';
    item.innerHTML = '<b></b><span></span>';
    item.querySelector('b').textContent = name;
    item.querySelector('span').textContent = value;
    container.appendChild(item);
  }
}

function countRows(counts) {
  const entries = Object.entries(counts || {}).sort((a, b) => b[1] - a[1] || a[0].localeCompare(b[0]));
  if (!entries.length) return '<div class="empty">none yet</div>';
  return entries.map(([name, count]) => (
    '<div class="count-row"><span>' + name + '</span><strong>' + count + '</strong></div>'
  )).join('');
}

function renderCategories(summaries) {
  const container = document.getElementById('categories');
  const names = Object.keys(summaries || {}).sort();
  container.innerHTML = '';
  if (!names.length) {
    container.innerHTML = '<section class="category"><h3>No captures yet</h3><p class="empty">Saved image counts will appear here by focus, location, tilt, and rotation.</p></section>';
    return;
  }
  for (const category of names) {
    const summary = summaries[category] || {};
    const section = document.createElement('section');
    section.className = 'category';
    const title = document.createElement('h3');
    title.innerHTML = '<span></span><small></small>';
    title.querySelector('span').textContent = category;
    title.querySelector('small').textContent = (summary.count || 0) + ' images, ' +
      (summary.checkerboard_count || 0) + ' with board, avg score ' + formatNumber(summary.average_score || 0, 1);
    section.appendChild(title);
    const groups = document.createElement('div');
    groups.className = 'count-groups';
    groups.innerHTML =
      '<div class="count-group"><b>Location</b>' + countRows(summary.location_counts) + '</div>' +
      '<div class="count-group"><b>Tilt</b>' + countRows(summary.tilt_counts) + '</div>' +
      '<div class="count-group"><b>Rotation</b>' + countRows(summary.rotation_counts) + '</div>';
    section.appendChild(groups);
    container.appendChild(section);
  }
}

function controlValueLabel(control) {
  if (control.value === null || control.value === undefined) return '?';
  if (control.type === 'bool') return control.value ? 'on' : 'off';
  if (control.type === 'select') {
    const option = (control.options || []).find((item) => Number(item.value) === Number(control.value));
    return option ? option.label : String(control.value);
  }
  return String(control.value);
}

function renderCameraControls(state) {
  const container = document.getElementById('cameraControls');
  const status = document.getElementById('cameraControlsStatus');
  const controls = state.controls || [];
  container.innerHTML = '';
  status.textContent = state.error ? 'control read error: ' + state.error : 'camera controls are recorded with each capture';
  const leftColumn = document.createElement('div');
  const rightColumn = document.createElement('div');
  leftColumn.className = 'control-column';
  rightColumn.className = 'control-column';
  container.appendChild(leftColumn);
  container.appendChild(rightColumn);
  const splitIndex = Math.ceil(controls.length / 2);

  controls.forEach((control, index) => {
    const row = document.createElement('div');
    row.className = 'camera-control';
    const label = document.createElement('span');
    label.textContent = control.label;
    const value = document.createElement('strong');
    value.id = 'controlValue_' + control.name;
    value.textContent = controlValueLabel(control);

    let input;
    let numberInput = null;
    if (control.type === 'select') {
      input = document.createElement('select');
      for (const option of control.options || []) {
        const item = document.createElement('option');
        item.value = option.value;
        item.textContent = option.label;
        input.appendChild(item);
      }
      input.value = control.value ?? '';
      input.addEventListener('focus', () => markControlEditing(control.name));
      input.addEventListener('blur', () => releaseControlEditing(control.name));
      input.addEventListener('change', () => setCameraControl(control.name, input.value));
    } else if (control.type === 'bool') {
      input = document.createElement('input');
      input.type = 'checkbox';
      input.checked = Boolean(control.value);
      input.addEventListener('change', () => setCameraControl(control.name, input.checked));
    } else {
      const inputs = document.createElement('div');
      inputs.className = 'control-inputs';
      const rangeInput = document.createElement('input');
      rangeInput.type = 'range';
      rangeInput.min = control.min;
      rangeInput.max = control.max;
      rangeInput.step = control.step || 1;
      rangeInput.value = control.value ?? control.min;
      numberInput = document.createElement('input');
      numberInput.type = 'number';
      numberInput.min = control.min;
      numberInput.max = control.max;
      numberInput.step = control.step || 1;
      numberInput.value = control.value ?? control.min;
      rangeInput.addEventListener('focus', () => markControlEditing(control.name));
      rangeInput.addEventListener('blur', () => releaseControlEditing(control.name));
      rangeInput.addEventListener('input', () => {
        markControlEditing(control.name);
        numberInput.value = rangeInput.value;
        const label = document.getElementById('controlValue_' + control.name);
        if (label) label.textContent = rangeInput.value;
      });
      rangeInput.addEventListener('change', () => {
        markControlEditing(control.name);
        numberInput.value = rangeInput.value;
        setCameraControl(control.name, rangeInput.value);
      });
      numberInput.addEventListener('focus', () => markControlEditing(control.name));
      numberInput.addEventListener('blur', () => releaseControlEditing(control.name));
      numberInput.addEventListener('keydown', (event) => {
        if (event.key === 'Enter') {
          event.preventDefault();
          numberInput.blur();
          setCameraControl(control.name, numberInput.value);
        }
      });
      numberInput.addEventListener('input', () => {
        markControlEditing(control.name);
        rangeInput.value = numberInput.value;
        const label = document.getElementById('controlValue_' + control.name);
        if (label) label.textContent = numberInput.value;
      });
      numberInput.addEventListener('change', () => {
        markControlEditing(control.name);
        rangeInput.value = numberInput.value;
        setCameraControl(control.name, numberInput.value);
      });
      inputs.appendChild(rangeInput);
      input = inputs;
    }

    const primaryInput = input.className === 'control-inputs' ? input.querySelector('input[type="range"]') : input;
    primaryInput.id = 'cameraControl_' + control.name;
    primaryInput.disabled = !control.available;
    if (numberInput) {
      numberInput.id = 'cameraControlNumber_' + control.name;
      numberInput.disabled = !control.available;
    }
    row.appendChild(label);
    row.appendChild(input);
    row.appendChild(numberInput || value);
    (index < splitIndex ? leftColumn : rightColumn).appendChild(row);
  });
}

async function setCameraControl(name, value) {
  if (name === undefined || value === undefined || value === '') {
    document.getElementById('cameraControlsStatus').textContent = 'control request skipped: missing name or value';
    activeControlName = null;
    controlEditing = false;
    return;
  }
  markControlEditing(name);
  const status = document.getElementById('cameraControlsStatus');
  let s;
  try {
    const r = await fetch('/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, value })
    });
    s = await r.json();
  } catch (error) {
    status.textContent = 'control request failed: ' + error.message;
    activeControlName = null;
    controlEditing = false;
    return;
  }
  if (!s.ok) {
    status.textContent = s.error;
    activeControlName = null;
    controlEditing = false;
    return;
  }
  status.textContent = 'set ' + name + ' = ' + s.control.value;
  const currentValue = s.control.value;
  const label = document.getElementById('controlValue_' + name);
  const input = document.getElementById('cameraControl_' + name);
  const number = document.getElementById('cameraControlNumber_' + name);
  if (label) label.textContent = String(currentValue);
  if (input && input.type !== 'checkbox' && document.activeElement !== input) input.value = currentValue;
  if (input && input.type === 'checkbox') input.checked = Boolean(currentValue);
  if (number && document.activeElement !== number) number.value = currentValue;
  activeControlName = null;
  controlEditing = false;
  renderCameraControls(s.control.controls);
}

async function refreshStatus() {
  if (statusInFlight) return;
  statusInFlight = true;
  let s;
  try {
    const r = await fetch('/status');
    s = await r.json();
  } catch (error) {
    document.getElementById('status').textContent = 'status request failed: ' + error.message;
    statusInFlight = false;
    return;
  }
  statusInFlight = false;
  document.getElementById('status').textContent = s.status + ' | saved ' + s.count;

  const metrics = s.metrics || {};
  const score = Number(metrics.score || 0);
  const scoreEl = document.getElementById('score');
  scoreEl.textContent = score.toFixed(0);
  scoreEl.className = 'score ' + scoreClass(score);
  document.querySelector('#scoreMeter span').style.width = Math.max(0, Math.min(100, score)) + '%';
  document.getElementById('scoreText').textContent =
    metrics.checkerboard_found ? 'usable board view' : 'checkerboard not detected';
  renderMetrics(metrics);

  if (!controlEditing && !isControlFocused()) {
    renderCameraControls(s.camera_controls || {});
  }
  renderCategories(s.categories || {});
}

async function capture() {
  document.getElementById('status').textContent = 'capturing high-res image...';
  let s;
  try {
    const r = await fetch('/capture', { method: 'POST' });
    s = await r.json();
  } catch (error) {
    document.getElementById('status').textContent = 'capture request failed: ' + error.message;
    return;
  }
  document.getElementById('status').textContent = s.ok ? 'saved ' + s.record.image : s.error;
  await refreshStatus();
}

document.addEventListener('keydown', (e) => {
  if (e.target && ['INPUT', 'BUTTON'].includes(e.target.tagName)) return;
  if (e.key === 'Enter' || e.key === ' ') capture();
});

refreshStatus();
setInterval(refreshStatus, 2000);
</script>
</body>
</html>
"""


def create_app(state: CameraState) -> Flask:
    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template_string(PAGE, session=state.session)

    @app.route("/stream")
    def stream():
        def frames():
            while state.running:
                if state.latest_jpeg is None:
                    time.sleep(0.05)
                    continue
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + state.latest_jpeg
                    + b"\r\n"
                )
                time.sleep(0.03)
        return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/capture", methods=["POST"])
    def capture():
        try:
            return jsonify({"ok": True, "record": state.capture_high_res()})
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500

    @app.route("/focus", methods=["POST"])
    def focus():
        data = flask_request.get_json(silent=True) or {}
        if "focus_absolute" not in data:
            return jsonify({"ok": False, "error": "missing focus_absolute"}), 400
        try:
            return jsonify({"ok": True, "focus": state.set_focus(int(data["focus_absolute"]))})
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500

    @app.route("/control", methods=["POST"])
    def control():
        data = flask_request.get_json(silent=True) or {}
        if "name" not in data or "value" not in data:
            return jsonify({"ok": False, "error": "missing name or value"}), 400
        try:
            return jsonify({"ok": True, "control": state.set_camera_control(data["name"], data["value"])})
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500

    @app.route("/status")
    def status():
        return jsonify(
            {
                "status": state.latest_status,
                "count": state.capture_count,
                "metrics": state.latest_metrics,
                "focus": state.focus_state(),
                "camera_controls": state.camera_control_state(),
                "categories": state.category_summary(),
            }
        )

    @app.route("/images/<path:name>")
    def image(name):
        return send_from_directory(state.images_dir, name)

    @app.route("/manifest.jsonl")
    def manifest():
        return send_from_directory(state.session, "manifest.jsonl")

    return app


def main() -> int:
    args = parse_args()
    state = CameraState(args)
    app = create_app(state)
    state.start()
    stop_flag = {"stop": False}

    def request_stop(signum, frame):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    server = make_server(args.host, args.port, app, threaded=True)
    server.timeout = 0.5
    if hasattr(server, "daemon_threads"):
        server.daemon_threads = True
    print(f"session={state.session}")
    print(f"open http://<pi-hostname-or-ip>:{args.port}/")
    try:
        while not stop_flag["stop"]:
            server.handle_request()
    finally:
        server.server_close()
        state.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
