#!/usr/bin/env python3
import argparse
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
from flask import Flask, Response, jsonify, redirect, render_template_string, send_from_directory


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
    parser.add_argument("--preview-height", type=int, default=480)
    parser.add_argument("--stream-width", type=int, default=320)
    parser.add_argument("--stream-height", type=int, default=240)
    parser.add_argument("--capture-width", type=int, default=3264)
    parser.add_argument("--capture-height", type=int, default=2448)
    parser.add_argument("--fps", type=float, default=30.0)
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
    return parser.parse_args()


def camera_source(value: str):
    try:
        return int(value)
    except ValueError:
        return value


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


class CameraState:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.session = args.output_root / args.session
        self.images_dir = self.session / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.session / "manifest.jsonl"
        self.session_info_path = self.session / "session.json"
        self.lock = threading.RLock()
        self.running = True
        self.cap = None
        self.latest_frame = None
        self.latest_jpeg = None
        self.latest_status = "starting"
        self.capture_count = 0
        self.records = []
        self.thread = threading.Thread(target=self.preview_loop, daemon=True)
        self.write_session_info()

    def write_session_info(self) -> None:
        data = {
            "camera": self.args.camera,
            "preview_size": [self.args.preview_width, self.args.preview_height],
            "stream_size": [self.args.stream_width, self.args.stream_height],
            "capture_size": [self.args.capture_width, self.args.capture_height],
            "checkerboard_inner_corners": [self.args.checkerboard_cols, self.args.checkerboard_rows],
            "autofocus": self.args.autofocus,
            "focus_absolute": self.args.focus_absolute,
            "created": datetime.now().isoformat(timespec="seconds"),
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
        self.cap = open_camera(
            self.args.camera,
            self.args.preview_width,
            self.args.preview_height,
            self.args.fps,
            self.args.fourcc,
            self.args.autofocus,
            self.args.focus_absolute,
        )
        for _ in range(max(0, self.args.warmup_frames)):
            self.cap.read()

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

                debug, found = self.draw_feedback(frame)
                preview = cv2.resize(
                    debug,
                    (self.args.stream_width, self.args.stream_height),
                    interpolation=cv2.INTER_AREA,
                )
                ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    self.latest_frame = frame
                    self.latest_jpeg = encoded.tobytes()
                    self.latest_status = "checkerboard found" if found else "checkerboard not found"
            except Exception as exc:
                self.latest_status = f"preview error: {exc}"
                time.sleep(0.5)

    def draw_feedback(self, frame):
        debug = frame.copy()
        found = False
        if self.args.checkerboard_cols and self.args.checkerboard_rows:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            pattern = (self.args.checkerboard_cols, self.args.checkerboard_rows)
            if hasattr(cv2, "findChessboardCornersSB"):
                found, corners = cv2.findChessboardCornersSB(gray, pattern)
            else:
                found, corners = cv2.findChessboardCorners(gray, pattern)
            if found:
                cv2.drawChessboardCorners(debug, pattern, corners, found)

        label = f"{self.latest_status} | saved {self.capture_count}"
        cv2.rectangle(debug, (0, 0), (debug.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(debug, label, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
        return debug, found

    def capture_high_res(self) -> dict:
        with self.lock:
            if self.cap is not None:
                self.cap.release()
                self.cap = None

            cap = open_camera(
                self.args.camera,
                self.args.capture_width,
                self.args.capture_height,
                15.0,
                self.args.fourcc,
                self.args.autofocus,
                self.args.focus_absolute,
            )
            frame = None
            actual = {
                "width": cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                "height": cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                "fps": cap.get(cv2.CAP_PROP_FPS),
            }
            for _ in range(max(1, self.args.warmup_frames)):
                ok, current = cap.read()
                if ok and current is not None:
                    frame = current
            cap.release()

            self.open_preview()

        if frame is None:
            raise RuntimeError("high-resolution capture failed")

        self.capture_count += 1
        image_name = f"cal_{self.capture_count:03d}.jpg"
        image_path = self.images_dir / image_name
        cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality])

        found = False
        if self.args.checkerboard_cols and self.args.checkerboard_rows:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            pattern = (self.args.checkerboard_cols, self.args.checkerboard_rows)
            if hasattr(cv2, "findChessboardCornersSB"):
                found, _ = cv2.findChessboardCornersSB(gray, pattern)
            else:
                found, _ = cv2.findChessboardCorners(gray, pattern)

        record = {
            "index": self.capture_count,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "image": f"images/{image_name}",
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
            "camera_actual": actual,
            "checkerboard_found": bool(found),
        }
        with self.manifest_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
        self.records.append(record)
        self.latest_status = f"saved {image_name}; checkerboard_found={found}"
        return record


PAGE = """<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Camera Calibration Capture</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 18px; max-width: 760px; }
    img { max-width: 100%; border: 1px solid #ccc; }
    button { font-size: 18px; padding: 10px 16px; margin: 10px 8px 10px 0; }
    code { background: #eee; padding: 2px 4px; }
    li { margin: 4px 0; }
  </style>
</head>
<body>
  <h1>Camera Calibration Capture</h1>
  <p>Session: <code>{{ session }}</code></p>
  <img src="/stream" alt="preview">
  <p>
    <button onclick="capture()">Capture</button>
    <button onclick="refreshStatus()">Refresh</button>
  </p>
  <p id="status">loading</p>
  <h2>Saved Images</h2>
  <ol id="images"></ol>
<script>
async function refreshStatus() {
  const r = await fetch('/status');
  const s = await r.json();
  document.getElementById('status').textContent = s.status + ' | saved ' + s.count;
  const list = document.getElementById('images');
  list.innerHTML = '';
  for (const rec of s.records) {
    const li = document.createElement('li');
    const a = document.createElement('a');
    a.href = '/' + rec.image;
    a.textContent = rec.image + ' ' + rec.width + 'x' + rec.height +
      ' checkerboard=' + rec.checkerboard_found;
    li.appendChild(a);
    list.appendChild(li);
  }
}
async function capture() {
  document.getElementById('status').textContent = 'capturing high-res image...';
  const r = await fetch('/capture', { method: 'POST' });
  const s = await r.json();
  document.getElementById('status').textContent = s.ok ? 'saved ' + s.record.image : s.error;
  await refreshStatus();
}
document.addEventListener('keydown', (e) => {
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

    @app.route("/status")
    def status():
        return jsonify({"status": state.latest_status, "count": state.capture_count, "records": state.records})

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
    print(f"session={state.session}")
    print(f"open http://<pi-hostname-or-ip>:{args.port}/")
    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        state.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
