# Wave Rover Development

This repo is the clean workspace for the Raspberry Pi 4 plus Waveshare WAVE ROVER/ESP32 stack.

Current contents:

- `agent_pack/`: collected rover docs, schematic crops, command notes, pinout notes, and vendor references.
- `tools/rover_status.py`: small serial status probe for the rover JSON API.
- `tools/requirements.txt`: Python packages used by local ESP32 tooling.
- `tools/camera_calibrate.py`: simple OpenCV-only checkerboard solve against a saved capture session.
- `tools/mrcal_corners_cache.py`: generate an `mrcal` corners cache from saved checkerboard images.
- `tools/mrcal_calibrate.py`: one-shot `mrcal` calibration wrapper that runs corner extraction, solve, plots, and analysis.
- `STATUS.md`: current hardware/software findings.
- `AUTO_DOCKING_PLAN.md`: current auto-docking development plan and hardware/control assumptions.

Auto-docking dock reference:

- The dock CAD places the AprilTag center point at a nominal 200 mm above the floor.
- Treat that as a design reference, not a measured truth. Manufacturing, assembly, glue thickness, tag placement, and contact tolerance can shift the real pose, so docking software should use calibration data.

Useful checks:

```bash
tools/rover_status.py --port /dev/serial0
tools/rover_status.py --port /dev/ttyUSB0
tools/.venv/bin/esptool --port /dev/ttyUSB0 --baud 115200 chip-id
tools/apriltag_probe.py --camera /dev/video0 --family tag16h5 --id 0 --tag-size 0.034 --width 1280 --height 720 --frames 120 --focus-absolute 432
tools/camera_cal_server.py --camera /dev/video0 --host 0.0.0.0 --port 8080 --focus-absolute 432
```

Camera calibration image collection:

- Run `tools/camera_cal_server.py --camera /dev/video0 --host 0.0.0.0 --port 8080`.
- Open `http://<rover-pi-ip>:8080/` from another device on the network.
- The current checkerboard is 9 squares across by 7 squares down, described as 4 white + 5 black across and 4 black + 3 white down. Use `8 x 6` inner corners for OpenCV checkerboard detection.
- Measure and record the physical square size before running the later calibration solve.
- Use the low-resolution browser preview to pose the checkerboard, then click Capture or press Enter in the browser. The page shows a rough score for checkerboard detection, sharpness, exposure, board size, board location, board tilt, and novelty versus saved images at the same focus.
- The camera runs continuously at capture resolution, defaulting to `1920x1080` MJPG at `--fps 30`; the browser preview is a resized copy of that same capture stream.
- Preview and stream defaults are `640x360` so the browser view keeps the same 16:9 aspect ratio as the default capture mode.
- Capture saves the latest full-resolution frame already in memory, so it does not switch camera modes before saving.
- The script saves focus-categorized images and sidecar metadata as `images/focus_####/cal_###.jpg` and `images/focus_####/cal_###.json`, plus `session.json` and append-only `manifest.jsonl` under `data/camera_calibration/captures/<timestamp>/`.
- Each sidecar records current camera image controls: autofocus, focus, exposure mode/time, dynamic framerate, gain, brightness, contrast, saturation, gamma, sharpness, backlight compensation, white balance, and power-line frequency.
- Focus is now exposed in the same camera-control list as the other sliders, with a numeric input beside every slider. Saved-image counts are summarized below the main preview/controls area by focus, board location, board tilt, and board rotation instead of loading capture thumbnails.
- The browser UI also exposes those camera image controls so exposure and white balance can be locked for repeatable capture.
- Tune controls in this order: manual exposure with dynamic framerate off, gain at 0, exposure time low enough to avoid clipped white squares, then fixed white balance. Leave brightness, contrast, gamma, and sharpness near defaults unless detection still needs help.
- Checkerboard scoring is throttled by `--score-interval`, defaulting to `0.20` seconds for about 5 Hz scoring, and runs on the resized preview copy rather than the full 8 MP frame.
- Full-resolution captures are still saved at capture resolution; sidecar metadata records that scoring came from a resized copy of the same full-resolution frame.
- Keep continuous autofocus disabled during calibration and docking. The capture server disables UVC continuous autofocus on launch unless `--autofocus` is passed. The current Arducam reports manual focus value `432`; use the same focus setting for calibration captures and later AprilTag pose estimation unless deliberately recalibrating.
- Relaunching with the same `--session` reloads existing `manifest.jsonl` records so the page can show existing categories and novelty scoring without rescanning all images.
- The actual calibration solve should be run separately against the saved images.

Camera calibration solve:

- Preferred one-shot command:

```bash
python3 tools/mrcal_calibrate.py \
  data/camera_calibration/captures/<timestamp> \
  --square-size 0.03 \
  --focal 2680
```

- For the current 9x7-square checkerboard, `mrcal` still cannot natively detect the `8x6` inner-corner grid through `mrgingham`, so the wrapper automatically generates an OpenCV-based `corners-opencv.vnl` cache and then runs `mrcal-calibrate-cameras` from that cache.
- If a future target is square, the same wrapper can use native `mrgingham` with `--detector auto`.
- Outputs are written under `<session>/calibration/mrcal/`:
  - `camera-0.cameramodel`
  - `corners-opencv.vnl` or `corners.vnl`
  - `plots/*.svg`
  - `summary.json`
  - `analysis.json`
  - `analysis.md`
- `analysis.md` reports both the solver-style RMS residual and the RMS of full 2-D residual magnitudes so the metrics are not conflated.
