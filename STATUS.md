# Current Status

Date: 2026-05-19

## Machine

- Hardware: Raspberry Pi 4 Model B Rev 1.4
- OS: Debian GNU/Linux 12 bookworm, arm64
- Kernel: `6.12.25+rpt-rpi-v8`
- Memory: 7.6 GiB RAM, 2.0 GiB swap
- Root filesystem: 57G total, 44G available at last check
- Failed systemd units: none
- Power state: `vcgencmd get_throttled` reports `throttled=0x0`

## Codex

- Codex is installed as a user-local standalone binary under `~/.local/share/codex-cli/0.131.0-alpha.16/bin/codex`, exposed through `~/.local/bin/codex`.
- `codex doctor` reports the install itself as consistent. The only current failure is `TERM=dumb` in this non-interactive session; that is a terminal environment issue, not a broken install.

## Rover Connections

- USB serial bridge: Silicon Labs CP210x, `/dev/ttyUSB0`
- GPIO UART: `/dev/serial0 -> ttyAMA0`
- GPIO14 is `TXD0`; GPIO15 is `RXD0`
- Serial settings: 115200 baud, newline-terminated JSON
- TX/RX cabling was corrected by swapping the pair. Both paths now work:
  - USB serial control works.
  - Pi GPIO UART control through `/dev/serial0` works.
  - GPIO-originated commands echo back and produce feedback.
- Latest rover feedback over `/dev/serial0`:
  - voltage: 11.820 V
  - motors: `L=0`, `R=0`
  - temperature field: about 61 C
- OLED command over GPIO succeeded. Last test text included `GPIO OK`.

## ESP32

- Chip detected by `esptool`: ESP32-D0WD-V3, revision v3.1
- Crystal: 40 MHz
- MAC: redacted
- `esptool` can enter the bootloader/upload-stub path over `/dev/ttyUSB0` without pressing physical buttons, and can reset the app afterward.
- Stock firmware is Waveshare `WAVE_ROVER_V0.9`, an Arduino-ESP32 sketch.
- Main stock interfaces:
  - UART0 JSON command parser via `Serial`
  - Wi-Fi AP/web UI, default AP `UGV` / `12345678`
  - HTTP `/js` path into the same JSON handler
  - ESP-NOW command receive path
  - LittleFS for config and mission files
  - OLED, INA219 voltage, IMU, motor control, optional arm/gimbal modules
- Stock partition table has OTA-style app slots (`app0`, `app1`) plus `otadata`, but the stock sketch does not appear to implement ArduinoOTA or a firmware upload endpoint. USB flashing is the confirmed recovery/update path.

## Notes

- Auto-docking tag print target:
  - Tag family: `tag16h5`
  - Tag ID: `0`
  - Cut piece: 40 mm square
  - Actual black outer tag square for pose estimation: 34 mm square, so use `tag_size_m=0.034`
  - Dock CAD nominal tag center height: 200 mm above floor, to be treated as a calibration seed only.
- Camera / AprilTag bring-up:
  - USB `Arducam_8mp` is available as `/dev/video0`.
  - OpenCV can capture from `/dev/video0`.
  - OpenCV AprilTag dictionary detection works for the printed `tag16h5` ID 0 tag.
  - Quick probe command detected the expected tag in 47 of 60 frames at requested `1280x720@30`; observed processing rate was about 4.3 FPS in the current Python/OpenCV probe.
  - `640x480` is faster, but the current tag view is only about 30 px per edge and can be intermittent depending on framing/lighting. Prefer `1280x720` for initial calibration and docking bring-up, then downshift only if the detection envelope remains reliable.
  - A warmed `1280x720` probe detected the expected tag in 21 of 30 frames with about 64 px mean edge length in the current setup.
  - Debug frame from the quick probe was saved outside the repo at `/tmp/wave_rover_apriltag_debug.jpg`.
- Camera calibration target:
  - Checkerboard squares: 9 across by 7 down.
  - User description: 4 white + 5 black across, 4 black + 3 white down.
  - OpenCV inner-corner pattern: `8 x 6`.
  - Calibration square size used in the current solve: `0.03 m`.
  - Installed `mrcal` still rejects native non-square board detection through `mrgingham`, so the current workflow uses an OpenCV-generated corners cache and then runs the native `mrcal` solve on that cache.
- Camera calibration capture workflow:
  - Use `tools/camera_cal_server.py --camera /dev/video0 --host 0.0.0.0 --port 8080`.
  - The camera now runs continuously at capture resolution, defaulting to `1920x1080` MJPG at `--fps 30`; browser preview is a resized copy of the same capture stream.
  - Preview and stream defaults are `640x360` to preserve the 16:9 aspect ratio of the default capture mode.
  - Preview scoring now reports checkerboard detection, sharpness, exposure, board size, board location, board tilt, and novelty versus saved images at the same focus value. Scoring is throttled by `--score-interval`, default `0.20` seconds.
  - Capture saves the latest full-resolution frame already in memory, so it does not switch camera modes or perform a high-resolution warmup before saving.
  - Saved images remain full resolution, but capture metadata scoring uses a resized copy of the same full-resolution frame instead of running checkerboard detection on the full 8 MP image.
  - Capture sessions save full-resolution images under `images/focus_####/`, one JSON sidecar per image, `session.json`, and append-only `manifest.jsonl`.
  - Each sidecar records current camera image controls: autofocus, focus, exposure mode/time, dynamic framerate, gain, brightness, contrast, saturation, gamma, sharpness, backlight compensation, white balance, and power-line frequency.
  - The browser shows count summaries by focus, relative board location, tilt, and rotation; it does not load saved-image thumbnails into the page.
  - The browser exposes the same camera image controls so exposure and white balance can be locked for repeatable capture.
  - Recommended tuning order: manual exposure, dynamic framerate off, gain `0`, reduce exposure time until white-square clipping is low, then lock white balance; leave brightness/contrast/gamma/sharpness close to defaults unless needed.
  - Reusing a `--session` reloads `manifest.jsonl` quickly for existing category count display and novelty scoring.
- Camera calibration solve workflow:
  - Preferred command is `python3 tools/mrcal_calibrate.py data/camera_calibration/captures/<timestamp> --square-size 0.03 --focal 2680`.
  - `tools/mrcal_calibrate.py` writes the corners cache, runs `mrcal-calibrate-cameras`, exports residual/distortion/uncertainty plots, and writes `summary.json`, `analysis.json`, and `analysis.md`.
  - `tools/camera_calibrate.py` remains available as a simple OpenCV-only baseline solve and cross-check.
- Current `mrcal` result for session `20260519_023511`:
  - Output directory: `data/camera_calibration/captures/20260519_023511/calibration/mrcal/`
  - Solver RMS reprojection error: about `0.33 px`
  - Worst residual: about `1.8 px`
  - Outliers rejected: `51 / 2064` points
  - Board observations used: `43`
  - Coverage convex hull: about `80.8%` of the imager
  - Current generated analysis reports an empty valid-intrinsics region for this solve, so downstream use should treat edge-of-frame behavior conservatively.
- Camera focus:
  - The capture server disables UVC continuous autofocus on launch unless `--autofocus` is passed.
  - Current UVC controls report `focus_automatic_continuous=0`, so continuous autofocus is already disabled.
  - Current manual `focus_absolute` value is `432` on the Arducam.
  - The capture browser exposes focus in the normal camera-control list with a numeric input beside the slider.
  - Calibration and docking should use the same locked focus value; changing focus after calibration can change effective intrinsics enough to hurt pose accuracy.
- Pi-side I2C was not reliable during probing and is not the preferred voltage source. Use ESP32 serial feedback for rover voltage.
- Old development folders were moved to `../archive` .
- Keep generated Python environments out of git. Recreate the tool venv with:

```bash
python3 -m venv tools/.venv
tools/.venv/bin/python -m pip install -r tools/requirements.txt
```
