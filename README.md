# Wave Rover Development

This repo is the clean workspace for the Raspberry Pi 4 plus Waveshare WAVE ROVER/ESP32 stack.

Current contents:

- `agent_pack/`: collected rover docs, schematic crops, command notes, pinout notes, and vendor references.
- `firmware/esp32/wave_rover_minimal/`: direct ESP-IDF lower-controller firmware for bounded relative moves over binary serial.
- `tools/rover_status.py`: small serial status probe for the rover JSON API.
- `tools/minimal_rover_serial.py`: host helper for the custom ESP-IDF binary serial protocol.
- `tools/light_ctrl.py`: tiny custom-light helper for the Pi-header lamp on `GPIO18` (`P12`) with ground on `P39`.
- `tools/camera_exposure_lock.py`: one-shot helper to read the current camera exposure and lock it into manual mode.
- `tools/rover_control.py`: basic Phase 3 control helper for body-frame `+X` / `+Z` motion pulses over `/dev/serial0`.
- `tools/rover_motion_probe.py`: motion and IMU probe utility that logs pulse-response runs under `data/motion_probes/`.
- `tools/tag_motion_probe.py`: combined AprilTag-pose plus IMU/motor pulse probe for tiny visible-tag motion tests.
- `tools/tag_motion_collect.py`: structured batch collector that runs repeated linear and/or turn probes and records a batch manifest plus per-run summaries under `data/tag_motion_batches/`.
- `tools/requirements.txt`: Python packages used by local ESP32 tooling.
- `tools/apriltag_pose.py`: live AprilTag pose estimator using the saved `mrcal` camera model plus OpenCV `solvePnP`.
- `tools/dock_pose_calibrate.py`: capture a short burst of docked-tag pose samples and save the dock target pose JSON for later controller use.
- `tools/dock_heading_step.py`: single-step heading-servo experiment tool that reports off-axis and centerline metrics in tag coordinates and can optionally execute one one-sided arc pulse.
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
tools/light_ctrl.py on
tools/light_ctrl.py off
tools/light_ctrl.py status
tools/camera_exposure_lock.py --camera /dev/video0 lock-current
tools/camera_exposure_lock.py --camera /dev/video0 status
tools/tag_motion_collect.py --camera /dev/video0 --focus-absolute 350 --program both --name structured
tools/tag_motion_probe.py --camera /dev/video0 --focus-absolute 350 --mode backward --pwm 0.14 --duration 0.40 --pre-seconds 2.0 --post-seconds 2.0 --name linear_backward_014
tools/rover_control.py forward-test --pwm 0.10 --duration 0.40
tools/rover_control.py pulse --x-pwm 0.00 --z-pwm 0.35 --duration 0.20
tools/rover_motion_probe.py forward --port /dev/serial0 --pwm 0.50 --duration 0.35
tools/rover_motion_probe.py turn-ccw --port /dev/serial0 --pwm 0.50 --duration 0.25
tools/minimal_rover_serial.py --port /dev/serial0 stop
tools/minimal_rover_serial.py --port /dev/serial0 --read-seconds 1.0 monitor
tools/minimal_rover_serial.py --port /dev/serial0 --read-seconds 1.2 move --z-deg 5 --max-time-ms 1000
tools/.venv/bin/esptool --port /dev/ttyUSB0 --baud 115200 chip-id
tools/apriltag_probe.py --camera /dev/video0 --family tag16h5 --id 0 --tag-size 0.034 --width 1280 --height 720 --frames 120 --focus-absolute 350
tools/apriltag_pose.py --camera /dev/video0 --family tag16h5 --id 0 --tag-size 0.034 --width 1280 --height 720 --frames 120 --focus-absolute 350
tools/apriltag_pose.py --camera /dev/video0 --family tag16h5 --id 0 --tag-size 0.034 --width 1280 --height 720 --frames 0 --focus-absolute 350 --serve --host 0.0.0.0 --port 8090
tools/dock_pose_calibrate.py --camera /dev/video0 --family tag16h5 --id 0 --tag-size 0.034 --width 1280 --height 720 --samples 25 --focus-absolute 350
tools/dock_heading_step.py --camera /dev/video0 --family tag16h5 --id 0 --tag-size 0.034 --width 3264 --height 2448 --fps 15 --autofocus --auto-exposure manual --exposure-time 220 --samples 3 --max-frames 12
tools/camera_cal_server.py --camera /dev/video0 --host 0.0.0.0 --port 8080 --focus-absolute 432
```

AprilTag pose and dock target calibration:

- `tools/apriltag_pose.py` loads the newest saved `mrcal` model by default from `data/camera_calibration/captures/*/calibration/mrcal/summary.json`. Pass `--model` explicitly if you want a different calibration.
- The current saved `mrcal` model from session `20260519_023511` was calibrated at `focus_absolute=350`. Use that same focus when running pose estimation against that model, or recalibrate if you want to standardize on a different focus such as `432`.
- Live pose output uses the OpenCV camera frame: `+X` right in the image, `+Y` down in the image, `+Z` forward from the camera.
- The docked target-pose capture utility saves to `config/auto_docking/docked_tag_pose.json` by default.
- The target-pose JSON stores the observed dock tag pose in that same camera frame so later controller code can compare live pose to target pose without an extra frame conversion.
- Daytime live testing on `2026-05-20` found that `1920x1080` and below could still miss the visible tag in OpenCV even after two small forward nudges, while `3264x2448 @ 15 fps` with `--autofocus --auto-exposure manual --exposure-time 220` could recover usable live detections again.
- Treat autofocus-based docking tests as controller bring-up only. The current saved calibration and docked target were captured at manual focus `350`, so autofocus changes effective intrinsics and can bias absolute pose even when relative heading/error trends are still useful.
- Dark-room testing already found an important lighting failure mode: if the scene is dark and only the tag is strongly illuminated, camera auto exposure can blow the tag out and detection can fail. Stable ambient light or controlled active illumination will matter for robust docking.
- Archived Waveshare Pi-side code suggests a reusable light-control path through ESP32 JSON `{"T":132,"IO4":..., "IO5":...}` for the board's 12 V switched outputs. That is an ESP32-controlled light path, not direct Pi GPIO lamp drive.
- User later clarified that the lamp relevant to docking on this rover is a custom install wired on the Raspberry Pi header and powered from Pi `5V`, so do not assume `T=132` controls the real installed lamp here.
- Confirmed custom-light pinout on this rover is `P39 = GND` and `P12 = GPIO18 / BCM18`.
- Quick live test showed the lamp turns on when `GPIO18` is driven high, so the current software convention is active-high.
- `tools/camera_exposure_lock.py` is the current quick way to let auto exposure settle and then freeze the current `exposure_time_absolute` into manual mode before pose or docking tests.
- First useful tag-assisted linear motion result on `2026-05-20`: a backward pulse at `0.14` PWM for `0.40 s` changed the observed tag pose by about `+0.082 m` in camera `+Z`, which is large enough to analyze. Earlier `0.10` PWM micro-pulses were too small to separate from tag-pose noise.
- Later sweep result from the farther aligned setup: `0.16` PWM linear pulses for `0.35 s` gave about `+3.3 cm` backward and `-3.8 cm` forward in camera `+Z`, while turn pulses near `0.40` PWM approached the current tag-visibility limit.
- `tools/dock_heading_step.py` reports:
  - `bearing`: horizontal tag-center bearing in camera coordinates
  - `off_axis`: lateral camera offset from the tag centerline, expressed in the tag frame
  - `heading_vs_normal`: camera forward-axis yaw relative to the dock tag normal, projected into the horizontal tag slice
  - `*_vs_target`: lateral/range error against the saved docked reference pose
- First live heading-step experiments on `2026-05-20` with the new tool showed:
  - Sense-only mode worked at `3264x2448 @ 15 fps` with `--autofocus --auto-exposure manual --exposure-time 220`.
  - A representative sense burst reported about `bearing=-2.14 deg`, `off_axis=-0.023 m`, and range about `0.625 m`, which is roughly `0.127 m` farther than the saved docked reference.
  - One-sided arc pulses at `R=0.16` for `0.25 s` and `R=0.22` for `0.40 s` while holding `L=0` produced no clearly measurable change in the visual heading metrics in that setup, so they should be treated as below the current useful heading-step threshold.
- For repeatable characterization, prefer `tools/tag_motion_collect.py` over ad hoc single runs. It records a timestamped batch directory with `batch.json`, append-only `runs.jsonl`, `summary.csv`, and one `session.json` per probe run so later analysis can combine multiple sessions reliably.

Current rover motion/control notes:

- Runtime motion control should use `/dev/serial0`. Opening `/dev/ttyUSB0` resets the ESP32 and is better treated as a flashing/debug path.
- The custom ESP-IDF firmware uses 460800-baud binary serial, not the stock 115200-baud JSON protocol.
- For USB flashing the custom ESP-IDF firmware, temporarily release the Pi TX line with `raspi-gpio set 14 ip`, flash through `/dev/ttyUSB0`, then restore runtime UART with `raspi-gpio set 14 a0`.
- Body-frame convention is right-handed with `+X` forward, `+Y` left, `+Z` up. Positive yaw / positive `omega_z` means CCW viewed from above.
- Stock `T=1` sign convention on this rover is `L > 0` = left forward, `R > 0` = right forward.
- Quick floor checks on `2026-05-20` found these practical minimums for visible response:
  - forward `+X`: about `0.10` PWM
  - Z turning: about `0.30` PWM gives first visible turn response but is unstable
  - Z turning: about `0.35` PWM is a better practical minimum for turn tests
- These are approximate thresholds only; they will move with floor friction, battery voltage, load, and tire condition.
- IMU use guidance:
  - for short turn pulses, integrated `gz` is more credible than fused yaw `y`
  - for straight pulses, heading disturbance is smaller, but accelerometers are still too noisy for useful distance estimation

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
