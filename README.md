# Wave Rover Development

This repo is the clean workspace for the Raspberry Pi 4 plus Waveshare WAVE ROVER/ESP32 stack.

Current contents:

- `agent_pack/`: collected rover docs, schematic crops, command notes, pinout notes, and vendor references.
- `tools/rover_status.py`: small serial status probe for the rover JSON API.
- `tools/requirements.txt`: Python packages used by local ESP32 tooling.
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
- Use the low-resolution browser preview to pose the checkerboard, then click Capture or press Enter in the browser.
- Preview runs at low resolution for responsiveness; each capture briefly switches to high resolution and saves one calibration image.
- The script saves only `images/cal_###.jpg`, `session.json`, and `manifest.jsonl` under `data/camera_calibration/captures/<timestamp>/`.
- Keep continuous autofocus disabled during calibration and docking. The current Arducam reports manual focus value `432`; use the same focus setting for calibration captures and later AprilTag pose estimation unless deliberately recalibrating.
- The actual calibration solve should be run separately against the saved images.
