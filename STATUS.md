# Current Status

Date: 2026-05-18

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
  - Physical square size still needs to be measured before running the calibration solve.
- Camera calibration capture workflow:
  - Use `tools/camera_cal_server.py --camera /dev/video0 --host 0.0.0.0 --port 8080`.
  - Preview is browser-based and low-resolution; saved calibration frames are high-resolution JPEGs.
  - Capture sessions intentionally save only full-resolution images plus `session.json` and `manifest.jsonl`.
- Camera focus:
  - Current UVC controls report `focus_automatic_continuous=0`, so continuous autofocus is already disabled.
  - Current manual `focus_absolute` value is `432` on the Arducam.
  - Calibration and docking should use the same locked focus value; changing focus after calibration can change effective intrinsics enough to hurt pose accuracy.
- Pi-side I2C was not reliable during probing and is not the preferred voltage source. Use ESP32 serial feedback for rover voltage.
- Old development folders were moved to `../archive` .
- Keep generated Python environments out of git. Recreate the tool venv with:

```bash
python3 -m venv tools/.venv
tools/.venv/bin/python -m pip install -r tools/requirements.txt
```
