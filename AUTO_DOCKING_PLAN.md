# Auto-Docking Plan

Date: 2026-05-18

## Goal

Build a basic Raspberry Pi top-level controller that uses the camera view of the dock AprilTag to guide the WAVE ROVER into the dock until charging contact is made.

The starting condition for this milestone is: the dock tag is already visible in the camera frame. The robot may start farther away, skewed, or at a large offset angle, but this milestone does not require room-scale navigation or searching from a place where the tag is not visible.

The dock CAD places the AprilTag center point at a nominal height of `200 mm` above the floor. Use this as an initial design reference only. The implementation should rely on measured calibration data because the real height and pose can shift with manufacturing tolerance, assembly tolerance, glue thickness, tag placement, and dock/robot contact variation.

## Current Hardware Status

- Rover base: WAVE ROVER 4WD chassis with four no-encoder DC motors.
- Motor topology: two motors are wired in parallel per side/channel. The ESP32 controls left/right motor groups, not individual wheel speeds.
- Motor control: stock JSON `T=1` commands are PWM-percentage style commands in `[-0.5, 0.5]`, not measured velocity commands.
- Lower controller: ESP32 running stock Waveshare `WAVE_ROVER_V0.9` firmware.
- Pi-to-ESP32 link: `/dev/serial0` works at 115200 baud with newline-terminated JSON.
- USB flashing path: `/dev/ttyUSB0` works with `esptool`, so custom ESP32 firmware is recoverable if needed.
- Live feedback works over serial. The ESP32 reports battery voltage, current motor command echo, temperature, and IMU attitude fields.
- Camera: USB `Arducam_8mp` is visible through V4L2 as `/dev/video0`; OpenCV can read frames. Raspberry Pi `rpicam` reports no CSI camera.
- Python environment gap: OpenCV is installed system-wide, but AprilTag Python detector packages still need to be installed or added to project requirements.

## ESP32 and IMU Capability

Use the stock ESP32 firmware first.

The current firmware exposes useful IMU data:

- `{"T":126}` returns yaw/pitch/roll plus gyro, accelerometer, magnetometer, and temperature fields.
- `{"T":130}` returns chassis feedback including voltage, motor command echo, yaw/pitch/roll, and temperature.

This is enough for a Pi-side, closed-loop-ish yaw controller:

- Read the starting yaw.
- Send slow differential motor commands with `T=1`.
- Poll yaw from `T=126` or `T=130`.
- Stop when the wrapped yaw delta reaches the target angle.
- Use a timeout, conservative PWM, and a final zero command.

This is not enough for reliable `move X distance` control:

- There are no wheel encoders.
- Motor command values are open-loop PWM, not wheel speed.
- Accelerometer integration will drift badly on this platform because of vibration, bias, slip, and start/stop jolts.

For distance-like movement, use calibrated open-loop time/PWM pulses only as a fallback, and correct position using camera/AprilTag feedback whenever possible.

Custom ESP32 firmware should be deferred unless stock firmware fails a concrete need, such as stop latency, watchdog behavior, command acknowledgement, telemetry rate, or motor ramp control. Even custom firmware will not provide true distance control without encoders or external position feedback.

## Development Process

1. Bring up camera and AprilTag detection.
   - Capture from `/dev/video0` using OpenCV/V4L2.
   - Prefer MJPG modes such as `640x480`, `1280x720`, or another mode that keeps tag detection responsive.
   - Use OpenCV's AprilTag dictionary support first; add a separate AprilTag detector dependency only if OpenCV detection is not reliable enough.
   - Current printed dock tag is `tag16h5` ID `0`; use actual tag size `0.034 m` for pose estimation.
   - Verify tag detection at expected dock distances, skew angles, and lighting.

2. Add calibration tools.
   - Calibrate camera intrinsics using OpenCV.
   - Keep image collection separate from the actual calibration solve. Use a browser preview server on the Pi: low-resolution live preview for positioning, high-resolution still capture for saved calibration images.
   - Current checkerboard is `9 x 7` squares, so use `8 x 6` inner corners for OpenCV. Measure the physical square size before the calibration solve.
   - Save intrinsics and distortion data in a repo-local config file.
   - Record AprilTag family and physical tag size.
   - Add docked-pose calibration: place the robot in confirmed charging-contact position and save the observed tag pose as the target. Seed expectations from the CAD nominal tag center height of `200 mm` above the floor, but never hard-code it as truth.
   - Add simple motor calibration for sign convention, minimum PWM that moves forward, minimum PWM that turns, and safe pulse durations.

3. Build the Pi-side rover control layer.
   - Wrap serial JSON commands to `/dev/serial0`.
   - Use only `T=1` motor commands for normal motion.
   - Provide immediate `stop()` and send zero on process exit.
   - Add a watchdog: stale camera frame, serial failure, tag loss, low voltage, timeout, or user interrupt all command zero.
   - Add an IMU yaw helper for approximate `turn_degrees()` behavior.

4. Implement visual docking control.
   - Estimate tag pose from each camera frame.
   - Compare live tag pose to the calibrated docked pose.
   - Use slow, stepwise control rather than smooth high-rate control.
   - First reduce large heading/lateral errors, then creep forward while correcting yaw, then use short final pulses near contact.
   - Stop immediately if the tag is lost; optionally allow a bounded slow yaw reacquisition only when the tag was seen recently.

5. Test in increasing risk order.
   - Camera-only detection and pose logging.
   - Dry-run docking controller with motors disabled and logged motor commands.
   - Motor sign and stop tests with wheels clear or robot restrained.
   - IMU yaw turn tests at low PWM.
   - Close centered docking attempt.
   - Skewed, offset, and farther visible-tag docking attempts.

## Safety Rules

- Never assume odometry exists.
- Never use encoder-only commands such as `T=13` as true closed-loop velocity control on this no-encoder rover.
- Keep docking PWM conservative until real tests establish safe values.
- Send repeated zero commands on failure paths.
- Require a human-accessible stop method during powered tests.
- Treat camera/AprilTag pose as the primary positioning feedback; treat IMU yaw as secondary heading feedback.

## Acceptance Criteria

- The Pi can detect the dock AprilTag reliably from the intended starting envelope.
- Camera intrinsics and docked tag pose can be calibrated and reloaded.
- The rover can perform approximate yaw turns using IMU feedback.
- The controller can command slow, bounded motor pulses and stop safely on all tested failure paths.
- From any start pose where the dock tag is visible within the tested envelope, the rover can drive into the dock and make charging contact.
