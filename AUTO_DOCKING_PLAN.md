# Auto-Docking Plan

Date: 2026-05-18

See also: `AUTO_DOCKING_DESIGN.md` for the intended docking behavior. This plan file is for implementation sequencing and assumptions; the design file is the behavior contract.

## Goal

Build a basic Raspberry Pi top-level controller that uses the camera view of the dock AprilTag to guide the WAVE ROVER into the dock until charging contact is made.

The starting condition for this milestone is: the dock tag is already visible in the camera frame. The robot may start farther away, skewed, or at a large offset angle, but this milestone does not require room-scale navigation or searching from a place where the tag is not visible.

The dock CAD places the AprilTag center point at a nominal height of `200 mm` above the floor. Use this as an initial design reference only. The implementation should rely on measured calibration data because the real height and pose can shift with manufacturing tolerance, assembly tolerance, glue thickness, tag placement, and dock/robot contact variation.

## Current Hardware Status

- Rover base: WAVE ROVER 4WD chassis with four no-encoder DC motors.
- Motor topology: two motors are wired in parallel per side/channel. The ESP32 controls left/right motor groups, not individual wheel speeds.
- Motor control: the current minimal firmware exposes bounded relative moves plus raw signed left/right PWM, not measured velocity control.
- Lower controller: ESP32 running the custom minimal firmware under `firmware/esp32/wave_rover_minimal/`.
- Pi-to-ESP32 runtime link: `/dev/serial0` works at `460800` baud with the compact binary protocol.
- USB flashing path: `/dev/ttyUSB0` works with `esptool`; use it for flashing/debug, not normal runtime control.
- Live feedback works over serial. The ESP32 reports telemetry including motion state, gyro Z, voltage, current, and motor command state.
- Camera: USB `Arducam_8mp` is visible through V4L2 as `/dev/video0`; OpenCV can read frames. Raspberry Pi `rpicam` reports no CSI camera.
- Visual pose tooling now uses system OpenCV plus the saved `mrcal` camera model; no separate AprilTag Python dependency is required for the current flow.

## ESP32 and IMU Capability

Use the current minimal ESP32 firmware path.

The current firmware exposes useful telemetry:

- fixed-rate binary `TELEMETRY` packets include gyro Z, commanded PWM, estimated bounded move progress, and power telemetry
- `CMD_MOVE_REL`, `CMD_PWM`, and `CMD_STOP` are the intended host-control primitives

This is enough for a Pi-side, closed-loop-ish yaw controller:

- Read the starting heading state from telemetry or re-sense with the camera.
- Send bounded turn or raw-PWM commands through the minimal protocol.
- Poll streamed telemetry while the move is active.
- Stop when the bounded move completes, or issue explicit `STOP` on any fault or abort condition.
- Use a timeout, conservative PWM, and a final zero command.

Current control findings from floor tests on `2026-05-20`:

- Runtime motion control should use `/dev/serial0`. Opening `/dev/ttyUSB0` resets the ESP32 and is better treated as a flashing/debug path.
- Use rover body-frame convention: `+X` forward, `+Y` left, `+Z` up. Positive yaw / positive `omega_z` / positive Z turn is counter-clockwise viewed from above.
- Stock `T=1` sign convention on this rover is:
  - `L > 0`: left side forward
  - `R > 0`: right side forward
  - Positive Z turn command: `L < 0`, `R > 0`
  - Negative Z turn command: `L > 0`, `R < 0`
- Practical minimum motion levels are roughly:
  - Forward `+X`: `0.10` PWM for first visible motion
  - Z turning: `0.30` PWM for first visible turn response, but unstable
  - Z turning: `0.35` PWM as a better practical minimum for turn tests
- These should be treated as floor-condition-dependent thresholds, not sharp calibrated values. Response near threshold is already weak and inconsistent, especially for turning because of wheel scrub.
- Under aggressive turn pulses, fused yaw `y` is much less trustworthy than integrated `gz`. For short turn estimation, prefer `gz` integration over direct use of absolute yaw.
- Under short straight pulses, heading disturbance is much smaller than under turn-in-place pulses, but accelerometer channels are still too noisy for distance estimation.

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
   - Treat illumination as part of the detection system. Dark-scene tests already showed that auto exposure can overreact when only the tag is brightly lit, causing the tag to wash out and detection to fail.

2. Add calibration tools.
   - Calibrate camera intrinsics using OpenCV.
   - Keep image collection separate from the actual calibration solve. Use a browser preview server on the Pi: low-resolution live preview for positioning, high-resolution still capture for saved calibration images.
   - Current checkerboard is `9 x 7` squares, so use `8 x 6` inner corners for OpenCV. Measure the physical square size before the calibration solve.
   - The current pose-estimation flow loads the saved `mrcal` model directly from the calibration session output.
   - Record AprilTag family and physical tag size.
   - Add docked-pose calibration: place the robot in confirmed charging-contact position and save the observed tag pose as the target. The current utility writes this to `config/auto_docking/docked_tag_pose.json`. Seed expectations from the CAD nominal tag center height of `200 mm` above the floor, but never hard-code it as truth.
   - Keep a separate visual edge-alignment target for the final pre-contact pose. The current execute path defaults to `config/auto_docking/dock_edge_tag_pose.json`, then finishes contact with a monitored push rather than assuming the visual edge pose itself means charging.
   - Add simple motor calibration for sign convention, minimum PWM that moves forward, minimum PWM that turns, and safe pulse durations.

3. Build the Pi-side rover control layer.
   - Wrap the current minimal ESP32 binary serial protocol on `/dev/serial0`.
   - Use `CMD_MOVE_REL`, `CMD_PWM`, and `CMD_STOP` as the normal motion/control primitives.
   - Provide immediate `stop()` and send zero on process exit.
   - Add a watchdog: stale camera frame, serial failure, tag loss, low voltage, timeout, or user interrupt all command zero.
   - Add an IMU yaw helper for approximate `turn_degrees()` behavior.
   - Encode the rover body-frame convention directly in the control layer: `+X` forward, positive Z turn = CCW/left.
   - Respect the current observed deadbands: forward commands below about `0.10` PWM may do nothing, and turn commands below about `0.30` to `0.35` PWM may be too weak or inconsistent to use.

4. Implement visual docking control.
   - Estimate tag pose from each camera frame. The current utility uses the OpenCV camera frame: `+X` right in image, `+Y` down in image, `+Z` forward from camera.
   - Compare live tag pose to the calibrated docked pose saved from the docked-pose calibration step.
   - Keep geometry-sensitive camera work at locked manual focus `350` unless a different focus has its own matching calibration.
   - Daytime can use auto exposure. Night should use fixed `exposure_time_absolute`, with `280` as the default and `300` or `320` as acceptable alternatives.
   - The rover's onboard light is unplugged, so assume external/manual lighting only.
   - Use slow, stepwise control rather than smooth high-rate control.
   - First reduce large heading/lateral errors, then creep forward while correcting yaw, then switch to a slow low-PWM final push near contact.
   - During that final push, monitor INA219 telemetry continuously and send zero immediately once charging is detected instead of waiting for the full push duration to expire.
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
- The final docking stage can hold a slow forward push into the dock and interrupt that push immediately when INA219 telemetry indicates charging.
- From any start pose where the dock tag is visible within the tested envelope, the rover can drive into the dock and make charging contact.
