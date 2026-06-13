# Live Visual Servo Side Design

Date: 2026-05-27

## Purpose

This is a sidecar design for exploring actual live visual servoing, separate from
the current staged `tools/auto_dock.py` planner.

The goal is not to replace the planner yet. The goal is to answer whether the
camera, AprilTag detector, serial loop, and low raw-PWM commands can support a
slow continuous controller whose failure mode is small expired motion leases
instead of one larger open-loop step.

## Current Timing Sample

Autofocus night-mode dry-run data was collected with:

```bash
python3 tools/live_visual_servo.py \
  --camera /dev/video1 \
  --duration-s 8 \
  --warmup-frames 3 \
  --command-period-s 0.35 \
  --autofocus \
  --low-light-preset \
  --name autofocus_timing_probe_path
```

Output:

- run directory: `data/live_visual_servo_runs/20260527_063815_autofocus_timing_probe_path`
- requested camera mode: `1280x720@30 MJPG`
- controls: continuous autofocus on, manual exposure `280`, gain `8`
- host control period: `0.35 s`
- proposed PWM lease duration: `450 ms`
- elapsed: `8.34 s`
- cycles: `23`, or `2.76 Hz`
- successful camera reads: `15`, or `1.80 Hz`
- expected tag matches: `6`, or `26%` of cycles and `40%` of successful reads
- median cycle time when frames were read: about `0.124 s`
- median AprilTag detection time: about `0.115 s`
- p90 AprilTag detection time: about `0.130 s`

This sample is not good enough for live motion yet because the camera stream
faulted during the run. Kernel logs around the test show UVC probe/control errors
and the device re-enumerated between `/dev/video0` and `/dev/video1`.

## Control Contract

Each top-level host cycle owns one short motor lease.

- The host loop runs at a target period, currently `350 ms`.
- Each cycle computes a small raw-PWM command from the latest tag pose.
- The firmware command duration is `period + margin`, currently `450 ms`.
- If the host loop stalls, the last command expires in firmware without another
  host action.
- If the host loop is merely slow or the camera drops frames, the rover moves as
  a sequence of short bounded pulses rather than continuing blindly.

The current firmware rejects a new `CMD_PWM` while an old one is active. The
sidecar tool therefore defaults to `STOP` before each new PWM lease when
`--execute` is used. This gives the desired lease-after-next-cycle timeout
without changing firmware preemption semantics.

## Initial PWM Envelope

Use conservative raw PWM for first live tests:

- default forward command: `140 milli`
- near-target forward command: `120 milli`
- maximum wheel command: `220 milli`
- steering correction: `8 milli/deg`, capped at `70 milli`
- default lease: `450 ms`

Existing raw-PWM motion notes suggest `160 milli` for `350 ms` produced about
`20 mm` forward movement in a night mapping run. A single stuck lease at the
current default should therefore be on the order of only a few centimeters, not a
full docking step. Re-check this after any floor, payload, tire, battery, or
lighting change.

## First Controller

The first live controller uses only simple image-pose terms:

- target range and lateral offset come from `config/auto_docking/dock_edge_tag_pose.json`
- lateral error is `live_tag_x - target_tag_x`
- range error is `live_tag_z - target_tag_z`
- bearing error is `atan2(lateral_error, live_tag_z)`
- positive bearing uses the existing `turn_sign=-1` convention so the rover
  turns right when the tag is to camera-right

The first control law is intentionally small:

- no tag: command zero, and with `--execute` send only STOP
- inside range/lateral/bearing deadbands: command zero
- too far: add slow forward PWM
- lateral or bearing error: add differential steering PWM
- large bearing: scale down forward PWM

This is a probing controller, not a complete docking policy. It does not yet
perform search, backoff, waypoint staging, contact push, or charge detection.

## Safety Rules

- Default mode is dry-run logging. Motion requires `--execute`.
- Do not run `--execute` while the camera stream is faulting or re-enumerating.
- Do not run `--execute` unless the tag has a stable recent match rate at the
  same camera mode.
- Keep `--preempt-with-stop` enabled unless firmware is changed to explicitly
  allow PWM replacement.
- If tag detection is lost, command zero and let the current lease expire.
- Treat autofocus pose as bring-up data, not calibrated absolute geometry.

## Tool

The sidecar implementation is `tools/live_visual_servo.py`.

Useful dry-run command:

```bash
python3 tools/live_visual_servo.py --camera /dev/video1 --autofocus --low-light-preset --duration-s 10
```

First motion command, only after camera stability is verified:

```bash
python3 tools/live_visual_servo.py \
  --camera /dev/video1 \
  --autofocus \
  --low-light-preset \
  --duration-s 5 \
  --command-period-s 0.35 \
  --lease-margin-s 0.10 \
  --execute
```
