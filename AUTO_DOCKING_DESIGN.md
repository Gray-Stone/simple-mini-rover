# Auto-Docking Design

Date: 2026-05-25

## Purpose

This document records the intended docking behavior before implementation details, tuning, or temporary expedients.

If the implementation and this document diverge, treat that as a bug or an explicitly unresolved design decision. Do not silently redefine the behavior in code.

## User Intent

The rover must not behave as if the AprilTag center itself were the final geometric goal.

The key user intent is:

- the rover should not simply point its centerline at the current tag XY location and drive toward it
- the rover should first stage onto a deliberate approach point on the dock centerline
- after reaching that approach point, the rover should align with the dock normal and only then drive straight into the dock
- the final dock region should be entered with small lateral error and small heading error, not with a large offset that happens to intersect the tag

This distinction is fundamental. "Point at the tag" and "be correctly lined up to dock" are separate behaviors.

## Terms

- `dock centerline`: the intended straight-in approach line for the dock
- `dock normal`: the heading that points straight into the dock
- `edge target`: the calibrated visual pre-contact pose in `config/auto_docking/dock_edge_tag_pose.json`
- `approach point`: a staging point on the dock centerline, farther away than the edge target, used to enter the final region in a controlled way
- `contact push`: the low-PWM final motion that continues until charging is detected or a bounded timeout ends

## Coordinate Frame Rule

The planner's centerline model is defined in the dock tag frame, not by the tag's screen position.

- The dock centerline is the tag-frame normal direction.
- The correct lateral reference is the calibrated target camera position expressed in the tag frame.
- A centered rover may still see the tag offset in the image if the camera is mounted off-center or skewed.
- Camera-frame tag translation is useful for reporting, visibility checks, and fallback behavior, but it is not the primary definition of dock centerline.

This requires calibrated geometry. Runs using continuous autofocus or a focus value that does not match the selected camera model and target pose should be treated as bring-up data, not as valid absolute docking geometry.

## Required Behavior

### 1. Starting assumption

- The dock tag is already visible.
- The rover may begin offset, skewed, or farther away.
- The controller is responsible for turning that visible-tag start state into a safe staged final approach.

### 2. Far-field behavior

When the rover is still outside the final approach region, it must reason about an approach point, not only the tag location.

- If the rover is already well centered and nearly aligned, it may drive toward the approach point.
- If the rover is laterally offset, it should plan a waypoint maneuver that moves it toward the dock centerline before the final approach.
- The canonical off-axis maneuver is:
  - turn toward the approach point
  - drive the slanted segment to that approach point
  - turn back to align with the dock normal, unless a deliberate hold-heading variant is chosen

The planner must prefer this staged behavior over "centerline intersects tag XY eventually".

### 3. Near-field behavior

Near the dock, the rover must become more conservative, not less.

- If the rover is too close and still misaligned, it should back off to create room rather than forcing a sharp correction inside the final region.
- The final region should be entered only from a pose that is already close to centered and close to the dock heading.
- A small straight drive is acceptable only when lateral and bearing errors are already within a narrow corridor and the move is effectively a continuation of a valid straight-in approach.

### 4. Final visual alignment

The visual objective before contact is the calibrated edge target, not the AprilTag center itself.

- The rover should compare live pose against the saved edge target.
- "Done" for the visual stage means the rover is in the pre-contact pose envelope.
- Visual completion alone does not imply charging contact.

### 5. Contact behavior

Once the visual edge target is satisfied:

- check whether charging is already present
- if not, execute a bounded low-PWM forward push
- stop that push immediately when charging is detected
- abort or report failure if the push completes without charging and retry policy is exhausted

### 6. Tag loss behavior

- Loss of tag is a fault condition, not a cue to continue blindly.
- A bounded small-angle reacquire scan is acceptable only as a short recovery attempt after recent visibility.
- If reacquisition fails, abort.

### 7. Safety and control constraints

- No wheel-encoder odometry is assumed.
- Straight-line move commands are open-loop lower-control actions corrected only by repeated re-sensing.
- The planner must therefore be geometrically conservative near the dock.
- The system should prefer creating room and re-staging over gambling on a close-in correction.

## Planner Contract

The docking planner should make decisions that fit one of these semantic categories:

- `waypoint`: explicit approach-point staging maneuver
- `drive`: straight continuation when already properly staged
- `turn`: bounded heading/bearing correction when appropriate and safe
- `backoff`: create room before attempting alignment
- `done`: visual pre-contact target reached
- `abort`: no safe continuation

The important contract is that `drive` should not be the default answer to a large off-axis state just because the tag remains visible.

## Explicit Non-Goals

- The rover does not need room-scale search behavior in this controller.
- The rover does not need to treat the raw tag center as a homing beacon.
- The rover does not need to force convergence from an unsafe close-in pose without backing out first.

## Review Rule

Future changes to `tools/auto_dock.py` should be reviewed against this document first:

- Does the planner still distinguish tag visibility from proper dock alignment?
- Does it still contain an explicit approach-point concept?
- Can it still explain why a chosen straight drive is safe?
- Does it avoid entering the final dock region with large lateral or heading error?

If those answers are not clear from the code and logs, the design has drifted and needs correction.
