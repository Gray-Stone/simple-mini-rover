This document will incrementally document current milestone.


## Auto-docking.

There is a very basic/simple dock, where I made. When drive into it head on, contact will be made to start charging.

The dock have a april tag on top of it, suppose to be in camera view-able height.

The current CAD places the dock AprilTag center point at a nominal 200 mm height above the floor. Treat this as a design reference only; the practical value may change due to manufacturing, assembly, glue thickness, tag placement, and dock/robot tolerance. The actual tag pose at the docked charging-contact position should be measured during calibration.

The goal is to have a basic top level control software on the pi, using camera feed to send feedback down to the base esp32, and guide it into the dock for proper docking.

### Calibrate

It is ok to have some calibration data saved. and a calibration process done by user once.

The camera intrentic needs calibrating. 

The position of the tag at the robot's docked position will be calibrated once as well.

Some very basic robot dynamic parameter can be calibrated. but they should be not fully truested as they could change.

### Control 

The Pi-esp32 control rate should not be fast, this is to accomedate the slow-ish respndong esp32 firmware right now. 

I do not expect a super smooth or continues docking procedure. so if the slow control rate cause many pause and steps during docking, its fine. 
