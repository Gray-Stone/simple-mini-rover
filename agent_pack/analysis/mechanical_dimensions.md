# Mechanical dimensions

Vendor reference images are included under `source_files/vendor_reference_images/`.

## Rover chassis / wheel dimensions

| Dimension | Value | Source note |
|---|---:|---|
| Overall outline | 194 x 168 x 100 mm | Product spec and dimension image |
| Chassis height | 33.70 mm | Product spec and dimension image |
| Body material | 2 mm 5052 aluminum alloy | Product spec |
| Weight | 860 g | Product spec |
| Tire diameter | 80 mm | Product spec |
| Tire width | 42.50 mm | Product spec and dimension image |
| Running speed | up to 1.25 m/s | Product spec |
| Vertical obstacle ability | 40 mm | Product spec |
| Driving payload | 0.8 kg | Product spec |
| Climbing ability | 22 deg | Product spec |
| Minimum turning radius | 0 m; in-situ rotation | Product spec |

## Top-view dimensions from `WAVE_ROVER-details-size-1.jpg`

| Feature | Value |
|---|---:|
| Full length | 194.00 mm |
| Wheelbase/front-to-rear tire reference | 183.00 mm |
| Inner top chassis reference | 97.00 mm |
| Body/chassis top width reference | 136.00 mm |
| Lower width reference | 183.00 mm |
| Central mounting rectangular region | 58.00 x 49.00 mm |
| Green plate/hole envelope shown on top view | 58.00 x 86.00 mm |
| Rear/front view full height | 100.00 mm |
| Chassis side height | 86.50 mm |
| Chassis ground clearance / lower height marker | 33.70 mm |
| Rear internal bracket width marker | 81.60 mm |

## Mounting plate dimensions from `WAVE_ROVER-details-size-2.jpg`

| Feature | Value |
|---|---:|
| Main long mounting spacing/envelope | 58.00 mm horizontal x 86.00 mm vertical |
| Raspberry-Pi-like mounting pattern shown | 58.00 mm x 49.00 mm |
| Square/gimbal-like central pattern shown | 34.00 mm x 34.00 mm |
| Left accessory/module vertical spacing | 46.80 mm |
| Left offset to main plate reference | 31.92 mm |
| Side bracket vertical heights | 13.50 mm and 12.50 mm |
| Side bracket lower hole spacing | 21.00 mm |

## Main driver board dimensions

| Feature | Value |
|---|---:|
| General Driver for Robots PCB dimensions | 65 x 65 mm |
| General Driver mounting hole spacing | 49 x 58 mm |
| General Driver mounting hole diameter | 3 mm |

## Motor dimensions

| Feature | Value |
|---|---:|
| Motor model | GF12-N20 Motor 12V200rpm Gearbox |
| Rated voltage | 12 V |
| Rated current | 0.055 A |
| Stall/locked-rotor current | 0.45 A |
| Rated torque | 0.09 kg.cm |
| Locked-rotor torque | 0.7 kg.cm |
| Rated output power | 1.5 W |
| No-load speed | 66 +/- 10% RPM |
| Motor size | 34 x 12 mm |
| Output shaft | 4 x 10 mm |

Note: the motor naming is confusing: Waveshare's WAVE ROVER product spec calls the motors "N20 12V 200RPM x4", while the wiki motor-spec table lists `GF12-N20 Motor 12V200rpm Gearbox` and then `No-load speed 66 +/- 10% RPM`. Treat the exact wheel RPM as something to measure if it matters for odometry/speed modeling.
