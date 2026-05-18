# WAVE ROVER command/API notes

The base rover can be controlled without a Raspberry Pi using Wi-Fi AP/web UI, HTTP JSON, USB serial, 40-pin UART serial, or ESP-NOW.

## Default network behavior

- Default AP SSID: `UGV`.
- Default AP password: `12345678`.
- Default AP IP: `192.168.4.1`.
- Web UI includes heartbeat detection. If movement commands stop, the robot stops shortly after.

## Serial behavior

- Serial JSON command link: 115200 baud in Waveshare example.
- pyserial example uses `dsrdtr=None`, then `ser.setRTS(False)` and `ser.setDTR(False)` after opening.
- JSON commands are newline-terminated in the serial example.

## Important movement commands

| Name | JSON | Notes |
|---|---|---|
| Speed/PWM-percentage style control | `{"T":1,"L":0.5,"R":0.5}` | Recommended WAVE ROVER command. `L` and `R` range `-0.5` to `+0.5`. For WAVE ROVER without encoders, `0.5` = 100% PWM on that side, `0.25` = 50% PWM. |
| Raw PWM input | `{"T":11,"L":164,"R":164}` | Debug command. `L`/`R` range `-255` to `+255`. Low absolute PWM may not move the gearmotors. |
| ROS velocity command | `{"T":13,"X":0.1,"Z":0.3}` | Wiki says this is only for UGV01 with encoder, not stock WAVE ROVER. Do not treat as closed-loop cmd_vel on the base rover. |
| Motor PID set | `{"T":2,"P":200,"I":2500,"D":0,"L":255}` | Wiki says only for UGV01 with encoder, not stock WAVE ROVER. |

## Feedback / sensors

| Function | JSON | Notes |
|---|---|---|
| IMU data | `{"T":126}` | Gets heading, geomagnetic field, acceleration, attitudes, temperature, etc. |
| Chassis feedback | `{"T":130}` | Request-response chassis feedback. |
| Serial continuous feedback off/on | `{"T":131,"cmd":0}` / `{"T":131,"cmd":1}` | Continuous feedback useful for ROS-style integrations. Default is off. |
| Serial echo off/on | `{"T":143,"cmd":0}` / `{"T":143,"cmd":1}` | Echo disabled by default. |

## OLED / IO / module commands

| Function | JSON | Notes |
|---|---|---|
| OLED line text | `{"T":3,"lineNum":0,"Text":"putYourTextHere"}` | lineNum 0..3. Overrides default robot status display. |
| Restore OLED status | `{"T":-3}` | Restores robot info display. |
| IO4/IO5 PWM | `{"T":132,"IO4":255,"IO5":255}` | Sets PWM of IO4 and IO5. |
| External module model | `{"T":4,"cmd":0}` | 0 null, 1 RoArm-M2, 3 gimbal. |
| Pan-tilt control | `{"T":133,"X":45,"Y":45,"SPD":0,"ACC":0}` | Only relevant if gimbal/pan-tilt is installed. |

## Safety note

Because stock WAVE ROVER lacks encoders, command values are not true velocity. Add an upper-level watchdog. Send repeated commands at a fixed rate and send zero on control loss.
