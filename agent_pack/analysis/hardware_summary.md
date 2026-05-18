# Hardware summary

Target hardware: Waveshare WAVE ROVER base chassis.

## Physical platform

- 4WD metal chassis.
- Four N20 12 V gearbox motors.
- Rubber tires, nylon wheel hubs.
- ESP32-based lower controller/driver board: Waveshare General Driver for Robots.
- 3S 18650 battery/UPS module support.
- No Raspberry Pi, Jetson, camera, lidar, or pan-tilt required for the base rover.
- Stock base WAVE ROVER motor configuration is the **4 DC motors without encoders / two groups** mode. The board supports encoder interfaces, but the WAVE ROVER motor spec and command docs state that its motors are without encoders.

## Control consequence

Do not assume true wheel odometry. Motor commands are open-loop PWM-derived commands. The JSON `T=1` speed command uses left/right values in `[-0.5, +0.5]`, but on WAVE ROVER this maps to a percentage of motor PWM rather than closed-loop wheel velocity.

## Main board

General Driver for Robots board:

- Main MCU: ESP32-WROOM-32 / ESP32-WROOM-32UE.
- Motor driver: TB6612FNG dual H-bridge.
- Power monitor: INA219, I2C address 0x42, 0.01 ohm shunt in demo code.
- IMU sensors: QMI8658C 6-axis IMU + AK09918C 3-axis magnetometer, through I2C/level-shift circuitry.
- Barometer/temperature sensor on schematic: BMP280.
- USB UART: CP2102-class circuits are documented by Waveshare for ESP32 UART and lidar UART paths.
- Serial bus servo interface: ST3215-compatible, UART1 via GPIO18/19 and TX-enable/buffer circuitry.
- SD card slot: SPI on GPIO12/13/14/15.
- OLED/I2C expansion: GPIO32 SDA, GPIO33 SCL.

## What to verify physically on your unit

- Which motor channel is left vs right. The board exposes Motor A and Motor B channels; firmware usually maps them to left/right. If the rover drives backward or turns inverted, swap command signs or channel mapping in software rather than rewiring first.
- Whether your specific board is populated exactly as the public General Driver schematic. The WAVE ROVER uses this board family, but Waveshare revisions may differ.
