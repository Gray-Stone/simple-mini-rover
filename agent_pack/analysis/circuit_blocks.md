# Circuit block summary

## Power input and regulation

- Main input: DC 7-13 V on the General Driver board; WAVE ROVER base uses a 3S lithium pack/UPS path and 12.6 V charger input.
- Main motor/bus-servo power net: `DC_IN` / raw input domain.
- 5 V regulator block: labeled 5V-5A / 5V Power for RPi/Jetson nano in the schematic. This is intended to power an upper computer, but power budgeting matters if a Raspberry Pi, camera, lights, servos, or lidar are added.
- 3.3 V rails power ESP32 logic, TB6612 logic, I2C peripherals, and sensor-level circuitry.
- INA219 monitors input voltage/current via a 0.01 ohm shunt in the demo/schematic path.

## MCU and programming

- ESP32-WROOM-32/32UE module.
- USB Type-C connector for ESP32 UART/upload.
- Auto-program circuit using DTR/RTS to control EN and GPIO0.
- Reset and boot/download buttons are present.

## Motor drive

- TB6612FNG dual H-bridge.
- Two no-encoder motors are connected in parallel on Motor A channel and two on Motor B channel.
- Motor A and B each have one PWM pin and two direction pins.
- STBY is tied to 3.3 V in the schematic crop; there is no separate GPIO standby control shown.

## Sensors and bus peripherals

- INA219 at I2C address 0x42 for battery/input voltage/current.
- QMI8658C 6-axis IMU and AK09918C magnetometer for 9-axis attitude/heading data.
- BMP280 appears in the schematic sensor block.
- SSD1306 OLED is connected over the same I2C expansion bus in demos.
- MicroSD is SPI over GPIO12/13/14/15.

## Host-computer interface

- 40-pin header exposes power, I2C, serial, and grounds for Raspberry Pi/Jetson-style host integration.
- Wiki describes host communication through serial JSON commands over the 40-pin UART or through USB serial if the board is disassembled/accessed.
- When using pyserial, set baud 115200 and force `setRTS(False)` and `setDTR(False)` after opening. This avoids the auto-program circuit holding the ESP32 in reset/download mode.

## External modules

- ST3215 serial bus servo interface can control a chain of bus servos.
- PWM servo signal interface is demonstrated on GPIO4.
- IO4/IO5 can be controlled by JSON command `T=132`.
