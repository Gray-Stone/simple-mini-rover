# ESP32 / General Driver pinout notes

Derived from the General Driver for Robots schematic PDF and Waveshare tutorial code snippets.

## TB6612FNG motor driver pins

The motor-without-encoder tutorial explicitly defines these pins:

| Function | ESP32 GPIO | TB6612FNG pin/signal | Notes |
|---|---:|---|---|
| Motor A PWM | GPIO25 | PWMA / schematic label `PMMA`, signal `S0` | LEDC PWM channel A in tutorial |
| Motor A direction 2 | GPIO17 | AIN2, signal `S1` | Direction pin |
| Motor A direction 1 | GPIO21 | AIN1, signal `S2` | Direction pin |
| Motor B direction 1 | GPIO22 | BIN1, signal `S3` | Direction pin |
| Motor B direction 2 | GPIO23 | BIN2, signal `S4` | Direction pin |
| Motor B PWM | GPIO26 | PWMB, signal `S5` | LEDC PWM channel B in tutorial |
| Standby | tied to 3.3 V | STBY | Not MCU controlled in schematic crop |
| Motor supply | DC_IN | VM1/VM2/VM3 | Raw motor/battery input path |
| Logic supply | 3.3 V | Vcc | TB6612 logic supply |

The schematic shows two 2-pin no-encoder motor connectors wired in parallel per channel:

| Connector | Nets | Channel |
|---|---|---|
| MOTOR-A1 | MA1 / MA2 | Motor A channel |
| MOTOR-A2 | MA1 / MA2 | Motor A channel |
| MOTOR-B1 | MB1 / MB2 | Motor B channel |
| MOTOR-B2 | MB1 / MB2 | Motor B channel |

There are also 6-pin motor-with-encoder headers:

| Header | Power / motor nets | Encoder-like nets | ESP32 pins |
|---|---|---|---|
| H3, Motor A encoder-capable header | MA1, MA2, 3V3, GND | A_C1, A_C2 | A_C1=GPIO34, A_C2=GPIO35 |
| H4, Motor B encoder-capable header | MB1, MB2, 3V3, GND | B_C1, B_C2 | B_C1=GPIO27, B_C2=GPIO16 |

For WAVE ROVER stock N20 motors, the no-encoder 2-pin motor path is the relevant one.

## I2C / OLED / INA219 / IMU

| Signal | ESP32 GPIO | Known attached devices / notes |
|---|---:|---|
| IIC_SDA / SDA | GPIO32 | INA219 power monitor, OLED expansion, IMU-related I2C/level-shift net |
| IIC_SCL / SCL | GPIO33 | INA219 power monitor, OLED expansion, IMU-related I2C/level-shift net |
| INA219 address | - | 0x42 in Waveshare demo code |
| OLED demo address | - | SSD1306 128x32, address 0x3C in Waveshare demo code |

## SD card

| SD/SPI signal | ESP32 GPIO |
|---|---:|
| SD_CS | GPIO15 |
| SPI_MO / MOSI | GPIO13 |
| SPI_CK / SCK | GPIO14 |
| SPI_SO / MISO | GPIO12 |

## Serial bus servo / UART1

| Function | ESP32 GPIO | Notes |
|---|---:|---|
| U1RXD / S_RXD | GPIO18 | ST3215 serial bus servo RX in tutorial |
| U1TXD / S_TXD | GPIO19 | ST3215 serial bus servo TX in tutorial |
| TXEN / DATA buffer | schematic control circuitry | Half-duplex bus-servo data path uses extra buffer/enable circuitry; do not assume direct single-wire UART without checking firmware/schematic. |

## USB / host serial

| Function | ESP32 signal | Notes |
|---|---|---|
| ESP32 USB serial | U0TX / U0RX | Used for USB control/upload. Wiki serial demo uses 115200 baud and explicitly clears RTS/DTR after opening. |
| 40-pin host UART | P_TX / P_RX | Intended for Raspberry Pi/Jetson host-to-ESP32 communication. Exact physical pin orientation should be confirmed from the board silkscreen/schematic. |

## Misc / expansion

| Signal | ESP32 GPIO | Notes |
|---|---:|---|
| IO4 | GPIO4 | PWM servo demo uses GPIO4. Also appears on expansion/pan-tilt-style header. |
| IO5 | GPIO5 | Extra GPIO/PWM-style expansion in schematic and JSON command docs. |
| IO0 | GPIO0 | Boot/download-related strap; avoid as a casual external signal unless you understand boot mode implications. |
