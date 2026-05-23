# WAVE ROVER Minimal ESP-IDF Firmware

This ESP-IDF project is the serial-only lower controller bring-up path for the WAVE ROVER ESP32.

The initial command surface is intentionally small:

- `CMD_STOP`
- `CMD_MOVE_REL` with body-frame `x_mm` and relative yaw `z_cdeg`
- `ACK`
- fixed-rate `TELEMETRY`

The current move implementation is a bounded first pass. Linear distance is still a PWM-time estimate. If the onboard QMI8658C gyro starts cleanly, yaw is integrated only while a move is active: the turn phase stops from integrated `gyro_z`, and the drive phase applies a small heading correction around the PWM-time distance estimate. The same dead-reckoning timing estimate plus headroom remains the turn safety limit, so bad gyro data cannot make a relative move run forever.

## Layout

- `main/protocol.*`: fixed-header binary protocol, CRC trailer, stream parser
- `main/imu.*`: direct QMI8658C gyro-only I2C bring-up on ESP32 `GPIO32/33`
- `main/motor.*`: TB6612 direction pins and LEDC PWM
- `main/app_main.c`: single ESP-IDF control task and motion state machine

Generated `build/` and local `sdkconfig` output are ignored. Project defaults live in `sdkconfig.defaults`.

## Wire Protocol

The on-wire frame is:

```text
[wr_header_t][payload bytes][crc16_ccitt]
```

`wr_header_t` is an 8-byte packed header:

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;       // 0x5752
    uint8_t version;      // 1
    uint8_t type;
    uint16_t payload_len; // payload only
    uint16_t seq;
} wr_header_t;
```

Payload bytes are copied into the type-specific packed struct after magic, length, version, and CRC checks. CRC is a wire trailer, not part of a command payload.

`CMD_MOVE_REL` currently carries:

```c
typedef struct __attribute__((packed)) {
    int32_t x_mm;
    int32_t z_cdeg;
    uint16_t max_time_ms;
    uint16_t flags;
} wr_cmd_move_rel_t;
```

`CMD_STOP` has zero payload. Stop is accepted immediately. A move received while another move is active gets a `BUSY` ack rather than being queued.

## Control Model

`app_main()` uses one fixed-rate task:

- drain bytes already buffered by the ESP-IDF UART driver
- decode complete packets and update the current motion state
- step the bounded motion state machine every `5 ms`
- emit state telemetry every `40 ms`

There is no application-level command queue. The ESP-IDF UART driver handles UART buffering below the parser.

Current calibration seeds are deliberately conservative and live in `main/app_main.c`:

- drive PWM: `0.160`
- turn PWM: `0.450`
- linear estimate: `100 mm/s`
- turn estimate: `20 deg/s`
- default move hard cap: `2500 ms`

These are bring-up values, not final motion constants.

The gyro path calibrates its Z-rate bias during boot while the motors are stopped. Keep the rover still for the first short startup window after flashing or reset if gyro-assisted turns are being evaluated.

## Build

With ESP-IDF exported:

```bash
cd firmware/esp32/wave_rover_minimal
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

The firmware runtime UART is ESP32 UART0 at `460800` baud, which is visible through both the USB bridge and the Pi runtime serial path on this rover.

On this rover the Pi `GPIO14/TXD0` runtime UART line also reaches ESP32 UART0 RX. USB flashing through `/dev/ttyUSB0` worked after temporarily releasing that Pi TX pin:

```bash
raspi-gpio set 14 ip
idf.py -p /dev/ttyUSB0 flash
raspi-gpio set 14 a0
```
