# Firmware

ESP32 lower-controller firmware lives under `esp32/`.

- `esp32/wave_rover_minimal/`: serial-only ESP-IDF firmware for bounded relative rover moves.

Generated ESP-IDF build trees and local `sdkconfig` files stay out of this repo. Project defaults belong in each firmware directory as `sdkconfig.defaults`.
