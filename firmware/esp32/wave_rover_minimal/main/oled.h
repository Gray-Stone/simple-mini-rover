#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint32_t write_errors;
    bool ready;
} wr_oled_t;

esp_err_t wr_oled_init_on_bus(wr_oled_t *oled, i2c_master_bus_handle_t bus);
bool wr_oled_ready(const wr_oled_t *oled);
esp_err_t wr_oled_show_voltage(wr_oled_t *oled, int32_t bus_mv);
