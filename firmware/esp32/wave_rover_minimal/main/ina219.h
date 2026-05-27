#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint32_t read_errors;
    bool ready;
} wr_ina219_t;

typedef struct {
    int32_t shunt_uv;
    int32_t bus_mv;
    int32_t current_ma;
} wr_ina219_sample_t;

esp_err_t wr_ina219_init_on_bus(wr_ina219_t *ina219, i2c_master_bus_handle_t bus);
bool wr_ina219_ready(const wr_ina219_t *ina219);
esp_err_t wr_ina219_read_sample(wr_ina219_t *ina219, wr_ina219_sample_t *sample);
