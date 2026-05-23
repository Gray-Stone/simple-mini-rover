#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    int32_t gyro_z_bias_cdeg_s;
    uint32_t read_errors;
    uint8_t address;
    bool ready;
} wr_imu_t;

esp_err_t wr_imu_init(wr_imu_t *imu);
bool wr_imu_ready(const wr_imu_t *imu);
esp_err_t wr_imu_read_yaw_rate_cdeg_s(wr_imu_t *imu, int32_t *rate_cdeg_s);
