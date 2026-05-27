#include "imu.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WR_QMI_I2C I2C_NUM_0
#define WR_QMI_SDA GPIO_NUM_32
#define WR_QMI_SCL GPIO_NUM_33
#define WR_QMI_I2C_HZ 400000
#define WR_QMI_TIMEOUT_MS 20

#define WR_QMI_ADDRESS_LOW 0x6A
#define WR_QMI_ADDRESS_HIGH 0x6B
#define WR_QMI_WHO_AM_I 0x00
#define WR_QMI_CTRL1 0x02
#define WR_QMI_CTRL3 0x04
#define WR_QMI_CTRL7 0x08
#define WR_QMI_GZ_L 0x3F

#define WR_QMI_WHO_AM_I_VALUE 0x05
#define WR_QMI_CTRL1_AUTO_INCREMENT 0x60
#define WR_QMI_CTRL3_GYRO_512_DPS_250_HZ 0x55
#define WR_QMI_CTRL7_GYRO_ONLY 0x02
#define WR_QMI_GYRO_LSB_PER_DPS 64
#define WR_QMI_BIAS_SAMPLES 64
#define WR_QMI_BIAS_MIN_SAMPLES 24
#define WR_QMI_BIAS_SAMPLE_MS 5
#define WR_QMI_GYRO_WAKE_MS 100

static esp_err_t write_reg(wr_imu_t *imu, uint8_t reg, uint8_t value)
{
    const uint8_t bytes[2] = {reg, value};
    return i2c_master_transmit(imu->dev, bytes, sizeof(bytes), WR_QMI_TIMEOUT_MS);
}

static esp_err_t read_reg(wr_imu_t *imu, uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(imu->dev, &reg, sizeof(reg), out, len, WR_QMI_TIMEOUT_MS);
}

static esp_err_t add_device(wr_imu_t *imu, uint8_t address)
{
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = WR_QMI_I2C_HZ,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(imu->bus, &config, &imu->dev), "wr_imu", "add");

    uint8_t who_am_i = 0;
    const esp_err_t read_error = read_reg(imu, WR_QMI_WHO_AM_I, &who_am_i, sizeof(who_am_i));
    if (read_error == ESP_OK && who_am_i == WR_QMI_WHO_AM_I_VALUE) {
        imu->address = address;
        return ESP_OK;
    }

    i2c_master_bus_rm_device(imu->dev);
    imu->dev = NULL;
    return read_error == ESP_OK ? ESP_ERR_NOT_FOUND : read_error;
}

static esp_err_t read_raw_yaw_rate_cdeg_s(wr_imu_t *imu, int32_t *rate_cdeg_s)
{
    uint8_t raw[2] = {0};
    ESP_RETURN_ON_ERROR(read_reg(imu, WR_QMI_GZ_L, raw, sizeof(raw)), "wr_imu", "gyro");

    const int16_t gyro_z = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
    *rate_cdeg_s = (int32_t)gyro_z * 100 / WR_QMI_GYRO_LSB_PER_DPS;
    return ESP_OK;
}

static esp_err_t calibrate_bias(wr_imu_t *imu)
{
    int64_t sum = 0;
    uint32_t samples = 0;

    for (uint32_t i = 0; i < WR_QMI_BIAS_SAMPLES; ++i) {
        int32_t rate_cdeg_s = 0;
        if (read_raw_yaw_rate_cdeg_s(imu, &rate_cdeg_s) == ESP_OK) {
            sum += rate_cdeg_s;
            ++samples;
        }
        vTaskDelay(pdMS_TO_TICKS(WR_QMI_BIAS_SAMPLE_MS));
    }

    if (samples < WR_QMI_BIAS_MIN_SAMPLES) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    imu->gyro_z_bias_cdeg_s = (int32_t)(sum / samples);
    return ESP_OK;
}

esp_err_t wr_i2c_bus_new(i2c_master_bus_handle_t *bus)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = WR_QMI_I2C,
        .sda_io_num = WR_QMI_SDA,
        .scl_io_num = WR_QMI_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_config, bus);
}

esp_err_t wr_imu_init_on_bus(wr_imu_t *imu, i2c_master_bus_handle_t bus)
{
    memset(imu, 0, sizeof(*imu));
    imu->bus = bus;

    esp_err_t error = add_device(imu, WR_QMI_ADDRESS_HIGH);
    if (error != ESP_OK) {
        error = add_device(imu, WR_QMI_ADDRESS_LOW);
    }
    if (error != ESP_OK) {
        return error;
    }

    ESP_RETURN_ON_ERROR(write_reg(imu, WR_QMI_CTRL7, 0x00), "wr_imu", "disable");
    ESP_RETURN_ON_ERROR(write_reg(imu, WR_QMI_CTRL1, WR_QMI_CTRL1_AUTO_INCREMENT), "wr_imu", "ctrl1");
    ESP_RETURN_ON_ERROR(write_reg(imu, WR_QMI_CTRL3, WR_QMI_CTRL3_GYRO_512_DPS_250_HZ), "wr_imu", "ctrl3");
    ESP_RETURN_ON_ERROR(write_reg(imu, WR_QMI_CTRL7, WR_QMI_CTRL7_GYRO_ONLY), "wr_imu", "enable");
    vTaskDelay(pdMS_TO_TICKS(WR_QMI_GYRO_WAKE_MS));
    ESP_RETURN_ON_ERROR(calibrate_bias(imu), "wr_imu", "bias");

    imu->ready = true;
    return ESP_OK;
}

esp_err_t wr_imu_init(wr_imu_t *imu)
{
    i2c_master_bus_handle_t bus = NULL;
    ESP_RETURN_ON_ERROR(wr_i2c_bus_new(&bus), "wr_imu", "bus");
    return wr_imu_init_on_bus(imu, bus);
}

bool wr_imu_ready(const wr_imu_t *imu)
{
    return imu->ready;
}

esp_err_t wr_imu_read_yaw_rate_cdeg_s(wr_imu_t *imu, int32_t *rate_cdeg_s)
{
    if (!imu->ready) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t raw_rate_cdeg_s = 0;
    const esp_err_t error = read_raw_yaw_rate_cdeg_s(imu, &raw_rate_cdeg_s);
    if (error != ESP_OK) {
        ++imu->read_errors;
        return error;
    }

    *rate_cdeg_s = raw_rate_cdeg_s - imu->gyro_z_bias_cdeg_s;
    return ESP_OK;
}
