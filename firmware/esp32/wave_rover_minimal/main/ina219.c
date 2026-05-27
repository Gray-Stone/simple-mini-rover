#include "ina219.h"

#include <string.h>

#include "esp_check.h"

#define WR_INA219_ADDRESS 0x42
#define WR_INA219_I2C_HZ 400000
#define WR_INA219_TIMEOUT_MS 20

#define WR_INA219_REG_CONFIG 0x00
#define WR_INA219_REG_SHUNT_VOLTAGE 0x01
#define WR_INA219_REG_BUS_VOLTAGE 0x02

#define WR_INA219_CONFIG_BUS_32V 0x2000
#define WR_INA219_CONFIG_GAIN_320MV 0x1800
#define WR_INA219_CONFIG_BADC_12BIT_1S 0x0180
#define WR_INA219_CONFIG_SADC_12BIT_1S 0x0018
#define WR_INA219_CONFIG_MODE_SHUNT_AND_BUS_CONTINUOUS 0x0007
#define WR_INA219_CONFIG_DEFAULT                                                                                 \
    (WR_INA219_CONFIG_BUS_32V | WR_INA219_CONFIG_GAIN_320MV | WR_INA219_CONFIG_BADC_12BIT_1S |                 \
     WR_INA219_CONFIG_SADC_12BIT_1S | WR_INA219_CONFIG_MODE_SHUNT_AND_BUS_CONTINUOUS)

#define WR_INA219_SHUNT_UV_PER_LSB 10
#define WR_INA219_BUS_MV_PER_LSB 4
#define WR_INA219_SHUNT_OHMS_NUM 1
#define WR_INA219_SHUNT_OHMS_DEN 100

static esp_err_t write_reg_u16(wr_ina219_t *ina219, uint8_t reg, uint16_t value)
{
    const uint8_t bytes[3] = {reg, (uint8_t)(value >> 8), (uint8_t)value};
    return i2c_master_transmit(ina219->dev, bytes, sizeof(bytes), WR_INA219_TIMEOUT_MS);
}

static esp_err_t read_reg_u16(wr_ina219_t *ina219, uint8_t reg, uint16_t *value)
{
    uint8_t bytes[2] = {0};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(ina219->dev, &reg, sizeof(reg), bytes, sizeof(bytes), WR_INA219_TIMEOUT_MS),
        "wr_ina219",
        "read");
    *value = (uint16_t)((uint16_t)bytes[0] << 8 | bytes[1]);
    return ESP_OK;
}

esp_err_t wr_ina219_init_on_bus(wr_ina219_t *ina219, i2c_master_bus_handle_t bus)
{
    memset(ina219, 0, sizeof(*ina219));
    ina219->bus = bus;

    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = WR_INA219_ADDRESS,
        .scl_speed_hz = WR_INA219_I2C_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(ina219->bus, &config, &ina219->dev), "wr_ina219", "add");
    ESP_RETURN_ON_ERROR(write_reg_u16(ina219, WR_INA219_REG_CONFIG, WR_INA219_CONFIG_DEFAULT), "wr_ina219", "config");

    ina219->ready = true;
    return ESP_OK;
}

bool wr_ina219_ready(const wr_ina219_t *ina219)
{
    return ina219->ready;
}

esp_err_t wr_ina219_read_sample(wr_ina219_t *ina219, wr_ina219_sample_t *sample)
{
    if (!ina219->ready) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw_shunt = 0;
    uint16_t raw_bus = 0;
    esp_err_t error = read_reg_u16(ina219, WR_INA219_REG_SHUNT_VOLTAGE, &raw_shunt);
    if (error != ESP_OK) {
        ++ina219->read_errors;
        return error;
    }
    error = read_reg_u16(ina219, WR_INA219_REG_BUS_VOLTAGE, &raw_bus);
    if (error != ESP_OK) {
        ++ina219->read_errors;
        return error;
    }

    const int16_t shunt_counts = (int16_t)raw_shunt;
    const int32_t shunt_uv = (int32_t)shunt_counts * WR_INA219_SHUNT_UV_PER_LSB;
    const int32_t bus_mv = (int32_t)((raw_bus >> 3) * WR_INA219_BUS_MV_PER_LSB);
    const int32_t current_ma = (shunt_uv * WR_INA219_SHUNT_OHMS_DEN) / (1000 * WR_INA219_SHUNT_OHMS_NUM);

    sample->shunt_uv = shunt_uv;
    sample->bus_mv = bus_mv;
    sample->current_ma = current_ma;
    return ESP_OK;
}
