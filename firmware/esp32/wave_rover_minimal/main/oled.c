#include "oled.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"

#define WR_OLED_ADDRESS 0x3C
#define WR_OLED_I2C_HZ 400000
#define WR_OLED_TIMEOUT_MS 40

#define WR_OLED_WIDTH 128
#define WR_OLED_HEIGHT 32
#define WR_OLED_PAGES (WR_OLED_HEIGHT / 8)

static const uint8_t *glyph_rows_for(char ch)
{
    static const uint8_t k_space[7] = {0, 0, 0, 0, 0, 0, 0};
    static const uint8_t k_dash[7] = {0, 0, 0, 0x1F, 0, 0, 0};
    static const uint8_t k_dot[7] = {0, 0, 0, 0, 0, 0x0C, 0x0C};
    static const uint8_t k_0[7] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
    static const uint8_t k_1[7] = {0x04, 0x0C, 0x14, 0x04, 0x04, 0x04, 0x1F};
    static const uint8_t k_2[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
    static const uint8_t k_3[7] = {0x1E, 0x01, 0x01, 0x06, 0x01, 0x01, 0x1E};
    static const uint8_t k_4[7] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
    static const uint8_t k_5[7] = {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E};
    static const uint8_t k_6[7] = {0x07, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E};
    static const uint8_t k_7[7] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
    static const uint8_t k_8[7] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
    static const uint8_t k_9[7] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x1C};
    static const uint8_t k_v[7] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04};

    switch (ch) {
    case '-':
        return k_dash;
    case '.':
        return k_dot;
    case '0':
        return k_0;
    case '1':
        return k_1;
    case '2':
        return k_2;
    case '3':
        return k_3;
    case '4':
        return k_4;
    case '5':
        return k_5;
    case '6':
        return k_6;
    case '7':
        return k_7;
    case '8':
        return k_8;
    case '9':
        return k_9;
    case 'V':
        return k_v;
    case ' ':
    default:
        return k_space;
    }
}

static esp_err_t write_bytes(wr_oled_t *oled, uint8_t control, const uint8_t *bytes, size_t len)
{
    uint8_t tx[1 + WR_OLED_WIDTH] = {0};
    if (len > WR_OLED_WIDTH) {
        return ESP_ERR_INVALID_SIZE;
    }

    tx[0] = control;
    if (len > 0) {
        memcpy(&tx[1], bytes, len);
    }
    return i2c_master_transmit(oled->dev, tx, len + 1, WR_OLED_TIMEOUT_MS);
}

static esp_err_t write_commands(wr_oled_t *oled, const uint8_t *commands, size_t len)
{
    return write_bytes(oled, 0x00, commands, len);
}

static esp_err_t write_data(wr_oled_t *oled, const uint8_t *data, size_t len)
{
    return write_bytes(oled, 0x40, data, len);
}

static void set_pixel(uint8_t *buffer, int x, int y)
{
    if (x < 0 || x >= WR_OLED_WIDTH || y < 0 || y >= WR_OLED_HEIGHT) {
        return;
    }

    buffer[(y / 8) * WR_OLED_WIDTH + x] |= (uint8_t)(1U << (y & 0x7));
}

static void draw_char_scaled(uint8_t *buffer, int x, int y, char ch, int scale)
{
    const uint8_t *rows = glyph_rows_for(ch);
    for (int row = 0; row < 7; ++row) {
        for (int col = 0; col < 5; ++col) {
            if ((rows[row] & (uint8_t)(1U << (4 - col))) == 0) {
                continue;
            }
            for (int dy = 0; dy < scale; ++dy) {
                for (int dx = 0; dx < scale; ++dx) {
                    set_pixel(buffer, x + col * scale + dx, y + row * scale + dy);
                }
            }
        }
    }
}

static void draw_string_scaled(uint8_t *buffer, int x, int y, const char *text, int scale)
{
    const int advance = 6 * scale;
    for (size_t i = 0; text[i] != '\0'; ++i) {
        draw_char_scaled(buffer, x + (int)i * advance, y, text[i], scale);
    }
}

static void format_voltage_text(char *text, size_t text_len, int32_t bus_mv)
{
    if (bus_mv <= 0) {
        snprintf(text, text_len, "--.-V");
        return;
    }

    const int32_t whole_v = bus_mv / 1000;
    const int32_t tenth_v = (bus_mv % 1000) / 100;
    snprintf(text, text_len, "%ld.%ldV", (long)whole_v, (long)tenth_v);
}

static esp_err_t render_voltage_screen(wr_oled_t *oled, int32_t bus_mv)
{
    char text[16] = {0};
    uint8_t buffer[WR_OLED_WIDTH * WR_OLED_PAGES] = {0};

    format_voltage_text(text, sizeof(text), bus_mv);

    const int scale = 2;
    const int advance = 6 * scale;
    const int text_len = (int)strlen(text);
    const int text_w = text_len > 0 ? text_len * advance - scale : 0;
    const int text_h = 7 * scale;
    const int origin_x = (WR_OLED_WIDTH - text_w) / 2;
    const int origin_y = (WR_OLED_HEIGHT - text_h) / 2;
    draw_string_scaled(buffer, origin_x, origin_y, text, scale);

    for (uint8_t page = 0; page < WR_OLED_PAGES; ++page) {
        const uint8_t page_setup[] = {
            (uint8_t)(0xB0 | page),
            0x00,
            0x10,
        };
        ESP_RETURN_ON_ERROR(write_commands(oled, page_setup, sizeof(page_setup)), "wr_oled", "page");
        ESP_RETURN_ON_ERROR(
            write_data(oled, &buffer[page * WR_OLED_WIDTH], WR_OLED_WIDTH),
            "wr_oled",
            "data");
    }
    return ESP_OK;
}

esp_err_t wr_oled_init_on_bus(wr_oled_t *oled, i2c_master_bus_handle_t bus)
{
    memset(oled, 0, sizeof(*oled));
    oled->bus = bus;

    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = WR_OLED_ADDRESS,
        .scl_speed_hz = WR_OLED_I2C_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(oled->bus, &config, &oled->dev), "wr_oled", "add");

    static const uint8_t init_commands[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x1F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x02,
        0xA1,
        0xC8,
        0xDA, 0x02,
        0x81, 0x8F,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0x2E,
        0xAF,
    };
    ESP_RETURN_ON_ERROR(write_commands(oled, init_commands, sizeof(init_commands)), "wr_oled", "init");
    ESP_RETURN_ON_ERROR(render_voltage_screen(oled, 0), "wr_oled", "clear");

    oled->ready = true;
    return ESP_OK;
}

bool wr_oled_ready(const wr_oled_t *oled)
{
    return oled->ready;
}

esp_err_t wr_oled_show_voltage(wr_oled_t *oled, int32_t bus_mv)
{
    if (!oled->ready) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t error = render_voltage_screen(oled, bus_mv);
    if (error != ESP_OK) {
        ++oled->write_errors;
    }
    return error;
}
