#include "motor.h"

#include <stdbool.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"

#define WR_PWM_MAX UINT32_C(1023)

static const gpio_num_t k_left_pwm = GPIO_NUM_25;
static const gpio_num_t k_left_in2 = GPIO_NUM_17;
static const gpio_num_t k_left_in1 = GPIO_NUM_21;
static const gpio_num_t k_right_in1 = GPIO_NUM_22;
static const gpio_num_t k_right_in2 = GPIO_NUM_23;
static const gpio_num_t k_right_pwm = GPIO_NUM_26;

static int16_t clamp_milli(int16_t value)
{
    if (value > 1000) {
        return 1000;
    }
    if (value < -1000) {
        return -1000;
    }
    return value;
}

static void set_side(
    gpio_num_t in1,
    gpio_num_t in2,
    ledc_channel_t channel,
    int16_t milli)
{
    const int16_t command = clamp_milli(milli);
    const bool forward = command >= 0;
    const uint32_t duty = (uint32_t)abs(command) * WR_PWM_MAX / 1000;

    gpio_set_level(in1, forward ? 1 : 0);
    gpio_set_level(in2, forward ? 0 : 1);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel));
}

void wr_motor_init(void)
{
    const gpio_config_t direction_pins = {
        .pin_bit_mask = BIT64(k_left_in1) | BIT64(k_left_in2) | BIT64(k_right_in1) | BIT64(k_right_in2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&direction_pins));

    const ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    const ledc_channel_config_t left_pwm = {
        .gpio_num = k_left_pwm,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_pwm));

    const ledc_channel_config_t right_pwm = {
        .gpio_num = k_right_pwm,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_pwm));
    wr_motor_stop();
}

void wr_motor_set_milli(int16_t left_milli, int16_t right_milli)
{
    set_side(k_left_in1, k_left_in2, LEDC_CHANNEL_0, left_milli);
    set_side(k_right_in1, k_right_in2, LEDC_CHANNEL_1, right_milli);
}

void wr_motor_stop(void)
{
    wr_motor_set_milli(0, 0);
}
