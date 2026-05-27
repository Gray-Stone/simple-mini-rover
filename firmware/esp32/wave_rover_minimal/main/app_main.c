#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ina219.h"
#include "imu.h"
#include "motor.h"
#include "oled.h"
#include "protocol.h"

#define WR_UART UART_NUM_0
#define WR_UART_BAUD 460800
#define WR_RX_BUF_SIZE 1024
#define WR_CONTROL_PERIOD_MS 5
#define WR_POWER_PERIOD_MS 40
#define WR_DISPLAY_PERIOD_MS 1000
#define WR_STATUS_TASK_STACK_SIZE 4096
#define WR_STATUS_TASK_PRIORITY tskIDLE_PRIORITY
#define WR_TELEMETRY_PERIOD_US UINT64_C(40000)
#define WR_DRIVE_DEFAULT_MILLI 400
#define WR_DRIVE_MIN_MAPPED_MILLI 250
#define WR_DRIVE_MAX_MAPPED_MILLI 700
#define WR_TURN_DEFAULT_MILLI 650
#define WR_TURN_MIN_MILLI 450
#define WR_TURN_MAX_MILLI 900
#define WR_TURN_CDEG_PER_S 900
#define WR_MIN_PHASE_MS 20
#define WR_TURN_IMU_HEADROOM_MS 350
#define WR_TURN_REACHED_CDEG 80
#define WR_DRIVE_HEADING_CORRECTION_MAX_MILLI 80
#define WR_DRIVE_HEADING_CORRECTION_DIV 4
#define WR_MOVE_DEADLINE_HEADROOM_MS 500
#define WR_MAX_PWM_MILLI 1000
#define WR_MAX_PWM_DURATION_MS 5000

typedef struct {
    wr_motion_phase_t phase;
    uint16_t active_seq;
    wr_cmd_move_rel_t target;
    int32_t x_est_mm;
    int32_t z_est_cdeg;
    uint64_t command_started_us;
    uint64_t command_deadline_us;
    uint64_t phase_started_us;
    uint64_t last_gyro_us;
    uint32_t phase_duration_ms;
    uint32_t phase_limit_ms;
    bool timed_out;
    bool use_imu;
    int16_t left_milli;
    int16_t right_milli;
    int16_t drive_base_milli;
} motion_state_t;

typedef struct {
    int16_t pwm_milli;
    uint16_t fwd_mm_s;
    uint16_t rev_mm_s;
    uint16_t fwd_startup_ms;
    uint16_t rev_startup_ms;
} drive_calibration_t;

typedef struct {
    wr_parser_t parser;
    i2c_master_bus_handle_t i2c_bus;
    motion_state_t motion;
    wr_imu_t imu;
    wr_ina219_t ina219;
    wr_oled_t oled;
    int32_t gyro_z_cdeg_s;
    int32_t bus_mv;
    int32_t current_ma;
    int32_t shunt_uv;
    uint16_t tx_seq;
    uint64_t next_telemetry_us;
} app_state_t;

static app_state_t g_app;
static const char *TAG = "wave_rover";

static uint32_t abs_i32(int32_t value)
{
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return (uint32_t)(value < 0 ? -value : value);
}

static uint32_t estimate_ms(uint32_t magnitude, uint32_t per_second)
{
    if (magnitude == 0) {
        return 0;
    }
    const uint32_t duration = (magnitude * 1000 + per_second - 1) / per_second;
    return duration < WR_MIN_PHASE_MS ? WR_MIN_PHASE_MS : duration;
}

static int32_t interpolate_i32(int32_t x, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    if (x1 == x0) {
        return y0;
    }
    return y0 + (int32_t)((int64_t)(y1 - y0) * (int64_t)(x - x0) / (int64_t)(x1 - x0));
}

static bool lookup_drive_calibration(int16_t abs_pwm, bool forward, uint32_t *speed_mm_s, uint32_t *startup_ms)
{
    static const drive_calibration_t table[] = {
        {.pwm_milli = 250, .fwd_mm_s = 244, .rev_mm_s = 250, .fwd_startup_ms = 73, .rev_startup_ms = 56},
        {.pwm_milli = 300, .fwd_mm_s = 317, .rev_mm_s = 337, .fwd_startup_ms = 53, .rev_startup_ms = 64},
        {.pwm_milli = 350, .fwd_mm_s = 388, .rev_mm_s = 414, .fwd_startup_ms = 39, .rev_startup_ms = 55},
        {.pwm_milli = 400, .fwd_mm_s = 474, .rev_mm_s = 496, .fwd_startup_ms = 40, .rev_startup_ms = 41},
        {.pwm_milli = 450, .fwd_mm_s = 551, .rev_mm_s = 568, .fwd_startup_ms = 38, .rev_startup_ms = 31},
        {.pwm_milli = 500, .fwd_mm_s = 635, .rev_mm_s = 652, .fwd_startup_ms = 37, .rev_startup_ms = 40},
        {.pwm_milli = 550, .fwd_mm_s = 673, .rev_mm_s = 678, .fwd_startup_ms = 31, .rev_startup_ms = 31},
        {.pwm_milli = 600, .fwd_mm_s = 730, .rev_mm_s = 750, .fwd_startup_ms = 26, .rev_startup_ms = 29},
        {.pwm_milli = 650, .fwd_mm_s = 801, .rev_mm_s = 827, .fwd_startup_ms = 31, .rev_startup_ms = 37},
        {.pwm_milli = 700, .fwd_mm_s = 891, .rev_mm_s = 897, .fwd_startup_ms = 32, .rev_startup_ms = 31},
    };
    const size_t count = sizeof(table) / sizeof(table[0]);
    if (abs_pwm < table[0].pwm_milli || abs_pwm > table[count - 1].pwm_milli) {
        return false;
    }

    for (size_t i = 0; i + 1 < count; ++i) {
        const drive_calibration_t *lo = &table[i];
        const drive_calibration_t *hi = &table[i + 1];
        if (abs_pwm < lo->pwm_milli || abs_pwm > hi->pwm_milli) {
            continue;
        }
        const uint16_t lo_speed = forward ? lo->fwd_mm_s : lo->rev_mm_s;
        const uint16_t hi_speed = forward ? hi->fwd_mm_s : hi->rev_mm_s;
        const uint16_t lo_startup = forward ? lo->fwd_startup_ms : lo->rev_startup_ms;
        const uint16_t hi_startup = forward ? hi->fwd_startup_ms : hi->rev_startup_ms;

        *speed_mm_s = (uint32_t)interpolate_i32(abs_pwm, lo->pwm_milli, lo_speed, hi->pwm_milli, hi_speed);
        *startup_ms = (uint32_t)interpolate_i32(abs_pwm, lo->pwm_milli, lo_startup, hi->pwm_milli, hi_startup);
        return true;
    }
    return false;
}

static uint32_t estimate_drive_ms(uint32_t magnitude, int16_t signed_pwm)
{
    if (magnitude == 0) {
        return 0;
    }
    uint32_t speed_mm_s = 0;
    uint32_t startup_ms = 0;
    const int16_t abs_pwm = signed_pwm < 0 ? -signed_pwm : signed_pwm;
    if (!lookup_drive_calibration(abs_pwm, signed_pwm > 0, &speed_mm_s, &startup_ms)) {
        return 0;
    }
    return estimate_ms(magnitude, speed_mm_s) + startup_ms;
}

static bool motion_active(const motion_state_t *motion)
{
    return motion->phase == WR_PHASE_TURN || motion->phase == WR_PHASE_DRIVE || motion->phase == WR_PHASE_PWM;
}

static void send_packet(uint8_t type, uint16_t seq, const void *payload, uint16_t payload_len)
{
    uint8_t frame[WR_MAX_FRAME];
    const size_t frame_len = wr_encode_frame(frame, sizeof(frame), type, seq, payload, payload_len);
    if (frame_len > 0) {
        uart_write_bytes(WR_UART, frame, frame_len);
    }
}

static void send_ack(uint16_t seq, uint8_t command_type, wr_ack_status_t status, uint16_t detail)
{
    const wr_ack_t ack = {
        .status = status,
        .command_type = command_type,
        .detail = detail,
    };
    send_packet(WR_PACKET_ACK, seq, &ack, sizeof(ack));
}

static void apply_output(motion_state_t *motion, int16_t left_milli, int16_t right_milli)
{
    motion->left_milli = left_milli;
    motion->right_milli = right_milli;
    wr_motor_set_milli(left_milli, right_milli);
}

static void stop_motion(motion_state_t *motion, wr_motion_phase_t terminal_phase)
{
    motion->phase = terminal_phase;
    apply_output(motion, 0, 0);
}

static void start_drive_phase(motion_state_t *motion, uint64_t now_us)
{
    motion->phase = WR_PHASE_DRIVE;
    motion->phase_started_us = now_us;
    motion->drive_base_milli = motion->target.x_mm >= 0 ? motion->target.drive_milli : -motion->target.drive_milli;
    motion->phase_duration_ms = estimate_drive_ms(abs_i32(motion->target.x_mm), motion->drive_base_milli);
    motion->phase_limit_ms = motion->phase_duration_ms;
    if (motion->phase_duration_ms == 0) {
        stop_motion(motion, WR_PHASE_DONE);
        return;
    }

    apply_output(motion, motion->drive_base_milli, motion->drive_base_milli);
}

static void start_move(motion_state_t *motion, uint16_t seq, const wr_cmd_move_rel_t *target)
{
    memset(motion, 0, sizeof(*motion));
    motion->active_seq = seq;
    motion->target = *target;

    const uint64_t now_us = esp_timer_get_time();
    const int16_t drive_pwm = target->x_mm >= 0 ? target->drive_milli : -target->drive_milli;
    const uint32_t turn_estimate_ms = estimate_ms(abs_i32(target->z_cdeg), WR_TURN_CDEG_PER_S);
    const uint32_t drive_estimate_ms = estimate_drive_ms(abs_i32(target->x_mm), drive_pwm);
    const uint32_t estimated_move_ms = turn_estimate_ms + drive_estimate_ms + WR_MOVE_DEADLINE_HEADROOM_MS;
    const uint32_t target_cap_ms = target->max_time_ms == 0 ? estimated_move_ms : target->max_time_ms;
    motion->command_started_us = now_us;
    motion->command_deadline_us = now_us + (uint64_t)target_cap_ms * 1000;

    motion->phase_duration_ms = turn_estimate_ms;
    motion->phase_limit_ms = motion->phase_duration_ms + motion->phase_duration_ms / 2 + WR_TURN_IMU_HEADROOM_MS;
    if (motion->phase_duration_ms == 0) {
        start_drive_phase(motion, now_us);
        return;
    }

    motion->phase = WR_PHASE_TURN;
    motion->phase_started_us = now_us;
    const int16_t pwm = target->z_cdeg >= 0 ? target->turn_milli : -target->turn_milli;
    apply_output(motion, -pwm, pwm);
}

static void start_pwm(motion_state_t *motion, uint16_t seq, const wr_cmd_pwm_t *target)
{
    memset(motion, 0, sizeof(*motion));
    motion->active_seq = seq;
    motion->phase = WR_PHASE_PWM;
    motion->phase_duration_ms = target->duration_ms;

    const uint64_t now_us = esp_timer_get_time();
    motion->command_started_us = now_us;
    motion->phase_started_us = now_us;
    motion->command_deadline_us =
        now_us + ((uint64_t)target->duration_ms + WR_CONTROL_PERIOD_MS * 4) * 1000;

    apply_output(motion, target->left_milli, target->right_milli);
}

static bool turn_reached(const motion_state_t *motion)
{
    const int32_t remaining = motion->target.z_cdeg - motion->z_est_cdeg;
    if (abs_i32(remaining) <= WR_TURN_REACHED_CDEG) {
        return true;
    }
    return motion->target.z_cdeg > 0 ? remaining < 0 : remaining > 0;
}

static int32_t linear_progress(int32_t target, uint64_t elapsed_us, uint32_t duration_ms)
{
    if (duration_ms == 0) {
        return target;
    }
    const uint64_t duration_us = (uint64_t)duration_ms * 1000;
    if (elapsed_us >= duration_us) {
        return target;
    }
    return (int32_t)((int64_t)target * (int64_t)elapsed_us / (int64_t)duration_us);
}

static int16_t clamp_heading_correction(int32_t correction)
{
    if (correction > WR_DRIVE_HEADING_CORRECTION_MAX_MILLI) {
        return WR_DRIVE_HEADING_CORRECTION_MAX_MILLI;
    }
    if (correction < -WR_DRIVE_HEADING_CORRECTION_MAX_MILLI) {
        return -WR_DRIVE_HEADING_CORRECTION_MAX_MILLI;
    }
    return (int16_t)correction;
}

static bool pwm_value_valid(int16_t pwm_milli)
{
    return pwm_milli >= -WR_MAX_PWM_MILLI && pwm_milli <= WR_MAX_PWM_MILLI;
}

static bool drive_pwm_mapped(int16_t drive_milli)
{
    return drive_milli >= WR_DRIVE_MIN_MAPPED_MILLI && drive_milli <= WR_DRIVE_MAX_MAPPED_MILLI;
}

static bool turn_pwm_valid(int16_t turn_milli)
{
    return turn_milli >= WR_TURN_MIN_MILLI && turn_milli <= WR_TURN_MAX_MILLI;
}

static void sample_imu(app_state_t *app, uint64_t now_us)
{
    int32_t rate_cdeg_s = 0;
    if (wr_imu_read_yaw_rate_cdeg_s(&app->imu, &rate_cdeg_s) != ESP_OK) {
        return;
    }

    app->gyro_z_cdeg_s = rate_cdeg_s;
    motion_state_t *motion = &app->motion;
    if (!motion_active(motion) || !motion->use_imu) {
        return;
    }

    if (motion->last_gyro_us != 0 && now_us > motion->last_gyro_us) {
        const uint64_t dt_us = now_us - motion->last_gyro_us;
        motion->z_est_cdeg += (int32_t)((int64_t)rate_cdeg_s * (int64_t)dt_us / INT64_C(1000000));
    }
    motion->last_gyro_us = now_us;
}

static void step_motion(motion_state_t *motion, uint64_t now_us)
{
    if (!motion_active(motion)) {
        return;
    }

    if (now_us >= motion->command_deadline_us) {
        motion->timed_out = true;
        stop_motion(motion, WR_PHASE_FAULT);
        return;
    }

    const uint64_t elapsed_us = now_us - motion->phase_started_us;
    if (motion->phase == WR_PHASE_PWM) {
        if (elapsed_us >= (uint64_t)motion->phase_duration_ms * 1000) {
            stop_motion(motion, WR_PHASE_DONE);
        }
        return;
    }

    if (motion->phase == WR_PHASE_TURN) {
        if (motion->use_imu) {
            if (turn_reached(motion)) {
                start_drive_phase(motion, now_us);
            } else if (elapsed_us >= (uint64_t)motion->phase_limit_ms * 1000) {
                motion->timed_out = true;
                stop_motion(motion, WR_PHASE_FAULT);
            }
            return;
        }

        motion->z_est_cdeg = linear_progress(motion->target.z_cdeg, elapsed_us, motion->phase_duration_ms);
        if (elapsed_us >= (uint64_t)motion->phase_duration_ms * 1000) {
            motion->z_est_cdeg = motion->target.z_cdeg;
            start_drive_phase(motion, now_us);
        }
        return;
    }

    motion->x_est_mm = linear_progress(motion->target.x_mm, elapsed_us, motion->phase_duration_ms);
    if (elapsed_us >= (uint64_t)motion->phase_duration_ms * 1000) {
        motion->x_est_mm = motion->target.x_mm;
        stop_motion(motion, WR_PHASE_DONE);
        return;
    }

    if (motion->use_imu) {
        const int32_t yaw_error = motion->target.z_cdeg - motion->z_est_cdeg;
        const int16_t correction = clamp_heading_correction(yaw_error / WR_DRIVE_HEADING_CORRECTION_DIV);
        apply_output(motion, motion->drive_base_milli - correction, motion->drive_base_milli + correction);
    }
}

static void send_telemetry(app_state_t *app)
{
    const motion_state_t *motion = &app->motion;
    uint8_t flags = 0;
    if (motion_active(motion)) {
        flags |= WR_TELEM_ACTIVE;
    }
    if (motion->timed_out) {
        flags |= WR_TELEM_TIMEOUT;
    }
    if (wr_imu_ready(&app->imu)) {
        flags |= WR_TELEM_IMU_READY;
    }
    if (wr_ina219_ready(&app->ina219)) {
        flags |= WR_TELEM_POWER_READY;
    }

    const wr_telemetry_t telemetry = {
        .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000),
        .active_seq = motion->active_seq,
        .phase = motion->phase,
        .flags = flags,
        .x_target_mm = motion->target.x_mm,
        .z_target_cdeg = motion->target.z_cdeg,
        .x_est_mm = motion->x_est_mm,
        .z_est_cdeg = motion->z_est_cdeg,
        .left_milli = motion->left_milli,
        .right_milli = motion->right_milli,
        .gyro_z_cdeg_s = app->gyro_z_cdeg_s,
        .bus_mv = app->bus_mv,
        .current_ma = app->current_ma,
        .shunt_uv = app->shunt_uv,
    };
    send_packet(WR_PACKET_TELEMETRY, ++app->tx_seq, &telemetry, sizeof(telemetry));
}

static void sample_power(app_state_t *app)
{
    wr_ina219_sample_t sample = {0};
    if (wr_ina219_read_sample(&app->ina219, &sample) != ESP_OK) {
        return;
    }

    app->bus_mv = sample.bus_mv;
    app->current_ma = sample.current_ma;
    app->shunt_uv = sample.shunt_uv;
}

static void status_task(void *ctx)
{
    app_state_t *app = ctx;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t display_divider = WR_DISPLAY_PERIOD_MS / WR_POWER_PERIOD_MS;
    uint32_t display_counter = 0;
    if (display_divider == 0) {
        display_divider = 1;
    }

    for (;;) {
        if (wr_ina219_ready(&app->ina219)) {
            sample_power(app);
        }
        ++display_counter;
        if (wr_oled_ready(&app->oled) && display_counter >= display_divider) {
            display_counter = 0;
            if (wr_oled_show_voltage(&app->oled, app->bus_mv) != ESP_OK) {
                ESP_LOGW(TAG, "OLED update failed");
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(WR_POWER_PERIOD_MS));
    }
}

static void handle_packet(const wr_header_t *header, const uint8_t *payload, void *ctx)
{
    app_state_t *app = ctx;
    motion_state_t *motion = &app->motion;

    if (header->version != WR_VERSION) {
        send_ack(header->seq, header->type, WR_ACK_BAD_VERSION, header->version);
        return;
    }

    if (header->type == WR_PACKET_CMD_STOP) {
        if (header->payload_len != 0) {
            send_ack(header->seq, header->type, WR_ACK_BAD_LENGTH, header->payload_len);
            return;
        }
        stop_motion(motion, WR_PHASE_IDLE);
        send_ack(header->seq, header->type, WR_ACK_OK, 0);
        return;
    }

    if (header->type == WR_PACKET_CMD_PWM) {
        if (header->payload_len != sizeof(wr_cmd_pwm_t)) {
            send_ack(header->seq, header->type, WR_ACK_BAD_LENGTH, header->payload_len);
            return;
        }
        if (motion_active(motion)) {
            send_ack(header->seq, header->type, WR_ACK_BUSY, motion->active_seq);
            return;
        }

        wr_cmd_pwm_t target;
        memcpy(&target, payload, sizeof(target));
        if (target.duration_ms == 0 || (target.left_milli == 0 && target.right_milli == 0)) {
            stop_motion(motion, WR_PHASE_IDLE);
            send_ack(header->seq, header->type, WR_ACK_OK, 0);
            return;
        }
        if (target.duration_ms > WR_MAX_PWM_DURATION_MS) {
            send_ack(header->seq, header->type, WR_ACK_BAD_VALUE, target.duration_ms);
            return;
        }
        if (!pwm_value_valid(target.left_milli) || !pwm_value_valid(target.right_milli)) {
            send_ack(header->seq, header->type, WR_ACK_BAD_VALUE, WR_MAX_PWM_MILLI);
            return;
        }

        start_pwm(motion, header->seq, &target);
        send_ack(header->seq, header->type, WR_ACK_OK, 0);
        return;
    }

    if (header->type == WR_PACKET_CMD_MOVE_REL) {
        if (header->payload_len != sizeof(wr_cmd_move_rel_t)) {
            send_ack(header->seq, header->type, WR_ACK_BAD_LENGTH, header->payload_len);
            return;
        }
        if (motion_active(motion)) {
            send_ack(header->seq, header->type, WR_ACK_BUSY, motion->active_seq);
            return;
        }

        wr_cmd_move_rel_t target;
        memcpy(&target, payload, sizeof(target));
        if (target.drive_milli == 0) {
            target.drive_milli = WR_DRIVE_DEFAULT_MILLI;
        }
        if (target.turn_milli == 0) {
            target.turn_milli = WR_TURN_DEFAULT_MILLI;
        }
        if (target.x_mm != 0 && !drive_pwm_mapped(target.drive_milli)) {
            send_ack(header->seq, header->type, WR_ACK_BAD_VALUE, target.drive_milli);
            return;
        }
        if (target.z_cdeg != 0 && !turn_pwm_valid(target.turn_milli)) {
            send_ack(header->seq, header->type, WR_ACK_BAD_VALUE, target.turn_milli);
            return;
        }
        if (target.x_mm == 0 && target.z_cdeg == 0) {
            stop_motion(motion, WR_PHASE_IDLE);
            send_ack(header->seq, header->type, WR_ACK_OK, 0);
            return;
        }
        start_move(motion, header->seq, &target);
        motion->use_imu = wr_imu_ready(&app->imu);
        motion->last_gyro_us = esp_timer_get_time();
        send_ack(header->seq, header->type, WR_ACK_OK, 0);
        return;
    }

    send_ack(header->seq, header->type, WR_ACK_BAD_COMMAND, header->type);
}

static void init_uart(void)
{
    const uart_config_t config = {
        .baud_rate = WR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(WR_UART, WR_RX_BUF_SIZE, WR_RX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(WR_UART, &config));
}

void app_main(void)
{
    memset(&g_app, 0, sizeof(g_app));
    init_uart();
    wr_parser_init(&g_app.parser, handle_packet, &g_app);
    send_telemetry(&g_app);
    vTaskDelay(pdMS_TO_TICKS(20));
    const esp_err_t i2c_error = wr_i2c_bus_new(&g_app.i2c_bus);
    if (i2c_error != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus unavailable: %s", esp_err_to_name(i2c_error));
    }
    const esp_err_t imu_error =
        g_app.i2c_bus != NULL ? wr_imu_init_on_bus(&g_app.imu, g_app.i2c_bus) : ESP_ERR_INVALID_STATE;
    if (imu_error != ESP_OK) {
        ESP_LOGW(TAG, "QMI8658 unavailable: %s", esp_err_to_name(imu_error));
    }
    const esp_err_t ina219_error =
        g_app.i2c_bus != NULL ? wr_ina219_init_on_bus(&g_app.ina219, g_app.i2c_bus) : ESP_ERR_INVALID_STATE;
    if (ina219_error != ESP_OK) {
        ESP_LOGW(TAG, "INA219 unavailable: %s", esp_err_to_name(ina219_error));
    }
    const esp_err_t oled_error =
        g_app.i2c_bus != NULL ? wr_oled_init_on_bus(&g_app.oled, g_app.i2c_bus) : ESP_ERR_INVALID_STATE;
    if (oled_error != ESP_OK) {
        ESP_LOGW(TAG, "OLED unavailable: %s", esp_err_to_name(oled_error));
    }
    wr_motor_init();
    if (wr_ina219_ready(&g_app.ina219) || wr_oled_ready(&g_app.oled)) {
        if (xTaskCreate(status_task, "wr_status", WR_STATUS_TASK_STACK_SIZE, &g_app, WR_STATUS_TASK_PRIORITY, NULL) !=
            pdPASS) {
            ESP_LOGW(TAG, "status task start failed");
        }
    }

    TickType_t last_wake = xTaskGetTickCount();
    uint8_t rx[128];
    for (;;) {
        const int got = uart_read_bytes(WR_UART, rx, sizeof(rx), 0);
        if (got > 0) {
            wr_parser_feed(&g_app.parser, rx, (size_t)got);
        }

        const uint64_t now_us = esp_timer_get_time();
        sample_imu(&g_app, now_us);
        step_motion(&g_app.motion, now_us);
        if (g_app.next_telemetry_us == 0 || now_us >= g_app.next_telemetry_us) {
            send_telemetry(&g_app);
            g_app.next_telemetry_us = now_us + WR_TELEMETRY_PERIOD_US;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(WR_CONTROL_PERIOD_MS));
    }
}
