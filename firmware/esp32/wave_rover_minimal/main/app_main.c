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
#include "imu.h"
#include "motor.h"
#include "protocol.h"

#define WR_UART UART_NUM_0
#define WR_UART_BAUD 460800
#define WR_RX_BUF_SIZE 1024
#define WR_CONTROL_PERIOD_MS 5
#define WR_TELEMETRY_PERIOD_US UINT64_C(40000)
#define WR_DRIVE_MILLI 160
#define WR_TURN_MILLI 450
#define WR_DRIVE_MM_PER_S 100
#define WR_TURN_CDEG_PER_S 2000
#define WR_MIN_PHASE_MS 20
#define WR_TURN_IMU_HEADROOM_MS 200
#define WR_TURN_REACHED_CDEG 80
#define WR_DRIVE_HEADING_CORRECTION_MAX_MILLI 80
#define WR_DRIVE_HEADING_CORRECTION_DIV 4
#define WR_DEFAULT_MOVE_CAP_MS 2500

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
} motion_state_t;

typedef struct {
    wr_parser_t parser;
    motion_state_t motion;
    wr_imu_t imu;
    int32_t gyro_z_cdeg_s;
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

static bool motion_active(const motion_state_t *motion)
{
    return motion->phase == WR_PHASE_TURN || motion->phase == WR_PHASE_DRIVE;
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
    motion->phase_duration_ms = estimate_ms(abs_i32(motion->target.x_mm), WR_DRIVE_MM_PER_S);
    motion->phase_limit_ms = motion->phase_duration_ms;
    if (motion->phase_duration_ms == 0) {
        stop_motion(motion, WR_PHASE_DONE);
        return;
    }

    const int16_t pwm = motion->target.x_mm >= 0 ? WR_DRIVE_MILLI : -WR_DRIVE_MILLI;
    apply_output(motion, pwm, pwm);
}

static void start_move(motion_state_t *motion, uint16_t seq, const wr_cmd_move_rel_t *target)
{
    memset(motion, 0, sizeof(*motion));
    motion->active_seq = seq;
    motion->target = *target;

    const uint64_t now_us = esp_timer_get_time();
    const uint32_t target_cap_ms = target->max_time_ms == 0 ? WR_DEFAULT_MOVE_CAP_MS : target->max_time_ms;
    motion->command_started_us = now_us;
    motion->command_deadline_us = now_us + (uint64_t)target_cap_ms * 1000;

    motion->phase_duration_ms = estimate_ms(abs_i32(target->z_cdeg), WR_TURN_CDEG_PER_S);
    motion->phase_limit_ms = motion->phase_duration_ms + motion->phase_duration_ms / 2 + WR_TURN_IMU_HEADROOM_MS;
    if (motion->phase_duration_ms == 0) {
        start_drive_phase(motion, now_us);
        return;
    }

    motion->phase = WR_PHASE_TURN;
    motion->phase_started_us = now_us;
    const int16_t pwm = target->z_cdeg >= 0 ? WR_TURN_MILLI : -WR_TURN_MILLI;
    apply_output(motion, -pwm, pwm);
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
        const int16_t base = motion->target.x_mm >= 0 ? WR_DRIVE_MILLI : -WR_DRIVE_MILLI;
        const int32_t yaw_error = motion->target.z_cdeg - motion->z_est_cdeg;
        const int16_t correction = clamp_heading_correction(yaw_error / WR_DRIVE_HEADING_CORRECTION_DIV);
        apply_output(motion, base - correction, base + correction);
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
    };
    send_packet(WR_PACKET_TELEMETRY, ++app->tx_seq, &telemetry, sizeof(telemetry));
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
    const esp_err_t imu_error = wr_imu_init(&g_app.imu);
    if (imu_error != ESP_OK) {
        ESP_LOGW(TAG, "QMI8658 unavailable: %s", esp_err_to_name(imu_error));
    }
    wr_motor_init();

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
