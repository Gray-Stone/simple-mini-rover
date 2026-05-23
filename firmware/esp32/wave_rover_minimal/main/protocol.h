#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WR_MAGIC UINT16_C(0x5752)
#define WR_VERSION UINT8_C(1)
#define WR_MAX_PAYLOAD UINT16_C(96)
#define WR_MAX_FRAME (sizeof(wr_header_t) + WR_MAX_PAYLOAD + sizeof(uint16_t))

typedef enum {
    WR_PACKET_CMD_STOP = 1,
    WR_PACKET_CMD_MOVE_REL = 2,
    WR_PACKET_ACK = 0x80,
    WR_PACKET_TELEMETRY = 0x81,
} wr_packet_type_t;

typedef enum {
    WR_ACK_OK = 0,
    WR_ACK_BAD_VERSION = 1,
    WR_ACK_BAD_LENGTH = 2,
    WR_ACK_BUSY = 3,
    WR_ACK_BAD_COMMAND = 4,
} wr_ack_status_t;

typedef enum {
    WR_PHASE_IDLE = 0,
    WR_PHASE_TURN = 1,
    WR_PHASE_DRIVE = 2,
    WR_PHASE_DONE = 3,
    WR_PHASE_FAULT = 4,
} wr_motion_phase_t;

typedef enum {
    WR_TELEM_ACTIVE = 1 << 0,
    WR_TELEM_TIMEOUT = 1 << 1,
    WR_TELEM_IMU_READY = 1 << 2,
} wr_telemetry_flag_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t payload_len;
    uint16_t seq;
} wr_header_t;

typedef struct __attribute__((packed)) {
    int32_t x_mm;
    int32_t z_cdeg;
    uint16_t max_time_ms;
    uint16_t flags;
} wr_cmd_move_rel_t;

typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t command_type;
    uint16_t detail;
} wr_ack_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint16_t active_seq;
    uint8_t phase;
    uint8_t flags;
    int32_t x_target_mm;
    int32_t z_target_cdeg;
    int32_t x_est_mm;
    int32_t z_est_cdeg;
    int16_t left_milli;
    int16_t right_milli;
    int32_t gyro_z_cdeg_s;
} wr_telemetry_t;

_Static_assert(sizeof(wr_header_t) == 8, "wire header size changed");
_Static_assert(sizeof(wr_cmd_move_rel_t) == 12, "move command size changed");
_Static_assert(sizeof(wr_ack_t) == 4, "ack size changed");
_Static_assert(sizeof(wr_telemetry_t) == 32, "telemetry size changed");

typedef void (*wr_packet_handler_t)(const wr_header_t *header, const uint8_t *payload, void *ctx);

typedef struct {
    uint8_t bytes[WR_MAX_FRAME];
    size_t used;
    wr_packet_handler_t handler;
    void *handler_ctx;
    uint32_t crc_errors;
    uint32_t length_errors;
} wr_parser_t;

void wr_parser_init(wr_parser_t *parser, wr_packet_handler_t handler, void *handler_ctx);
void wr_parser_feed(wr_parser_t *parser, const uint8_t *bytes, size_t len);
uint16_t wr_crc16_ccitt(const uint8_t *bytes, size_t len);
size_t wr_encode_frame(
    uint8_t *out,
    size_t out_size,
    uint8_t type,
    uint16_t seq,
    const void *payload,
    uint16_t payload_len);
