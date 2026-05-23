#include "protocol.h"

#include <string.h>

static bool magic_at_start(const wr_parser_t *parser)
{
    const uint16_t magic = WR_MAGIC;
    return parser->used >= sizeof(magic) && memcmp(parser->bytes, &magic, sizeof(magic)) == 0;
}

static void consume(wr_parser_t *parser, size_t len)
{
    if (len >= parser->used) {
        parser->used = 0;
        return;
    }
    memmove(parser->bytes, parser->bytes + len, parser->used - len);
    parser->used -= len;
}

uint16_t wr_crc16_ccitt(const uint8_t *bytes, size_t len)
{
    uint16_t crc = UINT16_C(0xffff);
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & UINT16_C(0x8000)) ? (uint16_t)((crc << 1) ^ UINT16_C(0x1021))
                                            : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void wr_parser_init(wr_parser_t *parser, wr_packet_handler_t handler, void *handler_ctx)
{
    memset(parser, 0, sizeof(*parser));
    parser->handler = handler;
    parser->handler_ctx = handler_ctx;
}

void wr_parser_feed(wr_parser_t *parser, const uint8_t *bytes, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        if (parser->used == sizeof(parser->bytes)) {
            consume(parser, 1);
            parser->length_errors++;
        }
        parser->bytes[parser->used++] = bytes[i];

        for (;;) {
            while (parser->used >= sizeof(uint16_t) && !magic_at_start(parser)) {
                consume(parser, 1);
            }
            if (parser->used < sizeof(wr_header_t)) {
                break;
            }

            wr_header_t header;
            memcpy(&header, parser->bytes, sizeof(header));
            if (header.payload_len > WR_MAX_PAYLOAD) {
                consume(parser, 1);
                parser->length_errors++;
                continue;
            }

            const size_t frame_len = sizeof(header) + header.payload_len + sizeof(uint16_t);
            if (parser->used < frame_len) {
                break;
            }

            uint16_t wire_crc = 0;
            memcpy(&wire_crc, parser->bytes + sizeof(header) + header.payload_len, sizeof(wire_crc));
            const uint16_t computed_crc = wr_crc16_ccitt(parser->bytes, sizeof(header) + header.payload_len);
            if (wire_crc == computed_crc && parser->handler != NULL) {
                parser->handler(&header, parser->bytes + sizeof(header), parser->handler_ctx);
            } else if (wire_crc != computed_crc) {
                parser->crc_errors++;
            }
            consume(parser, frame_len);
        }
    }
}

size_t wr_encode_frame(
    uint8_t *out,
    size_t out_size,
    uint8_t type,
    uint16_t seq,
    const void *payload,
    uint16_t payload_len)
{
    if (payload_len > WR_MAX_PAYLOAD) {
        return 0;
    }
    const size_t frame_len = sizeof(wr_header_t) + payload_len + sizeof(uint16_t);
    if (out == NULL || out_size < frame_len || (payload_len > 0 && payload == NULL)) {
        return 0;
    }

    const wr_header_t header = {
        .magic = WR_MAGIC,
        .version = WR_VERSION,
        .type = type,
        .payload_len = payload_len,
        .seq = seq,
    };
    memcpy(out, &header, sizeof(header));
    if (payload_len > 0) {
        memcpy(out + sizeof(header), payload, payload_len);
    }

    const uint16_t crc = wr_crc16_ccitt(out, sizeof(header) + payload_len);
    memcpy(out + sizeof(header) + payload_len, &crc, sizeof(crc));
    return frame_len;
}
