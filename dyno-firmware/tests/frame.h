#ifndef DYNO_TEST_FRAME_H
#define DYNO_TEST_FRAME_H
/* Request framing helpers shared by the native tests. */
#include "dyno.h"
#include <assert.h>
#include <string.h>

static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void put32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (8u * i));
}
static size_t frame(uint8_t *out, uint8_t command, uint16_t id, uint32_t epoch,
                    uint32_t capture, const uint8_t *payload, uint8_t n) {
    memset(out, 0, DYNO_FRAME_SIZE);
    out[0] = DYNO_MAGIC0; out[1] = DYNO_MAGIC1; out[2] = DYNO_PROTOCOL; out[3] = command;
    out[4] = (uint8_t)id; out[5] = (uint8_t)(id >> 8); out[7] = n;
    put32(out + 8, epoch); put32(out + 12, capture);
    if (n) memcpy(out + DYNO_HEADER_SIZE, payload, n);
    size_t size = DYNO_HEADER_SIZE + (size_t)n;
    uint16_t crc = dyno_crc16(out, size);
    out[size] = (uint8_t)crc; out[size + 1u] = (uint8_t)(crc >> 8);
    return size + 2u;
}
static uint16_t next_id = 1;
/* Sends one request and checks the response envelope; returns the result code. */
static uint8_t call(struct dyno_core *c, uint8_t command, uint32_t epoch, uint32_t capture,
                    const uint8_t *payload, uint8_t n, uint8_t out[DYNO_FRAME_SIZE]) {
    uint8_t request[DYNO_FRAME_SIZE];
    uint16_t id = next_id++;
    size_t size = frame(request, command, id, epoch, capture, payload, n);
    dyno_request(c, request, size, out);
    size_t last = dyno_response_size(command) - 2u;
    assert(out[0] == DYNO_MAGIC0 && out[1] == DYNO_MAGIC1 && out[2] == DYNO_PROTOCOL);
    assert(out[3] == command && out[4] == (uint8_t)id && out[5] == (uint8_t)(id >> 8));
    assert(dyno_crc16(out, last) == (uint16_t)(out[last] | out[last + 1u] << 8));
    return out[6];
}

#endif
