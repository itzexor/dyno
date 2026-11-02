#include "dyno.h"
#include "link.h"
#include "ads131m02.h"
#include "frame.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    static struct dyno_core c;
    uint8_t out[DYNO_FRAME_SIZE], payload[16];
    uint32_t tick = 0;
    dyno_init(&c, 0, 1000000);

    /* Epoch negotiation: a nonzero nonce is required once and then fixed. */
    assert(call(&c, DYNO_INFO, 0, 0, NULL, 0, out) == DYNO_BAD_VALUE);
    put32(payload, 0x12345678u);
    assert(call(&c, DYNO_INFO, 0, 0, payload, 4, out) == DYNO_OK);
    assert(get32(out + 8) == 0x12345678u && memcmp(out + 16, "DYNO-REF", 8) == 0);
    assert(get32(out + 24) == 1000000u && out[34] == DYNO_BLOCK_MAX);
    put32(payload, 0x9abcdef0u);
    assert(call(&c, DYNO_INFO, 0, 0, payload, 4, out) == DYNO_OK && c.epoch == 0x12345678u);
    const uint32_t epoch = c.epoch;
    assert(call(&c, DYNO_STATUS, epoch + 1u, 0, NULL, 0, out) == DYNO_STALE);

    /* Frames with a foreign magic are rejected without executing. */
    uint8_t foreign[DYNO_FRAME_SIZE];
    size_t foreign_size = frame(foreign, DYNO_STATUS, 99, epoch, 0, NULL, 0);
    foreign[0] = 'X';
    uint16_t crc = dyno_crc16(foreign, foreign_size - 2u);
    foreign[foreign_size - 2u] = (uint8_t)crc; foreign[foreign_size - 1u] = (uint8_t)(crc >> 8);
    dyno_request(&c, foreign, foreign_size, out);
    assert(out[6] == DYNO_BAD_FRAME);

    /* Tare needs a settled idle history. */
    assert(call(&c, DYNO_TARE, epoch, 0, NULL, 0, out) == DYNO_BAD_STATE);
    for (int i = 0; i < 20; ++i) dyno_sample(&c, tick += 4096u, 1000 + (i % 3), 0);
    assert(call(&c, DYNO_TARE, epoch, 0, NULL, 0, out) == DYNO_OK);
    assert((int32_t)get32(out + 16) == 1000);

    /* Capture IDs must be nonzero and increase; header capture is checked. */
    put32(payload, 0);
    assert(call(&c, DYNO_ARM, epoch, 0, payload, 4, out) == DYNO_BAD_VALUE);
    put32(payload, 5);
    assert(call(&c, DYNO_ARM, epoch, 0, payload, 4, out) == DYNO_OK && get32(out + 12) == 5u);
    assert(call(&c, DYNO_START, epoch, 4, NULL, 0, out) == DYNO_STALE);
    assert(call(&c, DYNO_START, epoch, 5, NULL, 0, out) == DYNO_OK);
    for (int i = 0; i < 10; ++i) dyno_sample(&c, tick += 4096u, 1000 + i, 0);

    /* READ pins a block; a retry gets the cached bytes, a new ID the same block. */
    uint8_t request[DYNO_FRAME_SIZE], first[DYNO_FRAME_SIZE], retry[DYNO_FRAME_SIZE];
    payload[0] = 8;
    size_t size = frame(request, DYNO_READ, 500, epoch, 5, payload, 1);
    dyno_request(&c, request, size, first);
    assert(first[6] == DYNO_OK && first[16 + 4] == 8);
    uint32_t token = get32(first + 16);
    assert(get32(first + 21) == 20u && (int32_t)(get32(first + 29) & 0xffffffu) == 1000);
    dyno_sample(&c, tick += 4096u, 1010, 0);
    dyno_request(&c, request, size, retry);
    assert(memcmp(first, retry, DYNO_FRAME_SIZE) == 0);
    assert(call(&c, DYNO_READ, epoch, 5, payload, 1, out) == DYNO_OK);
    assert(get32(out + 16) == token && memcmp(out + 20, first + 20, 5u + 12u * 8u) == 0);

    put32(payload, token + 1u);
    assert(call(&c, DYNO_ACK, epoch, 5, payload, 4, out) == DYNO_STALE);
    put32(payload, token);
    assert(call(&c, DYNO_ACK, epoch, 5, payload, 4, out) == DYNO_OK);
    assert(call(&c, DYNO_ACK, epoch, 5, payload, 4, out) == DYNO_OK); /* repeated ACK */

    /* While running, a minimum count defers partial blocks; STOP drains them. */
    payload[0] = 8;
    assert(call(&c, DYNO_READ, epoch, 5, payload, 1, out) == DYNO_OK && out[20] == 0);
    assert(call(&c, DYNO_STOP, epoch, 5, NULL, 0, out) == DYNO_OK);
    assert(call(&c, DYNO_READ, epoch, 5, payload, 1, out) == DYNO_OK && out[20] == 3);
    assert(get32(out + 21) == 28u && get32(out + 21 + 24) == 30u);
    put32(payload, get32(out + 16));
    assert(call(&c, DYNO_ACK, epoch, 5, payload, 4, out) == DYNO_OK);
    assert(call(&c, DYNO_STATUS, epoch, 5, NULL, 0, out) == DYNO_OK);
    assert(out[16] == DYNO_STOPPED && get32(out + 19) == 0 && get32(out + 35) == 31u);

    /* Overflow drops newest records but still consumes sequence numbers. */
    put32(payload, 5);
    assert(call(&c, DYNO_ARM, epoch, 5, payload, 4, out) == DYNO_BAD_VALUE);
    put32(payload, 6);
    assert(call(&c, DYNO_ARM, epoch, 5, payload, 4, out) == DYNO_OK);
    assert(call(&c, DYNO_START, epoch, 6, NULL, 0, out) == DYNO_OK);
    for (int i = 0; i < 300; ++i) dyno_sample(&c, tick += 4096u, 1000, 0);
    assert(call(&c, DYNO_STATUS, epoch, 6, NULL, 0, out) == DYNO_OK);
    assert(get32(out + 19) == DYNO_FIFO_SIZE && get32(out + 23) == 300u - DYNO_FIFO_SIZE);
    assert(get32(out + 31) & DYNO_FAULT_OVERFLOW);
    assert(call(&c, DYNO_RESET_ERRORS, epoch, 6, NULL, 0, out) == DYNO_BAD_STATE);

    /* Paged link: F0,page exposes a frozen 32-byte window without re-executing. */
    static struct dyno_link link;
    dyno_link_init(&link, &c);
    size = frame(request, DYNO_STATUS, 700, epoch, 6, NULL, 0);
    dyno_link_write(&link, request, size);
    assert(dyno_link_byte(&link, 40) == link.reply[40] && dyno_link_byte(&link, 41) == 0xff);
    const uint8_t select[] = {0xf0, 1};
    dyno_link_write(&link, select, sizeof(select));
    assert(dyno_link_byte(&link, 0) == link.reply[32] && dyno_link_byte(&link, 32) == 0xff);
    assert(link.reply[4] == (uint8_t)700u && link.reply[5] == (uint8_t)(700u >> 8));

    /* ADS131M02 frames: full-length CRC, sign extension and corruption. */
    uint8_t adc[12] = {0};
    adc[3] = 0x80; adc[4] = 0x00; adc[5] = 0x01;
    crc = dyno_crc16(adc, 9); adc[9] = (uint8_t)(crc >> 8); adc[10] = (uint8_t)crc;
    int32_t value = 0;
    assert(ads131_decode(adc, &value) && value == -8388607);
    adc[5] ^= 1;
    assert(!ads131_decode(adc, &value));
    ads131_command(adc, 0x6100, 0x0510);
    assert(adc[0] == 0x61 && adc[1] == 0 && adc[3] == 0x05 && adc[4] == 0x10 && adc[11] == 0);

    puts("Protocol: epoch, tare, capture lifecycle, pinned READ/ACK, retry cache, overflow, paging, ADC frames passed");
    return 0;
}
