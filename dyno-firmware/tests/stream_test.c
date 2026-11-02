/* Drives the register window the way a FIFO-accelerometer bulk reader does:
 * poll FIFO_COUNT, read 48-byte FIFO_DATA blocks, and run status queries
 * (INT_STATUS then a timed FIFO_COUNT). */
#include "dyno.h"
#include "frame.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

struct reader { uint16_t cached; };
struct entry { int32_t raw; uint16_t tick; uint8_t flags, seq; bool pad; };

/* One register transaction; the master clocks exactly `want` bytes. */
static size_t xfer(struct dyno_core *c, uint8_t reg, uint32_t now, uint8_t *out, size_t want) {
    uint8_t buffer[48];
    dyno_reg_select(c, reg);
    size_t n = dyno_reg_read(c, now, buffer);
    for (size_t i = 0; i < want; ++i) out[i] = i < n ? buffer[i] : 0xff;
    dyno_reg_done(c, want < n ? want : n);
    return n;
}
static uint16_t count_value(const uint8_t msg[2]) {
    return (uint16_t)(((msg[0] & 0x1fu) << 8) | msg[1]);
}
/* Mirrors the reader's poll: refresh the cached count only below one block. */
static bool poll(struct dyno_core *c, struct reader *r, uint32_t now, uint8_t block[48]) {
    uint8_t msg[2];
    if (r->cached < 48) { xfer(c, DYNO_REG_FIFO_COUNT, now, msg, 2); r->cached = count_value(msg); }
    if (r->cached < 48) return false;
    xfer(c, DYNO_REG_FIFO_DATA, now, block, 48);
    r->cached = (uint16_t)(r->cached - 48u);
    return true;
}
static uint16_t status_query(struct dyno_core *c, uint32_t now, uint8_t *int_status) {
    uint8_t msg[2];
    xfer(c, DYNO_REG_INT_STATUS, now, int_status, 1);
    xfer(c, DYNO_REG_FIFO_COUNT, now, msg, 2);
    return count_value(msg);
}
static struct entry decode(const uint8_t *e) {
    uint32_t raw = (uint32_t)e[0] | (uint32_t)e[1] << 8 | (uint32_t)e[2] << 16;
    struct entry out = {
        .raw = (int32_t)(raw & 0x7fffffu) - (int32_t)(raw & 0x800000u),
        .tick = (uint16_t)(e[3] | e[4] << 8), .flags = (uint8_t)(e[5] & DYNO_FLAG_MASK),
        .seq = (uint8_t)((e[5] >> DYNO_ENTRY_SEQ_SHIFT) & 15u), .pad = (e[5] & DYNO_ENTRY_PAD) != 0,
    };
    return out;
}

static int setter_calls;
static bool fake_setter(struct dyno_core *c, uint8_t osr, uint8_t gain) {
    ++setter_calls;
    c->adc_osr = osr; c->adc_gain = gain; c->adc_rate_mhz = 4000000000u / (128u << osr);
    c->adc_clock_reg = (uint16_t)(0x0102u | (unsigned)osr << 2u); c->adc_gain_reg = gain;
    return gain != 0; /* gain code 0 simulates a readback failure */
}

int main(void) {
    static struct dyno_core c;
    uint8_t out[DYNO_FRAME_SIZE], payload[8], block[48], int_status;
    uint32_t tick = 0x0001fff0u; /* crosses a 16-bit tick boundary */
    struct reader r = {0};
    dyno_init(&c, 0x1234u, 1000000);
    c.adc_set = fake_setter; c.adc_osr_min = 3;

    /* ADC profile: settled state only, validated codes, tare invalidated. */
    for (int i = 0; i < 20; ++i) dyno_sample(&c, tick += 1024u, 500, 0);
    assert(call(&c, DYNO_TARE, c.epoch, 0, NULL, 0, out) == DYNO_OK && c.tared);
    payload[0] = 2; payload[1] = 7;
    assert(call(&c, DYNO_SET_ADC, c.epoch, 0, payload, 2, out) == DYNO_BAD_VALUE);
    payload[0] = 5;
    assert(call(&c, DYNO_SET_ADC, c.epoch, 0, payload, 2, out) == DYNO_OK);
    assert(!c.tared && c.idle_count == 0 && setter_calls == 1);
    assert(call(&c, DYNO_GET_ADC, c.epoch, 0, NULL, 0, out) == DYNO_OK);
    assert(out[16] == 5 && out[17] == 7 && get32(out + 24) == 976562u);
    payload[1] = 0;
    assert(call(&c, DYNO_SET_ADC, c.epoch, 0, payload, 2, out) == DYNO_BAD_STATE);
    assert(c.faults & DYNO_FAULT_ADC);
    payload[1] = 7;
    assert(call(&c, DYNO_SET_ADC, c.epoch, 0, payload, 2, out) == DYNO_OK);
    assert(call(&c, DYNO_RESET_ERRORS, c.epoch, 0, NULL, 0, out) == DYNO_OK);

    /* Stream capture: mode byte on ARM; READ/ACK are refused. */
    for (int i = 0; i < 4; ++i) dyno_sample(&c, tick += 1024u, 500, 0);
    put32(payload, 1); payload[4] = 2;
    assert(call(&c, DYNO_ARM, c.epoch, 0, payload, 5, out) == DYNO_BAD_VALUE);
    payload[4] = 1;
    assert(call(&c, DYNO_ARM, c.epoch, 0, payload, 5, out) == DYNO_OK && c.streaming);
    assert(call(&c, DYNO_START, c.epoch, 1, NULL, 0, out) == DYNO_OK);
    assert(call(&c, DYNO_READ, c.epoch, 1, NULL, 0, out) == DYNO_BAD_STATE);

    /* Nothing buffered: the reader polls the count and reads nothing. */
    assert(!poll(&c, &r, tick, block) && r.cached == 0);
    const uint32_t first_seq = c.sequence;
    for (int i = 0; i < 21; ++i)
        dyno_sample(&c, tick += 1024u, i == 3 ? -5 : 1000 + i, i == 7 ? DYNO_ADC_CRC : 0);
    assert(call(&c, DYNO_STATUS, c.epoch, 1, NULL, 0, out) == DYNO_OK && get32(out + 19) == 21u);

    int blocks = 0; struct entry seen[64]; int n = 0;
    while (poll(&c, &r, tick, block)) {
        ++blocks;
        for (int i = 0; i < 8; ++i) seen[n++] = decode(block + 6 * i);
    }
    assert(blocks == 2 && n == 16 && r.cached == 30u);
    for (int i = 0; i < 16; ++i) {
        uint32_t expect_tick = 0x0001fff0u + 1024u * (uint32_t)(i + 25);
        assert(seen[i].tick == (uint16_t)expect_tick && !seen[i].pad);
        assert(seen[i].seq == ((first_seq + (uint32_t)i) & 15u));
        assert(seen[i].raw == (i == 3 ? -5 : 1000 + i));
        assert(seen[i].flags == (i == 7 ? DYNO_ADC_CRC : 0));
    }

    /* A status query latches bus time instead of the count; polls do not. */
    uint32_t latch_time = 0x00abcdefu;
    uint16_t value = status_query(&c, latch_time, &int_status);
    assert(value == ((latch_time >> DYNO_LATCH_SHIFT) & 0x1fffu));
    assert(int_status == 0);
    uint8_t msg[2];
    xfer(&c, DYNO_REG_FIFO_COUNT, latch_time, msg, 2);
    assert(count_value(msg) == 5u * DYNO_STREAM_ENTRY);
    /* Any other transaction between the two reads cancels the latch. */
    xfer(&c, DYNO_REG_INT_STATUS, latch_time, &int_status, 1);
    dyno_reg_select(&c, 0xf0);
    xfer(&c, DYNO_REG_FIFO_COUNT, latch_time, msg, 2);
    assert(count_value(msg) == 5u * DYNO_STREAM_ENTRY);

    /* A block clocked out only partially stays in the ring. */
    for (int i = 0; i < 3; ++i) dyno_sample(&c, tick += 1024u, 7, 0);
    uint8_t partial[48];
    xfer(&c, DYNO_REG_FIFO_DATA, tick, partial, 20);
    xfer(&c, DYNO_REG_FIFO_DATA, tick, block, 48);
    assert(memcmp(partial, block, 20) == 0);
    xfer(&c, DYNO_REG_FIFO_COUNT, tick, msg, 2);
    assert(count_value(msg) == 0);

    /* Overflow drops newest, flags INT_STATUS once, and leaves a sequence gap. */
    r.cached = 0;
    const uint32_t before = c.sequence;
    for (uint32_t i = 0; i < DYNO_STREAM_LIMIT + 5u; ++i) dyno_sample(&c, tick += 1024u, 1, 0);
    assert(c.overflow == 5u && (c.faults & DYNO_FAULT_OVERFLOW));
    status_query(&c, tick, &int_status);
    assert(int_status == (DYNO_INT_OVERFLOW | DYNO_INT_BLOCK_READY));
    status_query(&c, tick, &int_status);
    assert(int_status == DYNO_INT_BLOCK_READY);
    uint32_t drained = 0;
    while (poll(&c, &r, tick, block)) drained += 8;
    assert(drained == DYNO_STREAM_LIMIT - DYNO_STREAM_LIMIT % 8u);
    dyno_sample(&c, tick += 1024u, 1, 0);
    assert(DYNO_STREAM_LIMIT % 8u == 1u); /* the leftover entry plus this one */

    /* STOP pads the final partial block so the reader drains everything. */
    assert(call(&c, DYNO_STOP, c.epoch, 1, NULL, 0, out) == DYNO_OK);
    put32(payload, 2); payload[4] = 1;
    assert(call(&c, DYNO_ARM, c.epoch, 1, payload, 5, out) == DYNO_BAD_STATE);
    r.cached = 0;
    assert(poll(&c, &r, tick, block));
    struct entry last = decode(block), next = decode(block + 6), pad = decode(block + 12);
    assert(!last.pad && !next.pad && pad.pad && decode(block + 42).pad);
    assert(next.seq == ((before + DYNO_STREAM_LIMIT + 5u) & 15u)); /* gap of 5 is visible */
    assert(!poll(&c, &r, tick, block));
    assert(call(&c, DYNO_ARM, c.epoch, 1, payload, 5, out) == DYNO_OK);

    /* A block read before re-arming is never committed into the new capture. */
    assert(call(&c, DYNO_START, c.epoch, 2, NULL, 0, out) == DYNO_OK);
    for (int i = 0; i < 8; ++i) dyno_sample(&c, tick += 1024u, 3, 0);
    dyno_reg_select(&c, DYNO_REG_FIFO_DATA);
    assert(dyno_reg_read(&c, tick, block) == 48u);
    c.stream_generation = c.stream_generation + 1u; /* as a re-ARM would */
    dyno_reg_done(&c, 48);
    assert(c.stream_tail == 0);

    printf("Stream: ADC profile, block polling, tick/sequence packing, status latch, "
           "partial reads, overflow, STOP padding, re-arm passed\n");
    return 0;
}
