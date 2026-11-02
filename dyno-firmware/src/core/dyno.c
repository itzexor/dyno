#include "dyno.h"
#include <limits.h>
#include <stdatomic.h>
#include <string.h>

_Static_assert(sizeof(struct dyno_core) <= 8192u, "engine state exceeds the reserved RAM budget");
_Static_assert(DYNO_STREAM_SIZE % DYNO_STREAM_BLOCK == 0, "stream ring must hold whole blocks");
_Static_assert(DYNO_STREAM_SIZE * DYNO_STREAM_ENTRY <= sizeof(struct dyno_sample) * DYNO_FIFO_SIZE,
               "stream ring must fit in the FIFO's storage");
_Static_assert(DYNO_STREAM_SIZE * DYNO_STREAM_ENTRY <= 8190u, "FIFO_COUNT is a 13-bit byte count");

static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void put32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (8u * i));
}
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
uint16_t dyno_crc16(const uint8_t *data, size_t size) {
    uint16_t crc = 0xffffu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (unsigned b = 0; b < 8; ++b)
            crc = (uint16_t)((uint32_t)crc << 1 ^ ((crc & 0x8000u) ? 0x1021u : 0u));
    }
    return crc;
}
size_t dyno_response_size(uint8_t command) {
    /* READ has fixed padding; short replies keep ACK overhead bounded. */
    static const uint8_t sizes[] = {18, 42, 41, 31, 18, 22, 18, 18, 18, 128, 18, 40, 22, 18, 18, 30};
    return command < sizeof(sizes) ? sizes[command] : 18u;
}

/* Native builds have no interrupts; the board overrides these. */
__attribute__((weak)) uint32_t dyno_critical_enter(void) { return 0; }
__attribute__((weak)) void dyno_critical_exit(uint32_t state) { (void)state; }

void dyno_init(struct dyno_core *c, uint32_t epoch, uint32_t tick_hz) {
    memset(c, 0, sizeof(*c));
    c->epoch = epoch; c->tick_hz = tick_hz;
    c->config.trigger = 20000; c->config.release = 10000;
    c->config.confirmations = 3; c->config.timeout = tick_hz;
    dyno_identity(c, "DYNO-REF", DYNO_CAP_SYNTHETIC | DYNO_CAP_STREAM);
}
void dyno_identity(struct dyno_core *c, const char model[8], uint32_t capabilities) {
    memcpy(c->model, model, 8); c->capabilities = capabilities;
}
void dyno_clock(struct dyno_core *c, uint32_t now) {
    c->now = now;
    if (c->have_sample && (uint32_t)(now - c->last_tick) > c->config.timeout) {
        c->healthy = false; c->faults |= DYNO_FAULT_STALE;
    }
}

static uint32_t stream_count(const struct dyno_core *c) {
    return c->stream_head - c->stream_tail;
}
/* Nothing unread or pinned in either capture mode. */
static bool drained(const struct dyno_core *c) {
    return c->head == c->tail && !c->pending_count && stream_count(c) == 0;
}
static bool settled(const struct dyno_core *c) {
    return c->mode == DYNO_IDLE || (c->mode == DYNO_STOPPED && drained(c));
}
static void stream_put(struct dyno_core *c, uint32_t tick, int32_t raw, uint8_t bits) {
    uint32_t head = c->stream_head;
    uint8_t *e = &c->buf.stream[(head % DYNO_STREAM_SIZE) * DYNO_STREAM_ENTRY];
    for (unsigned b = 0; b < 3; ++b) e[b] = (uint8_t)((uint32_t)raw >> (8u * b));
    e[3] = (uint8_t)tick; e[4] = (uint8_t)(tick >> 8); e[5] = bits;
    /* Publish the entry only after its bytes are written. */
    atomic_signal_fence(memory_order_release);
    c->stream_head = head + 1u;
}
static void reset_idle_window(struct dyno_core *c) {
    c->idle_count = 0; c->idle_index = 0; c->idle_sum = 0; c->confirm = 0;
}

void dyno_sample(struct dyno_core *c, uint32_t tick, int32_t raw, uint8_t flags) {
    uint32_t seq = c->sequence++;
    c->last_tick = tick; c->now = tick; c->have_sample = true;
    if (raw <= -8388608 || raw >= 8388607) flags |= DYNO_SATURATED;
    if (raw < -8388608) raw = -8388608;
    if (raw > 8388607) raw = 8388607;
    c->raw = raw; c->healthy = flags == 0;
    if (flags != 0) {
        ++c->adc_errors; c->faults |= DYNO_FAULT_ADC;
        reset_idle_window(c);
    } else {
        /* Explicit tare is the only baseline update. Captures cannot absorb load. */
        if (settled(c) && !c->triggered) {
            if (c->idle_count == 32) c->idle_sum -= c->idle_history[c->idle_index];
            else ++c->idle_count;
            c->idle_history[c->idle_index] = raw;
            c->idle_index = (uint8_t)((c->idle_index + 1u) % 32u);
            c->idle_sum += raw; c->idle_min = raw; c->idle_max = raw;
            for (uint8_t i = 0; i < c->idle_count; ++i) {
                if (c->idle_history[i] < c->idle_min) c->idle_min = c->idle_history[i];
                if (c->idle_history[i] > c->idle_max) c->idle_max = c->idle_history[i];
            }
        }
        int32_t relative = raw - c->baseline;
        c->filtered += (relative - c->filtered) / 8;
        uint32_t magnitude = (uint32_t)(c->filtered < 0 ? -c->filtered : c->filtered);
        if (!c->tared) { c->confirm = 0; }
        else if (c->triggered) {
            if (magnitude < c->config.release) { c->triggered = false; c->confirm = 0; }
        } else if (magnitude > c->config.trigger) {
            int8_t sign = c->filtered < 0 ? -1 : 1;
            if (c->confirm == 0 || c->confirm_sign != sign) {
                c->crossing_tick = tick; c->crossing_sequence = seq; c->confirm = 0;
            }
            c->confirm_sign = sign;
            if (++c->confirm >= c->config.confirmations) c->triggered = true;
        } else c->confirm = 0;
    }
    if (c->mode != DYNO_RUNNING) return;
    if (c->streaming) {
        if (stream_count(c) >= DYNO_STREAM_LIMIT) {
            ++c->overflow; c->faults |= DYNO_FAULT_OVERFLOW;
            c->stream_overflows = c->stream_overflows + 1u;
            return; /* Drop newest; the sequence gap stays visible. */
        }
        stream_put(c, tick, raw, (uint8_t)((flags & DYNO_FLAG_MASK) |
                   (seq & 15u) << DYNO_ENTRY_SEQ_SHIFT));
        return;
    }
    if ((uint32_t)(c->head - c->tail) >= DYNO_FIFO_SIZE) {
        ++c->overflow; c->faults |= DYNO_FAULT_OVERFLOW;
        return; /* Drop newest; never overwrite a pinned/unread record. */
    }
    struct dyno_sample *s = &c->buf.fifo[c->head % DYNO_FIFO_SIZE];
    s->sequence = seq; s->tick = tick; s->raw = raw; s->flags = flags;
    ++c->head;
}

static uint8_t execute(struct dyno_core *c, uint8_t command, const uint8_t *p,
                       uint8_t n, uint8_t *out, uint8_t *length) {
    *length = 0;
    switch (command) {
    case DYNO_INFO:
        if (n != 0 && n != 4) return DYNO_BAD_VALUE;
        /* A board starts with epoch=0. The first master supplies a random nonce,
         * avoiding flash writes and dependence on uninitialized SRAM entropy.
         * Later INFO calls cannot change an established epoch/capture. */
        if (c->epoch == 0) {
            if (n != 4 || get32(p) == 0) return DYNO_BAD_VALUE;
            c->epoch = get32(p);
        }
        memcpy(out, c->model, 8);
        put32(out + 8, c->tick_hz); put32(out + 12, c->capabilities);
        put16(out + 16, DYNO_FIFO_SIZE); out[18] = DYNO_BLOCK_MAX;
        memcpy(out + 19, DYNO_VERSION, 5); *length = 24; break;
    case DYNO_STATUS:
        if (n) return DYNO_BAD_VALUE;
        out[0] = c->mode; out[1] = (uint8_t)c->healthy; out[2] = (uint8_t)c->triggered;
        put32(out + 3, c->streaming ? stream_count(c) : c->head - c->tail);
        put32(out + 7, c->overflow);
        put32(out + 11, c->adc_errors); put32(out + 15, c->faults);
        put32(out + 19, c->sequence); *length = 23; break;
    case DYNO_GET_CONFIG:
        if (n) return DYNO_BAD_VALUE;
        put32(out, c->config.trigger); put32(out + 4, c->config.release);
        put32(out + 8, c->config.timeout); out[12] = c->config.confirmations;
        *length = 13; break;
    case DYNO_SET_CONFIG:
        if (!settled(c)) return DYNO_BAD_STATE;
        if (n != 13 || get32(p) == 0 || get32(p) > 16777215u ||
            get32(p + 4) == 0 || get32(p + 4) >= get32(p) ||
            get32(p + 8) == 0 || get32(p + 8) >= 0x80000000u || p[12] == 0 || p[12] > 32)
            return DYNO_BAD_VALUE;
        c->config.trigger = get32(p); c->config.release = get32(p + 4);
        c->config.timeout = get32(p + 8); c->config.confirmations = p[12]; break;
    case DYNO_TARE:
        if (n) return DYNO_BAD_VALUE;
        if (!settled(c) || !c->healthy || c->triggered || c->idle_count < 16 ||
            (uint32_t)(c->idle_max - c->idle_min) > c->config.release / 2u)
            return DYNO_BAD_STATE;
        c->baseline = (int32_t)(c->idle_sum / c->idle_count);
        c->filtered = 0; c->confirm = 0; c->tared = true;
        put32(out, (uint32_t)c->baseline); *length = 4; break;
    case DYNO_ARM: {
        /* Optional mode byte: 0 = READ/ACK blocks, 1 = register-window stream. */
        if ((n != 4 && n != 5) || get32(p) == 0 || get32(p) <= c->capture) return DYNO_BAD_VALUE;
        if (n == 5 && p[4] > 1) return DYNO_BAD_VALUE;
        if (c->mode != DYNO_IDLE && c->mode != DYNO_STOPPED) return DYNO_BAD_STATE;
        if (!drained(c)) return DYNO_BAD_STATE;
        bool stream = n == 5 && p[4] == 1;
        if (stream && !(c->capabilities & DYNO_CAP_STREAM)) return DYNO_UNSUPPORTED;
        uint32_t key = dyno_critical_enter();
        c->stream_head = 0; c->stream_tail = 0;
        c->stream_overflows = 0; c->reg_seen_overflows = 0;
        c->stream_generation = c->stream_generation + 1u;
        c->reg_pending_size = 0;
        dyno_critical_exit(key);
        c->head = 0; c->tail = 0;
        c->streaming = stream;
        c->capture = get32(p); c->mode = DYNO_ARMED;
        c->block_token = 0; c->last_ack = 0; break;
    }
    case DYNO_START:
        if (n) return DYNO_BAD_VALUE;
        if (c->mode != DYNO_ARMED || !c->healthy) return DYNO_BAD_STATE;
        c->mode = DYNO_RUNNING; break;
    case DYNO_STOP:
        if (n) return DYNO_BAD_VALUE;
        if (c->mode != DYNO_RUNNING && c->mode != DYNO_ARMED && c->mode != DYNO_STOPPED)
            return DYNO_BAD_STATE;
        if (c->streaming && c->mode == DYNO_RUNNING) {
            /* Pad to a whole block so a block-reading master drains everything. */
            while (c->stream_head % DYNO_STREAM_BLOCK)
                stream_put(c, c->last_tick, 0, DYNO_ENTRY_PAD);
        }
        c->mode = DYNO_STOPPED; break;
    case DYNO_READ: {
        if (n > 1 || (n == 1 && (p[0] == 0 || p[0] > DYNO_BLOCK_MAX))) return DYNO_BAD_VALUE;
        if (c->mode != DYNO_RUNNING && c->mode != DYNO_STOPPED) return DYNO_BAD_STATE;
        if (c->streaming) return DYNO_BAD_STATE;
        uint32_t count = c->head - c->tail;
        if (count > DYNO_BLOCK_MAX) count = DYNO_BLOCK_MAX;
        if (n == 1 && count < p[0] && c->mode == DYNO_RUNNING) count = 0;
        if (!c->pending_count && count) { c->pending_count = (uint8_t)count; ++c->block_token; }
        put32(out, c->block_token); out[4] = c->pending_count;
        for (uint32_t i = 0; i < c->pending_count; ++i) {
            const struct dyno_sample *s = &c->buf.fifo[(c->tail + i) % DYNO_FIFO_SIZE];
            uint8_t *dest = out + 5 + 12u * i;
            put32(dest, s->sequence); put32(dest + 4, s->tick);
            for (unsigned b = 0; b < 3; ++b) dest[8u + b] = (uint8_t)((uint32_t)s->raw >> (8u * b));
            dest[11] = s->flags;
        }
        *length = (uint8_t)(5u + 12u * c->pending_count); break;
    }
    case DYNO_ACK:
        if (n != 4) return DYNO_BAD_VALUE;
        if (c->streaming) return DYNO_BAD_STATE;
        if (get32(p) == c->last_ack && c->last_ack) break;
        if (!c->pending_count || get32(p) != c->block_token) return DYNO_STALE;
        c->tail += c->pending_count; c->pending_count = 0;
        c->last_ack = c->block_token; break;
    case DYNO_PROBE:
        if (n) return DYNO_BAD_VALUE;
        put32(out, (uint32_t)c->raw); put32(out + 4, (uint32_t)c->baseline);
        put32(out + 8, (uint32_t)c->filtered); out[12] = (uint8_t)c->triggered;
        out[13] = (uint8_t)c->tared;
        put32(out + 14, c->crossing_tick); put32(out + 18, c->crossing_sequence);
        *length = 22; break;
    case DYNO_TIME:
        if (n) return DYNO_BAD_VALUE;
        put32(out, c->now); *length = 4; break;
    case DYNO_RESET_ERRORS:
        if (n) return DYNO_BAD_VALUE;
        if (c->mode != DYNO_IDLE && c->mode != DYNO_STOPPED) return DYNO_BAD_STATE;
        c->overflow = 0; c->adc_errors = 0; c->faults = 0; break;
    case DYNO_SET_ADC:
        if (n != 2 || p[0] > 7 || p[0] < c->adc_osr_min || p[1] > 7) return DYNO_BAD_VALUE;
        if (!c->adc_set) return DYNO_UNSUPPORTED;
        if (!settled(c)) return DYNO_BAD_STATE;
        /* A new rate or gain invalidates the idle window and the tare. */
        reset_idle_window(c); c->tared = false; c->filtered = 0; c->triggered = false;
        if (!c->adc_set(c, p[0], p[1])) {
            ++c->adc_errors; c->faults |= DYNO_FAULT_ADC; c->healthy = false;
            return DYNO_BAD_STATE;
        }
        break;
    case DYNO_GET_ADC:
        if (n) return DYNO_BAD_VALUE;
        out[0] = c->adc_osr; out[1] = c->adc_gain;
        put16(out + 2, c->adc_mode_reg); put16(out + 4, c->adc_clock_reg);
        put16(out + 6, c->adc_gain_reg); put32(out + 8, c->adc_rate_mhz);
        *length = 12; break;
    default: return DYNO_UNSUPPORTED;
    }
    return DYNO_OK;
}
void dyno_request(struct dyno_core *c, const uint8_t *r, size_t size, uint8_t out[DYNO_FRAME_SIZE]) {
    if (size == c->last_request_size && size && memcmp(r, c->last_request, size) == 0) {
        memcpy(out, c->last_response, DYNO_FRAME_SIZE); return;
    }
    memset(out, 0, DYNO_FRAME_SIZE);
    out[0] = DYNO_MAGIC0; out[1] = DYNO_MAGIC1; out[2] = DYNO_PROTOCOL;
    uint8_t result = DYNO_BAD_FRAME, length = 0;
    if (size >= DYNO_HEADER_SIZE + 2u && size <= DYNO_FRAME_SIZE) {
        out[3] = r[3]; out[4] = r[4]; out[5] = r[5];
        uint16_t crc = dyno_crc16(r, size - 2u);
        if (r[0] == DYNO_MAGIC0 && r[1] == DYNO_MAGIC1 && r[2] == DYNO_PROTOCOL && r[6] == 0 &&
            r[7] <= DYNO_PAYLOAD_MAX && size == DYNO_HEADER_SIZE + (size_t)r[7] + 2u &&
            r[size - 2u] == (uint8_t)crc && r[size - 1u] == (uint8_t)(crc >> 8)) {
            if (r[3] != DYNO_INFO && get32(r + 8) != c->epoch) result = DYNO_STALE;
            else if (r[3] >= DYNO_START && r[3] <= DYNO_ACK && get32(r + 12) != c->capture)
                result = DYNO_STALE;
            else result = execute(c, r[3], r + DYNO_HEADER_SIZE, r[7],
                                  out + DYNO_HEADER_SIZE, &length);
        }
    }
    out[6] = result; out[7] = length;
    put32(out + 8, c->epoch); put32(out + 12, c->capture);
    size_t response_size = dyno_response_size(out[3]);
    put16(out + response_size - 2u, dyno_crc16(out, response_size - 2u));
    if (result != DYNO_BAD_FRAME) {
        memcpy(c->last_request, r, size); c->last_request_size = size;
        memcpy(c->last_response, out, DYNO_FRAME_SIZE);
    }
}

bool dyno_reg_owns(uint8_t reg) {
    return reg == DYNO_REG_INT_STATUS || reg == DYNO_REG_FIFO_COUNT || reg == DYNO_REG_FIFO_DATA;
}
void dyno_reg_select(struct dyno_core *c, uint8_t reg) {
    if (dyno_reg_owns(reg)) { c->reg_selected = reg; return; }
    /* Any other transaction breaks an INT_STATUS -> FIFO_COUNT pair. */
    c->reg_selected = 0; c->reg_previous = 0;
}
size_t dyno_reg_read(struct dyno_core *c, uint32_t now, uint8_t out[48]) {
    size_t size = 0;
    uint8_t reg = c->reg_selected;
    switch (reg) {
    case DYNO_REG_INT_STATUS: {
        uint32_t seen = c->stream_overflows;
        uint8_t value = stream_count(c) >= DYNO_STREAM_BLOCK ? DYNO_INT_BLOCK_READY : 0u;
        if (seen != c->reg_seen_overflows) { value |= DYNO_INT_OVERFLOW; c->reg_seen_overflows = seen; }
        out[0] = value; size = 1; break;
    }
    case DYNO_REG_FIFO_COUNT: {
        uint32_t value;
        if (c->reg_previous == DYNO_REG_INT_STATUS) {
            /* A status query: report the bus-time latch instead of a count. */
            value = (now >> DYNO_LATCH_SHIFT) & 0x1fffu;
        } else {
            value = stream_count(c) * DYNO_STREAM_ENTRY;
        }
        out[0] = (uint8_t)(value >> 8); out[1] = (uint8_t)value; size = 2; break;
    }
    case DYNO_REG_FIFO_DATA: {
        uint32_t tail = c->stream_tail, count = c->stream_head - tail;
        atomic_signal_fence(memory_order_acquire);
        if (count > DYNO_STREAM_BLOCK) count = DYNO_STREAM_BLOCK;
        for (uint32_t i = 0; i < count; ++i)
            memcpy(out + i * DYNO_STREAM_ENTRY,
                   &c->buf.stream[((tail + i) % DYNO_STREAM_SIZE) * DYNO_STREAM_ENTRY],
                   DYNO_STREAM_ENTRY);
        c->reg_pending_end = tail + count;
        c->reg_pending_generation = c->stream_generation;
        c->reg_pending_size = (uint8_t)(count * DYNO_STREAM_ENTRY);
        size = c->reg_pending_size; break;
    }
    default: break;
    }
    c->reg_previous = reg;
    return size;
}
void dyno_reg_done(struct dyno_core *c, size_t sent) {
    /* Entries leave the ring only after the master clocked out the whole block. */
    if (c->reg_pending_size && sent >= c->reg_pending_size &&
        c->reg_pending_generation == c->stream_generation)
        c->stream_tail = c->reg_pending_end;
    c->reg_pending_size = 0;
}
