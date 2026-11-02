#ifndef DYNO_H
#define DYNO_H

/* Portable acquisition and protocol engine. No MMIO, startup, or board driver.
 * The platform MUST serialize all API calls except the dyno_reg_* register
 * window, which may run in the bus ISR (see below).
 * epoch must be unique across resets; zero lets the first INFO negotiate it.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DYNO_FRAME_SIZE 128u
#define DYNO_HEADER_SIZE 16u
#define DYNO_PAYLOAD_MAX 110u
#define DYNO_FIFO_SIZE 256u
#define DYNO_BLOCK_MAX 8u
#define DYNO_PROTOCOL 1u
#define DYNO_MAGIC0 'D'
#define DYNO_MAGIC1 'Y'
#define DYNO_VERSION "0.2.0"

/* Stream ring: packed 6-byte entries read in 8-entry (48-byte) blocks. Seven
 * slots stay reserved so STOP can always pad the final partial block. */
#define DYNO_STREAM_ENTRY 6u
#define DYNO_STREAM_BLOCK 8u
#define DYNO_STREAM_SIZE 680u
#define DYNO_STREAM_LIMIT (DYNO_STREAM_SIZE - (DYNO_STREAM_BLOCK - 1u))

/* Register window, compatible with a FIFO accelerometer's polling sequence. */
#define DYNO_REG_INT_STATUS 0x3au
#define DYNO_REG_FIFO_COUNT 0x72u
#define DYNO_REG_FIFO_DATA 0x74u
#define DYNO_INT_OVERFLOW 0x10u
#define DYNO_INT_BLOCK_READY 0x01u
#define DYNO_LATCH_SHIFT 4u

enum dyno_command {
    DYNO_INFO = 1, DYNO_STATUS, DYNO_GET_CONFIG, DYNO_SET_CONFIG,
    DYNO_TARE, DYNO_ARM, DYNO_START, DYNO_STOP, DYNO_READ, DYNO_ACK,
    DYNO_PROBE, DYNO_TIME, DYNO_RESET_ERRORS, DYNO_SET_ADC, DYNO_GET_ADC
};
enum dyno_result { DYNO_OK, DYNO_BAD_FRAME, DYNO_BAD_STATE,
                   DYNO_BAD_VALUE, DYNO_STALE, DYNO_UNSUPPORTED };
enum dyno_mode { DYNO_IDLE, DYNO_ARMED, DYNO_RUNNING, DYNO_STOPPED };
enum dyno_sample_flags { DYNO_SATURATED = 1, DYNO_ADC_CRC = 2,
                         DYNO_ADC_MISSED = 4, DYNO_FLAG_MASK = 7 };
enum dyno_entry_bits { DYNO_ENTRY_SEQ_SHIFT = 3, DYNO_ENTRY_PAD = 0x80 };
enum dyno_fault { DYNO_FAULT_OVERFLOW = 1, DYNO_FAULT_ADC = 2,
                  DYNO_FAULT_STALE = 4 };
enum dyno_capability { DYNO_CAP_SYNTHETIC = 1, DYNO_CAP_ADC = 2, DYNO_CAP_PAGED = 4,
                       DYNO_CAP_STREAM = 8, DYNO_CAP_ADC_CONFIG = 16 };

struct dyno_sample { uint32_t sequence, tick; int32_t raw; uint8_t flags; };
struct dyno_config { uint32_t trigger, release, timeout; uint8_t confirmations; };
struct dyno_core;
/* Applies an ADC profile (OSR code 0-7 = 128..16384, gain code 0-7 = 1..128)
 * and updates the adc_* readback fields. Returns false if readback failed. */
typedef bool (*dyno_adc_setter)(struct dyno_core *core, uint8_t osr, uint8_t gain);

struct dyno_core {
    union {
        struct dyno_sample fifo[DYNO_FIFO_SIZE];
        uint8_t stream[DYNO_STREAM_SIZE * DYNO_STREAM_ENTRY];
    } buf;
    struct dyno_config config;
    uint32_t epoch, capture, tick_hz, now, sequence;
    uint32_t head, tail, overflow, adc_errors, faults;
    uint32_t block_token, last_ack, last_tick, crossing_tick, crossing_sequence;
    int32_t raw, baseline, filtered, idle_min, idle_max;
    int64_t idle_sum;
    int32_t idle_history[32];
    uint8_t mode, pending_count, idle_count, idle_index, confirm;
    int8_t confirm_sign;
    bool triggered, healthy, have_sample, tared, streaming;
    uint8_t last_request[DYNO_FRAME_SIZE], last_response[DYNO_FRAME_SIZE];
    size_t last_request_size;
    char model[8];
    uint32_t capabilities;
    /* Stream ring. The foreground owns stream_head and stream_overflows; the
     * bus ISR owns stream_tail and the reg_* state. */
    volatile uint32_t stream_head, stream_tail, stream_overflows, stream_generation;
    uint32_t reg_seen_overflows, reg_pending_end, reg_pending_generation;
    uint8_t reg_selected, reg_previous, reg_pending_size;
    /* ADC profile, maintained by the platform's setter. */
    dyno_adc_setter adc_set;
    uint8_t adc_osr, adc_gain, adc_osr_min;
    uint16_t adc_mode_reg, adc_clock_reg, adc_gain_reg;
    uint32_t adc_rate_mhz;
};

uint16_t dyno_crc16(const uint8_t *data, size_t size);
size_t dyno_response_size(uint8_t command);
void dyno_init(struct dyno_core *core, uint32_t epoch, uint32_t tick_hz);
void dyno_identity(struct dyno_core *core, const char model[8], uint32_t capabilities);
void dyno_clock(struct dyno_core *core, uint32_t now);
/* Every observed DRDY consumes one sequence, including invalid/lost events.
 * timestamp is actual event time, not a reconstructed fixed-rate timestamp.
 */
void dyno_sample(struct dyno_core *core, uint32_t tick, int32_t raw, uint8_t flags);
void dyno_request(struct dyno_core *core, const uint8_t *request, size_t size,
                  uint8_t response[DYNO_FRAME_SIZE]);

/* Register window, callable from the bus ISR concurrently with the foreground
 * API. select: a one-byte register write. read: at the read address, fills
 * out (up to 48 bytes) and returns its length; now is latched by a
 * FIFO_COUNT read that directly follows an INT_STATUS read. done: at STOP,
 * with the number of bytes the master clocked out. */
void dyno_reg_select(struct dyno_core *core, uint8_t reg);
bool dyno_reg_owns(uint8_t reg);
size_t dyno_reg_read(struct dyno_core *core, uint32_t now, uint8_t out[48]);
void dyno_reg_done(struct dyno_core *core, size_t sent);

/* Short critical sections around foreground updates of ISR-owned state.
 * The platform provides these; native builds may make them no-ops. */
uint32_t dyno_critical_enter(void);
void dyno_critical_exit(uint32_t state);
#endif
