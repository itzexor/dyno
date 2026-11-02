#include "dyno.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    assert(dyno_crc16((const uint8_t *)"123456789", 9) == 0x29b1u);
    struct dyno_core c;
    dyno_init(&c, 7, 1000000);
    assert(sizeof(c) < 8192); /* leaves headroom in the verified 16 KiB SRAM */
    dyno_sample(&c, 0xfffffff0u, 100, 0);
    dyno_clock(&c, 0x20u);
    assert(c.healthy);
    dyno_clock(&c, 1000020u);
    assert(!c.healthy && (c.faults & DYNO_FAULT_STALE));
    dyno_sample(&c, 1000030, INT32_MAX, 0);
    assert(!c.healthy && c.raw == 8388607 && c.adc_errors == 1);
    uint8_t input[256], out[128];
    memset(input, 0xa5, sizeof(input));
    for (size_t n = 0; n <= sizeof(input); ++n) {
        dyno_request(&c, input, n, out);
        assert(out[6] == DYNO_BAD_FRAME);
        size_t last = dyno_response_size(out[3]) - 2u;
        assert(dyno_crc16(out, last) == ((uint16_t)out[last] | (uint16_t)((uint16_t)out[last + 1u] << 8)));
    }
    printf("C core: CRC vector, bounded malformed requests, stale/wrapped clock, saturation, %zu-byte state passed\n", sizeof(c));
    return 0;
}
