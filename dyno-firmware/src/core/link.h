#ifndef DYNO_LINK_H
#define DYNO_LINK_H
#include "dyno.h"

/* Complete-transaction adapter; the board serializes write/publish against
 * the I2C ISR. F0,page selects a frozen 32-byte window without re-executing
 * the original command, for I2C masters limited to short reads. */
struct dyno_link {
    struct dyno_core *core;
    uint8_t reply[DYNO_FRAME_SIZE];
    uint8_t page;
    bool paged;
};
void dyno_link_init(struct dyno_link *link, struct dyno_core *core);
void dyno_link_write(struct dyno_link *link, const uint8_t *data, size_t size);
uint8_t dyno_link_byte(const struct dyno_link *link, size_t index);
#endif
