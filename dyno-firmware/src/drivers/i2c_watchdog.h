#ifndef DYNO_I2C_WATCHDOG_H
#define DYNO_I2C_WATCHDOG_H
#include <stdbool.h>
#include <stdint.h>

/* Call with interrupts excluded when sharing this state with the I2C ISR. */
struct dyno_i2c_watchdog {
    uint32_t last_progress;
    bool active;
};
static inline void dyno_i2c_watchdog_reset(struct dyno_i2c_watchdog *watchdog,
                                        uint32_t now) {
    watchdog->last_progress = now;
    watchdog->active = false;
}
static inline void dyno_i2c_watchdog_progress(struct dyno_i2c_watchdog *watchdog,
                                           uint32_t now) {
    watchdog->last_progress = now;
    watchdog->active = true;
}
static inline bool dyno_i2c_watchdog_expired(struct dyno_i2c_watchdog *watchdog,
                                          uint32_t now, bool busy) {
    if (!busy) {
        watchdog->active = false;
        return false;
    }
    if (!watchdog->active) {
        /* BUSY precedes the address IRQ: idle time is never a bus stall. */
        dyno_i2c_watchdog_progress(watchdog, now);
        return false;
    }
    return (uint32_t)(now - watchdog->last_progress) > 100000u;
}
#endif
