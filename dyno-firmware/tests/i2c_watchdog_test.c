#include "i2c_watchdog.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    struct dyno_i2c_watchdog watchdog;
    dyno_i2c_watchdog_reset(&watchdog, 0);
    /* Regression: first START after seconds of idle, with
       BUSY asserted but the address byte/ADDR interrupt not yet complete. */
    assert(!dyno_i2c_watchdog_expired(&watchdog, 1000000u, false));
    assert(!dyno_i2c_watchdog_expired(&watchdog, 2000000u, true));
    assert(!dyno_i2c_watchdog_expired(&watchdog, 2000080u, true));
    dyno_i2c_watchdog_progress(&watchdog, 2000090u);
    assert(!dyno_i2c_watchdog_expired(&watchdog, 2100090u, true));
    assert(dyno_i2c_watchdog_expired(&watchdog, 2100091u, true));

    /* STOP/idle followed by another START gets a fresh timeout. */
    assert(!dyno_i2c_watchdog_expired(&watchdog, 3000000u, false));
    assert(!dyno_i2c_watchdog_expired(&watchdog, 4000000u, true));
    dyno_i2c_watchdog_progress(&watchdog, 4099999u);
    assert(!dyno_i2c_watchdog_expired(&watchdog, 4100001u, true));

    /* ADDR may interrupt before the first foreground BUSY observation. */
    dyno_i2c_watchdog_reset(&watchdog, 0);
    dyno_i2c_watchdog_progress(&watchdog, 5000000u);
    assert(!dyno_i2c_watchdog_expired(&watchdog, 5000001u, true));
    assert(dyno_i2c_watchdog_expired(&watchdog, 5100001u, true));

    /* A genuine stall still times out across the 32-bit microsecond wrap. */
    dyno_i2c_watchdog_reset(&watchdog, 0);
    assert(!dyno_i2c_watchdog_expired(&watchdog, UINT32_MAX - 15u, true));
    assert(!dyno_i2c_watchdog_expired(&watchdog, 16u, true));
    assert(dyno_i2c_watchdog_expired(&watchdog, 100000u, true));
    puts("I2C watchdog: idle START, progress, stalled bus and timer wrap passed");
    return 0;
}
