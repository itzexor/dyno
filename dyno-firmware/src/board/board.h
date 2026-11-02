#ifndef DYNO_BOARD_H
#define DYNO_BOARD_H
#include "stm32f072xb.h"
#include "dyno.h"
#include "link.h"

#ifndef DYNO_I2C_ADDRESS
#define DYNO_I2C_ADDRESS 0x2au
#endif
#ifndef DYNO_USE_ADC
#define DYNO_USE_ADC 0
#endif
#ifndef DYNO_BUILD_ID
#define DYNO_BUILD_ID 0u
#endif

extern struct dyno_core instrument;
extern uint8_t board_snapshot[32];
extern volatile uint32_t bus_requests, bus_errors;
extern uint16_t adc_id, adc_mode, adc_clock, adc_gain;
extern bool adc_ready;
extern uint8_t clock_source;
extern uint32_t flash_crc;
extern uint32_t SystemCoreClock;

static inline uint32_t board_now(void) { return TIM2->CNT; }
static inline uint32_t irq_lock(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static inline void irq_unlock(uint32_t p) { __set_PRIMASK(p); }
void board_delay(uint32_t us);
void i2c_init(void);
void i2c_service(void);
void adc_init(void);
void adc_service(void);
void usb_init(void);
void usb_service(void);
#endif
