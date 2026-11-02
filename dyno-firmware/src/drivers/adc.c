#include "board.h"
#include "ads131m02.h"
#include <stdatomic.h>

/* Boot profile: channel 0, gain 128, OSR 4096 -> 976.5625 SPS at the 4 MHz
 * modulator clock (8 MHz CLKIN). No internal gain calibration. */
#define DEFAULT_OSR 5u
#define DEFAULT_GAIN 7u
#define MIN_OSR 3u          /* 1024 -> 3906 SPS; faster rates cannot be streamed */
#define QUEUE_SIZE 32u
#define SETTLE_CONVERSIONS 4u

uint16_t adc_id, adc_mode, adc_clock, adc_gain;
bool adc_ready;

/* DRDY ISR -> foreground queue. The ISR owns queue_head and dropped. */
struct conversion { uint32_t tick, dropped; int32_t raw; uint8_t flags; };
static struct conversion queue[QUEUE_SIZE];
static volatile uint32_t queue_head, queue_tail;
static uint32_t dropped;
static uint8_t settling;

static bool spi_wait(uint32_t mask, bool set, uint32_t start) {
    while (((SPI1->SR & mask) != 0) != set)
        if ((uint32_t)(board_now() - start) >= 1000u) return false;
    return true;
}
static bool exchange(const uint8_t out[12], uint8_t in[12]) {
    uint32_t start = board_now();
    GPIOA->BSRR = 1u << 31;
    for (unsigned i = 0; i < 12; ++i) {
        if (!spi_wait(SPI_SR_TXE, true, start)) goto error;
        *(volatile uint8_t *)&SPI1->DR = out[i];
        if (!spi_wait(SPI_SR_RXNE, true, start)) goto error;
        in[i] = *(volatile uint8_t *)&SPI1->DR;
    }
    if (!spi_wait(SPI_SR_BSY, false, start)) goto error;
    GPIOA->BSRR = 1u << 15; return true;
error:
    GPIOA->BSRR = 1u << 15; return false;
}
static bool command(uint16_t cmd, uint16_t value) {
    uint8_t out[12], in[12]; ads131_command(out, cmd, value);
    bool ok = exchange(out, in); board_delay(5); return ok;
}
static bool read_register(uint8_t reg, uint16_t *value) {
    uint8_t out[12] = {0}, in[12];
    if (!command((uint16_t)(0xa000u | (uint16_t)reg << 7), 0) || !exchange(out, in)) return false;
    *value = (uint16_t)((uint16_t)in[0] << 8 | in[1]); board_delay(5); return true;
}
static bool write_register(uint8_t reg, uint16_t value) {
    return command((uint16_t)(0x6000u | (uint16_t)reg << 7), value);
}
/* Register writes and readback only: well under a millisecond, so a held
 * bus read never stretches toward the master's transaction timeout. */
static bool configure(uint8_t osr, uint8_t gain) {
    uint16_t clock = (uint16_t)(0x0100u | (uint16_t)osr << 2 | 2u);
    if (!write_register(2, 0x0510) || !write_register(3, clock) || !write_register(4, gain) ||
        !read_register(2, &adc_mode) || !read_register(3, &adc_clock) || !read_register(4, &adc_gain))
        return false;
    instrument.adc_mode_reg = adc_mode; instrument.adc_clock_reg = adc_clock;
    instrument.adc_gain_reg = adc_gain;
    if ((adc_mode & 0x3f3fu) != 0x0510 || adc_clock != clock || adc_gain != gain) return false;
    instrument.adc_osr = osr; instrument.adc_gain = gain;
    instrument.adc_rate_mhz = 4000000000u / (128u << osr);
    return true;
}
static bool set_profile(struct dyno_core *core, uint8_t osr, uint8_t gain) {
    (void)core;
    NVIC_DisableIRQ(EXTI4_15_IRQn);
    adc_ready = false;
    bool ok = configure(osr, gain);
    queue_tail = queue_head; /* Discard conversions from the previous profile. */
    settling = SETTLE_CONVERSIONS;
    EXTI->PR = 1u << 6;
    if (ok) { NVIC_EnableIRQ(EXTI4_15_IRQn); adc_ready = true; }
    return ok;
}
void adc_init(void) {
    GPIOA->BSRR = 1u << 15;
    GPIOA->MODER = (GPIOA->MODER & ~((3u << 30) | (3u << 6))) | (1u << 30) | (2u << 6);
    GPIOA->AFR[0] &= ~(15u << 12);
    GPIOB->MODER = (GPIOB->MODER & ~((3u << 6) | (3u << 8) | (3u << 10) | (3u << 12))) |
                    (2u << 6) | (2u << 8) | (2u << 10);
    GPIOB->AFR[0] &= ~((15u << 12) | (15u << 16) | (15u << 20));
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << 12)) | (1u << 12);
    GPIOB->OSPEEDR |= (3u << 6) | (3u << 8) | (3u << 10);
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM15EN;
    TIM15->PSC = 2; TIM15->ARR = 1; TIM15->CCR2 = 1;
    TIM15->CCMR1 = (6u << 12) | TIM_CCMR1_OC2PE;
    TIM15->CCER = TIM_CCER_CC2E; TIM15->BDTR = TIM_BDTR_MOE;
    TIM15->EGR = TIM_EGR_UG; TIM15->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_BR_0;
    SPI1->CR2 = (7u << 8) | SPI_CR2_FRXTH; SPI1->CR1 |= SPI_CR1_SPE;
    instrument.adc_set = set_profile; instrument.adc_osr_min = MIN_OSR;
    board_delay(10000);
    if (!command(0x0011, 0)) goto error;
    board_delay(2000);
    if (!command(0x0033, 0)) goto error;
    board_delay(2000);
    if (!read_register(0, &adc_id) || (adc_id & 0xff00u) != 0x2200u) goto error;
    if (!configure(DEFAULT_OSR, DEFAULT_GAIN)) goto error;
    board_delay(30000);
    uint8_t zeros[12] = {0}, discard[12];
    if (!exchange(zeros, discard) || !exchange(zeros, discard)) goto error;
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~(15u << 8)) | (1u << 8);
    EXTI->RTSR &= ~(1u << 6); EXTI->FTSR |= 1u << 6;
    EXTI->PR = 1u << 6; EXTI->IMR |= 1u << 6;
    NVIC_SetPriority(EXTI4_15_IRQn, 0); NVIC_EnableIRQ(EXTI4_15_IRQn);
    adc_ready = true; return;
error:
    instrument.faults |= DYNO_FAULT_ADC; ++instrument.adc_errors; adc_ready = false;
}
/* Highest priority: read each conversion at its DRDY edge so a foreground
 * stall cannot lose it. The edge time is taken before the SPI transfer. */
void EXTI4_15_IRQHandler(void) {
    if (!(EXTI->PR & (1u << 6))) return;
    uint32_t tick = board_now();
    EXTI->PR = 1u << 6;
    uint8_t out[12] = {0}, in[12]; int32_t raw = 0; uint8_t flags = 0;
    if (!exchange(out, in)) flags = DYNO_ADC_MISSED;
    else if (!ads131_decode(in, &raw)) flags = DYNO_ADC_CRC;
    uint32_t head = queue_head;
    if (head - queue_tail >= QUEUE_SIZE) { ++dropped; return; }
    struct conversion *slot = &queue[head % QUEUE_SIZE];
    slot->tick = tick; slot->raw = raw; slot->flags = flags; slot->dropped = dropped;
    dropped = 0;
    atomic_signal_fence(memory_order_release);
    queue_head = head + 1u;
}
void adc_service(void) {
    while (queue_tail != queue_head) {
        atomic_signal_fence(memory_order_acquire);
        struct conversion e = queue[queue_tail % QUEUE_SIZE];
        queue_tail = queue_tail + 1u;
        /* Lost conversions keep their sequence numbers; the next is flagged. */
        if (e.dropped) { instrument.sequence += e.dropped; e.flags |= DYNO_ADC_MISSED; }
        if (settling) { --settling; ++instrument.sequence; continue; }
        dyno_sample(&instrument, e.tick, e.raw, e.flags);
    }
}
