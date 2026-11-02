#include "board.h"
#include <string.h>

struct dyno_core instrument;
uint8_t board_snapshot[32];
uint8_t clock_source;
uint32_t SystemCoreClock = 48000000;
uint32_t flash_crc;
extern const uint32_t vectors[48];
static uint32_t synthetic_period = 1024;

uint32_t dyno_critical_enter(void) { return irq_lock(); }
void dyno_critical_exit(uint32_t state) { irq_unlock(state); }

/* The synthetic source follows the same OSR codes as the ADC. */
static bool synthetic_profile(struct dyno_core *core, uint8_t osr, uint8_t gain) {
    core->adc_osr = osr; core->adc_gain = gain;
    core->adc_rate_mhz = 4000000000u / (128u << osr);
    synthetic_period = (128u << osr) / 4u;
    return true;
}

void board_delay(uint32_t us) {
    uint32_t start = board_now();
    while ((uint32_t)(board_now() - start) < us) { IWDG->KR = 0xaaaau; }
}
static void remap_vectors(void) {
    SysTick->CTRL = 0;
    NVIC->ICER[0] = 0xffffffffu; NVIC->ICPR[0] = 0xffffffffu;
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    for (unsigned i = 0; i < 48; ++i) ((volatile uint32_t *)0x20000000u)[i] = vectors[i];
    __DSB(); SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) | 3u;
    __DSB(); __ISB();
}
static bool wait_bits(volatile uint32_t *reg, uint32_t mask, uint32_t expected, uint32_t limit) {
    uint32_t start = board_now();
    while ((*reg & mask) != expected) {
        if ((uint32_t)(board_now() - start) >= limit) return false;
    }
    return true;
}
static void clocks(void) {
    /* HSI48 also provides a recoverable clock if the expected 8 MHz HSE fails. */
    RCC->CR2 |= RCC_CR2_HSI48ON;
    while (!(RCC->CR2 & RCC_CR2_HSI48RDY)) {}
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    RCC->CFGR = RCC_CFGR_SW_HSI48;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI48) {}
    RCC->CR &= ~RCC_CR_PLLON;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST; RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
    TIM2->PSC = 47; TIM2->ARR = 0xffffffffu; TIM2->EGR = TIM_EGR_UG; TIM2->CR1 = TIM_CR1_CEN;
    RCC->CR |= RCC_CR_HSEON;
    if (wait_bits(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY, 30000)) {
        RCC->CFGR2 = 0;
        RCC->CFGR = RCC_CFGR_SW_HSI48 | RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6;
        RCC->CR |= RCC_CR_PLLON;
        if (wait_bits(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY, 10000)) {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            if (wait_bits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL, 10000)) clock_source = 1;
        }
    }
    RCC->CFGR3 = clock_source ? RCC_CFGR3_USBSW : 0;
    if (!clock_source) {
        RCC->APB1ENR |= RCC_APB1ENR_CRSEN;
        CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
    }
}
static uint32_t crc32_flash(void) {
    uint32_t crc = 0xffffffffu;
    const uint8_t *flash = (const uint8_t *)0x08000000u;
    for (size_t i = 0; i < 131072; ++i) {
        crc ^= flash[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
        if ((i & 1023u) == 0) IWDG->KR = 0xaaaau;
    }
    return ~crc;
}
static void snapshot(void) {
    uint8_t data[32];
    memcpy(data, "DYBD", 4);
    uint32_t words[] = {DYNO_BUILD_ID, board_now(), bus_requests, bus_errors};
    memcpy(data + 4, words, sizeof(words));
    uint16_t regs[] = {adc_id, adc_clock, adc_gain, adc_mode};
    memcpy(data + 20, regs, sizeof(regs));
    data[28] = DYNO_USE_ADC; data[29] = clock_source;
    data[30] = (uint8_t)adc_ready; data[31] = 3;
    uint32_t key = irq_lock(); memcpy(board_snapshot, data, sizeof(data)); irq_unlock(key);
}
void board_main(void) {
    remap_vectors(); clocks();
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
    GPIOC->BSRR = 1u << 13;
    GPIOC->MODER = (GPIOC->MODER & ~(3u << 26)) | (1u << 26);
    /* PB13/14 are never initialized as push-pull outputs. */
    GPIOB->OTYPER |= (1u << 13) | (1u << 14);
    GPIOB->MODER &= ~((3u << 26) | (3u << 28));
    IWDG->KR = 0xcccc; /* Start LSI before waiting for prescaler/reload updates. */
    IWDG->KR = 0x5555; IWDG->PR = 4; IWDG->RLR = 1250;
    while (IWDG->SR) { IWDG->KR = 0xaaaa; }
    IWDG->KR = 0xaaaa; IWDG->KR = 0xcccc;
    flash_crc = crc32_flash();
    dyno_init(&instrument, 0, 1000000);
    const uint32_t caps = DYNO_CAP_PAGED | DYNO_CAP_STREAM | DYNO_CAP_ADC_CONFIG;
    dyno_identity(&instrument, DYNO_USE_ADC ? "DYNO-ADC" : "DYNO-SYN",
                  caps | (DYNO_USE_ADC ? DYNO_CAP_ADC : DYNO_CAP_SYNTHETIC));
    if (DYNO_USE_ADC) adc_init();
    else {
        instrument.adc_set = synthetic_profile; instrument.adc_osr_min = 3;
        synthetic_profile(&instrument, 5, 7);
    }
    i2c_init(); usb_init(); __enable_irq();
    uint32_t next = board_now(), last_blink = next, last_snapshot = next;
    while (true) {
        uint32_t now = board_now();
        if (!DYNO_USE_ADC && (int32_t)(now - next) >= 0) {
            /* Actual service timestamps; gaps are flagged rather than backfilled. */
            uint8_t flags = (uint32_t)(now - next) >= synthetic_period ? DYNO_ADC_MISSED : 0;
            int32_t raw = 100000 + (int32_t)((instrument.sequence % 100u) * 100u) - 5000;
            dyno_sample(&instrument, now, raw, flags); next = now + synthetic_period;
        }
        if (DYNO_USE_ADC) adc_service();
        dyno_clock(&instrument, board_now());
        i2c_service(); usb_service();
        if ((uint32_t)(now - last_snapshot) >= 1000) { snapshot(); last_snapshot = now; }
        if ((uint32_t)(now - last_blink) >= 500000) {
            GPIOC->ODR ^= 1u << 13; last_blink = now;
        }
        IWDG->KR = 0xaaaa;
    }
}
