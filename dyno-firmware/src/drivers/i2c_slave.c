#include "board.h"
#include "i2c_watchdog.h"
#include <string.h>

volatile uint32_t bus_requests, bus_errors;
static struct dyno_link link;
static uint8_t rx[128], mailbox[128], tx[128];
static volatile size_t rx_size, mailbox_size;
static size_t tx_size, tx_index;
static volatile bool pending, held, writing;
static bool invalid_rx, register_read;
/* What the next master read returns: the framed reply, the diagnostic
 * snapshot, or a register-window value computed in the ISR. */
static enum { SOURCE_REPLY, SOURCE_BOARD, SOURCE_REGISTER } source;
static struct dyno_i2c_watchdog watchdog;

static void reset_bus(void) {
    I2C2->CR1 = 0;
    I2C2->ICR = 0x3f38u;
    I2C2->TIMINGR = 0x20303e5du; /* slave uses external SCL; conservative filter delays */
    I2C2->OAR1 = (DYNO_I2C_ADDRESS << 1) | I2C_OAR1_OA1EN;
    I2C2->CR2 = 0;
    I2C2->CR1 = I2C_CR1_PE | I2C_CR1_ADDRIE | I2C_CR1_RXIE |
                 I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE;
    rx_size = 0; held = false; writing = false; invalid_rx = false;
    tx_index = 0; register_read = false;
    dyno_i2c_watchdog_reset(&watchdog, board_now());
}
void i2c_init(void) {
    dyno_link_init(&link, &instrument);
    GPIOB->OTYPER |= (1u << 13) | (1u << 14);
    GPIOB->PUPDR &= ~((3u << 26) | (3u << 28));
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~((15u << 20) | (15u << 24))) | (5u << 20) | (5u << 24);
    GPIOB->OSPEEDR |= (3u << 26) | (3u << 28);
    GPIOB->MODER = (GPIOB->MODER & ~((3u << 26) | (3u << 28))) | (2u << 26) | (2u << 28);
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST; RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    reset_bus(); NVIC_SetPriority(I2C2_IRQn, 1); NVIC_EnableIRQ(I2C2_IRQn);
}
static void receive_end(void) {
    if (!writing) return;
    writing = false;
    if (invalid_rx || !rx_size) { if (invalid_rx) ++bus_errors; return; }
    if (rx_size == 1 && dyno_reg_owns(rx[0])) {
        dyno_reg_select(&instrument, rx[0]); source = SOURCE_REGISTER; return;
    }
    dyno_reg_select(&instrument, 0);
    if (rx_size == 2 && rx[0] == 0xf0 && rx[1] < 4) {
        link.page = rx[1]; link.paged = true; source = SOURCE_REPLY; return;
    }
    if (rx_size == 1 && rx[0] == 0xf1) { source = SOURCE_BOARD; return; }
    if (pending) { ++bus_errors; return; }
    link.paged = false; link.page = 0;
    memcpy(mailbox, rx, rx_size); mailbox_size = rx_size;
    __DMB(); pending = true; source = SOURCE_REPLY;
}
static void begin_read(uint32_t now) {
    if (source == SOURCE_REGISTER) {
        tx_size = dyno_reg_read(&instrument, now, tx); register_read = true;
    } else if (source == SOURCE_BOARD) { memcpy(tx, board_snapshot, 32); tx_size = 32; }
    else {
        tx_size = link.paged ? 32u : dyno_response_size(link.reply[3]);
        for (size_t i = 0; i < tx_size; ++i) tx[i] = dyno_link_byte(&link, i);
    }
    tx_index = 0; held = false;
    /* Flush a byte prefetched during an earlier master read. */
    I2C2->ISR = I2C_ISR_TXE;
    I2C2->CR1 |= I2C_CR1_TXIE;
    I2C2->ICR = I2C_ICR_ADDRCF;
}
void I2C2_IRQHandler(void) {
    uint32_t status = I2C2->ISR, now = board_now();
    dyno_i2c_watchdog_progress(&watchdog, now);
    if (status & I2C_ISR_RXNE) {
        uint8_t byte = (uint8_t)I2C2->RXDR;
        if (rx_size < sizeof(rx)) rx[rx_size++] = byte;
        else { invalid_rx = true; I2C2->CR2 |= I2C_CR2_NACK; }
    }
    if (status & (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)) {
        ++bus_errors; reset_bus(); return;
    }
    if (status & I2C_ISR_NACKF) {
        I2C2->CR1 &= ~I2C_CR1_TXIE; I2C2->ICR = I2C_ICR_NACKCF;
    }
    if (status & I2C_ISR_STOPF) {
        receive_end(); I2C2->CR1 &= ~I2C_CR1_TXIE;
        if (register_read) { dyno_reg_done(&instrument, tx_index); register_read = false; }
        I2C2->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
        if (held) { held = false; I2C2->ICR = I2C_ICR_ADDRCF; }
        I2C2->CR1 |= I2C_CR1_ADDRIE;
    }
    if ((status & I2C_ISR_ADDR) && (I2C2->CR1 & I2C_CR1_ADDRIE)) {
        receive_end();
        I2C2->CR1 &= ~I2C_CR1_TXIE;
        if (status & I2C_ISR_DIR) {
            /* Register reads never wait on the foreground: the master's
               timestamp brackets this ISR, not a queued framed request. */
            if (pending && source != SOURCE_REGISTER) { held = true; I2C2->CR1 &= ~I2C_CR1_ADDRIE; }
            else begin_read(now);
        } else {
            rx_size = 0; invalid_rx = false; writing = true;
            I2C2->CR2 &= ~I2C_CR2_NACK;
            I2C2->ICR = I2C_ICR_ADDRCF;
        }
    }
    if ((I2C2->ISR & I2C_ISR_TXIS) && (I2C2->CR1 & I2C_CR1_TXIE))
        I2C2->TXDR = tx_index < tx_size ? tx[tx_index++] : 0xff;
}
void i2c_service(void) {
    if (pending) {
        uint8_t response[128];
        dyno_request(&instrument, mailbox, mailbox_size, response);
        uint32_t key = irq_lock();
        memcpy(link.reply, response, sizeof(response));
        /* A page selector may already have arrived after the command STOP. */
        if (!link.paged) link.page = 0;
        pending = false; ++bus_requests;
        if (held) { begin_read(board_now()); I2C2->CR1 |= I2C_CR1_ADDRIE; }
        irq_unlock(key);
    }
    /* START sets BUSY before ADDR can interrupt. Start the stall timer on that
       transition, and keep the time/state snapshot atomic with ISR progress. */
    uint32_t key = irq_lock();
    bool busy = (I2C2->ISR & I2C_ISR_BUSY) != 0;
    if (dyno_i2c_watchdog_expired(&watchdog, board_now(), busy)) {
        ++bus_errors; reset_bus();
    }
    irq_unlock(key);
}
