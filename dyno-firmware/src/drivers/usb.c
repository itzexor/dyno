#include "board.h"
#include "tusb.h"
#include <string.h>

static const tusb_desc_device_t device = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = 64, .idVendor = 0x0483, .idProduct = 0x5740,
    .bcdDevice = 0x0100, .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3,
    .bNumConfigurations = 1
};
static const uint8_t configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(0, 4, 0x82, 8, 0x01, 0x81, 64)
};
const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&device; }
const uint8_t *tud_descriptor_configuration_cb(uint8_t index) { (void)index; return configuration; }
const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t result[40];
    char serial[25];
    const char *text;
    size_t count;
    if (index == 0) { result[1] = 0x0409; count = 1; }
    else {
        if (index == 1) text = "Dyno";
        else if (index == 2) text = "Dyno load cell interface";
        else if (index == 4) text = "Bench diagnostics";
        else if (index == 3) {
            const uint8_t *uid = (const uint8_t *)0x1ffff7acu;
            for (unsigned i = 0; i < 12; ++i) {
                serial[2u*i] = "0123456789ABCDEF"[uid[i] >> 4];
                serial[2u*i+1u] = "0123456789ABCDEF"[uid[i] & 15u];
            }
            serial[24] = 0; text = serial;
        } else return NULL;
        count = strlen(text); if (count > 39) count = 39;
        for (size_t i = 0; i < count; ++i) result[i+1] = (uint8_t)text[i];
    }
    result[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2u * count + 2u));
    return result;
}
void USB_IRQHandler(void) { tud_int_handler(0); }
uint32_t tusb_time_millis_api(void) { return board_now() / 1000u; }
void usb_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;
    RCC->APB1RSTR |= RCC_APB1RSTR_USBRST; RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST;
    NVIC_SetPriority(USB_IRQn, 2);
    tusb_init();
}
static void hex32(char out[8], uint32_t value) {
    for (unsigned i = 0; i < 8; ++i) out[i] = "0123456789abcdef"[(value >> (28u - 4u*i)) & 15u];
}
void usb_service(void) {
    static uint8_t rx[128], reply[128];
    static size_t used, expected, send_size, sent;
    static uint32_t last_byte;
    tud_task();
    if (!tud_connected() || (used && (uint32_t)(board_now() - last_byte) > 500000)) {
        used = 0; expected = 0;
    }
    if (send_size) {
        uint32_t n = tud_cdc_write(reply + sent, (uint32_t)(send_size - sent));
        sent += n; tud_cdc_write_flush();
        if (sent == send_size) { send_size = 0; sent = 0; }
        return;
    }
    unsigned budget = 128;
    while (budget-- && tud_cdc_available()) {
        uint8_t byte;
        if (tud_cdc_read(&byte, 1) != 1) break;
        last_byte = board_now();
        if (used == sizeof(rx)) { used = 0; expected = 0; }
        rx[used++] = byte;
        if (used >= 2 && rx[0] == DYNO_MAGIC0 && rx[1] == DYNO_MAGIC1) {
            if (used == 8) {
                expected = 18u + rx[7];
                if (expected > sizeof(rx)) { used = 0; expected = 0; }
            }
            if (expected && used == expected) {
                dyno_request(&instrument, rx, used, reply);
                send_size = dyno_response_size(reply[3]); used = 0; expected = 0; break;
            }
        } else if (byte == '\n') {
            if (used == 6 && memcmp(rx, "board\n", 6) == 0) {
                memcpy(reply, board_snapshot, 32); send_size = 32;
            } else if (used == 10 && memcmp(rx, "checksums\n", 10) == 0) {
                memcpy(reply, "flash_crc32=", 12); hex32((char *)reply + 12, flash_crc);
                memcpy(reply + 20, " build_id=", 10); hex32((char *)reply + 30, DYNO_BUILD_ID);
                memcpy(reply + 38, "\r\n", 2); send_size = 40;
            } else {
                const char *text = DYNO_USE_ADC ? "dyno " DYNO_VERSION " adc address=0x2a\r\n"
                                                 : "dyno " DYNO_VERSION " synthetic address=0x2a\r\n";
                send_size = strlen(text); memcpy(reply, text, send_size);
            }
            used = 0; expected = 0; break;
        }
    }
}
