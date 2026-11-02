#include "ads131m02.h"
#include "dyno.h"
#include <string.h>

void ads131_command(uint8_t frame[12], uint16_t command, uint16_t value) {
    memset(frame, 0, 12);
    frame[0] = (uint8_t)(command >> 8); frame[1] = (uint8_t)command;
    frame[3] = (uint8_t)(value >> 8); frame[4] = (uint8_t)value;
}
bool ads131_decode(const uint8_t frame[12], int32_t *channel0) {
    uint16_t full = (uint16_t)((uint16_t)frame[9] << 8 | frame[10]);
    uint16_t short_crc = (uint16_t)((uint16_t)frame[6] << 8 | frame[7]);
    bool valid = (frame[11] == 0 && dyno_crc16(frame, 9) == full) ||
                 (frame[8] == 0 && dyno_crc16(frame, 6) == short_crc);
    uint32_t raw = (uint32_t)frame[3] << 16 | (uint32_t)frame[4] << 8 | frame[5];
    *channel0 = (int32_t)(raw & 0x7fffffu) - (int32_t)(raw & 0x800000u);
    return valid;
}
