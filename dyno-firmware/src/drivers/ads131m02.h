#ifndef ADS131M02_H
#define ADS131M02_H
#include <stdbool.h>
#include <stdint.h>
void ads131_command(uint8_t frame[12], uint16_t command, uint16_t value);
bool ads131_decode(const uint8_t frame[12], int32_t *channel0);
#endif
