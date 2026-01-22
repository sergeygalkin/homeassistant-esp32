#pragma once
#include <stdint.h>

void ws2812_init(void);
void ws2812_set(uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_brightness(uint8_t brightness); // 0..255
