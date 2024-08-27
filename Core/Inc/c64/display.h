#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern uint16_t color_bright;
extern uint16_t color_dark;
extern uint8_t display_zoom;

void display_clear();
void display_set_pixel(uint16_t x, uint16_t y, bool state);
