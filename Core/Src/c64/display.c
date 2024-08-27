#include "display.h"

uint16_t color_bright = (0b01110 << 11) | (0b011011 << 5) | 0b11101;
uint16_t color_dark = (0b00101 << 11) | (0b001000 << 5) | 0b11010;
uint8_t display_zoom = 0;

void display_clear()
{
	for (uint16_t x = 0; x < 500; x++)
	{
		for (uint16_t y = 0; y < 500; y++)
		{
			display_set_pixel(x, y, 0);
		}
	}
}

__attribute__((weak)) void display_set_pixel(uint16_t x, uint16_t y, bool state)
{
}
