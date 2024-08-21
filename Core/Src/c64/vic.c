#include "vic.h"

uint8_t vic_reg[48];
uint8_t vic_line = 0;

int8_t vic_read(uint16_t addr_off)
{
	if (addr_off == 0x12)
		return vic_line = (vic_line + 1) % 24;
	else if (addr_off == 0x19)
		return 0; // NTSC
	else
		return vic_reg[addr_off % 0x30];
}

void vic_write(uint16_t addr_off, int8_t d)
{
	vic_reg[addr_off % 0x30] = d;
}

void vic_write_screen(uint16_t addr_off, int8_t d)
{
	if (addr_off < 1000)
	{
		uint16_t screen_x = addr_off % 40;
		uint16_t screen_y = addr_off / 40;
		if (display_init_write(8 * screen_x, 8 * screen_y))
		{
			for (uint8_t char_y = 0; char_y < 8; char_y++)
			{
				uint16_t char_offset = (uint8_t)vic_reg[0x18] >> 1 ? 0x0000 : 0x0800;
				uint8_t char_line = characters[(uint8_t)d * 8 + char_y + char_offset];
				for (uint8_t char_x = 0; char_x < (display_zoom ? 16 : 8); char_x++)
					display_write_pixel((char_line >> (7 - (char_x % 8))) & 1);
			}
		}
	}
}
