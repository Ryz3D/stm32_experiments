#include "memory.h"

int8_t zeropage[256], stack[256], sysvar[512], screen[1024], cia2_reg[16];
int8_t basicram[38912];
uint8_t ram[4096] = {
	120, 162, 0, 160, 0, 200, 208, 253, 232, 208, 250, 108, 252, 255 };

int8_t mem_read(uint16_t addr)
{
	uint16_t addr_off = addr;
	if (addr < 0x0100)
	{
		// zeropage
		return zeropage[addr];
	}
	else if (addr < 0x0200)
	{
		addr_off -= 0x0100;
		// stack
		return stack[addr_off];
	}
	else if (addr < 0x0400)
	{
		addr_off -= 0x0200;
		// sysvar
		return sysvar[addr_off];
	}
	else if (addr < 0x0800)
	{
		addr_off -= 0x0400;
		// screen
		return screen[addr_off];
	}
	else if (addr < 0xa000)
	{
		addr_off -= 0x0800;
		// basic ram
		return basicram[addr_off];
	}
	else if (addr < 0xc000)
	{
		addr_off -= 0xa000;
		// basic rom
		return basic[addr_off];
	}
	else if (addr < 0xd000)
	{
		addr_off -= 0xc000;
		// free ram
		return ram[addr_off];
	}
	else if (addr < 0xe000)
	{
		if (addr < 0xd400)
		{
			addr_off -= 0xd000;
			// vic registers
			return vic_read(addr_off);
		}
		else if (addr < 0xd800)
		{
			addr_off -= 0xd400;
			// sid registers
		}
		else if (addr < 0xdc00)
		{
			addr_off -= 0xd800;
			// color ram
		}
		else if (addr < 0xdd00)
		{
			addr_off -= 0xdc00;
			return cia_read(addr_off % 16);
		}
		else if (addr < 0xde00)
		{
			addr_off -= 0xdd00;
			// cia 2
			return cia2_reg[addr_off % 16];
		}
		else
		{
			addr_off -= 0xde00;
			// interface expansions
		}
	}
	else
	{
		addr_off -= 0xe000;
		// kernal rom
		return kernal[addr_off];
	}
	return 0;
}

uint16_t mem_read16(uint16_t addr0)
{
	return ((uint16_t)(uint8_t)mem_read(addr0 + 1) << 8) | (uint16_t)(uint8_t)mem_read(addr0);
}

void mem_write(uint16_t addr, int8_t d)
{
	uint16_t addr_off = addr;
	if (addr < 0x0100)
	{
		// zeropage
		zeropage[addr] = d;
	}
	else if (addr < 0x0200)
	{
		addr_off -= 0x0100;
		// stack
		stack[addr_off] = d;
	}
	else if (addr < 0x0400)
	{
		addr_off -= 0x0200;
		// sysvar
		sysvar[addr_off] = d;
	}
	else if (addr < 0x0800)
	{
		addr_off -= 0x0400;
		// screen
		screen[addr_off] = d;
		vic_write_screen(addr_off, d);
	}
	else if (addr < 0xa000)
	{
		addr_off -= 0x0800;
		// basic ram
		basicram[addr_off] = d;
	}
	else if (addr < 0xc000)
	{
		addr_off -= 0xa000;
		// basic rom
	}
	else if (addr < 0xd000)
	{
		addr_off -= 0xc000;
		// free ram
		ram[addr_off] = d;

		if (addr == 0xcff1)
			display_zoom = d;
		if (addr == 0xcff2)
		{
			if (d == 0)
			{
				// Blau
				color_bright = (0b11001 << 11) | (0b111000 << 5) | 0b11111;
				color_dark = (0b00101 << 11) | (0b001000 << 5) | 0b11010;
			}
			else if (d == 1)
			{
				// Dunkelgruen
				color_bright = (0b01011 << 11) | (0b100011 << 5) | 0b01000;
				color_dark = (0b00001 << 11) | (0b001010 << 5) | 0b00100;
			}
			else if (d == 2)
			{
				// Hellgruen
				color_bright = (0b00001 << 11) | (0b001010 << 5) | 0b00100;
				color_dark = (0b01011 << 11) | (0b100011 << 5) | 0b01000;
			}
			else if (d == 3)
			{
				// Schwarz/Weiss
				color_bright = (0b11111 << 11) | (0b111111 << 5) | 0b11111;
				color_dark = (0b00000 << 11) | (0b000000 << 5) | 0b00000;
			}
			else if (d == 4)
			{
				// Weiss/Schwarz
				color_bright = (0b00000 << 11) | (0b000000 << 5) | 0b00000;
				color_dark = (0b11111 << 11) | (0b111111 << 5) | 0b11111;
			}
			else if (d == 5)
			{
				// Grau/Blau
				color_bright = (0b01010 << 11) | (0b110000 << 5) | 0b11111;
				color_dark = (0b00100 << 11) | (0b001000 << 5) | 0b00100;
			}
		}
	}
	else if (addr < 0xd400)
	{
		addr_off -= 0xd000;
		// vic registers
		vic_write(addr_off, d);
	}
	else if (addr < 0xd800)
	{
		addr_off -= 0xd400;
		// sid registers
	}
	else if (addr < 0xdc00)
	{
		addr_off -= 0xd800;
		// color ram
	}
	else if (addr < 0xdd00)
	{
		addr_off -= 0xdc00;
		// cia 1
		cia_write(addr_off % 16, d);
	}
	else if (addr < 0xde00)
	{
		addr_off -= 0xdd00;
		// cia 2
		cia2_reg[addr_off % 16] = d;
	}
	else if (addr < 0xe000)
	{
		addr_off -= 0xde00;
		// interface expansions
	}
	else
	{
		addr_off -= 0xe000;
		// kernal rom
	}
}
