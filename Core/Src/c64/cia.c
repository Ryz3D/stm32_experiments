#include "cia.h"

uint8_t port_a = 0;

uint8_t gpio_read(uint8_t pin)
{
}

void gpio_write(uint8_t pin, uint8_t state)
{
}

int8_t cia_read(uint8_t addr_off)
{
	if (addr_off == 0x00)
		return port_a;
	else if (addr_off == 0x01)
	{
		uint8_t port_b = 0;

		port_b |= gpio_read(0) << 3;
		port_b |= gpio_read(1) << 6;
		port_b |= gpio_read(2) << 5;
		port_b |= gpio_read(3) << 4;
		port_b |= gpio_read(4) << 7;
		port_b |= gpio_read(5) << 2;
		port_b |= gpio_read(6) << 1;
		port_b |= gpio_read(7) << 0;

		return port_b;
	}
	else if (addr_off == 0x02)
		return 0xff;
	else if (addr_off == 0x03)
		return 0x00;
	return 0;
}

void cia_write(uint8_t addr_off, int8_t d)
{
	if (addr_off == 0x00)
	{
		port_a = d;

		gpio_write(8, (port_a >> 0) & 1);
		gpio_write(9, (port_a >> 6) & 1);
		gpio_write(10, (port_a >> 5) & 1);
		gpio_write(11, (port_a >> 4) & 1);
		gpio_write(12, (port_a >> 3) & 1);
		gpio_write(13, (port_a >> 2) & 1);
		gpio_write(14, (port_a >> 1) & 1);
		gpio_write(15, (port_a >> 7) & 1);
	}
	else if (addr_off == 0x02)
		; // pin_mode(d, 9);
	else if (addr_off == 0x03)
		; // pin_mode(d, 1);
}
