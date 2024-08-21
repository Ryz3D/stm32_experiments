#include "cia.h"

void pin_mode(uint8_t ddr, uint8_t offset)
{
	/*
	 gpio_config_t io_conf_in, io_conf_out;

	 io_conf_in.mode = GPIO_MODE_INPUT;
	 io_conf_in.pull_up_en = GPIO_PULLUP_ENABLE;
	 io_conf_in.pull_down_en = GPIO_PULLDOWN_DISABLE;
	 io_conf_in.intr_type = GPIO_INTR_DISABLE;
	 io_conf_in.pin_bit_mask = 0;

	 io_conf_out.mode = GPIO_MODE_OUTPUT;
	 io_conf_out.pull_up_en = GPIO_PULLUP_DISABLE;
	 io_conf_out.pull_down_en = GPIO_PULLDOWN_DISABLE;
	 io_conf_out.intr_type = GPIO_INTR_DISABLE;
	 io_conf_out.pin_bit_mask = 0;

	 for (uint8_t i = 0; i < 8; i++)
	 {
	 if ((ddr >> i) & 1)
	 io_conf_out.pin_bit_mask |= 1ULL << (offset + i);
	 else
	 io_conf_in.pin_bit_mask |= 1ULL << (offset + i);
	 }

	 if (io_conf_in.pin_bit_mask)
	 ESP_ERROR_CHECK(gpio_config(&io_conf_in));
	 if (io_conf_out.pin_bit_mask)
	 ESP_ERROR_CHECK(gpio_config(&io_conf_out));
	 */
}

uint8_t port_a = 0;

int8_t cia_read(uint8_t addr_off)
{
	if (addr_off == 0x00)
		return port_a;
	else if (addr_off == 0x01)
	{
		uint8_t port_b = 0;

		/*
		 port_b |= gpio_get_level(1) << 3;
		 port_b |= gpio_get_level(2) << 6;
		 port_b |= gpio_get_level(3) << 5;
		 port_b |= gpio_get_level(4) << 4;
		 port_b |= gpio_get_level(5) << 7;
		 port_b |= gpio_get_level(6) << 2;
		 port_b |= gpio_get_level(7) << 1;
		 port_b |= gpio_get_level(8) << 0;
		 */

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

		/*
		 gpio_set_level(9, (port_a >> 0) & 1);
		 gpio_set_level(10, (port_a >> 6) & 1);
		 gpio_set_level(11, (port_a >> 5) & 1);
		 gpio_set_level(12, (port_a >> 4) & 1);
		 gpio_set_level(13, (port_a >> 3) & 1);
		 gpio_set_level(14, (port_a >> 2) & 1);
		 gpio_set_level(15, (port_a >> 1) & 1);
		 gpio_set_level(16, (port_a >> 7) & 1);
		 */
	}
	else if (addr_off == 0x02)
		pin_mode(d, 9);
	else if (addr_off == 0x03)
		pin_mode(d, 1);
}
