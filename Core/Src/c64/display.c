#include "display.h"

uint16_t color_bright = (0b01110 << 11) | (0b011011 << 5) | 0b11101;
uint16_t color_dark = (0b00101 << 11) | (0b001000 << 5) | 0b11010;
uint8_t display_zoom = 0;

void display_init()
{
	/*
	 gpio_config_t io_conf;
	 io_conf.mode = GPIO_MODE_OUTPUT;
	 io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	 io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	 io_conf.intr_type = GPIO_INTR_DISABLE;
	 io_conf.pin_bit_mask = 0;
	 io_conf.pin_bit_mask |= (1ULL << PIN_LCD_RD) | (1ULL << PIN_LCD_WR) | (1ULL << PIN_LCD_RS) | (1ULL << PIN_LCD_CS) | (1ULL << PIN_LCD_RST) | (1ULL << PIN_LCD_D0) | (1ULL << PIN_LCD_D1)
	 | (1ULL << PIN_LCD_D2) | (1ULL << PIN_LCD_D3) | (1ULL << PIN_LCD_D4) | (1ULL << PIN_LCD_D5) | (1ULL << PIN_LCD_D6) | (1ULL << PIN_LCD_D7);
	 ESP_ERROR_CHECK(gpio_config(&io_conf));

	 gpio_set_level(PIN_LCD_CS, 1);
	 gpio_set_level(PIN_LCD_WR, 1);
	 gpio_set_level(PIN_LCD_RD, 1);
	 gpio_set_level(PIN_LCD_RS, 1);
	 gpio_set_level(PIN_LCD_RST, 0);
	 vTaskDelay(10 / portTICK_PERIOD_MS);
	 gpio_set_level(PIN_LCD_RST, 1);
	 gpio_set_level(PIN_LCD_CS, 0);

	 display_write_c(0x00);
	 display_write_c(0x00);
	 display_write_c(0x00);

	 vTaskDelay(200 / portTICK_PERIOD_MS);

	 display_write_c(0xF1);
	 display_write(0x36);
	 display_write(0x04);
	 display_write(0x00);
	 display_write(0x3C);
	 display_write(0x0F);
	 display_write(0x8F);
	 display_write_c(0xF2);
	 display_write(0x18);
	 display_write(0xA3);
	 display_write(0x12);
	 display_write(0x02);
	 display_write(0xB2);
	 display_write(0x12);
	 display_write(0xFF);
	 display_write(0x10);
	 display_write(0x00);
	 display_write_cd16(0xF8, 0x2104);
	 display_write_cd16(0xF9, 0x0008);
	 display_write_cd(0xB4, 0x00);
	 display_write_cd(0xC1, 0x41);
	 display_write_c(0xC5);
	 display_write(0x41);
	 display_write(0x41);
	 display_write(0x41);
	 display_write(0x41);
	 display_write_c(0xE0);
	 display_write(0x0F);
	 display_write(0x1F);
	 display_write(0x1C);
	 display_write(0x0C);
	 display_write(0x0F);
	 display_write(0x08);
	 display_write(0x48);
	 display_write(0x98);
	 display_write(0x37);
	 display_write(0x0A);
	 display_write(0x13);
	 display_write(0x04);
	 display_write(0x11);
	 display_write(0x0D);
	 display_write(0x00);
	 display_write_c(0xE1);
	 display_write(0x0F);
	 display_write(0x32);
	 display_write(0x2E);
	 display_write(0x0B);
	 display_write(0x0D);
	 display_write(0x05);
	 display_write(0x47);
	 display_write(0x75);
	 display_write(0x37);
	 display_write(0x06);
	 display_write(0x10);
	 display_write(0x03);
	 display_write(0x24);
	 display_write(0x20);
	 display_write(0x00);
	 display_write_cd(0x3A, 0x55);
	 display_write_c(0x11);
	 vTaskDelay(120 / portTICK_PERIOD_MS);
	 display_write_c(0x29);
	 */

	// rotation 0: 0x48, 1: 0x38, 2: 0x88, 3: 0xe8
	display_write_cd(0x36, 0xe8);

	display_clear();
}

void display_write(uint8_t data)
{
	/*
	 gpio_set_level(PIN_LCD_D0, (data >> 0) & 1);
	 gpio_set_level(PIN_LCD_D1, (data >> 1) & 1);
	 gpio_set_level(PIN_LCD_D2, (data >> 2) & 1);
	 gpio_set_level(PIN_LCD_D3, (data >> 3) & 1);
	 gpio_set_level(PIN_LCD_D4, (data >> 4) & 1);
	 gpio_set_level(PIN_LCD_D5, (data >> 5) & 1);
	 gpio_set_level(PIN_LCD_D6, (data >> 6) & 1);
	 gpio_set_level(PIN_LCD_D7, (data >> 7) & 1);
	 gpio_set_level(PIN_LCD_WR, 0);
	 gpio_set_level(PIN_LCD_WR, 1);
	 */
}

void display_write16(uint16_t data)
{
	display_write(data >> 8);
	display_write(data & 0xff);
}

void display_write_c(uint8_t command)
{
	// gpio_set_level(PIN_LCD_RS, 0);
	display_write(command);
	// gpio_set_level(PIN_LCD_RS, 1);
}

void display_write_c16(uint16_t command)
{
	// gpio_set_level(PIN_LCD_RS, 0);
	display_write16(command);
	// gpio_set_level(PIN_LCD_RS, 1);
}

void display_write_cd(uint8_t command, uint8_t data)
{
	display_write_c(command);
	display_write(data);
}

void display_write_cd16(uint16_t command, uint16_t data)
{
	display_write_c16(command);
	display_write16(data);
}

void display_set_space(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	display_write_c(0x2A);
	display_write16(x1);
	display_write16(x2);
	display_write_c(0x2B);
	display_write16(y1);
	display_write16(y2);
}

void display_write_pixel(bool state)
{
	display_write16(state ? color_bright : color_dark);
	if (display_zoom)
		display_write16(state ? color_bright : color_dark);
}

bool display_init_write(uint16_t x, uint16_t y)
{
	uint16_t x_off = display_zoom ? 0 : 80;
	uint16_t y_off = display_zoom ? 0 : 60;
	uint16_t factor = display_zoom ? 2 : 1;
	uint16_t char_size = display_zoom ? 15 : 7;
	if (x * factor + x_off + char_size >= 480 || y * factor + y_off + char_size >= 320)
		return false;
	display_set_space(x * factor + x_off, y * factor + y_off, x * factor + x_off + char_size, y * factor + y_off + char_size);
	display_write_c(0x2C);
	return true;
}

void display_set_pixel(uint16_t x, uint16_t y, bool state)
{
	display_set_space(x, y, x, y);
	display_write_c(0x2C);
	display_write_pixel(state);
}

void display_clear()
{
	display_set_space(0, 0, 480, 320);
	display_write_c(0x2C);
	for (uint32_t i = 0; i < 480 * 320; i++)
		display_write_pixel(0);
}
