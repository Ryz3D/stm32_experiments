#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define PIN_LCD_RD 18
#define PIN_LCD_WR 19
#define PIN_LCD_RS 20
#define PIN_LCD_CS 21
#define PIN_LCD_RST 33
#define PIN_LCD_D0 34
#define PIN_LCD_D1 35
#define PIN_LCD_D2 36
#define PIN_LCD_D3 37
#define PIN_LCD_D4 38
#define PIN_LCD_D5 39
#define PIN_LCD_D6 40
#define PIN_LCD_D7 41

extern uint16_t color_bright;
extern uint16_t color_dark;
extern uint8_t display_zoom;

void display_init();
void display_write(uint8_t data);
void display_write16(uint16_t data);
void display_write_c(uint8_t command);
void display_write_c16(uint16_t command);
void display_write_cd(uint8_t command, uint8_t data);
void display_write_cd16(uint16_t command, uint16_t data);
void display_set_space(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void display_write_pixel(bool state);
bool display_init_write(uint16_t x, uint16_t y);
void display_set_pixel(uint16_t x, uint16_t y, bool state);
void display_clear();
