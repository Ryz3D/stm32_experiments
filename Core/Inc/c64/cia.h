#pragma once

#include <stdint.h>
#include <stdbool.h>

extern uint8_t port_a;

__attribute__((weak))   uint8_t gpio_read_row(uint8_t row);
__attribute__((weak)) void gpio_write_col(uint8_t col, uint8_t state);
int8_t cia_read(uint8_t addr_off);
void cia_write(uint8_t addr_off, int8_t d);
