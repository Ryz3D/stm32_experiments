#pragma once

#include <stdint.h>
#include <stdbool.h>

extern uint8_t port_a;

__attribute__((weak))  uint8_t gpio_read(uint8_t pin);
__attribute__((weak)) void gpio_write(uint8_t pin, uint8_t state);
int8_t cia_read(uint8_t addr_off);
void cia_write(uint8_t addr_off, int8_t d);
