#pragma once

#include <stdint.h>
#include <stdbool.h>

extern uint8_t port_a;

int8_t cia_read(uint8_t addr_off);
void cia_write(uint8_t addr_off, int8_t d);
