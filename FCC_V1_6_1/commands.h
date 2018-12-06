#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

#include "atmel_start_pins.h"
#include "subbus.h"

#define CMD_BASE_ADDR 0x1A
#define CMD_HIGH_ADDR 0x1A

extern subbus_driver_t sb_cmd;

#endif
