#pragma once
#include <stdint.h>

#define AS5600_ADDR          0x36
#define AS5600_REG_STATUS    0x0B
#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_REG_AGC       0x1A
#define AS5600_REG_MAGNITUDE 0x1B

int as5600_check(uint8_t *agc);

int as5600_start(void);

int as5600_get(uint16_t *raw);
