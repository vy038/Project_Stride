#pragma once
#include <stddef.h>
#include <stdint.h>

int i2c_init(void);

int i2c_recover(void);

int i2c_read_start(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);

int i2c_read_poll(void);

int i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);
int i2c_write(uint8_t addr, uint8_t reg, const uint8_t *buf, size_t n);
