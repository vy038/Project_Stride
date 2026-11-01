#pragma once
#include <stddef.h>
#include <stdint.h>

int spi_init(void);

int spi_xfer(const uint8_t *tx, uint8_t *rx, size_t n);
