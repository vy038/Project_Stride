#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @file spi.h
 * @brief SPI2 master, polled, for the MCP2515. Superloop and boot only, never from an ISR.
 *
 * Mode 0 (clock idles low, sample on the rising edge), 8-bit, MSB first, SPI_HZ (42 MHz / 16).
 * PB13 SCK, PB14 MISO, PB15 MOSI on AF5. CS (PB12) is a plain GPIO so it can stay low across a
 * multi-byte instruction: spi_cs(true), any number of spi_xfer, spi_cs(false).
 */

int spi_init(void);

/** @brief CS low (true) or high (false). */
void spi_cs(bool select);

/**
 * @brief Full duplex: sends n bytes and receives n at the same time.
 * @param tx bytes to send, NULL sends zeros
 * @param rx received bytes, NULL discards them
 * @return 0, or -ETIMEDOUT if a byte didn't complete (the peripheral is broken, not the slave)
 */
int spi_xfer(const uint8_t *tx, uint8_t *rx, size_t n);
