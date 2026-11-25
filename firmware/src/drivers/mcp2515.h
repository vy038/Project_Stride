#pragma once
#include "can/can_proto.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file mcp2515.h
 * @brief MCP2515 CAN controller over SPI2. Superloop and boot only, like spi.h.
 *
 * 8 MHz crystal, 500 kbit/s, standard 11-bit frames. Sends through TXB0 only; receives into
 * RXB0, rolling over into RXB1 when RXB0 is full. Register map and bit timing: MCP2515 datasheet,
 * values cross-checked against the autowp library the bring-up ran on.
 */

#define MCP2515_MODE_NORMAL   0x00
#define MCP2515_MODE_LOOPBACK 0x40 // frames come straight back, nothing goes on the bus, no ACK needed

#define MCP2515_EFLG_RX0OVR 0x40 // a frame arrived with both RX buffers full and was lost
#define MCP2515_EFLG_RX1OVR 0x80
#define MCP2515_EFLG_TXBO   0x20 // bus-off: the chip gave up transmitting

/**
 * @brief Reset, check it answers, set bit timing, receive-any, RX interrupts, then enter mode.
 * @return 0, -ENODEV no reply (power, CS, MISO wiring), -EIO mode change refused, -ETIMEDOUT SPI
 */
int mcp2515_init(uint8_t mode);

/**
 * @brief PC7 as a falling-edge EXTI at priority 3. Call after mcp2515_init.
 * @return 0, or -EIO if INT is already low with nothing pending (stuck line): poll instead
 */
int mcp2515_int_init(void);

/** @brief INT is active low and stays low while any enabled flag is set. */
bool mcp2515_int_low(void);

/**
 * @brief Queue one frame in TXB0 and request sending. Doesn't wait for it to go out.
 * @return 0, -EBUSY TXB0 still sending the last one (nobody ACKing keeps it busy forever),
 *         -EINVAL dlc > 8 or id > 11 bits, -ETIMEDOUT SPI
 */
int mcp2515_send(const can_frame_t *f);

/**
 * @brief Take the oldest received frame. Extended and remote frames are read and dropped.
 * @return 0 frame valid, -EAGAIN nothing waiting, -ETIMEDOUT SPI
 */
int mcp2515_recv(can_frame_t *f);

/**
 * @brief Read the error counters and flags, clear the latched overflow and error-interrupt flags.
 * @param tec, rec out, may be NULL. Error-passive at 128, bus-off at 256 (TEC)
 * @return EFLG before clearing (MCP2515_EFLG_*), or -ETIMEDOUT
 */
int mcp2515_errors(uint8_t *tec, uint8_t *rec);
