#pragma once
#include <stddef.h>
#include <stdint.h>

/**
 * @file i2c.h
 * @brief I2C1 (PB8 SCL, PB9 SDA, 400 kHz) for the AS5600: blocking reads at boot,
 *        non-blocking reads from the tick, and bus recovery from the superloop.
 *
 * Non-blocking reads use HAL's interrupt mode (I2C1 EV/ER at NVIC priority 1), not DMA.
 * In this HAL version HAL_I2C_Mem_Read_DMA polls the whole address phase before handing
 * the 2 data bytes to DMA, with timeouts on HAL_GetTick, which is frozen inside the tick.
 *
 * Who may call what:
 *   - i2c_init, i2c_read, i2c_write: main, before the tick starts reading.
 *   - i2c_read_start, i2c_read_poll: the tick ISR only.
 *   - i2c_recover: the superloop only, every pass.
 * Addresses are 7-bit (0x36 for the AS5600); the driver shifts them for HAL.
 */

/** Live timing of the last completed non-blocking read, in CPU cycles (84 per us). */
typedef struct {
    uint32_t xfer_cycles; // start to completion, wall time
    uint32_t isr_cycles;  // CPU time spent inside the I2C interrupts for that read
} i2c_timing_t;

/** @brief Clocks, pins (AF4 open drain), 400 kHz, I2C interrupts at priority 1. @return 0 or -EIO. */
int i2c_init(void);

/**
 * @brief Blocking register read, bounded by I2C_TIMEOUT_US (SysTick based, so main only).
 * @return 0, -ETIMEDOUT, -EBUSY, -EINVAL, or -EIO (e.g. no ACK).
 */
int i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);

/** @brief Blocking register write, same rules and returns as i2c_read. */
int i2c_write(uint8_t addr, uint8_t reg, const uint8_t *buf, size_t n);

/**
 * @brief Start a register read and return immediately. Tick only.
 *
 * buf must stay valid until the read completes (make it static).
 * @return 0 started, -EBUSY a read is still running, -EIO the bus needs recovery
 *         (or just failed to start, which also flags it for recovery).
 */
int i2c_read_start(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);

/**
 * @brief Check the read started by i2c_read_start. Tick only.
 *
 * On any error the I2C interrupts are switched off at once (an error can leave the event
 * interrupt firing forever) and the bus is flagged for i2c_recover.
 * @return 0 done (buf is valid), -EINPROGRESS still running, -EAGAIN nothing was started,
 *         -EIO error or recovery pending, -ETIMEDOUT running longer than I2C_TIMEOUT_US.
 */
int i2c_read_poll(void);

/**
 * @brief Recover the bus if a read failed. Superloop only; call every pass, it is cheap
 *        when there is nothing to do.
 *
 * Peripheral off, pins as GPIO, up to 9 SCL pulses until a stuck slave releases SDA,
 * START + STOP, then a full re-init (HAL_I2C_Init includes the SWRST). Attempts are at
 * least I2C_RECOVER_MS apart so a dead sensor can't turn the superloop into a recovery loop.
 * @return 0 nothing to do, 1 recovered, -EAGAIN too soon since the last attempt,
 *         -EIO SDA still held low or re-init failed (tried again later).
 */
int i2c_recover(void);

/** @brief Timing of the last completed non-blocking read. */
i2c_timing_t i2c_last_timing(void);
