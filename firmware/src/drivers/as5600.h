#pragma once
#include <stdint.h>

/**
 * @file as5600.h
 * @brief AS5600 12-bit magnetic angle sensor on I2C1 (see i2c.h for who may call what).
 *
 * RAW ANGLE is 0..4095 per turn, 1.534 mrad per count. Register map from the AS5600 datasheet.
 */

#define AS5600_ADDR          0x36 // 7-bit
#define AS5600_REG_STATUS    0x0B
#define AS5600_REG_RAW_ANGLE 0x0C // 0x0C high nibble, 0x0D low byte; a 2-byte read gets both
#define AS5600_REG_AGC       0x1A
#define AS5600_REG_MAGNITUDE 0x1B

#define AS5600_STATUS_MD 0x20 // magnet detected
#define AS5600_STATUS_ML 0x10 // magnet too weak (AGC at maximum gain)
#define AS5600_STATUS_MH 0x08 // magnet too strong (AGC at minimum gain)
#define AS5600_AGC_MAX   128  // AGC range at 3.3 V supply (0..255 at 5 V)
#define AS5600_POWERUP_MS 10  // datasheet power-up time before the first read

/**
 * @brief Boot check, blocking, main only. Waits out the power-up time first.
 *
 * Never enable the joint if this fails: a dead AS5600 once shorted 3V3.
 * @param agc  out: the AGC value, may be NULL. Mid-range means a good magnet gap.
 * @return 0 ok, -ENODEV no magnet, -ERANGE magnet too weak/strong or AGC pinned,
 *         or an i2c_read error.
 */
int as5600_check(uint8_t *agc);

/** @brief Start reading RAW ANGLE in the background. Tick only. @return i2c_read_start's result. */
int as5600_start(void);

/**
 * @brief Collect the read started by as5600_start. Tick only.
 * @param raw out: 0..4095, only written on success.
 * @return i2c_read_poll's result (0 means raw is valid).
 */
int as5600_get(uint16_t *raw);
