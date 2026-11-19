/**
 * @file as5600.c
 * @brief AS5600 boot check and background RAW ANGLE reads. See as5600.h.
 */

#include "drivers/as5600.h"
#include "drivers/i2c.h"
#include "stm32f4xx_hal.h"
#include <errno.h>

// the I2C interrupt fills this after as5600_start has returned, so it must outlive the call
static uint8_t raw_buf[2];

int as5600_check(uint8_t *agc) {
    while (HAL_GetTick() < AS5600_POWERUP_MS) {} // SysTick starts at ~power-on

    uint8_t status, gain;
    int r = i2c_read(AS5600_ADDR, AS5600_REG_STATUS, &status, 1);
    if (r != 0) return r;
    r = i2c_read(AS5600_ADDR, AS5600_REG_AGC, &gain, 1);
    if (r != 0) return r;
    if (agc) *agc = gain;

    if (!(status & AS5600_STATUS_MD)) return -ENODEV;
    if (status & (AS5600_STATUS_ML | AS5600_STATUS_MH)) return -ERANGE;
    if (gain == 0 || gain >= AS5600_AGC_MAX) return -ERANGE; // pinned: the gap is wrong
    return 0;
}

int as5600_start(void) { return i2c_read_start(AS5600_ADDR, AS5600_REG_RAW_ANGLE, raw_buf, sizeof raw_buf); }

int as5600_get(uint16_t *raw) {
    int r = i2c_read_poll();
    if (r != 0) return r;
    *raw = (uint16_t)(((raw_buf[0] & 0x0F) << 8) | raw_buf[1]); // top nibble of 0x0C isn't angle
    return 0;
}
