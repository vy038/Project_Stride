#include "drivers/i2c.h"
#include <errno.h>

int i2c_init(void) {
    return -ENOSYS; // TODO
}

int i2c_recover(void) {
    return -ENOSYS; // TODO: SWRST, 9 SCL clocks until SDA releases, STOP, re-init
}

int i2c_read_start(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
    (void)addr;
    (void)reg;
    (void)buf;
    (void)n;
    // TODO: DMA register read, 400 kHz. I2C EV/ER and DMA at NVIC priority 1, above PendSV
    return -ENOSYS;
}

int i2c_read_poll(void) {
    // TODO: on error disable the I2C IRQs and leave i2c_recover to the superloop (bring-up IRQ storm)
    return -ENOSYS;
}

int i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
    (void)addr;
    (void)reg;
    (void)buf;
    (void)n;
    return -ENOSYS; // TODO
}

int i2c_write(uint8_t addr, uint8_t reg, const uint8_t *buf, size_t n) {
    (void)addr;
    (void)reg;
    (void)buf;
    (void)n;
    return -ENOSYS; // TODO
}
