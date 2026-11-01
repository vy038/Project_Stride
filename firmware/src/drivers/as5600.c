#include "drivers/as5600.h"
#include <errno.h>

int as5600_check(uint8_t *agc) {
    (void)agc;
    // TODO: STATUS MD set, ML and MH clear, AGC not pinned at 0 or 128. A dead AS5600 once shorted 3V3
    return -ENOSYS;
}

int as5600_start(void) {
    return -ENOSYS; // TODO: i2c_read_start into a static 2-byte buffer
}

int as5600_get(uint16_t *raw) {
    (void)raw;
    return -ENOSYS; // TODO: i2c_read_poll, then ((b[0] & 0x0F) << 8) | b[1]
}
