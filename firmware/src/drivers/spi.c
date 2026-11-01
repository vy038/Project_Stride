#include "drivers/spi.h"
#include <errno.h>

int spi_init(void) {
    return -ENOSYS; // TODO
}

int spi_xfer(const uint8_t *tx, uint8_t *rx, size_t n) {
    (void)tx;
    (void)rx;
    (void)n;
    return -ENOSYS; // TODO
}
