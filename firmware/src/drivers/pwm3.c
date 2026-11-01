#include "drivers/pwm3.h"
#include <errno.h>

int pwm3_init(void) {
    return -ENOSYS; // TODO
}

int pwm3_set(const float duty[3]) {
    (void)duty;
    return -ENOSYS; // TODO
}

int pwm3_enable(bool on) {
    (void)on;
    return -ENOSYS; // TODO
}
