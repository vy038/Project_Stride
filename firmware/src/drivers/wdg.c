#include "drivers/wdg.h"
#include "config.h"
#include "stm32f4xx.h"
#include <errno.h>

int wdg_init(uint32_t timeout_ms) {
    (void)timeout_ms;
    return -ENOSYS; // TODO
}

void wdg_kick(void) {
    // TODO
}

bool wdg_caused_reset(void) {
    return false; // TODO: RCC->CSR IWDGRSTF, then clear with RMVF
}

void HardFault_Handler(void) {
    EN_PORT->BSRR = 1u << (EN_PIN + 16);
    for (;;) {
    }
}
