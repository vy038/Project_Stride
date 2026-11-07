/**
 * @file wdg.c
 * @brief IWDG driver and HardFault handler. See wdg.h for the design rules.
 */

#include "drivers/wdg.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <errno.h>

#define WDG_RELOAD_MAX 4095u // IWDG_RLR is 12 bits

static IWDG_HandleTypeDef hiwdg;

int wdg_init(uint32_t timeout_ms) {
    // prescaler /32 on a nominal 32 kHz LSI is a 1 kHz count = timeout: reload == ms
    if (timeout_ms == 0u || timeout_ms > WDG_RELOAD_MAX) {
        return -EINVAL;
    }

    // freeze counter while the core is halted at breakpoint
    // MUST be set before debugger halts
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    // watchdog configuration: prescaler, reload, start registers
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = timeout_ms;
    // HAL unlocks the registers, writes prescaler and reload, waits for
    // update to cross into the LSI domain (bounded), then starts counter once init
    return HAL_IWDG_Init(&hiwdg) == HAL_OK ? 0 : -EIO;
    // starting the IWDG also turns the LSI oscillator on in hardware
}

// writes reload key 0xAAAA, counter restarts from reload value
void wdg_kick(void) {
    HAL_IWDG_Refresh(&hiwdg);
}

bool wdg_caused_reset(void) {
    static bool captured;
    static bool was_watchdog;

    // config flags survive reset but cleared by RMVF, so read them once and latch again
    // clearing is needed still, else IWDGRSTF stays set until next power-on reset and every boot after reads wdg reset
    if (!captured) {
        was_watchdog = (RCC->CSR & RCC_CSR_IWDGRSTF) != 0u;
        RCC->CSR |= RCC_CSR_RMVF;
        captured = true;
    }
    return was_watchdog;
}

// minimal implementation, cut motor driver first for safety, then wait for watchdog to reset chip
void HardFault_Handler(void) {
    EN_PORT->BSRR = 1u << (EN_PIN + 16);
    for (;;) {
    }
}
