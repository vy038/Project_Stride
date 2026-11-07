#pragma once
#include <stdbool.h>
#include <stdint.h>

/**
 * @file wdg.h
 * @brief Independent watchdog (IWDG) and the HardFault last-resort handler.
 *
 * The IWDG is a free-running down-counter clocked by the chip's internal LSI
 * oscillator, independent of the main clock tree. If it reaches zero the chip
 * resets. Once started it cannot be stopped, only reset.
 *
 * Rule: wdg_kick() is called from main's superloop ONLY, never from the tick
 * ISR or PendSV. A kick must mean "the lowest-priority code is still running",
 * which is the first thing to die when CAN, I2C recovery or a bad loop hangs.
 * If an ISR kicked, interrupts alone would keep a dead superloop looking alive.
 *
 * Source of the numbers (RM0368 IWDG chapter, STM32F401xE datasheet):
 *   - LSI is nominally 32 kHz, but the datasheet spread is about 17 to 47 kHz.
 *   - Prescaler /32 gives a count rate of LSI/32, nominally 1 kHz, so the reload
 *     value in counts equals the timeout in milliseconds (at nominal LSI).
 *   - Reload is 12 bits, so the maximum is 4095.
 *   - Real timeout = reload * 32 / f_LSI. A SLOW LSI gives a LONG timeout:
 *     17 kHz -> 1.88 T, 47 kHz -> 0.68 T. For T = 150 that is about 102 to 282 ms.
 *     Pick T so that BOTH ends are acceptable (see config.h WDG_TIMEOUT_MS).
 */

/**
 * @brief Start the watchdog. Cannot be undone short of a reset.
 *
 * Also freezes the counter while the core is halted by a debugger, otherwise
 * every breakpoint would reset the chip.
 *
 * @param timeout_ms  Nominal timeout, 1 to 4095. The real value varies with the
 *                    LSI spread, see the file comment.
 * @return 0 on success, -EINVAL if timeout_ms is out of range, -EIO if the
 *         peripheral did not accept the configuration in time.
 * @note   Call once, from main, after all slow initialisation is finished:
 *         the counter is already running when this returns.
 */
int wdg_init(uint32_t timeout_ms);

/** @brief Restart the countdown. Superloop only, see the file comment. */
void wdg_kick(void);

/**
 * @brief Report whether the previous reset was caused by the watchdog.
 *
 * Reads RCC->CSR IWDGRSTF on the first call, then clears the hardware reset
 * flags (RMVF) so a later normal reset is not misreported. The answer is
 * latched, so it is safe to call more than once per boot.
 *
 * @return true if the last reset was an IWDG reset.
 */
bool wdg_caused_reset(void);

/**
 * @brief The last line of defence for watchdog. Cut the motor driver first, then wait for the
 * watchdog to reset the chip.
 *
 * Deliberately no HAL and no function calls: the HAL may be what crashed. The
 * upper half of BSRR clears a pin in a single atomic store, and EN's clock and
 * output mode were set up at the very start of main, so this works from any
 * point after boot. If the watchdog has not been started yet, the chip stays
 * here with EN low, which is the safe state.
 */
void HardFault_Handler(void);
