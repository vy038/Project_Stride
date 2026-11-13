#pragma once
#include <stdbool.h>

/**
 * @file pwm3.h
 * @brief Three-phase PWM on TIM3 (PA6, PA7, PB0) and the driver's EN pin.
 *
 * Center-aligned at PWM_HZ (25 kHz): the counter runs 0 -> ARR -> 0, so one period is
 * 2 x ARR counts and ARR = 84 MHz / (2 x 25 kHz) = 1680. Duty d maps to CCR = d x ARR.
 * New duties are preloaded and take effect at the next period, so a pulse is never cut.
 *
 * Safe state: all duties 0.5 (no voltage across the motor) and EN low (driver outputs off).
 * EN is the main breaker: on last, off first.
 */

/**
 * @brief Configure the pins and TIM3, start the counter at 50% duty on all three phases.
 *
 * Leaves EN low. Call once from main, after the clock is configured.
 * @return 0.
 */
int pwm3_init(void);

/**
 * @brief Set the three phase duties, 0..1. Values are clamped to that range.
 *
 * Safe to call from the tick ISR: three register writes, no HAL.
 * @return 0, or -EINVAL (nothing written) if any duty is NaN or infinite.
 */
int pwm3_set(const float duty[3]);

/**
 * @brief Switch the driver on or off with its EN pin.
 *
 * Off drives EN low first, then returns all duties to 0.5. On only raises EN, so set the
 * duties you want before calling it. One atomic BSRR store each way, safe from an ISR.
 * @return 0.
 */
int pwm3_enable(bool on);
