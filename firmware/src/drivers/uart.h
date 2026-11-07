#pragma once

/**
 * @file uart.h
 * @brief Debug console on USART2 (PA2 TX / PA3 RX), the ST-LINK virtual COM port.
 *
 * 115200 baud, 8N1, blocking transmit, no receive path in use yet. newlib's
 * printf is retargeted to this port via _write(), so printf output appears on
 * the host's /dev/serial/by-id/... device with no extra wiring.
 *
 * Context rules:
 *   - Call printf ONLY from main's loop or app level. Never from the tick ISR or PendSV:
 *     at 115200 baud one byte takes ~87 us, so a 40-character line blocks for
 *     ~3.5 ms, which would stall the 200 us control tick.
 *   - Output is blocking but bounded (see UART_TX_BASE_TIMEOUT_MS in uart.c), so
 *     a stuck peripheral cannot starve the watchdog forever.
 *
 * This is intended for bring-up and debug, not for production.
 */

/**
 * @brief Initialise USART2 and route stdout to it.
 *
 * Enables the GPIOA and USART2 clocks, configures PA2/PA3 as alternate
 * function 7, sets 115200 8N1, and disables stdio buffering so every printf
 * reaches the wire immediately.
 *
 * Requires the system clock to be configured first: the baud rate divisor is
 * computed from the APB1 clock (42 MHz after SystemClock_Config).
 *
 * @note On a HAL init failure this halts in an infinite loop. A debug console
 *       that silently does not work is worse than one that visibly hangs
 *       during bring-up, and the motor driver's EN is already low by then.
 */
void uart_init(void);
