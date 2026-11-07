/**
 * @file uart.c
 * @brief USART2 debug console and newlib printf retarget. See uart.h.
 */

#include "drivers/uart.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

/**
 * Transmit timeout is a fixed base plus a per-byte allowance.
 * At 115200 baud a byte takes ~0.087 ms, so 1 ms per 8 bytes is about 1.4x the
 * real time: generous enough not to false-trigger, tight enough to bound a hang.
 * HAL timeouts count SysTick ms, so this only works from main context where
 * SysTick runs, which matches the "printf from main only" rule.
 */
#define UART_TX_BASE_TIMEOUT_MS 2u
#define UART_TX_BYTES_PER_MS    8u

static UART_HandleTypeDef huart2;

void uart_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    // PA2 = USART2_TX, PA3 = USART2_RX, both alternate function 7 on the F401.
    // Pull-up on RX keeps the idle line high if the ST-LINK is unplugged.
    GPIO_InitTypeDef pins = {
        .Pin = GPIO_PIN_2 | GPIO_PIN_3,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF7_USART2,
    };
    HAL_GPIO_Init(GPIOA, &pins);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        for (;;) {
        }
    }

    // Unbuffered: each printf fragment goes straight to _write. Costs a few more
    // small transmits, but avoids lost output if we reset or fault before a flush.
    setvbuf(stdout, NULL, _IONBF, 0);
}

/**
 * newlib's printf/puts/putchar all end in _write(). Providing it sends stdout
 * (and stderr) out USART2.
 *
 * @param fd   file descriptor; ignored, stdout and stderr both go to the UART.
 * @param buf  bytes to send.
 * @param len  number of bytes.
 * @return     bytes written, or -1 if the transmit timed out or failed. newlib
 *             treats -1 as an I/O error rather than retrying forever.
 */
int _write(int fd, char *buf, int len) {
    (void)fd;
    if (len <= 0) {
        return 0;
    }

    // HAL_UART_Transmit takes a 16-bit size, so send in chunks rather than
    // silently truncating a long write.
    int sent = 0;
    while (sent < len) {
        int remaining = len - sent;
        uint16_t chunk = (uint16_t)(remaining > UINT16_MAX ? UINT16_MAX : remaining);
        uint32_t timeout_ms = UART_TX_BASE_TIMEOUT_MS + chunk / UART_TX_BYTES_PER_MS;

        if (HAL_UART_Transmit(&huart2, (uint8_t *)buf + sent, chunk, timeout_ms) != HAL_OK) {
            return -1;
        }
        sent += chunk;
    }
    return len;
}
