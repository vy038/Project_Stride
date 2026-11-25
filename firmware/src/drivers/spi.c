/**
 * @file spi.c
 * @brief SPI2 master, register level. See spi.h.
 */

#include "drivers/spi.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <errno.h>

_Static_assert(TIM_CLK_HZ / 2 / 16 == SPI_HZ, "SPI_HZ must be APB1 (42 MHz) / 16, the BR setting below");

#define BYTE_TIMEOUT_CYC (TIM_CLK_HZ / 10000) // 100 us; a byte takes 3 us

static void pin_af5(GPIO_TypeDef *port, uint32_t pin) {
    GPIO_InitTypeDef g = {.Pin = 1u << pin, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL,
                          .Speed = GPIO_SPEED_FREQ_MEDIUM, .Alternate = GPIO_AF5_SPI2};
    HAL_GPIO_Init(port, &g);
}

int spi_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    spi_cs(false); // high before it becomes an output, so the MCP2515 never sees a stray select
    GPIO_InitTypeDef cs = {.Pin = 1u << CAN_CS_PIN, .Mode = GPIO_MODE_OUTPUT_PP, .Speed = GPIO_SPEED_FREQ_MEDIUM};
    HAL_GPIO_Init(CAN_CS_PORT, &cs);
    pin_af5(CAN_SCK_PORT, CAN_SCK_PIN);
    pin_af5(CAN_MISO_PORT, CAN_MISO_PIN);
    pin_af5(CAN_MOSI_PORT, CAN_MOSI_PIN);

    // master, /16 (BR = 011), mode 0, 8-bit, MSB first. SSM+SSI: NSS handled in software and held
    // "high" internally, otherwise a floating NSS pin drops the peripheral out of master mode
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI2->CR1 |= SPI_CR1_SPE;
    return 0;
}

void spi_cs(bool select) { CAN_CS_PORT->BSRR = select ? 1u << (CAN_CS_PIN + 16) : 1u << CAN_CS_PIN; }

static bool wait_flag(uint32_t flag) {
    uint32_t t0 = DWT->CYCCNT;
    while (!(SPI2->SR & flag))
        if (DWT->CYCCNT - t0 > BYTE_TIMEOUT_CYC) return false;
    return true;
}

int spi_xfer(const uint8_t *tx, uint8_t *rx, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (!wait_flag(SPI_SR_TXE)) return -ETIMEDOUT;
        SPI2->DR = tx ? tx[i] : 0;
        // RXNE means this byte's 8 clocks are done, so CS can go high right after the last one
        if (!wait_flag(SPI_SR_RXNE)) return -ETIMEDOUT;
        uint8_t b = (uint8_t)SPI2->DR;
        if (rx) rx[i] = b;
    }
    return 0;
}
