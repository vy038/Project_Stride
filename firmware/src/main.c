#include "bldc/joint.h"
#include "can/can_node.h"
#include "cfc_ctrl.h"
#include "config.h"
#include "drivers/uart.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

static joint_t joint;
static cfc_check_t cfc_check;

void SysTick_Handler(void) { HAL_IncTick(); }

void TIM2_IRQHandler(void) {
    // TODO: clear UIF
    //   as5600_get: raw from the DMA read started last tick (failed -> count toward ENC_LOST_N, hold angle)
    //   as5600_start: next read
    //   multi-turn unwrap -> joint angle -> velocity estimate -> publish via seqlock
    //   theta_e from angle (or joint.theta_e while calibrating) -> foc_svpwm(uq, theta_e, SUPPLY_V) -> pwm3_set
    //   every FOC_HZ / CFC_CTRL_HZ ticks: pend PendSV
}

void EXTI9_5_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(1u << CAN_INT_PIN);
    can_node_on_int();
}

void PendSV_Handler(void) {
    // TODO: seqlock_read the telemetry, joint_step(&joint, angle, vel),
    //       hand uq / theta_e to the tick with interrupts masked for the copy.
}

// 8 MHz HSE (bypass to ST-LINK clock instead of crystal) to 84 MHz SYSCLK

//   VCO in  = HSE / M       = 8 MHz / 8    = 1 MHz      (must be 1-2 MHz, lower is better)
//   VCO out = VCO in * N    = 1 MHz * 336  = 336 MHz    (must be 100-432 MHz)
//   SYSCLK  = VCO out / P   = 336 MHz / 4  = 84 MHz     (max 84 on F401)
//   USB/SDIO= VCO out / Q   = 336 MHz / 7  = 48 MHz     (unused, Q just has to be legal (2-15))
//   AHB = /1 = 84 MHz, APB1 = /2 = 42 MHz (max 42), APB2 = /1 = 84 MHz
//   APB1 timers run at 2x APB1 = 84 MHz (42MHz x 2) because APB1 prescaler isn't 1 to 1
static void SystemClock_Config(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); // 84 MHz needs scale 1

    RCC_OscInitTypeDef osc = {
        .OscillatorType = RCC_OSCILLATORTYPE_HSE,
        .HSEState = RCC_HSE_BYPASS,
        .PLL = {.PLLState = RCC_PLL_ON,
                .PLLSource = RCC_PLLSOURCE_HSE,
                .PLLM = 8,
                .PLLN = 336,
                .PLLP = RCC_PLLP_DIV4,
                .PLLQ = 7},
    };
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) for (;;) {}

    RCC_ClkInitTypeDef clk = {
        .ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2,
        .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
        .AHBCLKDivider = RCC_SYSCLK_DIV1,
        .APB1CLKDivider = RCC_HCLK_DIV2,
        .APB2CLKDivider = RCC_HCLK_DIV1,
    };
    // flash latency is set inside ClockConfig, before the switch: 2 wait states at 84 MHz, 3.3 V.
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) for (;;) {}
}

// pull en pin low when configuring clock, ensures nothing runs 
static void en_pin_low(void) {
    EN_PORT->BSRR = 1u << (EN_PIN + 16); // ODR bit low first (it resets to 0 anyway)
    GPIO_InitTypeDef g = {.Pin = 1u << EN_PIN, .Mode = GPIO_MODE_OUTPUT_PP, .Speed = GPIO_SPEED_FREQ_LOW};
    HAL_GPIO_Init(EN_PORT, &g);
}

int main(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    en_pin_low();
    HAL_Init();
    SystemClock_Config(); // CfC step cost (~14.1k cycles) was measured at 84 MHz
    uart_init();
    // temporary uart printf to check the clock is actually 84 MHz
    printf("hello, SYSCLK %lu Hz\r\n", HAL_RCC_GetSysClockFreq());

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // TODO: i2c_init + as5600_check, pwm3_init (EN low), spi_init + mcp2515_init
    // TODO: wdg_caused_reset -> FAULT_DEADMAN

    cfc_check = cfc_ctrl_check();
    joint_init(&joint, cfc_check.ok);
    can_node_init();

    // TODO: NVIC priorities: tick 0, I2C EV/ER + its DMA 1, PendSV 2, EXTI9_5 (MCP2515 INT) 3
    // TODO: start TICK_TIM at FOC_HZ, wdg_init(WDG_TIMEOUT_MS)

    for (;;) {
        // remove later: CYCCNT delta over 1000 SysTick ms should be 84,000,000
        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() == t0) {} // align to a tick edge
        t0 = HAL_GetTick();
        uint32_t c0 = DWT->CYCCNT;
        while (HAL_GetTick() - t0 < 1000) {}
        printf("cycles/1000ms: %lu\r\n", DWT->CYCCNT - c0);
        
        // TODO: wdg_kick, can_node_poll, i2c_recover after an I2C error
    }
}
