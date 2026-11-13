/**
 * @file pwm3.c
 * @brief TIM3 center-aligned PWM and EN. See pwm3.h.
 */

#include "drivers/pwm3.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <errno.h>
#include <math.h>

// PWM mode 1: output high while CNT < CCR, in both count directions. with center-aligned
// counting that gives a pulse of 2 x CCR counts out of 2 x ARR, so duty = CCR / ARR
#define OC_PWM1 (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1)

static void pin_af2(GPIO_TypeDef *port, uint32_t pin) {
    GPIO_InitTypeDef g = {
        .Pin = 1u << pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF2_TIM3,
    };
    HAL_GPIO_Init(port, &g);
}

int pwm3_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    PWM_TIM->PSC = 0; // count at the full 84 MHz timer clock
    PWM_TIM->ARR = PWM_ARR;
    PWM_TIM->CCR1 = PWM_ARR / 2; // 50%: no voltage across the motor
    PWM_TIM->CCR2 = PWM_ARR / 2;
    PWM_TIM->CCR3 = PWM_ARR / 2;

    // channel 1 and 2 live in CCMR1, channel 3 in CCMR2. OCxPE = preload, so a new CCR
    // waits for the next update event instead of landing mid-pulse
    PWM_TIM->CCMR1 = OC_PWM1 | TIM_CCMR1_OC1PE | (OC_PWM1 << 8) | TIM_CCMR1_OC2PE;
    PWM_TIM->CCMR2 = OC_PWM1 | TIM_CCMR2_OC3PE;
    PWM_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E; // active high

    // CMS_0 = center-aligned mode 1 (only settable while stopped). ARPE preloads ARR too
    PWM_TIM->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE;
    PWM_TIM->EGR = TIM_EGR_UG; // load the preloaded ARR and CCRs now
    PWM_TIM->CR1 |= TIM_CR1_CEN;

    // pins last, so they only ever see a running 50% signal. EN stays low
    pin_af2(PWM_A_PORT, PWM_A_PIN);
    pin_af2(PWM_B_PORT, PWM_B_PIN);
    pin_af2(PWM_C_PORT, PWM_C_PIN);
    return 0;
}

int pwm3_set(const float duty[3]) {
    if (!isfinite(duty[0]) || !isfinite(duty[1]) || !isfinite(duty[2])) {
        return -EINVAL; // a NaN cast to an integer CCR is undefined
    }
    PWM_TIM->CCR1 = (uint32_t)(fminf(fmaxf(duty[0], 0.0f), 1.0f) * PWM_ARR);
    PWM_TIM->CCR2 = (uint32_t)(fminf(fmaxf(duty[1], 0.0f), 1.0f) * PWM_ARR);
    PWM_TIM->CCR3 = (uint32_t)(fminf(fmaxf(duty[2], 0.0f), 1.0f) * PWM_ARR);
    return 0;
}

int pwm3_enable(bool on) {
    if (on) {
        EN_PORT->BSRR = 1u << EN_PIN;
    } else {
        EN_PORT->BSRR = 1u << (EN_PIN + 16); // breaker off first
        PWM_TIM->CCR1 = PWM_ARR / 2;
        PWM_TIM->CCR2 = PWM_ARR / 2;
        PWM_TIM->CCR3 = PWM_ARR / 2;
    }
    return 0;
}
