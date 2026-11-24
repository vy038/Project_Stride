#include "bldc/foc.h"
#include "bldc/joint.h"
#include "bldc/vel.h"
#include "can/can_node.h"
#include "cfc_ctrl.h"
#include "config.h"
#include "drivers/as5600.h"
#include "drivers/i2c.h"
#include "drivers/pwm3.h"
#include "drivers/uart.h"
#include "drivers/wdg.h"
#include "seqlock.h"
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>

// temporary test
#ifndef WDG_TEST
#define WDG_TEST 0
#endif

// temporary open-loop spin: build with -DOPENLOOP_TEST=1. motor free, off the leg, supply at 0.5 A.
// reverse with -DOPENLOOP_W_E=-6.2831853f. 1 electrical rev/s = 51.4 deg of shaft per second,
// so the shaft makes one full turn in 7 s
#ifndef OPENLOOP_TEST
#define OPENLOOP_TEST 0
#endif
#ifndef OPENLOOP_UQ
#define OPENLOOP_UQ 1.0f // volts, start at 1, never above 2
#endif
#ifndef OPENLOOP_W_E
#define OPENLOOP_W_E 6.2831853f // electrical rad/s, 1 rev/s
#endif
#define OPENLOOP_MAX_TICKS (10 * FOC_HZ) // stops itself after 10 s: it heats the motor even stalled

static volatile uint32_t openloop_ticks_left; // set by main, counted down by the tick

static joint_t joint;
static cfc_check_t cfc_check;

void SysTick_Handler(void) { HAL_IncTick(); }

// written only by the tick, read from main
// min/max are since boot, so main never resets them
static volatile uint32_t tick_count;
static volatile uint32_t isr_min = UINT32_MAX, isr_max;
static volatile uint32_t period_min = UINT32_MAX, period_max;

// encoder telemetry. the tick is the only writer, lower priorities read it with seqlock_read.
// on a miss raw/angle hold the last good reading; angle is NAN until the first one
typedef struct {
    float angle, vel; // rad, rad/s
    uint32_t fails;
    uint16_t raw;
    bool lost;
} enc_telem_t;
static enc_telem_t enc_telem;
static seqlock_t enc_lock;
static volatile uint32_t enc_lost_events, enc_start_max;

void TIM2_IRQHandler(void) {
    static uint32_t last_entry;
    uint32_t entry = DWT->CYCCNT;

    // SR bits clear on writing 0, 1s are ignored
    TICK_TIM->SR = ~TIM_SR_UIF;

    if (tick_count > 0) { // first tick has no previous entry to measure from
        uint32_t period = entry - last_entry;
        if (period < period_min) period_min = period;
        if (period > period_max) period_max = period;
    }
    last_entry = entry;
    tick_count++;

    // encoder: collect the read started last tick, then start the next, so reads never overlap
    static enc_telem_t e = {.angle = NAN};
    static uint32_t fail_run;
    uint16_t raw;
    int r = as5600_get(&raw);
    if (r == 0) {
        e.raw = raw;
        e.angle = ENC_DIR * (raw * AS5600_RAD_PER_COUNT - ENC_ZERO_RAD);
        fail_run = 0;
        e.lost = false;
    } else if (r != -EAGAIN) { // -EAGAIN: nothing was started yet, not a failure
        e.fails++;             // includes a read still running a tick later
        if (fail_run < ENC_LOST_N && ++fail_run == ENC_LOST_N) {
            e.lost = true;
            enc_lost_events++;
        }
    }
    if (r != -EINPROGRESS) { // never start over a read that's still running
        uint32_t t = DWT->CYCCNT;
        as5600_start();
        uint32_t c = DWT->CYCCNT - t;
        if (c > enc_start_max) enc_start_max = c;
    }

    // velocity at the outer-loop rate, its window and filter assume that dt
    // TODO: move into PendSV with the outer loop
    static vel_t vel;
    static uint32_t vel_div;
    if (++vel_div == FOC_HZ / CFC_CTRL_HZ) {
        vel_div = 0;
        if (isfinite(e.angle)) e.vel = vel_step(&vel, e.angle);
    }
    seqlock_write(&enc_lock, &enc_telem, &e, sizeof e);

    // TODO:
    //   theta_e from angle (or joint.theta_e while calibrating) -> foc_svpwm(uq, theta_e, SUPPLY_V) -> pwm3_set
    //   every FOC_HZ / CFC_CTRL_HZ ticks: pend PendSV

    // open loop: rotate the field ourselves, the rotor gets dragged along
    if (openloop_ticks_left > 0) {
        static float theta_e;
        theta_e = fmodf(theta_e + OPENLOOP_W_E / FOC_HZ, 6.2831853f); // keep it small, float loses precision
        float duty[3];
        foc_svpwm(fminf(OPENLOOP_UQ, 2.0f), theta_e, SUPPLY_V, duty);
        pwm3_set(duty);
        if (--openloop_ticks_left == 0) pwm3_enable(false);
    }

    // grabs min and max cycles to get jitter
    uint32_t cycles = DWT->CYCCNT - entry;
    if (cycles < isr_min) isr_min = cycles;
    if (cycles > isr_max) isr_max = cycles;
}

void EXTI9_5_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(1u << CAN_INT_PIN);
    can_node_on_int();
}

void PendSV_Handler(void) {
    // TODO: seqlock_read the telemetry, joint_step(&joint, angle, vel),
    //       hand uq / theta_e to the tick with interrupts masked for the copy.
}

// 8 MHz HSE (bypass, driven by the ST-LINK) to 84 MHz SYSCLK. Each stage has a hardware limit:
//   VCO in  = HSE / M      = 8 MHz / 4   = 2 MHz
//       1-2 MHz; the phase comparator needs a slow clean reference (2 is recommended for less jitter)
//   VCO out = VCO in * N   = 2 MHz * 168 = 336 MHz
//       192-432 MHz on F401; the VCO only locks in this band, so it can't run at 84 directly
//   SYSCLK  = VCO out / P  = 336 MHz / 4 = 84 MHz
//       max 84; P is 2, 4, 6 or 8 and only 4 keeps the VCO in band (2 -> 168, 6 -> 504)
//   USB/SDIO= VCO out / Q  = 336 MHz / 7 = 48 MHz
//       unused; second output of the same VCO, Q just has to be legal (2-15)
//   AHB /1 = 84 MHz
//       max 84; feeds the CPU, memory and DMA
//   APB1 /2 = 42 MHz
//       max 42, so it needs /2; slow bus for UART2, I2C1, SPI2, TIM2/3
//   APB2 /1 = 84 MHz
//       max 84; fast bus
//   APB1 timers = 2 x 42 = 84 MHz
//       timers on a bus with a prescaler other than /1 run at 2x the bus clock
static void SystemClock_Config(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2); // scale to 2 for <= 84 MHz

    RCC_OscInitTypeDef osc = {
        .OscillatorType = RCC_OSCILLATORTYPE_HSE,
        .HSEState = RCC_HSE_BYPASS,
        .PLL = {.PLLState = RCC_PLL_ON,
                .PLLSource = RCC_PLLSOURCE_HSE,
                .PLLM = 4,
                .PLLN = 168,
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

// pull both driver EN pins low before configuring anything
// the other joint's EN mis JTAG NJTRST, so it idles high
static void en_pin_low(void) {
    EN_PORT->BSRR = EN_ALL_RESET; // ODR bits low first (they reset to 0 anyway)
    GPIO_InitTypeDef g = {.Pin = (1u << EN_PIN) | (1u << OTHER_EN_PIN), .Mode = GPIO_MODE_OUTPUT_PP, .Speed = GPIO_SPEED_FREQ_LOW};
    HAL_GPIO_Init(EN_PORT, &g);
}

// TIM2 update interrupt at FOC_HZ
// 84 MHz / (PSC+1) = 1 MHz count, 1 MHz / (ARR+1) = 5 kHz, so 200 us per tick
static void tick_start(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    TICK_TIM->PSC = TIM_CLK_HZ / TICK_CNT_HZ - 1;
    TICK_TIM->ARR = TICK_CNT_HZ / FOC_HZ - 1;
    TICK_TIM->EGR = TIM_EGR_UG;  // PSC only loads on an update event, so force one
    TICK_TIM->SR = ~TIM_SR_UIF;  // UG also set UIF, clear it or the ISR fires straight away
    TICK_TIM->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);
    TICK_TIM->CR1 = TIM_CR1_CEN;
}

// kick only if this loop is running AND the tick has fired since the last kick. if either dies
// the kicks stop and the IWDG resets the chip. a dead tick would otherwise leave PWM at its last duty
static void kick_if_tick_alive(void) {
    static uint32_t last_seen;
    uint32_t now = tick_count;
    if (now != last_seen) {
        last_seen = now;
        wdg_kick();
    }
}

// for printing floats without float printf. NAN prints as 0
static long milli(float x) { return isfinite(x) ? lroundf(x * 1000.0f) : 0; }

// everything the superloop must do every pass, including inside temporary busy-waits
static uint32_t i2c_recoveries;
static void superloop_service(void) {
    kick_if_tick_alive();
    if (i2c_recover() == 1) i2c_recoveries++;
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
    pwm3_init(); // 50% on all three, EN still low

    // blocking boot check, before the tick starts its own reads
    uint8_t agc = 0;
    int enc_check = i2c_init();
    if (enc_check == 0) enc_check = as5600_check(&agc);
    printf("as5600: %s (err %d), agc %u of %u\r\n", enc_check == 0 ? "ok" : "FAILED, never enable", enc_check, agc,
           AS5600_AGC_MAX);
    // TODO: refuse joint_enable when enc_check != 0, spi_init + mcp2515_init
    // TODO: report FAULT_DEADMAN to the host over CAN once joint faults exist
    bool deadman = wdg_caused_reset(); // must be read before anything else clears RCC->CSR
    printf("reset cause: %s\r\n", deadman ? "IWDG (watchdog)" : "other");

    cfc_check = cfc_ctrl_check();
    joint_init(&joint, cfc_check.ok);
    can_node_init();

    // TODO: NVIC priorities: I2C EV/ER + its DMA 1, PendSV 2, EXTI9_5 (MCP2515 INT) 3
    tick_start();

    // after slow init: the counter runs when this exits
    if (wdg_init(WDG_TIMEOUT_MS) != 0) for (;;) {}

#if OPENLOOP_TEST
    printf("open loop: %d mV, %d mrad/s electrical, 10 s\r\n", (int)(OPENLOOP_UQ * 1000), (int)(OPENLOOP_W_E * 1000));
    openloop_ticks_left = OPENLOOP_MAX_TICKS; // the tick sets duties from here on
    pwm3_enable(true);                        // breaker on last
#endif

    for (;;) {
        // remove later: CYCCNT delta over 1000 SysTick ms should be 84,000,000
        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() == t0) {} // align to a tick edge
        t0 = HAL_GetTick();
        uint32_t c0 = DWT->CYCCNT;
        uint32_t n0 = tick_count;
        // once per ms: sample raw for the noise figure. every 100 ms: print angle and velocity
        enc_telem_t e;
        seqlock_read(&enc_lock, &e, &enc_telem, sizeof e);
        uint16_t raw0 = e.raw;
        int32_t sum = 0;
        uint64_t sumsq = 0;
        uint32_t n = 0, last_ms = UINT32_MAX;
        for (uint32_t ms; (ms = HAL_GetTick() - t0) < 1000;) {
            superloop_service(); // the busy-wait is part of the superloop
            if (ms == last_ms) continue;
            last_ms = ms;
            seqlock_read(&enc_lock, &e, &enc_telem, sizeof e);
            int32_t d = (int32_t)e.raw - raw0; // offset from the first sample keeps the sums small
            sum += d;
            sumsq += (uint64_t)((int64_t)d * d);
            n++;
            if (ms % 100 == 0) printf("angle %ld mrad  vel %ld mrad/s\r\n", milli(e.angle), milli(e.vel));
        }
        // std of raw in counts, only meaningful with the shaft still
        float mean = (float)sum / n;
        float noise = sqrtf(fmaxf((float)sumsq / n - mean * mean, 0.0f));
        // use DWT->CYCCNT as the counter so it doesn't affect tick ISR (independent of tick timer)
        uint32_t cycles = DWT->CYCCNT - c0;
        uint32_t ticks = tick_count - n0;
        printf("cycles/1000ms: %lu  ticks/1000ms: %lu\r\n", cycles, ticks);
        // remove later: budget is 16800 cycles, period should sit within a few cycles of 16800
        printf("isr cycles: %lu-%lu  period: %lu-%lu\r\n", isr_min, isr_max, period_min, period_max);
        i2c_timing_t it = i2c_last_timing();
        printf("enc raw %u  noise %ld mcounts (%lu samples)  fails %lu  lost %d (%lu times)  recoveries %lu\r\n",
               e.raw, milli(noise), n, e.fails, e.lost, enc_lost_events, i2c_recoveries);
        printf("i2c read %lu cyc, isr %lu cyc, start max %lu cyc\r\n", it.xfer_cycles, it.isr_cycles, enc_start_max);
#if OPENLOOP_TEST
        printf("open loop: %s\r\n", openloop_ticks_left ? "running" : "stopped, EN low");
#endif

        // watchdog test
#if WDG_TEST == 1 // build with -DWDG_TEST=1: stop kicking, chip must reset
        printf("no more kicks\r\n");
        for (;;) {}
#elif WDG_TEST == 2 // build with -DWDG_TEST=2: HardFault. Driver supply OFF.
        printf("calling null\r\n");
        ((void (*)(void))0)();
#elif WDG_TEST == 3 // build with -DWDG_TEST=3: tick dies, loop keeps going, chip must still reset
        printf("tick stopped\r\n");
        NVIC_DisableIRQ(TIM2_IRQn);
        for (;;) {
            kick_if_tick_alive();
        }
#endif
        // TODO: can_node_poll

        superloop_service();
    }
}
