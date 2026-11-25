#include "bldc/calib.h"
#include "bldc/foc.h"
#include "bldc/joint.h"
#include "bldc/vel.h"
#include "can/can_node.h"
#include "can/can_proto.h"
#include "cfc_ctrl.h"
#include "config.h"
#include "drivers/as5600.h"
#include "drivers/i2c.h"
#include "drivers/mcp2515.h"
#include "drivers/pwm3.h"
#include "drivers/spi.h"
#include "drivers/uart.h"
#include "drivers/wdg.h"
#include "ring.h"
#include "seqlock.h"
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

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

// temporary commutation test: build with -DCOMMUTATE_TEST=1. motor free, off the leg.
// calibrates, then holds COMMUTATE_UQ with the field placed from the encoder. 10 s total, then EN low
#ifndef COMMUTATE_TEST
#define COMMUTATE_TEST 0
#endif
#ifndef COMMUTATE_UQ
#define COMMUTATE_UQ 0.5f // volts, + spins one way, - the other. clamped to CLOSEDLOOP_V
#endif
_Static_assert(!(OPENLOOP_TEST && COMMUTATE_TEST), "one motor test at a time");

// temporary leg test, build with -DJOINT_TEST=1 (hold zero), 2 (also step +-0.15 rad through zero
// every 2 s) or 3 (gravity sweep: hold -0.4..0.4..-0.4 rad, 3 s each, print the mean uq per angle).
// enables 3 s after boot, off after 60 s. SUPPORT THE LEG: any fault drops the knee ~70 deg
#ifndef JOINT_TEST
#define JOINT_TEST 0
#endif
_Static_assert(!(JOINT_TEST && (OPENLOOP_TEST || COMMUTATE_TEST)), "one motor test at a time");

// temporary CAN bench tests, build with -DCAN_TEST=1, 2 or 3:
//   1: SPI only. wire MISO (PB14) to MOSI (PB15), MCP2515 unplugged. sends 0..255, checks the echo
//   2: MCP2515 loopback mode, no bus needed. a frame a second comes back through the RX path and is compared
//   3: real bus, normal mode. a frame every 100 ms (id node<<5 | 0x1F, 4-byte counter), prints what arrives
#ifndef CAN_TEST
#define CAN_TEST 0
#endif

// temporary: busy-wait this long in every PendSV, standing in for the ~168 us CfC step
#ifndef PENDSV_LOAD_US
#define PENDSV_LOAD_US 0
#endif

static volatile uint32_t openloop_ticks_left, commutate_ticks_left; // set by main, counted down by the tick
static volatile int commutate_state = CALIB_RUNNING; // CALIB_DONE once spinning, < 0 calibration failed
static volatile float cal_dir, cal_zero;

// PendSV owns the joint once the tick runs. everyone else sends it commands through cmd_ring
// (superloop pushes, PendSV pops, lock-free single producer/consumer) and reads ctrl_telem
static joint_t joint;
static joint_cmd_t cmd_buf[8];
static ring_t cmd_ring;
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
    float angle; // rad
    uint32_t fails;
    uint32_t lost_events; // counts up each time lost is raised, so a short outage can't be missed
    uint16_t raw;
    bool lost;
} enc_telem_t;
static enc_telem_t enc_telem;
static seqlock_t enc_lock;
static volatile uint32_t enc_start_max;

// PendSV -> tick. the tick outranks PendSV, so a seqlock would spin forever if the tick caught a
// write half done; PendSV masks interrupts for the copy instead. the tick just reads it
typedef struct {
    calib_t cal;
    float uq, theta_e; // theta_e only used while calibrating, after that it comes from the encoder
    bool calibrating, en;
} drive_t;
static drive_t drive;

// PendSV -> superloop, for prints. here the reader is lower priority, so a seqlock works
typedef struct {
    float vel, vel_sp, uq, target;
    uint8_t state, faults;
    int8_t cmd_err; // result of the last command, 0 or -errno
} ctrl_telem_t;
static ctrl_telem_t ctrl_telem;
static seqlock_t ctrl_lock;
static volatile uint32_t pendsv_count, pendsv_period_min = UINT32_MAX, pendsv_period_max, pendsv_max;

// step-response recorder, leg-test builds only: the dump blocks the superloop for ~4 s, which
// would starve CAN and trip the host timeout in a normal build.
// PendSV appends one row per run, the superloop dumps it once full.
// printing from PendSV would block it for ms per line and break the timing being measured
#define RECORDER (JOINT_TEST != 0)
#if RECORDER
#define REC_N (4 * CFC_CTRL_HZ) // 4 s, 40 KB
typedef struct {
    uint32_t n; // outer-loop runs since boot
    float angle, vel_sp, vel, uq;
} rec_row_t;
static rec_row_t rec[REC_N];
static volatile uint32_t rec_len; // REC_N means full and waiting to be dumped
#endif

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
        // wrap to +-pi around the zero, so the 0/2pi jump of raw can't land inside the travel
        e.angle = ENC_DIR * remainderf(raw * AS5600_RAD_PER_COUNT - ENC_ZERO_RAD, 6.2831853f);
        fail_run = 0;
        e.lost = false;
    } else if (r != -EAGAIN) { // -EAGAIN: nothing was started yet, not a failure
        e.fails++;             // includes a read still running a tick later
        if (fail_run < ENC_LOST_N && ++fail_run == ENC_LOST_N) {
            e.lost = true;
            e.lost_events++;
        }
    }
    if (r != -EINPROGRESS) { // never start over a read that's still running
        uint32_t t = DWT->CYCCNT;
        as5600_start();
        uint32_t c = DWT->CYCCNT - t;
        if (c > enc_start_max) enc_start_max = c;
    }

    seqlock_write(&enc_lock, &enc_telem, &e, sizeof e);

    // outer loop every 10th tick: PendSV runs as soon as this handler returns, on this tick's angle
    static uint32_t ctrl_div;
    bool ctrl_tick = ++ctrl_div == FOC_HZ / CFC_CTRL_HZ;
    if (ctrl_tick) {
        ctrl_div = 0;
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    // commutate every tick from the fresh angle; uq only changes at 500 Hz
    static bool en;
    if (drive.en && isfinite(e.angle)) {
        float th = drive.calibrating ? drive.theta_e : calib_theta_e(&drive.cal, e.angle);
        float duty[3];
        foc_svpwm(drive.uq, th, SUPPLY_V, duty);
        pwm3_set(duty);
    }
    if (drive.en != en) { // EN after the first duties, off before anything else
        en = drive.en;
        pwm3_enable(en);
    }

#if COMMUTATE_TEST
    if (commutate_ticks_left > 0) {
        static calib_t cal; // zeroed is the same as calib_start
        static float uq, theta_e;
        int stop = e.lost || !isfinite(e.angle) ? -EIO : 0;
        if (!stop && commutate_state == CALIB_RUNNING) {
            if (ctrl_tick) { // calibration is written for the outer-loop rate
                int r = calib_step(&cal, e.angle, &uq, &theta_e);
                if (r == CALIB_DONE) {
                    cal_dir = cal.dir;
                    cal_zero = cal.zero_elec;
                    commutate_state = CALIB_DONE;
                }
                if (r < 0) stop = r;
            }
        } else if (!stop) {
            uq = fminf(fmaxf(COMMUTATE_UQ, -CLOSEDLOOP_V), CLOSEDLOOP_V);
            theta_e = calib_theta_e(&cal, e.angle);
        }
        float duty[3];
        foc_svpwm(uq, theta_e, SUPPLY_V, duty);
        pwm3_set(duty);
        if (stop) {
            commutate_state = stop;
            commutate_ticks_left = 0;
            pwm3_enable(false);
        } else if (--commutate_ticks_left == 0) {
            pwm3_enable(false);
        }
    }
#endif

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

// outer loop at CFC_CTRL_HZ, pended by the tick. priority 2, so the tick and I2C preempt it
void PendSV_Handler(void) {
    static uint32_t last_entry;
    uint32_t entry = DWT->CYCCNT;
    if (pendsv_count > 0) {
        uint32_t period = entry - last_entry;
        if (period < pendsv_period_min) pendsv_period_min = period;
        if (period > pendsv_period_max) pendsv_period_max = period;
    }
    last_entry = entry;
    pendsv_count++;

    static int8_t cmd_err;
    joint_cmd_t cmd;
    while (ring_pop(&cmd_ring, &cmd)) cmd_err = (int8_t)joint_apply(&joint, &cmd);

    enc_telem_t e;
    seqlock_read(&enc_lock, &e, &enc_telem, sizeof e);
    static vel_t vs;
    float vel = isfinite(e.angle) ? vel_step(&vs, e.angle) : NAN;
    // only while driving, like the bring-up. the counter catches an outage that started and ended
    // between two runs; while idle it's just followed, so an old outage can't fault a later enable
    static uint32_t lost_seen;
    bool driving = joint.state == JOINT_CALIBRATING || joint.state == JOINT_CLOSED_PID ||
                   joint.state == JOINT_CLOSED_CFC;
    if (driving && (e.lost || e.lost_events != lost_seen)) joint_fault(&joint, FAULT_ENCODER_LOST);
    lost_seen = e.lost_events;
    joint_step(&joint, e.angle, vel);

    bool active = joint.state != JOINT_IDLE && joint.state != JOINT_FAULT;
    drive_t d = {.cal = joint.cal, .uq = joint.uq, .theta_e = joint.theta_e,
                 .calibrating = joint.state == JOINT_CALIBRATING, .en = active};
    __disable_irq(); // a few cycles; the tick waits at most this long
    drive = d;
    __enable_irq();

    ctrl_telem_t t = {.vel = vel, .vel_sp = joint.vel_sp, .uq = joint.uq, .target = joint.target,
                      .state = joint.state, .faults = joint.faults, .cmd_err = cmd_err};
    seqlock_write(&ctrl_lock, &ctrl_telem, &t, sizeof t);

#if RECORDER
    // record from each start of closed loop until the buffer is full; dumping frees it again
    static bool was_closed;
    bool closed = joint.state == JOINT_CLOSED_PID || joint.state == JOINT_CLOSED_CFC;
    if (closed && !was_closed && rec_len == 0) rec_len = 1;
    was_closed = closed;
    if (rec_len > 0 && rec_len <= REC_N) {
        rec[rec_len - 1] = (rec_row_t){pendsv_count, e.angle, joint.vel_sp, vel, joint.uq};
        rec_len++;
    }
#endif

#if PENDSV_LOAD_US
    while (DWT->CYCCNT - entry < PENDSV_LOAD_US * (TIM_CLK_HZ / 1000000)) {}
#endif
    uint32_t c = DWT->CYCCNT - entry;
    if (c > pendsv_max) pendsv_max = c;
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
// the other joint's EN is JTAG NJTRST, so it idles high
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

// dump the recorder as CSV once full. blocking is fine here, but keep kicking between lines
static void rec_dump(void) {
#if RECORDER
    if (rec_len <= REC_N) return;
    printf("rec: t_ms,angle_mrad,vel_sp_mrad_s,vel_mrad_s,uq_mv\r\n");
    for (uint32_t i = 0; i < REC_N; i++) {
        const rec_row_t *r = &rec[i];
        printf("%lu,%ld,%ld,%ld,%ld\r\n", (r->n - rec[0].n) * (1000 / CFC_CTRL_HZ), milli(r->angle),
               milli(r->vel_sp), milli(r->vel), milli(r->uq));
        kick_if_tick_alive();
    }
    printf("rec: end\r\n");
    rec_len = 0;
#endif
}

// returns false if the ring is full (PendSV not running)
static bool joint_push(const joint_cmd_t *c) { return ring_push(&cmd_ring, c); }
static inline bool joint_cmd(joint_cmd_op_t op, float arg) { return joint_push(&(joint_cmd_t){op, arg}); }

#if JOINT_TEST
static void joint_test_service(void) {
    static uint32_t next_ms = 3000, step;
    uint32_t now = HAL_GetTick();
    if (now < next_ms || next_ms == UINT32_MAX) return;
    ctrl_telem_t c;
    seqlock_read(&ctrl_lock, &c, &ctrl_telem, sizeof c);
    if (now >= 63000) {
        joint_cmd(JCMD_DISABLE, 0);
        printf("joint test: 60 s up, disabled\r\n");
        next_ms = UINT32_MAX;
    } else if (c.state == JOINT_FAULT) { // never re-enable after a fault, reset the board instead
        printf("joint test: FAULT 0x%02x, stays off\r\n", c.faults);
        next_ms = UINT32_MAX;
    } else if (c.state == JOINT_IDLE) { // refused, e.g. outside travel: move it near zero, it retries
        if (c.cmd_err) printf("joint test: enable refused (err %d), retrying\r\n", c.cmd_err);
        joint_cmd(JCMD_ENABLE, 0);
        next_ms = now + 1000;
    } else if (JOINT_TEST == 2 && c.state == JOINT_CLOSED_PID) {
        static const float seq[] = {0.15f, 0.0f, -0.15f, 0.0f};
        joint_cmd(JCMD_TARGET, seq[step++ % 4]);
        next_ms = now + 2000;
    } else if (JOINT_TEST == 3 && c.state == JOINT_CLOSED_PID) {
        // up then down, so friction pulls each angle both ways and averages out in the fit.
        // every 100 ms: 30 per hold, the last 10 (1 s, settled) are averaged
        static float uq_sum, a_sum;
        uint32_t k = step % 30, hold = step / 30;
        if (hold >= 17) {
            next_ms = UINT32_MAX;
            joint_cmd(JCMD_DISABLE, 0);
            printf("grav: done, disabled\r\n");
            return;
        }
        if (k == 0) joint_cmd(JCMD_TARGET, -0.4f + 0.1f * (float)(hold <= 8 ? hold : 16 - hold));
        if (k >= 20) {
            enc_telem_t e;
            seqlock_read(&enc_lock, &e, &enc_telem, sizeof e);
            uq_sum += c.uq;
            a_sum += e.angle;
        }
        if (k == 29) {
            printf("grav: %ld,%ld\r\n", milli(a_sum / 10), milli(uq_sum / 10)); // angle mrad, uq mV
            uq_sum = a_sum = 0;
        }
        step++;
        next_ms = now + 100;
    } else {
        next_ms = now + 100;
    }
}
#endif

#if CAN_TEST >= 2
static uint32_t can_test_sent;
static can_frame_t can_test_frame;
#if CAN_TEST == 2
static uint32_t can_test_ok, can_test_bad;
#endif

static void can_test_service(void) {
    static uint32_t next_ms;
    uint32_t now = HAL_GetTick();
    if (now < next_ms) return;
    next_ms = now + (CAN_TEST == 2 ? 1000 : 100);
#if CAN_TEST == 2
    // last second's frame should be back by now. compare every field: a wrong ID shift still
    // gets the data right
    if (can_test_sent) {
        can_node_stats_t c = can_node_stats();
        bool same = c.last.id == can_test_frame.id && c.last.dlc == can_test_frame.dlc &&
                    memcmp(c.last.data, can_test_frame.data, can_test_frame.dlc) == 0;
        if (same) can_test_ok++;
        else can_test_bad++;
    }
    // 0x5A5 alternates bits across the SIDH/SIDL split; the DLC walks 0..8
    can_test_frame = (can_frame_t){.id = 0x5A5, .dlc = (uint8_t)(can_test_sent % 9)};
    for (int i = 0; i < 8; i++) can_test_frame.data[i] = (uint8_t)(can_test_sent + 17 * i);
#else
    can_test_frame = (can_frame_t){.id = can_make_id(CAN_NODE, 0x1F), .dlc = 4};
    memcpy(can_test_frame.data, &can_test_sent, 4);
#endif
    if (can_node_send(&can_test_frame) == 0) can_test_sent++;
}
#endif

// everything the superloop must do every pass, including inside temporary busy-waits
static uint32_t i2c_recoveries;
static void superloop_service(void) {
    kick_if_tick_alive();
    if (i2c_recover() == 1) i2c_recoveries++;
    rec_dump();
    enc_telem_t e;
    ctrl_telem_t c;
    seqlock_read(&enc_lock, &e, &enc_telem, sizeof e);
    seqlock_read(&ctrl_lock, &c, &ctrl_telem, sizeof c);
    can_node_telem_t t = {.angle = e.angle, .vel = c.vel, .state = (joint_state_t)c.state,
                          .joint_faults = c.faults, .cmd_err = c.cmd_err, .i2c_err = e.fails};
    can_node_poll(HAL_GetTick(), &t, joint_push);
#if CAN_TEST >= 2
    can_test_service();
#endif
#if JOINT_TEST
    joint_test_service();
#endif
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
    bool deadman = wdg_caused_reset(); // must be read before anything else clears RCC->CSR
    printf("reset cause: %s\r\n", deadman ? "IWDG (watchdog)" : "other");

    cfc_check = cfc_ctrl_check();
    joint_init(&joint, cfc_check.ok, enc_check == 0);
    ring_init(&cmd_ring, cmd_buf, 8, sizeof cmd_buf[0]);
#if CAN_TEST == 1
    {
        spi_init();
        uint8_t tx[256], rx[256];
        for (int i = 0; i < 256; i++) tx[i] = (uint8_t)i;
        spi_cs(true);
        int r = spi_xfer(tx, rx, sizeof tx);
        spi_cs(false);
        int bad = 0;
        for (int i = 0; i < 256; i++) bad += rx[i] != tx[i];
        printf("spi loopback: err %d, %d of 256 bytes wrong\r\n", r, bad);
    }
#endif
    int can_err = can_node_init(CAN_TEST == 2 ? MCP2515_MODE_LOOPBACK : MCP2515_MODE_NORMAL,
                                deadman ? FAULT_DEADMAN : 0); // STATUS reports it until the next ENABLE
    printf("can: %s (err %d), INT %s\r\n", can_err == 0 ? "ok" : "FAILED", can_err,
           can_node_stats().int_active ? "on" : "off, polling");

    NVIC_SetPriority(PendSV_IRQn, 2); // below the tick (0) and I2C (1)
    tick_start();

    // after slow init: the counter runs when this exits
    if (wdg_init(WDG_TIMEOUT_MS) != 0) for (;;) {}

#if OPENLOOP_TEST
    printf("open loop: %d mV, %d mrad/s electrical, 10 s\r\n", (int)(OPENLOOP_UQ * 1000), (int)(OPENLOOP_W_E * 1000));
    openloop_ticks_left = OPENLOOP_MAX_TICKS; // the tick sets duties from here on
    pwm3_enable(true);                        // breaker on last
#endif
#if COMMUTATE_TEST
    printf("commutate: calibrating, then %d mV for the rest of 10 s. hands off\r\n", (int)(COMMUTATE_UQ * 1000));
    commutate_ticks_left = 10 * FOC_HZ;
    pwm3_enable(true);
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
            if (ms % 100 == 0) {
                ctrl_telem_t c;
                seqlock_read(&ctrl_lock, &c, &ctrl_telem, sizeof c);
                printf("angle %ld mrad  target %ld  vel %ld mrad/s  uq %ld mV  state %u faults 0x%02x\r\n",
                       milli(e.angle), milli(c.target), milli(c.vel), milli(c.uq), c.state, c.faults);
            }
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
        // period should sit near 168000 (2 ms); max cycles includes time the tick preempted it
        printf("pendsv: %lu runs  period %lu-%lu  max %lu cyc\r\n", pendsv_count, pendsv_period_min,
               pendsv_period_max, pendsv_max);
        i2c_timing_t it = i2c_last_timing();
        printf("enc raw %u  noise %ld mcounts (%lu samples)  fails %lu  lost %d (%lu times)  recoveries %lu\r\n",
               e.raw, milli(noise), n, e.fails, e.lost, e.lost_events, i2c_recoveries);
        printf("i2c read %lu cyc, isr %lu cyc, start max %lu cyc\r\n", it.xfer_cycles, it.isr_cycles, enc_start_max);
#if CAN_TEST >= 2
        {
            can_node_stats_t c = can_node_stats();
            printf("can: sent %lu", can_test_sent);
#if CAN_TEST == 2
            printf("  ok %lu  bad %lu", can_test_ok, can_test_bad);
#endif
            printf("  rx %lu  dropped %lu  hw_ovf %lu  tx_fail %lu  tec %u rec %u eflg 0x%02x\r\n", c.rx,
                   c.rx_dropped, c.hw_overflow, c.tx_fail, c.tec, c.rec, c.eflg);
            printf("can: last id 0x%03x dlc %u:", c.last.id, c.last.dlc);
            for (int i = 0; i < c.last.dlc; i++) printf(" %02x", c.last.data[i]);
            printf("\r\n");
        }
#endif
#if OPENLOOP_TEST
        printf("open loop: %s\r\n", openloop_ticks_left ? "running" : "stopped, EN low");
#endif
#if COMMUTATE_TEST
        if (commutate_state < 0) {
            printf("commutate: calibration FAILED (err %d), EN low\r\n", commutate_state);
        } else if (commutate_state == CALIB_DONE) {
            // simplefoc's zero_electric_angle for the same rotor, to compare with the bring-up's print
            float sfoc = fmodf(cal_zero + cal_dir * ENC_DIR * POLE_PAIRS * ENC_ZERO_RAD, 6.2831853f);
            if (sfoc < 0) sfoc += 6.2831853f;
            printf("commutate: dir %+d  zero_elec %ld mrad (simplefoc %ld)  %s\r\n", (int)cal_dir, milli(cal_zero),
                   milli(sfoc), commutate_ticks_left ? "spinning" : "stopped, EN low");
        }
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
        superloop_service();
    }
}
