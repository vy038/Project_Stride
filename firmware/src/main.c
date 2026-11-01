#include "bldc/joint.h"
#include "can/can_node.h"
#include "cfc_ctrl.h"
#include "config.h"
#include "stm32f4xx_hal.h"

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

int main(void) {
    // TODO: EN pin low before anything else can fail
    HAL_Init();
    // TODO: SystemClock_Config, 84 MHz. The CfC step cost (~14.1k cycles) was measured at 84 MHz.
    // TODO: i2c_init + as5600_check, pwm3_init (EN low), spi_init + mcp2515_init
    // TODO: wdg_caused_reset -> FAULT_DEADMAN

    cfc_check = cfc_ctrl_check();
    joint_init(&joint, cfc_check.ok);
    can_node_init();

    // TODO: NVIC priorities: tick 0, I2C EV/ER + its DMA 1, PendSV 2, EXTI9_5 (MCP2515 INT) 3
    // TODO: start TICK_TIM at FOC_HZ, wdg_init(WDG_TIMEOUT_MS)

    for (;;) {
        // TODO: wdg_kick, can_node_poll, i2c_recover after an I2C error
    }
}
