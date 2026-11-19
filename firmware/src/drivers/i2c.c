/**
 * @file i2c.c
 * @brief I2C1 driver: blocking boot reads, interrupt-driven tick reads, bus recovery. See i2c.h.
 */

#include "drivers/i2c.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <errno.h>
#include <stdbool.h>

#define I2C_IRQ_PRIO   1 // above PendSV (2), below the tick (0)
#define I2C_TIMEOUT_MS (I2C_TIMEOUT_US / 1000)
_Static_assert(I2C_TIMEOUT_US % 1000 == 0 && I2C_TIMEOUT_US >= 1000, "HAL timeouts are whole ms");

static I2C_HandleTypeDef hi2c;

// set by the HAL callbacks (priority 1), cleared by the tick before each start
static volatile bool read_done, read_err;
static volatile uint32_t isr_cycles, done_at;
// tick only
static bool in_flight;
static uint32_t started_at;
static i2c_timing_t last_timing;
// set by the tick on an error, cleared by i2c_recover once the bus works again.
// while set, the tick never touches the peripheral, so recovery can't be interrupted by a read
static volatile bool need_recover;

static uint32_t us_to_cycles(uint32_t us) { return us * (SystemCoreClock / 1000000u); }

static void delay_us(uint32_t us) {
    uint32_t t0 = DWT->CYCCNT;
    while (DWT->CYCCNT - t0 < us_to_cycles(us)) {}
}

static void pin(GPIO_TypeDef *port, uint32_t n, uint32_t mode) {
    GPIO_InitTypeDef g = {
        .Pin = 1u << n,
        .Mode = mode,
        .Pull = GPIO_NOPULL, // external pull-ups; the internal ones are too weak for 400 kHz
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF4_I2C1,
    };
    HAL_GPIO_Init(port, &g);
}

static void irqs(bool on) {
    if (on) {
        NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
        NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
        NVIC_EnableIRQ(I2C1_EV_IRQn);
        NVIC_EnableIRQ(I2C1_ER_IRQn);
    } else {
        NVIC_DisableIRQ(I2C1_EV_IRQn);
        NVIC_DisableIRQ(I2C1_ER_IRQn);
    }
}

static int to_errno(HAL_StatusTypeDef s) {
    switch (s) {
    case HAL_OK: return 0;
    case HAL_TIMEOUT: return -ETIMEDOUT;
    case HAL_BUSY: return -EBUSY;
    default: return -EIO;
    }
}

// pins, peripheral and interrupts. shared by init and recovery.
// pins and clocks are done here instead of in HAL_I2C_MspInit, so the weak empty one is fine
static int setup(void) {
    pin(SCL_PORT, SCL_PIN, GPIO_MODE_AF_OD);
    pin(SDA_PORT, SDA_PIN, GPIO_MODE_AF_OD);

    hi2c.Instance = ENC_I2C;
    hi2c.Init.ClockSpeed = I2C_HZ;
    hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2; // fast mode (400 kHz) timing
    hi2c.Init.OwnAddress1 = 0;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    hi2c.State = HAL_I2C_STATE_RESET; // forget any half-finished transfer, unlocks the handle
    if (HAL_I2C_Init(&hi2c) != HAL_OK) return -EIO; // includes a SWRST of the peripheral

    NVIC_SetPriority(I2C1_EV_IRQn, I2C_IRQ_PRIO);
    NVIC_SetPriority(I2C1_ER_IRQn, I2C_IRQ_PRIO);
    irqs(true);
    return 0;
}

int i2c_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    return setup();
}

int i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
    if (n == 0 || n > UINT16_MAX) return -EINVAL;
    return to_errno(HAL_I2C_Mem_Read(&hi2c, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)n,
                                     I2C_TIMEOUT_MS));
}

int i2c_write(uint8_t addr, uint8_t reg, const uint8_t *buf, size_t n) {
    if (n == 0 || n > UINT16_MAX) return -EINVAL;
    return to_errno(HAL_I2C_Mem_Write(&hi2c, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf,
                                      (uint16_t)n, I2C_TIMEOUT_MS));
}

// stop using the peripheral until the superloop has recovered it. interrupts off first:
// an error can leave the event interrupt firing forever (the bring-up's IRQ storm)
static void fail(void) {
    irqs(false);
    in_flight = false;
    need_recover = true;
}

int i2c_read_start(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
    if (need_recover) return -EIO;
    if (in_flight) return -EBUSY;
    if (n == 0 || n > UINT16_MAX) return -EINVAL;
    // HAL spins up to 25 ms waiting for BUSY to clear before it starts. inside the tick that
    // would stall control, so check it here. a finished read clears BUSY within microseconds
    if (ENC_I2C->SR2 & I2C_SR2_BUSY) {
        fail();
        return -EIO;
    }

    read_done = false;
    read_err = false;
    isr_cycles = 0;
    started_at = DWT->CYCCNT;
    // sets START and enables the interrupts, then returns. the event interrupt does the rest
    if (HAL_I2C_Mem_Read_IT(&hi2c, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)n) != HAL_OK) {
        fail();
        return -EIO;
    }
    in_flight = true;
    return 0;
}

int i2c_read_poll(void) {
    if (!in_flight) return need_recover ? -EIO : -EAGAIN;
    if (read_err) {
        fail();
        return -EIO;
    }
    if (read_done) {
        in_flight = false;
        last_timing.xfer_cycles = done_at - started_at;
        last_timing.isr_cycles = isr_cycles;
        return 0;
    }
    // no completion and no error: a stuck bus or an interrupt that never comes
    if (DWT->CYCCNT - started_at > us_to_cycles(I2C_TIMEOUT_US)) {
        fail();
        return -ETIMEDOUT;
    }
    return -EINPROGRESS;
}

static bool sda_high(void) { return (SDA_PORT->IDR & (1u << SDA_PIN)) != 0; }
static void scl(bool high) { SCL_PORT->BSRR = high ? 1u << SCL_PIN : 1u << (SCL_PIN + 16); }
static void sda(bool high) { SDA_PORT->BSRR = high ? 1u << SDA_PIN : 1u << (SDA_PIN + 16); }

int i2c_recover(void) {
    static uint32_t last_attempt_ms;
    static bool attempted;

    if (!need_recover) return 0;
    uint32_t now = HAL_GetTick();
    if (attempted && now - last_attempt_ms < I2C_RECOVER_MS) return -EAGAIN;
    attempted = true;
    last_attempt_ms = now;

    irqs(false);
    __HAL_I2C_DISABLE(&hi2c); // peripheral off so it lets go of the pins

    // bus clear (I2C spec): pins as plain open-drain outputs, released high. a slave that was
    // cut off mid-byte is still holding SDA low, waiting for clocks. give it up to 9
    // (8 data bits + ACK) until it lets go
    scl(true);
    sda(true);
    pin(SCL_PORT, SCL_PIN, GPIO_MODE_OUTPUT_OD);
    pin(SDA_PORT, SDA_PIN, GPIO_MODE_OUTPUT_OD);
    delay_us(5);
    for (int i = 0; i < 9 && !sda_high(); i++) {
        scl(false);
        delay_us(5); // 5 us half period = 100 kHz, slow enough for any slave
        scl(true);
        delay_us(5);
    }
    // START then STOP (SDA falls, then rises, while SCL is high) resets every slave's state
    sda(false);
    delay_us(5);
    sda(true);
    delay_us(5);
    bool released = sda_high();

    int r = setup(); // pins back to AF4, re-init, interrupts back on
    if (r != 0 || !released) {
        irqs(false); // still broken: stay off, try again in I2C_RECOVER_MS
        return -EIO;
    }
    need_recover = false; // last: the tick may start reads again from here
    return 1;
}

i2c_timing_t i2c_last_timing(void) { return last_timing; }

// the vector table jumps here; HAL's handlers advance the transfer one step per interrupt
// and call the callbacks below when it finishes or fails. the cycle count is for i2c_last_timing
void I2C1_EV_IRQHandler(void) {
    uint32_t t = DWT->CYCCNT;
    HAL_I2C_EV_IRQHandler(&hi2c);
    isr_cycles += DWT->CYCCNT - t;
}

void I2C1_ER_IRQHandler(void) {
    uint32_t t = DWT->CYCCNT;
    HAL_I2C_ER_IRQHandler(&hi2c);
    isr_cycles += DWT->CYCCNT - t;
}

// interrupt context: set a flag and return, the tick picks it up
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h) {
    (void)h;
    done_at = DWT->CYCCNT;
    read_done = true;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h) {
    (void)h;
    read_err = true;
}
