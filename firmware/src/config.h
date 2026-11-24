#pragma once

// ---------- motor and supply ----------
#define SUPPLY_V     22.0f
#define POLE_PAIRS   7
#define CLOSEDLOOP_V 3.0f
#define VLIMIT_MIN_V 0.5f
#define ALIGN_V      2.0f
#define CALIB_SETTLE_MS 700   // per hold, same as SimpleFOC's alignSensor
#define CALIB_MOVE_MS   250
#define CALIB_STEP_E    1.5707963f // nudge, electrical rad: 13 deg of shaft at 7 pole pairs
#define PWM_HZ       25000

// ---------- loop rates ----------
#define FOC_HZ      5000
#define CFC_CTRL_HZ 500
#define TICK_TIM    TIM2
#define TIM_CLK_HZ  84000000 // APB1 timer clock, 2 x 42 MHz
#define TICK_CNT_HZ 1000000  // tick timer count rate after the prescaler
#define PWM_ARR     (TIM_CLK_HZ / (2 * PWM_HZ)) // center-aligned: period is 2 x ARR counts, 1680

_Static_assert(TIM_CLK_HZ % (2 * PWM_HZ) == 0, "PWM period must be a whole number of counts");
_Static_assert(PWM_ARR <= 0xFFFF, "TIM3 is 16-bit");

_Static_assert(FOC_HZ % CFC_CTRL_HZ == 0, "PendSV fires every FOC_HZ / CFC_CTRL_HZ ticks, must be whole");
_Static_assert(TICK_CNT_HZ % FOC_HZ == 0, "tick period must be a whole number of counts");

// ---------- joint behaviour ----------
#define TRAVEL_RAD  0.5f
#define CFC_MAX_ERR 0.3f

// ---------- PID, angle cascade (bring-up values, PORTING.md section 3) ----------
#define PID_ANGLE_P  20.0f
#define VEL_LIMIT    10.0f
#define PID_VEL_P    0.2f
#define PID_VEL_I    2.0f
#define PID_VEL_RAMP 1000.0f

// ---------- encoder ----------
#define I2C_HZ         400000
#define I2C_TIMEOUT_US 2000
// blind time before encoder-lost. at the 10 rad/s limit a 10 ms stale angle is 0.1 rad, 0.7 rad
// (40 deg) electrical, so torque keeps the right sign. long enough to ride through one recovery
#define ENC_LOST_MS    10
#define ENC_LOST_N     (ENC_LOST_MS * FOC_HZ / 1000) // consecutive failed tick reads, 50 at 5 kHz
#define I2C_RECOVER_MS 50 // min time between recovery attempts, so a dead sensor can't spin the loop

_Static_assert(ENC_LOST_MS * FOC_HZ % 1000 == 0, "ENC_LOST_MS must be a whole number of ticks");

// velocity estimate, runs at CFC_CTRL_HZ. lag ~ N*dt/2 + Tf = 5 + 5 ms, quantum 1.534 mrad / 10 ms
// = 0.15 rad/s. starting point, check against the velocity loop crossover (GUIDE.md). retrain if changed
#define VEL_WINDOW_N 5
#define VEL_TF_S     0.005f
#define VEL_DT       (1.0f / CFC_CTRL_HZ)

// ---------- safety ----------
#define WDG_TIMEOUT_MS  150
#define HOST_TIMEOUT_MS 500

// ---------- CAN, MCP2515 on SPI2 ----------
#define MCP2515_XTAL_HZ 8000000
#define SPI_HZ          4000000
#define STATUS_MS       100
#define FEEDBACK_MS     20

#define CAN_CS_PORT   GPIOB
#define CAN_CS_PIN    12
#define CAN_SCK_PORT  GPIOB
#define CAN_SCK_PIN   13
#define CAN_MISO_PORT GPIOB
#define CAN_MISO_PIN  14
#define CAN_MOSI_PORT GPIOB
#define CAN_MOSI_PIN  15
#define CAN_INT_PORT  GPIOC
#define CAN_INT_PIN   7

// ---------- this node: knee ----------
#define CAN_NODE     CAN_NODE_KNEE
#define ENC_ZERO_RAD 5.470f // raw angle at the zero pose. bring-up re-measured this after the encoder swap
#define ENC_DIR      (-1.0f)

#define PWM_TIM    TIM3
#define PWM_A_PORT GPIOA
#define PWM_A_PIN  6
#define PWM_B_PORT GPIOA
#define PWM_B_PIN  7
#define PWM_C_PORT GPIOB
#define PWM_C_PIN  0
#define EN_PORT    GPIOB
#define EN_PIN     5
#define OTHER_EN_PIN 4 // hip driver EN, same port. never driven high by this build
#define EN_ALL_RESET ((1u << (EN_PIN + 16)) | (1u << (OTHER_EN_PIN + 16))) // BSRR value clearing both

#define ENC_I2C  I2C1
#define SCL_PORT GPIOB
#define SCL_PIN  8
#define SDA_PORT GPIOB
#define SDA_PIN  9
