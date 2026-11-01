#pragma once

// ---------- motor and supply ----------
#define SUPPLY_V     22.0f
#define POLE_PAIRS   7
#define CLOSEDLOOP_V 3.0f
#define VLIMIT_MIN_V 0.5f
#define ALIGN_V      2.0f
#define PWM_HZ       25000

// ---------- loop rates ----------
#define FOC_HZ      5000
#define CFC_CTRL_HZ 500
#define TICK_TIM    TIM2

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
#define ENC_LOST_N     10

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
#define ENC_ZERO_RAD 2.499f
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

#define ENC_I2C  I2C1
#define SCL_PORT GPIOB
#define SCL_PIN  8
#define SDA_PORT GPIOB
#define SDA_PIN  9
