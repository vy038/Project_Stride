// Stride CAN protocol v0. The single source of truth for frame layout; pc/can_host.py mirrors it.
// C port of the bring-up's stride_can.h, plus FAULT_ESTOP.
//
// Bus: classic CAN 2.0A, 11-bit IDs, 500 kbit/s.
// ID  = (node_id << 5) | cmd        node_id 1..63, cmd 0..31  (same scheme as ODrive CANSimple)
// ID 0x000 = E-STOP, all nodes. Lowest ID wins arbitration, so it beats everything else on the bus.
// Multi-byte fields are little-endian. Floats are IEEE-754 float32.
//
// Host -> leg
//   cmd  name          DLC  payload
//   0x00 HEARTBEAT     0    -                      keeps a CAN-enabled leg alive (any frame does)
//   0x01 DISABLE       0    -                      driver off
//   0x02 ENABLE        0    -                      calibrate if needed, then closed-loop hold
//   0x03 SET_TARGET    4    f32 angle [rad]        closed-loop position target
//   0x04 SET_VLIMIT    4    f32 volts              clamped to 0.5..6 V (the node clamps to CLOSEDLOOP_V)
//
// Leg -> host
//   0x10 STATUS        8    u8 state, u8 faults, u8 last_cmd, u8 rx_count,
//                           u16 i2c_err, u16 ms_since_host            (10 Hz)
//   0x11 FEEDBACK      8    f32 angle [rad], f32 velocity [rad/s]     (50 Hz)
//
// Safety: if the leg was enabled over CAN and hears no frame from the host for
// HOST_TIMEOUT_MS (config.h), it disables itself and sets FAULT_HOST_TIMEOUT.
#pragma once
#include <stdint.h>
#include <string.h>

#define CAN_BITRATE_BPS 500000
#define CAN_ID_ESTOP    0x000
#define CAN_NODE_KNEE   1
#define CAN_NODE_HIP    2

enum {
    CMD_HEARTBEAT = 0x00,
    CMD_DISABLE = 0x01,
    CMD_ENABLE = 0x02,
    CMD_SET_TARGET = 0x03,
    CMD_SET_VLIMIT = 0x04,
    MSG_STATUS = 0x10,
    MSG_FEEDBACK = 0x11,
};

enum { STATE_IDLE = 0, STATE_OPEN_LOOP = 1, STATE_CLOSED_LOOP = 2, STATE_ENCODER = 3 };

enum {
    FAULT_ENCODER_LOST = 1 << 0,
    FAULT_DEADMAN = 1 << 1, // now: last reset was the IWDG
    FAULT_HOST_TIMEOUT = 1 << 2,
    FAULT_CALIB_FAILED = 1 << 3,
    FAULT_BAD_FRAME = 1 << 4, // wrong DLC or non-finite float
    FAULT_ESTOP = 1 << 5,     // ID 0x000 seen. Tells an E-STOP apart from a DISABLE
};

typedef struct {
    uint16_t id; // 11-bit standard ID
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

static inline uint16_t can_make_id(uint8_t node, uint8_t cmd) { return (uint16_t)((node << 5) | (cmd & 0x1F)); }
static inline uint8_t can_id_node(uint16_t id) { return (uint8_t)(id >> 5); }
static inline uint8_t can_id_cmd(uint16_t id) { return id & 0x1F; }

static inline void can_put_f32(uint8_t *p, float v) { memcpy(p, &v, 4); } // STM32 is little-endian
static inline float can_get_f32(const uint8_t *p) {
    float v;
    memcpy(&v, p, 4);
    return v;
}
static inline void can_put_u16(uint8_t *p, uint16_t v) {
    p[0] = v & 0xFF;
    p[1] = v >> 8;
}
