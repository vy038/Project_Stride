// Stride CAN protocol v1. The single source of truth for frame layout; pc/can_host.py mirrors it.
// Readable version with examples: V1_NOTES.md section 5. Change this file and can_host.py together.
//
// Bus: classic CAN 2.0A, 11-bit IDs, 500 kbit/s. Data frames only (extended/remote ignored).
// ID  = (node_id << 5) | cmd        node_id 1..63, cmd 0..31  (same scheme as ODrive CANSimple)
// ID 0x000 = E-STOP, all nodes. Lowest ID wins arbitration, so it beats everything else on the bus.
// Multi-byte fields are little-endian. Floats are IEEE-754 float32.
//
// Host -> leg
//   cmd  name          DLC  payload
//   0x00 HEARTBEAT     any  -                      keeps a CAN-enabled leg alive (any frame for it does)
//   0x01 DISABLE       any  -                      driver off
//   0x02 ENABLE        any  -                      clear faults, calibrate if needed, then hold where it is
//   0x03 SET_TARGET    4    f32 angle [rad]        clamped to +-TRAVEL_RAD
//   0x04 SET_VLIMIT    4    f32 volts              clamped to VLIMIT_MIN_V..CLOSEDLOOP_V (0.5..3 V)
//   SET_* with DLC != 4 or a non-finite float, or an unknown cmd: ignored, FAULT_BAD_FRAME set.
//
// Leg -> host
//   0x10 STATUS        8    u8 state, u8 faults, u8 last_cmd, i8 last_err,
//                           u16 i2c_err, u16 ms_since_host            (10 Hz)
//   0x11 FEEDBACK      8    f32 angle [rad], f32 velocity [rad/s]     (50 Hz; NaN angle = no reading yet)
//
// Safety: if the leg was enabled over CAN and hears no frame from the host for
// HOST_TIMEOUT_MS (config.h), it disables itself and sets FAULT_HOST_TIMEOUT.
// E-STOP disables and sets FAULT_ESTOP; only an explicit ENABLE runs the leg again.
//
// v0 -> v1: STATUS byte 3 was rx_count, now the last command's result (0 or -errno: -34 outside
// travel, -19 encoder failed its boot check). States 4..6 and FAULT_ESTOP added.
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

// 1 and 3 were bring-up modes, kept so the numbers mean the same thing in old logs
enum {
    STATE_IDLE = 0,
    STATE_OPEN_LOOP = 1,
    STATE_CLOSED_LOOP = 2, // PID
    STATE_ENCODER = 3,
    STATE_CALIBRATING = 4,
    STATE_CLOSED_CFC = 5,
    STATE_FAULT = 6,
};

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
