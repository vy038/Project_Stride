#include "can/can_node.h"
#include "drivers/mcp2515.h"
#include "ring.h"
#include <stdbool.h>

static can_frame_t rx_buf[32];
static ring_t rx;
static uint32_t rx_dropped;
static bool int_active;
static volatile bool int_flag;

static void drain(void) {
    can_frame_t f;
    while (mcp2515_recv(&f) == 0)
        if (!ring_push(&rx, &f)) rx_dropped++;
    // TODO: clear MCP2515 error flags too, or INT stays asserted
}

static void service(void) {
    int_flag = false;
    for (int n = 0; n < 4; n++) {
        drain();
        if (!mcp2515_int_low()) return;
    }
    int_flag = true;
}

void can_node_init(void) {
    ring_init(&rx, rx_buf, 32, sizeof rx_buf[0]);
    int_active = mcp2515_int_init() == 0;
}

void can_node_on_int(void) { int_flag = true; }

void can_node_poll(joint_t *j, uint32_t now_ms, float angle, float vel) {
    if (!int_active)
        drain();
    else if (int_flag)
        service();
    (void)j;
    (void)now_ms;
    (void)angle;
    (void)vel;
    can_frame_t f;
    while (ring_pop(&rx, &f)) {
        // TODO, same behaviour as the bring-up (verified on the bench 2026-10-26):
        //   ID 0x000: joint_disable, FAULT_ESTOP, leave CAN control. Stays off under heartbeat.
        //   other nodes: ignore. Own node: any frame refreshes the host time.
        //   ENABLE: joint_enable, enter CAN control. DISABLE: joint_disable, leave CAN control.
        //   SET_TARGET / SET_VLIMIT: DLC 4 and finite, else FAULT_BAD_FRAME. Unknown cmd: FAULT_BAD_FRAME.
        //   joint_* calls run with BASEPRI masking PendSV: joint_t belongs to the 500 Hz loop.
    }
    // TODO: CAN control and no host frame for HOST_TIMEOUT_MS: joint_disable, FAULT_HOST_TIMEOUT.
    // TODO: FEEDBACK every FEEDBACK_MS (sensor angle, vel), STATUS every STATUS_MS.
}
