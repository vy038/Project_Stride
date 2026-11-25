/**
 * @file can_node.c
 * @brief CAN node: receive path, error housekeeping. See can_node.h.
 */

#include "can/can_node.h"
#include "drivers/mcp2515.h"
#include "ring.h"
#include <errno.h>

#define ERR_POLL_MS 100

static can_frame_t rx_buf[32];
static ring_t rx;
static can_node_stats_t st;
static bool up;
static volatile bool int_flag;

static void drain(void) {
    can_frame_t f;
    while (mcp2515_recv(&f) == 0) {
        st.rx++;
        if (!ring_push(&rx, &f)) st.rx_dropped++;
    }
}

// INT is level: it stays low while anything is pending, but the EXTI only fires on the falling edge.
// so drain, then look at the pin again. capped, so a stuck line can't hold the superloop (and the kick)
static void service(void) {
    int_flag = false;
    for (int n = 0; n < 4; n++) {
        drain();
        if (!mcp2515_int_low()) return;
    }
    int_flag = true; // still low: try again next pass
}

int can_node_init(uint8_t mode) {
    ring_init(&rx, rx_buf, 32, sizeof rx_buf[0]);
    int r = mcp2515_init(mode);
    if (r) return r;
    up = true;
    st.int_active = mcp2515_int_init() == 0;
    return 0;
}

void can_node_on_int(void) { int_flag = true; }

void can_node_poll(uint32_t now_ms) {
    if (!up) return;
    if (!st.int_active)
        drain();
    else if (int_flag)
        service();

    static uint32_t last_err_ms;
    if (now_ms - last_err_ms >= ERR_POLL_MS) {
        last_err_ms = now_ms;
        int e = mcp2515_errors(&st.tec, &st.rec);
        if (e >= 0) {
            st.eflg = (uint8_t)e;
            if (e & (MCP2515_EFLG_RX0OVR | MCP2515_EFLG_RX1OVR)) st.hw_overflow++;
        }
    }

    can_frame_t f;
    while (ring_pop(&rx, &f)) {
        st.last = f;
        // TODO, same behaviour as the bring-up (verified on the bench 2026-10-26):
        //   ID 0x000: disable, FAULT_ESTOP, leave CAN control. Stays off under heartbeat.
        //   other nodes: ignore. Own node: any frame refreshes the host time.
        //   ENABLE / DISABLE / SET_TARGET / SET_VLIMIT: push into main's joint command ring (PendSV
        //   owns joint_t). SET_*: DLC 4 and finite, else FAULT_BAD_FRAME. Unknown cmd: FAULT_BAD_FRAME.
    }
    // TODO: CAN control and no host frame for HOST_TIMEOUT_MS: disable, FAULT_HOST_TIMEOUT.
    // TODO: FEEDBACK every FEEDBACK_MS (sensor angle, vel), STATUS every STATUS_MS.
}

int can_node_send(const can_frame_t *f) {
    int r = up ? mcp2515_send(f) : -ENODEV;
    if (r) st.tx_fail++;
    return r;
}

can_node_stats_t can_node_stats(void) { return st; }
