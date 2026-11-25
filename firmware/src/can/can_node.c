/**
 * @file can_node.c
 * @brief CAN node: receive path, error housekeeping. See can_node.h.
 */

#include "can/can_node.h"
#include "can/can_dispatch.h"
#include "config.h"
#include "drivers/mcp2515.h"
#include "ring.h"
#include <errno.h>
#include <string.h>

#define ERR_POLL_MS 100

static can_frame_t rx_buf[32];
static ring_t rx;
static can_node_stats_t st;
static bool up, protocol;
static volatile bool int_flag;

// protocol state, superloop only
static uint8_t node_faults, last_cmd = 0xFF;
static bool can_control, heard_host;
static uint32_t last_host_ms;

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

int can_node_init(uint8_t mode, uint8_t boot_faults) {
    ring_init(&rx, rx_buf, 32, sizeof rx_buf[0]);
    node_faults = boot_faults;
    protocol = mode == MCP2515_MODE_NORMAL;
    int r = mcp2515_init(mode);
    if (r) return r;
    up = true;
    st.int_active = mcp2515_int_init() == 0;
    return 0;
}

void can_node_on_int(void) { int_flag = true; }

static void send(uint8_t cmd, const uint8_t d[8]) {
    can_frame_t f = {.id = can_make_id(CAN_NODE, cmd), .dlc = 8};
    memcpy(f.data, d, 8);
    can_node_send(&f);
}

static uint16_t sat16(uint32_t v) { return v > 0xFFFF ? 0xFFFF : (uint16_t)v; }

void can_node_poll(uint32_t now_ms, const can_node_telem_t *t, bool (*push)(const joint_cmd_t *)) {
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
        if (!protocol) continue;
        can_action_t a = can_dispatch(&f, CAN_NODE);
        if (a.from_host) {
            last_host_ms = now_ms;
            heard_host = true;
            last_cmd = a.cmd;
        }
        if (a.clear_faults) node_faults = 0;
        node_faults |= a.set_faults;
        if (a.can_control) can_control = a.can_control > 0;
        // the queue only fills if PendSV isn't running, and then the joint is off anyway
        if (a.has_cmd) push(&a.cmd_out);
    }
    if (!protocol) return;

    // dead-man on the CAN side: a CAN-enabled leg that stops hearing the host turns itself off.
    // only while it's actually driving, so a refused ENABLE (still idle) can't time out
    bool driving = t->state == JOINT_CALIBRATING || t->state == JOINT_CLOSED_PID || t->state == JOINT_CLOSED_CFC;
    if (can_control && driving && now_ms - last_host_ms > HOST_TIMEOUT_MS) {
        push(&(joint_cmd_t){JCMD_DISABLE, 0});
        node_faults |= FAULT_HOST_TIMEOUT;
        can_control = false;
    }

    static uint32_t last_fb_ms, last_status_ms;
    if (now_ms - last_fb_ms >= FEEDBACK_MS) {
        last_fb_ms = now_ms;
        uint8_t d[8];
        can_put_f32(d, t->angle);
        can_put_f32(d + 4, t->vel);
        send(MSG_FEEDBACK, d);
    }
    if (now_ms - last_status_ms >= STATUS_MS) {
        last_status_ms = now_ms;
        uint8_t d[8] = {can_state_code(t->state), (uint8_t)(t->joint_faults | node_faults), last_cmd,
                        (uint8_t)t->cmd_err};
        can_put_u16(d + 4, sat16(t->i2c_err));
        can_put_u16(d + 6, heard_host ? sat16(now_ms - last_host_ms) : 0xFFFF);
        send(MSG_STATUS, d);
    }
}

int can_node_send(const can_frame_t *f) {
    int r = up ? mcp2515_send(f) : -ENODEV;
    if (r) st.tx_fail++;
    return r;
}

can_node_stats_t can_node_stats(void) { return st; }
