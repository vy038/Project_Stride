#pragma once
#include "bldc/joint.h"
#include "can/can_proto.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file can_node.h
 * @brief This joint's CAN node: owns the MCP2515, turns frames into joint commands, reports back.
 *
 * Superloop only, except can_node_on_int (EXTI). The INT pin only says "something is waiting";
 * all SPI work happens in can_node_poll, so the superloop never gets interrupted mid-transaction.
 */

typedef struct {
    uint32_t rx;          // frames taken off the chip
    uint32_t rx_dropped;  // ring full, frame lost in software
    uint32_t hw_overflow; // chip RX buffers both full, frame lost in hardware (polled too slowly)
    uint32_t tx_fail;     // can_node_send refused or SPI failed
    uint8_t tec, rec, eflg; // last read error counters and flags
    bool int_active;      // false: INT stuck or missing, polling every pass
    can_frame_t last;     // most recent frame popped (for bench tests)
} can_node_stats_t;

/** What the node reports, gathered by the caller from the control telemetry each pass. */
typedef struct {
    float angle, vel;     // rad, rad/s. angle NaN until the first encoder reading
    joint_state_t state;
    uint8_t joint_faults; // FAULT_* bits the joint set itself
    int8_t cmd_err;       // result of the last joint command, 0 or -errno
    uint32_t i2c_err;     // failed encoder reads since boot
} can_node_telem_t;

/**
 * @param mode MCP2515_MODE_NORMAL, or MCP2515_MODE_LOOPBACK for a driver bench test: no protocol,
 *             no STATUS/FEEDBACK, frames are only received (they'd come straight back otherwise)
 * @param boot_faults reported until the next ENABLE, e.g. FAULT_DEADMAN after a watchdog reset
 * @return 0, or mcp2515_init's error. On error the node stays off and poll does nothing.
 */
int can_node_init(uint8_t mode, uint8_t boot_faults);

/** @brief EXTI9_5 handler hook: only sets a flag. */
void can_node_on_int(void);

/**
 * @brief Every superloop pass: drain the chip, dispatch frames, host timeout, STATUS and FEEDBACK.
 * @param push queues a command for the joint's owner; false if its queue is full
 */
void can_node_poll(uint32_t now_ms, const can_node_telem_t *t, bool (*push)(const joint_cmd_t *));

/** @return 0 or mcp2515_send's error. */
int can_node_send(const can_frame_t *f);

can_node_stats_t can_node_stats(void);
