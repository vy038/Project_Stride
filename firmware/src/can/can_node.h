#pragma once
#include "can/can_proto.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file can_node.h
 * @brief This joint's CAN node: owns the MCP2515, receives into a ring, (later) dispatches commands.
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

/**
 * @param mode MCP2515_MODE_NORMAL, or MCP2515_MODE_LOOPBACK for a bench test with no bus
 * @return 0, or mcp2515_init's error. On error the node stays off and poll does nothing.
 */
int can_node_init(uint8_t mode);

/** @brief EXTI9_5 handler hook: only sets a flag. */
void can_node_on_int(void);

/** @brief Every superloop pass: drain the chip, handle frames, refresh the error counters. */
void can_node_poll(uint32_t now_ms);

/** @return 0 or mcp2515_send's error. */
int can_node_send(const can_frame_t *f);

can_node_stats_t can_node_stats(void);
