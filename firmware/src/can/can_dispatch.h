#pragma once
#include "bldc/joint.h"
#include "can/can_proto.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file can_dispatch.h
 * @brief Received frame -> what to do. Pure, no hardware, tested on the PC.
 *
 * Rules (same as the bring-up, plus the E-STOP fault): see can_proto.h.
 */

typedef struct {
    bool from_host;     // addressed to this node: refresh the host timer, count it, record last_cmd
    uint8_t cmd;        // the command number, valid when from_host
    bool has_cmd;       // push cmd_out to the joint
    joint_cmd_t cmd_out;
    uint8_t set_faults; // OR into the node's faults
    bool clear_faults;  // ENABLE: start from a clean slate (the joint clears its own)
    int8_t can_control; // +1 enabled over CAN (host timeout applies), -1 not any more, 0 unchanged
} can_action_t;

can_action_t can_dispatch(const can_frame_t *f, uint8_t node);

/** @brief joint_state_t -> STATE_* on the wire. */
uint8_t can_state_code(joint_state_t s);
