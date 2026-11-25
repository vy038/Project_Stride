/**
 * @file can_dispatch.c
 * @brief Frame -> action. See can_dispatch.h.
 */

#include "can/can_dispatch.h"
#include <math.h>

can_action_t can_dispatch(const can_frame_t *f, uint8_t node) {
    can_action_t a = {0};
    if (f->id == CAN_ID_ESTOP) { // any node's E-STOP, any DLC. not a host frame: it can't keep a leg alive
        a.has_cmd = true;
        a.cmd_out = (joint_cmd_t){JCMD_DISABLE, 0};
        a.set_faults = FAULT_ESTOP;
        a.can_control = -1;
        return a;
    }
    if (can_id_node(f->id) != node) return a;

    a.from_host = true;
    a.cmd = can_id_cmd(f->id);
    switch (a.cmd) {
    case CMD_HEARTBEAT: break;
    case CMD_DISABLE:
        a.has_cmd = true;
        a.cmd_out = (joint_cmd_t){JCMD_DISABLE, 0};
        a.can_control = -1;
        break;
    case CMD_ENABLE:
        a.has_cmd = true;
        a.cmd_out = (joint_cmd_t){JCMD_ENABLE, 0};
        a.clear_faults = true;
        a.can_control = 1;
        break;
    case CMD_SET_TARGET:
    case CMD_SET_VLIMIT: {
        // a NaN target would poison every step after it, so a bad payload never reaches the joint
        float v = f->dlc == 4 ? can_get_f32(f->data) : NAN;
        if (!isfinite(v)) {
            a.set_faults = FAULT_BAD_FRAME;
            break;
        }
        a.has_cmd = true;
        a.cmd_out = (joint_cmd_t){a.cmd == CMD_SET_TARGET ? JCMD_TARGET : JCMD_VLIMIT, v};
        break;
    }
    default: a.set_faults = FAULT_BAD_FRAME; break;
    }
    return a;
}

uint8_t can_state_code(joint_state_t s) {
    switch (s) {
    case JOINT_IDLE: return STATE_IDLE;
    case JOINT_CALIBRATING: return STATE_CALIBRATING;
    case JOINT_CLOSED_PID: return STATE_CLOSED_LOOP;
    case JOINT_CLOSED_CFC: return STATE_CLOSED_CFC;
    case JOINT_FAULT: return STATE_FAULT;
    }
    return STATE_FAULT;
}
