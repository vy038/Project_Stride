#include "bldc/joint.h"
#include "can/can_proto.h"
#include "cfc_ctrl.h"
#include "config.h"
#include <errno.h>
#include <math.h>

#define CTRL_DT (1.0f / CFC_CTRL_HZ)

static float clampf(float x, float lim) { return fminf(fmaxf(x, -lim), lim); }

void joint_init(joint_t *j, bool cfc_ok, bool enc_ok) {
    *j = (joint_t){
        .state = JOINT_IDLE,
        .cfc_ok = cfc_ok,
        .enc_ok = enc_ok,
        .angle = NAN,
        .uq_limit = CLOSEDLOOP_V,
        .pid_angle = {.kp = PID_ANGLE_P, .out_limit = VEL_LIMIT},
        .pid_vel = {.kp = PID_VEL_P, .ki = PID_VEL_I, .i_limit = PID_VEL_LIMIT, .out_limit = PID_VEL_LIMIT,
                    .ramp = PID_VEL_RAMP},
    };
}

// voltage that holds the leg against gravity at this angle, so the integral doesn't have to find it
static float grav_ff(float angle) { return GRAV_K * sinf(angle + GRAV_PHI); }

static void enter_pid(joint_t *j) {
    pid_reset(&j->pid_angle);
    pid_reset(&j->pid_vel);
    // the PID's share of the current uq, so pid + feedforward continues from it with no jump
    j->pid_vel.integral = clampf(j->uq - grav_ff(j->angle), j->pid_vel.i_limit);
    j->pid_vel.prev_out = j->pid_vel.integral;
    j->state = JOINT_CLOSED_PID;
}

void joint_step(joint_t *j, float angle, float vel) {
    j->angle = angle;
    j->vel_sp = 0.0f;
    if (j->state == JOINT_IDLE || j->state == JOINT_FAULT) {
        j->uq = 0.0f;
        return;
    }
    if (!isfinite(angle) || !isfinite(vel)) {
        joint_fault(j, FAULT_ENCODER_LOST);
        return;
    }

    if (j->state == JOINT_CALIBRATING) {
        int r = calib_step(&j->cal, angle, &j->uq, &j->theta_e);
        if (r < 0) {
            joint_fault(j, FAULT_CALIB_FAILED);
        } else if (r == CALIB_DONE) {
            j->calibrated = true;
            j->uq = grav_ff(angle); // start already holding gravity, the integral starts at 0
            j->target = clampf(angle, TRAVEL_RAD);
            enter_pid(j);
        }
        j->uq = clampf(j->uq, j->uq_limit);
        return;
    }

    float err = j->target - angle;
    if (j->state == JOINT_CLOSED_CFC) {
        if (fabsf(err) <= CFC_MAX_ERR) {
            float u = cfc_ctrl_step(err, vel, angle);
            if (isfinite(u)) {
                j->uq = clampf(u, j->uq_limit);
                return;
            }
        }
        j->cfc_fallbacks++;
        enter_pid(j);
    }

    j->vel_sp = pid_step(&j->pid_angle, err, CTRL_DT);
    j->uq = clampf(pid_step(&j->pid_vel, j->vel_sp - vel, CTRL_DT) + grav_ff(angle), j->uq_limit);
}

int joint_enable(joint_t *j) {
    if (j->state != JOINT_IDLE && j->state != JOINT_FAULT) return 0;
    if (!j->enc_ok) return -ENODEV;
    if (!(fabsf(j->angle) <= TRAVEL_RAD)) return -ERANGE;
    j->faults = 0;
    j->uq = 0.0f;
    j->target = j->angle;
    if (j->calibrated) {
        j->uq = grav_ff(j->angle); // same as after calibration
        enter_pid(j);
    } else {
        calib_start(&j->cal);
        j->state = JOINT_CALIBRATING;
    }
    return 0;
}

void joint_disable(joint_t *j) {
    j->state = JOINT_IDLE;
    j->uq = 0.0f;
}

void joint_fault(joint_t *j, uint8_t fault_bits) {
    j->faults |= fault_bits;
    j->state = JOINT_FAULT;
    j->uq = 0.0f;
}

int joint_set_target(joint_t *j, float rad) {
    if (!isfinite(rad)) return -EINVAL;
    j->target = clampf(rad, TRAVEL_RAD);
    return 0;
}

int joint_set_vlimit(joint_t *j, float volts) {
    if (!isfinite(volts)) return -EINVAL;
    j->uq_limit = fminf(fmaxf(volts, VLIMIT_MIN_V), CLOSEDLOOP_V);
    j->pid_vel.i_limit = fminf(j->uq_limit, PID_VEL_LIMIT);
    j->pid_vel.out_limit = j->pid_vel.i_limit;
    return 0;
}

int joint_apply(joint_t *j, const joint_cmd_t *c) {
    switch (c->op) {
    case JCMD_ENABLE: return joint_enable(j);
    case JCMD_DISABLE: joint_disable(j); return 0;
    case JCMD_TARGET: return joint_set_target(j, c->arg);
    case JCMD_VLIMIT: return joint_set_vlimit(j, c->arg);
    case JCMD_CFC: return joint_set_cfc(j, c->arg != 0.0f);
    }
    return -EINVAL;
}

int joint_set_cfc(joint_t *j, bool on) {
    if (!on) {
        if (j->state == JOINT_CLOSED_CFC) enter_pid(j);
        return 0;
    }
    if (j->state == JOINT_CLOSED_CFC) return 0;
    if (!j->cfc_ok) return -ENOTSUP;
    if (j->state != JOINT_CLOSED_PID) return -EPERM;
    cfc_ctrl_reset();
    j->state = JOINT_CLOSED_CFC;
    return 0;
}
