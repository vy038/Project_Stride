#pragma once
#include "bldc/calib.h"
#include "bldc/pid.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum { JOINT_IDLE, JOINT_CALIBRATING, JOINT_CLOSED_PID, JOINT_CLOSED_CFC, JOINT_FAULT } joint_state_t;

typedef struct {
    joint_state_t state;
    uint8_t faults;
    bool cfc_ok;
    bool enc_ok; // boot check passed. never enable without it
    bool calibrated;
    float angle;
    float target;
    float vel_sp; // angle loop output, rad/s. 0 outside the PID
    float uq;
    float theta_e;
    float uq_limit;
    uint32_t cfc_fallbacks;
    calib_t cal;
    pid_ctrl_t pid_angle;
    pid_ctrl_t pid_vel;
} joint_t;

void joint_init(joint_t *j, bool cfc_ok, bool enc_ok);

void joint_step(joint_t *j, float angle, float vel);

/** @return 0, -ENODEV boot encoder check failed, -ERANGE outside TRAVEL_RAD or no angle yet */
int joint_enable(joint_t *j);
void joint_disable(joint_t *j);
void joint_fault(joint_t *j, uint8_t fault_bits);

int joint_set_target(joint_t *j, float rad);
int joint_set_vlimit(joint_t *j, float volts);
int joint_set_cfc(joint_t *j, bool on);
