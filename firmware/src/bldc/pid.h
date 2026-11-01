#pragma once

typedef struct {
    float kp, ki, kd;
    float i_limit;
    float out_limit;
    float ramp;
    float integral, prev_err, prev_out;
} pid_ctrl_t;

void pid_reset(pid_ctrl_t *p);
float pid_step(pid_ctrl_t *p, float err, float dt);
