#include "pid.h"
#include <math.h>

static float clampf(float x, float lim) { return fminf(fmaxf(x, -lim), lim); }

void pid_reset(pid_ctrl_t *p) {
    p->integral = 0.0f;
    p->prev_err = 0.0f;
    p->prev_out = 0.0f;
}

float pid_step(pid_ctrl_t *p, float err, float dt) {
    p->integral = clampf(p->integral + p->ki * dt * 0.5f * (err + p->prev_err), p->i_limit);
    float d = p->kd * (err - p->prev_err) / dt;
    float out = clampf(p->kp * err + p->integral + d, p->out_limit);
    if (p->ramp > 0.0f) out = p->prev_out + clampf(out - p->prev_out, p->ramp * dt);
    p->prev_err = err;
    p->prev_out = out;
    return out;
}
