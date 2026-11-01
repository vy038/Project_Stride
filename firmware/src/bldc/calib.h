#pragma once

typedef enum { CALIB_DONE = 0, CALIB_RUNNING = 1 } calib_status_t;

typedef struct {
    // TODO: procedure state
    float dir;
    float zero_elec;
} calib_t;

void calib_start(calib_t *c);

int calib_step(calib_t *c, float angle, float *uq, float *theta_e);
