/**
 * @file calib.c
 * @brief Align-and-nudge electrical calibration. See calib.h.
 */

#include "bldc/calib.h"
#include "config.h"
#include <errno.h>
#include <math.h>

#define PI       3.14159265f
#define HOLD_E   (1.5f * PI) // field at hold_e + pi/2 = 2pi, so the d-axis settles at electrical 0
#define SETTLE_N (CALIB_SETTLE_MS * CFC_CTRL_HZ / 1000)
#define MOVE_N   (CALIB_MOVE_MS * CFC_CTRL_HZ / 1000)

enum { HOLD, MOVE, HOLD2 };

void calib_start(calib_t *c) { *c = (calib_t){.phase = HOLD}; }

int calib_step(calib_t *c, float angle, float *uq, float *theta_e) {
    *uq = ALIGN_V;
    c->n++;
    switch (c->phase) {
    case HOLD:
        *theta_e = HOLD_E;
        if (c->n < SETTLE_N) return CALIB_RUNNING;
        c->a0 = angle;
        break;
    case MOVE: // ramp, a jump would kick the rotor
        *theta_e = HOLD_E + CALIB_STEP_E * c->n / MOVE_N;
        if (c->n < MOVE_N) return CALIB_RUNNING;
        break;
    default:
        *theta_e = HOLD_E + CALIB_STEP_E;
        if (c->n < SETTLE_N) return CALIB_RUNNING;
        *uq = 0.0f;
        float moved = angle - c->a0;
        if (!(fabsf(fabsf(moved) * POLE_PAIRS - CALIB_STEP_E) < 0.5f * CALIB_STEP_E)) return -EIO;
        c->dir = moved > 0 ? 1.0f : -1.0f;
        // d-axis was at electrical 0 in the first hold: dir * pp * a0 - zero = 0
        c->zero_elec = fmodf(c->dir * POLE_PAIRS * c->a0, 2 * PI);
        if (c->zero_elec < 0) c->zero_elec += 2 * PI;
        return CALIB_DONE;
    }
    c->phase++;
    c->n = 0;
    return CALIB_RUNNING;
}

float calib_theta_e(const calib_t *c, float angle) {
    float th = c->dir * POLE_PAIRS * angle - c->zero_elec;
    return c->dir > 0 ? th : th + PI;
}
