#pragma once
#include <stdint.h>

/**
 * @file calib.h
 * @brief Electrical calibration: where the rotor's magnets sit relative to the joint angle.
 *
 * Align then nudge, never a full sweep (the sweep swings the leg ~77 deg):
 *   1. hold the field at 3pi/2 at ALIGN_V for CALIB_SETTLE_MS, the rotor's d-axis settles at electrical 0.
 *      the joint angle there gives zero_elec (same hold angle as SimpleFOC, so the numbers compare)
 *   2. ramp the field CALIB_STEP_E forward over CALIB_MOVE_MS, hold again
 *   3. the sign of the move gives dir; its size must be about CALIB_STEP_E / POLE_PAIRS
 * Worst-case motion: the snap in 1 (up to half an electrical turn, 26 deg) plus the nudge (13 deg).
 *
 * Non-blocking: calib_step is called at CFC_CTRL_HZ and returns every time. Pure logic, tested on the PC.
 */

typedef enum { CALIB_DONE = 0, CALIB_RUNNING = 1 } calib_status_t;

typedef struct {
    uint8_t phase;
    uint32_t n;      // calls spent in this phase
    float a0;        // joint angle at the end of the first hold
    float dir;       // +1 if advancing the field raises the joint angle
    float zero_elec; // rotor electrical angle is dir * POLE_PAIRS * angle - zero_elec, rad, 0..2pi
} calib_t;

void calib_start(calib_t *c);

/**
 * @brief One step at CFC_CTRL_HZ.
 * @param angle joint angle, rad
 * @param uq, theta_e out: what to drive this step
 * @return CALIB_RUNNING, CALIB_DONE, or -EIO if the rotor didn't move the expected amount
 *         (stuck, a dead phase, wrong POLE_PAIRS). uq is 0 once it returns anything but RUNNING.
 */
int calib_step(calib_t *c, float angle, float *uq, float *theta_e);

/**
 * @brief Commutation angle for foc_svpwm after a successful calibration.
 *
 * Flips the field by pi when dir is -1, so +uq always pushes the joint angle up whatever the
 * phase wiring. The PID and CfC rely on that sign.
 */
float calib_theta_e(const calib_t *c, float angle);
