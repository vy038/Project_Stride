#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

/**
 * @file vel.h
 * @brief Joint velocity from the quantised angle: windowed difference, then a first-order low-pass.
 *
 * Pure logic, fixed dt (VEL_DT), so it runs the same on the PC. Lag is about
 * VEL_WINDOW_N * VEL_DT / 2 + VEL_TF_S. The CfC trains on this signal, so changing either
 * constant means retraining. How they were picked: GUIDE.md, velocity estimate.
 */

typedef struct {
    float hist[VEL_WINDOW_N]; // last N angles, ring
    float y;                  // filtered velocity
    uint8_t i;                // oldest entry, overwritten next
    bool primed;
} vel_t;

/**
 * @brief One step at VEL_DT. Zero-init the state; the first call fills the window and returns 0.
 * @param angle joint angle, rad, finite. No unwrapping: the travel stays clear of the 0/2pi jump.
 * @return velocity, rad/s
 */
float vel_step(vel_t *v, float angle);
