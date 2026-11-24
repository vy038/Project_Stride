/**
 * @file vel.c
 * @brief Windowed difference plus first-order low-pass. See vel.h.
 */

#include "bldc/vel.h"

#define ALPHA (VEL_DT / (VEL_TF_S + VEL_DT))

float vel_step(vel_t *v, float angle) {
    if (!v->primed) { // window full of the first angle, so it starts at 0 instead of a huge jump
        for (int k = 0; k < VEL_WINDOW_N; k++) v->hist[k] = angle;
        v->y = 0.0f;
        v->i = 0;
        v->primed = true;
    }
    float raw = (angle - v->hist[v->i]) / (VEL_WINDOW_N * VEL_DT); // now vs N samples ago
    v->hist[v->i] = angle;
    v->i = (uint8_t)((v->i + 1) % VEL_WINDOW_N);
    v->y += ALPHA * (raw - v->y);
    return v->y;
}
