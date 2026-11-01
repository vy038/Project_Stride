#include "foc.h"
#include <math.h>

#define SQRT3_2 0.866025404f

static float clamp01(float x) { return fminf(fmaxf(x, 0.0f), 1.0f); }

void foc_svpwm(float uq, float theta_e, float vbus, float duty[3]) {
    float alpha = -uq * sinf(theta_e);
    float beta = uq * cosf(theta_e);
    float v[3] = {alpha, -0.5f * alpha + SQRT3_2 * beta, -0.5f * alpha - SQRT3_2 * beta};
    float mid = 0.5f * (fmaxf(v[0], fmaxf(v[1], v[2])) + fminf(v[0], fminf(v[1], v[2])));
    float k = vbus > 0.0f ? 1.0f / vbus : 0.0f;
    for (int i = 0; i < 3; i++) duty[i] = clamp01((v[i] - mid) * k + 0.5f);
}
