#include "cfc_ctrl.h"
#include "cfc.h"
#include "cfc_ctrl_io.h"
#include "cfc_golden_nb.h"
#include "config.h"
#include <math.h>

_Static_assert(CFC_NB_HIDDEN_DIM <= CFC_HIDDEN_DIM, "no-backbone hidden state must fit in cfc_state_t");
_Static_assert(GOLDEN_NB_IN_DIM == CFC_NB_INPUT_DIM && GOLDEN_NB_OUT_DIM == CFC_NB_HIDDEN_DIM,
               "golden_nb does not match weights_nobackbone.h");
#if CFC_CTRL_TRAINED
_Static_assert(CFC_CTRL_IN_DIM == CFC_NB_INPUT_DIM, "controller I/O header does not match weights_nobackbone.h");
#endif

cfc_check_t cfc_ctrl_check(void) {
    cfc_check_t r = {CFC_CTRL_TRAINED, 0.0f, -1};
    cfc_state_t s;
    float y[CFC_HIDDEN_DIM];
    cfc_init(&s);
    for (int t = 0; t < GOLDEN_NB_STEPS; t++) {
        cfc_step_nobackbone(&s, GOLDEN_NB_INPUTS + t * GOLDEN_NB_IN_DIM, GOLDEN_NB_TS[t], y);
        for (int i = 0; i < GOLDEN_NB_OUT_DIM; i++) {
            float e = fabsf(y[i] - GOLDEN_NB_OUTPUTS[t * GOLDEN_NB_OUT_DIM + i]);
            if (!(e <= 1e30f)) e = 1e30f;
            if (e > r.max_abs_err) r.max_abs_err = e;
            if (e > 1e-5f && r.first_bad_step < 0) r.first_bad_step = t;
        }
    }
    r.ok = r.ok && r.first_bad_step < 0;
    return r;
}

static cfc_state_t ctrl_state;

void cfc_ctrl_reset(void) { cfc_init(&ctrl_state); }

float cfc_ctrl_step(float err, float vel, float angle) {
#if CFC_CTRL_TRAINED
    const float raw[CFC_CTRL_IN_DIM] = {err, vel, sinf(angle), cosf(angle)};
    float in[CFC_CTRL_IN_DIM], h[CFC_HIDDEN_DIM];
    for (int i = 0; i < CFC_CTRL_IN_DIM; i++) in[i] = (raw[i] - CFC_CTRL_IN_MEAN[i]) / CFC_CTRL_IN_STD[i];
    cfc_step_nobackbone(&ctrl_state, in, 1.0f, h);
    float y = CFC_CTRL_OUT_B;
    for (int i = 0; i < CFC_NB_HIDDEN_DIM; i++) y += CFC_CTRL_OUT_W[i] * h[i];
    return y * CFC_CTRL_OUT_STD + CFC_CTRL_OUT_MEAN;
#else
    (void)err;
    (void)vel;
    (void)angle;
    return 0.0f;
#endif
}
