#pragma once
#include <stdbool.h>

typedef struct {
    bool ok;
    float max_abs_err;
    int first_bad_step;
} cfc_check_t;

cfc_check_t cfc_ctrl_check(void);

void cfc_ctrl_reset(void);

float cfc_ctrl_step(float err, float vel, float angle);
