#pragma once
#include <stdbool.h>

int pwm3_init(void);

int pwm3_set(const float duty[3]);

int pwm3_enable(bool on);
