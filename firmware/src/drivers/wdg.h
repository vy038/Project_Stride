#pragma once
#include <stdbool.h>
#include <stdint.h>

int wdg_init(uint32_t timeout_ms);
void wdg_kick(void);
bool wdg_caused_reset(void);
