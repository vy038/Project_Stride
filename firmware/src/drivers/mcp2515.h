#pragma once
#include "can/can_proto.h"
#include <stdbool.h>

int mcp2515_init(void);
int mcp2515_int_init(void);
bool mcp2515_int_low(void);

int mcp2515_send(const can_frame_t *f);

int mcp2515_recv(can_frame_t *f);
