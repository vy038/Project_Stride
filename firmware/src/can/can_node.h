#pragma once
#include "bldc/joint.h"
#include <stdint.h>

void can_node_init(void);

void can_node_on_int(void);

void can_node_poll(joint_t *j, uint32_t now_ms, float angle, float vel);
