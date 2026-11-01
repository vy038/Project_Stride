#include "drivers/mcp2515.h"
#include "config.h"
#include "stm32f4xx.h"
#include <errno.h>

int mcp2515_init(void) {
    return -ENOSYS; // TODO
}

int mcp2515_int_init(void) {
    return -ENOSYS; // TODO: GPIOC clock, PC7 pull-up input, SYSCFG EXTICR to port C, EXTI7 falling edge, NVIC priority 3, MCP2515 CANINTE
}

bool mcp2515_int_low(void) {
    return !(CAN_INT_PORT->IDR & (1u << CAN_INT_PIN));
}

int mcp2515_send(const can_frame_t *f) {
    (void)f;
    return -ENOSYS; // TODO
}

int mcp2515_recv(can_frame_t *f) {
    (void)f;
    return -ENOSYS; // TODO
}
