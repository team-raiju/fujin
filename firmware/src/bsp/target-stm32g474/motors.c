#include "st/hal.h"

#include "bsp/motors.h"

void bsp_motors_init() {
    MX_TIM1_Init();
}
