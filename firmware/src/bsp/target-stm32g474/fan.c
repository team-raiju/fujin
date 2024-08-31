#include "st/hal.h"

#include "bsp/fan.h"

void bsp_fan_init() {
    MX_TIM3_Init();
}
