#include "st/hal.h"

#include "bsp/buzzer.h"

/// @section Interface implementation

void bsp_buzzer_init() {
    MX_TIM8_Init();
}
