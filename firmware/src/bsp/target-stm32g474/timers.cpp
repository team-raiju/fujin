#include "st/hal.h"

#include "bsp/timers.hpp"

namespace bsp {

/// @section Interface implementation

void timers::init(void) {
    MX_TIM5_Init();
    HAL_TIM_Base_Start(&htim5);
}

uint32_t get_tick_ms(void) {
    return HAL_GetTick();
}

uint32_t get_tick_us(void) {
    return __HAL_TIM_GET_COUNTER(&htim5);
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void delay_us(uint32_t us) {
    uint32_t start = get_tick_us();
    volatile uint32_t wait = us;

    if (wait < HAL_MAX_DELAY) {
        wait += 1;
    }

    while ((get_tick_us() - start) < wait) {
        __NOP();
    }
}

} // namespace bsp
