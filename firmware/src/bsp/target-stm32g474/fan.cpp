#include "st/hal.h"

#include "bsp/fan.hpp"
#include "utils/math.h"

namespace bsp::fan {

/// @section Constants

#define MAX_SPEED (100)
#define COUNTER_PERIOD_MAX (999)

#define PWM_CH TIM_CHANNEL_2

/// @section Interface implementation

void init() {
    MX_TIM3_Init();

    HAL_TIM_PWM_Start(&htim3, PWM_CH);
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH, 0);
}

void set(uint8_t speed) {
    speed = min(speed, MAX_SPEED);

    uint32_t counter_value = map(speed, 0, MAX_SPEED, 0, COUNTER_PERIOD_MAX);
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH, counter_value);
}

} // namespace
