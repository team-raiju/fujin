#include "st/hal.h"

#include "bsp/fan.hpp"
#include "utils/math.hpp"

namespace bsp::fan {

/// @section Constants

static constexpr uint16_t counter_period_min = 0;
static constexpr uint16_t counter_period_max = 999;

#define PWM_CH TIM_CHANNEL_2
#define PWM_TIMER_INSTANCE &htim3
#define PWM_TIMER_INIT_FUNCTION MX_TIM3_Init

/// @section Interface implementation

void init() {
    PWM_TIMER_INIT_FUNCTION();

    HAL_TIM_PWM_Start(PWM_TIMER_INSTANCE, PWM_CH);
    __HAL_TIM_SET_COMPARE(PWM_TIMER_INSTANCE, PWM_CH, 0);
}

void set(uint16_t speed) {
    speed = std::min(speed, counter_period_max);

    __HAL_TIM_SET_COMPARE(PWM_TIMER_INSTANCE, PWM_CH, speed);
}

float get_max_fan_voltage(void) {
    return 11.0f;
}

} // namespace
