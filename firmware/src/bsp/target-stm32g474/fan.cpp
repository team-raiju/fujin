#include "st/hal.h"

#include "bsp/fan.hpp"
#include "utils/math.hpp"

namespace bsp::fan {

/// @section Constants

static constexpr uint8_t max_speed = 100;
static constexpr uint16_t counter_period_max = 999;

#define PWM_CH TIM_CHANNEL_2

/// @section Interface implementation

void init() {
    MX_TIM3_Init();

    HAL_TIM_PWM_Start(&htim3, PWM_CH);
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH, 0);
}

void set(uint8_t speed) {
    speed = std::min(speed, max_speed);

    auto counter_value = map<uint32_t>(speed, 0, max_speed, 0, counter_period_max);
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH, counter_value);
}

} // namespace
