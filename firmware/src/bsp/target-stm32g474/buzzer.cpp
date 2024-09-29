#include "st/hal.h"

#include "bsp/buzzer.hpp"
#include "utils/math.hpp"

namespace bsp::buzzer {

/// @section Constants

#define CPU_FREQUENCY (170000000UL) // SystemCoreClock
#define COUNTER_PERIOD (100)

/// @section Interface implementation

void init() {
    MX_TIM8_Init();
}

void start(void) {
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}

void stop(void) {
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
}

void set_volume(uint8_t volume) {
    static constexpr uint8_t max_volume = 100;
    volume = std::min(volume, max_volume);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, volume);
}

void set_frequency(uint16_t hz) {
    uint16_t preescaler_value = ((CPU_FREQUENCY / COUNTER_PERIOD) / hz) - 1;
    __HAL_TIM_SET_PRESCALER(&htim8, preescaler_value);
}

} // namespace
