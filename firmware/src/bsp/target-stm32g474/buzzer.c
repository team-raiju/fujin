#include "st/hal.h"

#include "bsp/buzzer.h"
#include "utils.h"

/// @section Constants

#define CPU_FREQUENCY (170000000UL) // SystemCoreClock
#define COUNTER_PERIOD (100)

/// @section Interface implementation

void bsp_buzzer_init() {
    MX_TIM8_Init();
}

void bsp_buzzer_start(void) {
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}

void bsp_buzzer_stop(void) {
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
}

void bsp_buzzer_set_volume(uint8_t volume) {
    volume = min(volume, 100);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, volume);
}

void bsp_buzzer_set_frequency(uint16_t hz) {
    uint16_t preescaler_value = ((CPU_FREQUENCY / COUNTER_PERIOD) / hz) - 1;
    __HAL_TIM_SET_PRESCALER(&htim8, preescaler_value);
}
