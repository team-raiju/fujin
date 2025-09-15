#include "st/hal.h"

#include "bsp/motors.hpp"
#include "utils/math.hpp"

namespace bsp::motors {

/// @section Constants

#define LEFT_PWM_CH TIM_CHANNEL_4
#define LEFT_DIR_PIN GPIO_PIN_2
#define RIGHT_PWM_CH TIM_CHANNEL_2
#define RIGHT_DIR_PIN GPIO_PIN_0

/// @section Private functions

void set_pwm(uint16_t channel, uint16_t pin, int16_t speed) {
    HAL_GPIO_WritePin(GPIOC, pin, speed < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, channel, abs(speed));
}

/// @section Interface implementation

void init() {
    MX_TIM1_Init();

    /* Motor Right */
    HAL_TIM_PWM_Start(&htim1, RIGHT_PWM_CH);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_PWM_CH, 0);

    /* Motor Left */
    HAL_TIM_PWM_Start(&htim1, LEFT_PWM_CH);
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM_CH, 0);
}

void set(int16_t speed_left, int16_t speed_right) {
    speed_right = constrain(speed_right, -MAX_SPEED, MAX_SPEED);
    speed_left = constrain(speed_left, -MAX_SPEED, MAX_SPEED);

    // Inverted in the board
    speed_right = -speed_right;

    set_pwm(LEFT_PWM_CH, LEFT_DIR_PIN, speed_left);
    set_pwm(RIGHT_PWM_CH, RIGHT_DIR_PIN, speed_right);
}

} // namespace
