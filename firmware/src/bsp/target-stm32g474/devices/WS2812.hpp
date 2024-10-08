#pragma once

#include <cstddef>
#include <functional>

#include "../st/hal.h"

#include "utils/math.hpp"

namespace devices {

template <size_t _amount>
class WS2812 {
public:
    WS2812(TIM_HandleTypeDef* timer, uint32_t channel) : timer(timer), timer_channel(channel) {}

    void set(uint8_t num, uint32_t encoded_color) {
        if (num >= _amount) {
            return;
        }

        for (int8_t i = (WS2812_PACKET_SIZE - 1); i >= 0; i--) {
            pwm_data[(num * WS2812_PACKET_SIZE) + i] = (encoded_color & (1 << i)) ? PWM_BIT_1 : PWM_BIT_0;
        }
    }

    void send() {
        for (int8_t i = 0; i < FINAL_PADDING_SIZE; i++) {
            pwm_data[(WS2812_PACKET_SIZE * _amount) + i] = 0;
        }

        HAL_TIM_PWM_Start_DMA(timer, timer_channel, pwm_data, WS2812_PWM_SIZE);
    }

    void pulse_finished_callback(TIM_HandleTypeDef* htim) {
        if (htim->Instance == timer->Instance) {
            HAL_TIM_PWM_Stop_DMA(htim, timer_channel);
        }
    }

    static constexpr size_t amount = _amount;

private:
    static constexpr uint16_t WS2812_PACKET_SIZE = 24;
    static constexpr uint16_t FINAL_PADDING_SIZE = 2;

    static constexpr uint16_t WS2812_PWM_SIZE = ((WS2812_PACKET_SIZE * _amount) + FINAL_PADDING_SIZE);

    /* PWM BIT1 must be a number so that the pwm has 580ns to 1000ns high time */
    /* PWM BIT0 must be a number so that the pwm has 220ns to 380ns high time */

    /* Here we are using BIT1 approximately 625ns and BIT0 approximately 305ns */
    /* Considering timer frequency = 85MHz */

    static constexpr uint32_t PWM_BIT_1 = std::round(85 * 0.625);
    static constexpr uint32_t PWM_BIT_0 = std::round(85 * 0.305);

    uint32_t pwm_data[WS2812_PWM_SIZE];

    TIM_HandleTypeDef* timer;
    uint32_t timer_channel;
};

}
