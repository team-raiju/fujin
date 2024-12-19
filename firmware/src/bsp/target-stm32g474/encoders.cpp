#include "st/hal.h"

#include "bsp/encoders.hpp"
#include "pin_mapping.h"

namespace bsp::encoders {

static EncoderCallback cb_encoder_l;
static EncoderCallback cb_encoder_r;
static float linear_velocity_m_s;

/// @section Interface implementation

void init() {
    // We are not using the SPI interface, only the interruptions
    // MX_SPI1_Init();
}

void register_callback_encoder_left(EncoderCallback callback) {
    cb_encoder_l = callback;
}

void register_callback_encoder_right(EncoderCallback callback) {
    cb_encoder_r = callback;
}

void gpio_exti_callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ENCODER_LEFT_A_PIN) {
        if (!cb_encoder_l) {
            return;
        }

        cb_encoder_l(HAL_GPIO_ReadPin(ENCODER_LEFT_B_PORT, ENCODER_LEFT_B_PIN) == GPIO_PIN_SET ? CCW : CW);
    }

    if (GPIO_Pin == ENCODER_RIGHT_A_PIN) {
        if (!cb_encoder_r) {
            return;
        }

        cb_encoder_r(HAL_GPIO_ReadPin(ENCODER_RIGHT_B_PORT, ENCODER_RIGHT_B_PIN) == GPIO_PIN_SET ? CW : CCW);
    }
}

void set_linear_velocity_m_s(float speed) {
    linear_velocity_m_s = speed;
}

float get_linear_velocity_m_s() {
    return linear_velocity_m_s;
}

}
