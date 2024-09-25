#include "st/hal.h"

#include "bsp/encoders.hpp"
#include "pin_mapping.h"

namespace bsp::encoders {

static EncoderCallback cb_encoder_l;
static EncoderCallback cb_encoder_r;

/// @section Interface implementation

void init() {}

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

        if (HAL_GPIO_ReadPin(ENCODER_LEFT_B_PORT, ENCODER_LEFT_B_PIN) == GPIO_PIN_SET) {
            cb_encoder_l(CW);
        } else {
            cb_encoder_l(CCW);
        }
    }

    if (GPIO_Pin == ENCODER_RIGHT_A_PIN) {
        if (!cb_encoder_r) {
            return;
        }

        if (HAL_GPIO_ReadPin(ENCODER_RIGHT_B_PORT, ENCODER_RIGHT_B_PIN) == GPIO_PIN_SET) {
            cb_encoder_r(CW);
        } else {
            cb_encoder_r(CCW);
        }
    }
}

}
