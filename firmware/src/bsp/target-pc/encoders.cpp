#include <iostream>

#include "bsp/encoders.hpp"

namespace bsp::encoders {

/// @section Interface implementation

void init() {}

void register_callback_encoder_left(EncoderCallback callback) {
    (void)callback;
}

void register_callback_encoder_right(EncoderCallback callback) {
    (void)callback;
}

void set_linear_velocity_m_s(float speed) {
    (void)speed;
}

float get_linear_velocity_m_s() {
    return 0;
}

}
