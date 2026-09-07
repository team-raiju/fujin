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

void update_velocities(float target_accel_m_s2) {
    (void)target_accel_m_s2;
}

float get_linear_velocity_m_s() {
    return 0;
}

float get_left_linear_velocity_m_s() {
    return 0;
}

float get_right_linear_velocity_m_s() {
    return 0;
}

}
