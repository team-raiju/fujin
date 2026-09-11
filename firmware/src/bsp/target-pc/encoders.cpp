#include "bsp/encoders.hpp"

namespace bsp::encoders {

static EncoderData left_encoder{0, DirectionType::CW, 0, 0, 0.0f, 0.0f};
static EncoderData right_encoder{0, DirectionType::CW, 0, 0, 0.0f, 0.0f};

void init() {}

void reset() {
    clear_ticks();
    reset_velocities();
}

void clear_ticks() {
    left_encoder.ticks = 0;
    right_encoder.ticks = 0;
}

EncoderData get_data(EncoderSide side) {
    return (side == LEFT) ? left_encoder : right_encoder;
}

void update_ticks() {}

void reset_velocities() {
    left_encoder.linear_vel_m_s = 0.0f;
    right_encoder.linear_vel_m_s = 0.0f;
    left_encoder.ang_vel_rad_s = 0.0f;
    right_encoder.ang_vel_rad_s = 0.0f;
}

void update_velocities(float target_accel_m_s2) {
    (void)target_accel_m_s2;
}

void set_right_ang_vel_rad_s(float speed) {
    right_encoder.ang_vel_rad_s = speed;
}

void set_left_ang_vel_rad_s(float speed) {
    left_encoder.ang_vel_rad_s = speed;
}

float get_linear_velocity_m_s() {
    return 0.0f;
}

float get_left_linear_velocity_m_s() {
    return 0.0f;
}

float get_right_linear_velocity_m_s() {
    return 0.0f;
}

float get_filtered_velocity_m_s() {
    return 0.0f;
}

float get_right_filtered_ang_vel_rad_s() {
    return 0.0f;
}

float get_left_filtered_ang_vel_rad_s() {
    return 0.0f;
}

float get_encoder_dist_mm_pulse() {
    return 0.0f;
}

} // namespace bsp::encoders
