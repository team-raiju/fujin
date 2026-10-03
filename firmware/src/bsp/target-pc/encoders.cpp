#include "bsp/encoders.hpp"
#include "services/config.hpp"
#include "amaterasu/amaterasu.hpp"
#include <algorithm>
#include <cmath>

namespace bsp::encoders {

static constexpr float WHEEL_TO_ENCODER_RATIO = (1.0f);
static constexpr float ENCODER_PPR = (1024.0f);
static constexpr float PULSES_PER_WHEEL_ROTATION = (WHEEL_TO_ENCODER_RATIO * ENCODER_PPR);

static EncoderData left_encoder{0, DirectionType::CW, 0, 0, 0.0f, 0.0f};
static EncoderData right_encoder{0, DirectionType::CW, 0, 0, 0.0f, 0.0f};

static int32_t last_amaterasu_ticks_l = 0;
static int32_t last_amaterasu_ticks_r = 0;
static int32_t last_delta_l = 0;
static int32_t last_delta_r = 0;
static float linear_velocity_m_s = 0.0f;
static float filtered_velocity_m_s = 0.0f;
static float right_filtered_ang_vel_rad_s = 0.0f;
static float left_filtered_ang_vel_rad_s = 0.0f;

void init() {
    reset();
}

void reset() {
    clear_ticks();
    reset_velocities();
    last_amaterasu_ticks_l = amaterasu::get_encoder_ticks(0);
    last_amaterasu_ticks_r = amaterasu::get_encoder_ticks(1);
    last_delta_l = 0;
    last_delta_r = 0;
}

void clear_ticks() {
    left_encoder.ticks = 0;
    right_encoder.ticks = 0;
}

EncoderData get_data(EncoderSide side) {
    return (side == LEFT) ? left_encoder : right_encoder;
}

void update_ticks() {
    int32_t curr_l = amaterasu::get_encoder_ticks(0);
    int32_t curr_r = amaterasu::get_encoder_ticks(1);

    int32_t delta_l = curr_l - last_amaterasu_ticks_l;
    int32_t delta_r = curr_r - last_amaterasu_ticks_r;

    last_amaterasu_ticks_l = curr_l;
    last_amaterasu_ticks_r = curr_r;
    last_delta_l = delta_l;
    last_delta_r = delta_r;

    left_encoder.ticks += delta_l;
    right_encoder.ticks += delta_r;

    left_encoder.direction = (delta_l >= 0) ? DirectionType::CW : DirectionType::CCW;
    right_encoder.direction = (delta_r >= 0) ? DirectionType::CW : DirectionType::CCW;
}

void reset_velocities() {
    left_encoder.linear_vel_m_s = 0.0f;
    right_encoder.linear_vel_m_s = 0.0f;
    left_encoder.ang_vel_rad_s = 0.0f;
    right_encoder.ang_vel_rad_s = 0.0f;
    linear_velocity_m_s = 0.0f;
    filtered_velocity_m_s = 0.0f;
    left_filtered_ang_vel_rad_s = 0.0f;
    right_filtered_ang_vel_rad_s = 0.0f;
    last_delta_l = 0;
    last_delta_r = 0;
}

void update_velocities(float target_accel_m_s2) {
    (void)target_accel_m_s2;
    float dt_s = services::Config::CONTROL_PERIOD_S;
    float dist_pulse_m = get_encoder_dist_mm_pulse() / 1000.0f;
    float wheel_radius_m = services::Config::wheel_radius_mm / 1000.0f;

    left_encoder.linear_vel_m_s = (last_delta_l * dist_pulse_m) / dt_s;
    right_encoder.linear_vel_m_s = (last_delta_r * dist_pulse_m) / dt_s;

    if (wheel_radius_m > 0.0f) {
        set_left_ang_vel_rad_s(left_encoder.linear_vel_m_s / wheel_radius_m);
        set_right_ang_vel_rad_s(right_encoder.linear_vel_m_s / wheel_radius_m);
    }

    linear_velocity_m_s = (left_encoder.linear_vel_m_s + right_encoder.linear_vel_m_s) / 2.0f;
    filtered_velocity_m_s += 0.30f * (linear_velocity_m_s - filtered_velocity_m_s);
    left_filtered_ang_vel_rad_s += 0.30f * (left_encoder.ang_vel_rad_s - left_filtered_ang_vel_rad_s);
    right_filtered_ang_vel_rad_s += 0.30f * (right_encoder.ang_vel_rad_s - right_filtered_ang_vel_rad_s);
}

void set_right_ang_vel_rad_s(float speed) {
    right_encoder.ang_vel_rad_s = speed;
}

void set_left_ang_vel_rad_s(float speed) {
    left_encoder.ang_vel_rad_s = speed;
}

float get_linear_velocity_m_s() {
    return linear_velocity_m_s;
}

float get_left_linear_velocity_m_s() {
    return left_encoder.linear_vel_m_s;
}

float get_right_linear_velocity_m_s() {
    return right_encoder.linear_vel_m_s;
}

float get_filtered_velocity_m_s() {
    return filtered_velocity_m_s;
}

float get_right_filtered_ang_vel_rad_s() {
    return right_filtered_ang_vel_rad_s;
}

float get_left_filtered_ang_vel_rad_s() {
    return left_filtered_ang_vel_rad_s;
}

float get_encoder_dist_mm_pulse() {
    float wheel_perimeter_mm = (2.0f * static_cast<float>(M_PI) * services::Config::wheel_radius_mm);
    return (wheel_perimeter_mm / PULSES_PER_WHEEL_ROTATION);
}

} // namespace bsp::encoders
