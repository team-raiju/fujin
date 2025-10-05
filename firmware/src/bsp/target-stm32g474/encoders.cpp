#include "st/hal.h"

#include "bsp/encoders.hpp"
#include "bsp/timers.hpp"
#include "pin_mapping.h"

namespace bsp::encoders {

// Min measured vel is ENCODER_DIST_MM_PULSE / 10 = 0.00776 m/s
static constexpr uint32_t MAX_TIME_WITHOUT_ENCODER_US = 10000;

static constexpr float WHEEL_RADIUS_CM = (1.275);
static constexpr float WHEEL_RADIUS_M = (WHEEL_RADIUS_CM / 100.0);
static constexpr float WHEEL_PERIMETER_CM = (M_TWOPI * WHEEL_RADIUS_CM);

static constexpr float WHEEL_TO_ENCODER_RATIO = (1.0);
static constexpr float ENCODER_PPR = (1024.0);
static constexpr float PULSES_PER_WHEEL_ROTATION = (WHEEL_TO_ENCODER_RATIO * ENCODER_PPR);

static constexpr float WHEELS_DIST_CM = (7.0);
static constexpr float ENCODER_DIST_CM_PULSE = (WHEEL_PERIMETER_CM / PULSES_PER_WHEEL_ROTATION);
static constexpr float ENCODER_DIST_MM_PULSE = (ENCODER_DIST_CM_PULSE * 10.0);


static float linear_velocity_m_s;

float filtered_velocity_m_s;
float last_velocity_m_s;

float right_filtered_ang_vel_rad_s;
float left_filtered_ang_vel_rad_s;

uint32_t last_update_vel_time;
uint32_t delta_vel_time;

EncoderData left_encoder;
EncoderData right_encoder;

/// @section Interface implementation

void init() {
    reset();
}

void reset() {
    left_encoder = {0, DirectionType::CW, 0, MAX_TIME_WITHOUT_ENCODER_US + 1, 0, 0};
    right_encoder = {0, DirectionType::CW, 0, MAX_TIME_WITHOUT_ENCODER_US + 1, 0, 0};
    reset_velocities();
}

void clear_ticks() {
    left_encoder.ticks = 0;
    right_encoder.ticks = 0;
}

EncoderData get_data(EncoderSide side) {
    return side == LEFT ? left_encoder : right_encoder;
}

void gpio_exti_callback(uint16_t GPIO_Pin) {
    uint32_t current_time_us = bsp::get_tick_us();

    if (GPIO_Pin == ENCODER_LEFT_A_PIN) {

        left_encoder.last_delta_tick_time = current_time_us - left_encoder.last_update_tick_time;
        left_encoder.last_update_tick_time = current_time_us;
        left_encoder.direction = HAL_GPIO_ReadPin(ENCODER_LEFT_B_PORT, ENCODER_LEFT_B_PIN) == GPIO_PIN_SET ? CCW : CW;
        left_encoder.ticks = left_encoder.direction == CW ? left_encoder.ticks - 1 : left_encoder.ticks + 1;

    } else if (GPIO_Pin == ENCODER_RIGHT_A_PIN) {

        right_encoder.last_delta_tick_time = current_time_us - right_encoder.last_update_tick_time;
        right_encoder.last_update_tick_time = current_time_us;
        right_encoder.direction =
            HAL_GPIO_ReadPin(ENCODER_RIGHT_B_PORT, ENCODER_RIGHT_B_PIN) == GPIO_PIN_SET ? CW : CCW;
        right_encoder.ticks = right_encoder.direction == CW ? right_encoder.ticks - 1 : right_encoder.ticks + 1;
    }
}

void update_ticks() {}

void reset_velocities() {
    linear_velocity_m_s = 0;
    filtered_velocity_m_s = 0;
    last_velocity_m_s = 0;
    right_filtered_ang_vel_rad_s = 0;
    left_filtered_ang_vel_rad_s = 0;
    last_update_vel_time = 0;
    delta_vel_time = MAX_TIME_WITHOUT_ENCODER_US + 1;
}

void set_right_ang_vel_rad_s(float speed) {
    right_encoder.ang_vel_rad_s = speed;
    right_filtered_ang_vel_rad_s = speed * 0.1 + right_filtered_ang_vel_rad_s * 0.9;
}

void set_left_ang_vel_rad_s(float speed) {
    left_encoder.ang_vel_rad_s = speed;
    left_filtered_ang_vel_rad_s = speed * 0.1 + left_filtered_ang_vel_rad_s * 0.9;
}

void update_velocities() {

    delta_vel_time = bsp::get_tick_us() - last_update_vel_time;
    last_update_vel_time = bsp::get_tick_us();

    if (left_encoder.last_delta_tick_time == 0 || right_encoder.last_delta_tick_time == 0) {
        return;
    }

    uint32_t delta_time_tick_left = bsp::get_tick_us() - left_encoder.last_update_tick_time;
    uint32_t delta_time_tick_right = bsp::get_tick_us() - right_encoder.last_update_tick_time;

    /* Left wheel */
    if (delta_time_tick_left > MAX_TIME_WITHOUT_ENCODER_US) {
        left_encoder.linear_vel_m_s = 0;
    } else {
        int32_t ticks = left_encoder.ticks;
        if (ticks == 0) { // No tick detected in the last delta_vel_time (e.g 1ms)
            ticks = left_encoder.direction == CW ? -1 : 1;
        }

        left_encoder.linear_vel_m_s = ((ticks * ENCODER_DIST_MM_PULSE) / (float)delta_vel_time) * MM_PER_US_TO_M_PER_S;
    }

    /* Right wheel */
    if (delta_time_tick_right > MAX_TIME_WITHOUT_ENCODER_US) {
        right_encoder.linear_vel_m_s = 0;
    } else {
        int32_t ticks = right_encoder.ticks;
        if (ticks == 0) { // No tick detected in the last delta_vel_time (e.g 1ms)
            ticks = right_encoder.direction == CW ? -1 : 1;
        }

        right_encoder.linear_vel_m_s = ((ticks * ENCODER_DIST_MM_PULSE) / (float)delta_vel_time) * MM_PER_US_TO_M_PER_S;
    }

    set_left_ang_vel_rad_s(left_encoder.linear_vel_m_s / WHEEL_RADIUS_M);
    set_right_ang_vel_rad_s(right_encoder.linear_vel_m_s / WHEEL_RADIUS_M);

    linear_velocity_m_s = (left_encoder.linear_vel_m_s + right_encoder.linear_vel_m_s) / 2.0;

    // Butterworth filter - https://www.meme.net.au/butterworth.html
    // filtered_velocity_m_s = (0.112157918)*(linear_velocity_m_s + last_velocity_m_s) +
    // (0.775684163)*(filtered_velocity_m_s); // Low pass 40hz filtered_velocity_m_s =
    // (0.072960747)*(linear_velocity_m_s + last_velocity_m_s) + (0.854078506)*(filtered_velocity_m_s); // Low pass 25hz
    filtered_velocity_m_s = linear_velocity_m_s * 0.1 + filtered_velocity_m_s * 0.9;
    last_velocity_m_s = linear_velocity_m_s;
}

float get_linear_velocity_m_s() {
    return linear_velocity_m_s;
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
    return ENCODER_DIST_MM_PULSE;
}

}
