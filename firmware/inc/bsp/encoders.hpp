#pragma once

#include <cstdint>
#include <functional>
#include "utils/math.hpp"


namespace bsp::encoders {

static constexpr float MM_PER_US_TO_M_PER_S = 1000.0; // 1mm/us = 1000m/s


/// @section Custom types

enum DirectionType { CW, CCW };
enum EncoderSide { LEFT, RIGHT };

struct EncoderData {
    int32_t ticks;
    DirectionType direction;
    uint32_t last_update_tick_time;
    uint32_t last_delta_tick_time;
    float linear_vel_m_s;
    float ang_vel_rad_s;
};


/// @section Interface definitions

void init();
void reset();
void clear_ticks();
EncoderData get_data(EncoderSide side);

void update_ticks();
void reset_velocities();
void update_velocities();

void set_right_ang_vel_rad_s(float speed);
void set_left_ang_vel_rad_s(float speed);
float get_linear_velocity_m_s();
float get_filtered_velocity_m_s();
float get_right_filtered_ang_vel_rad_s();
float get_left_filtered_ang_vel_rad_s();
float get_encoder_dist_mm_pulse();

}
