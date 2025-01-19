#pragma once

#include <cstdint>
#include <functional>

namespace bsp::encoders {

/// @section Custom types

// TODO: Enum the type
enum DirectionType { CW, CCW };

typedef std::function<void(DirectionType type)> EncoderCallback;

/// @section Interface definitions

void init();

void register_callback_encoder_left(EncoderCallback callback);
void register_callback_encoder_right(EncoderCallback callback);
void reset_linear_velocity_m_s();
void set_linear_velocity_m_s(float speed);
void set_right_ang_vel_rad_s(float speed);
void set_left_ang_vel_rad_s(float speed); 
float get_linear_velocity_m_s();
float get_filtered_velocity_m_s();
float get_right_filtered_ang_vel_rad_s();
float get_left_filtered_ang_vel_rad_s();

}
