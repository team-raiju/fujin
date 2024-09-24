#pragma once

#include <cstdint>

namespace bsp::imu {

/// @section Custom types

enum ImuResult {
    OK,
    ERROR,
};

/// @section Interface definition

ImuResult init();
ImuResult update();

// All angles are related to the Z axis
float get_angle();
void reset_angle();
float get_rps();

void update_g_bias();
void set_g_bias(int32_t bias);
int32_t get_g_bias();

} // namespace
