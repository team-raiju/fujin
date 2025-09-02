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
float get_incremental_angle();
void reset_angle();
float get_rad_per_s();

float get_z_acceleration();

void set_g_bias_z(float z_gbias);
float get_g_bias_z();

void enable_motion_gc_filter(bool enable);

bool is_imu_emergency();

} // namespace
