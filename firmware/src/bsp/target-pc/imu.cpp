#include "bsp/imu.hpp"

namespace bsp::imu {

static float g_bias_z = 0.0f;
static float current_angle = 0.0f;

ImuResult init() {
    return OK;
}

ImuResult update() {
    return OK;
}

void reset() {
    current_angle = 0.0f;
}

float get_angle() {
    return current_angle;
}

float get_incremental_angle() {
    return 0.0f;
}

void reset_angle() {
    current_angle = 0.0f;
}

float get_rad_per_s() {
    return 0.0f;
}

float get_raw_rad_per_s() {
    return 0.0f;
}

float get_z_acceleration() {
    return 0.0f;
}

void set_g_bias_z(float z_gbias) {
    g_bias_z = z_gbias;
}

float get_g_bias_z() {
    return g_bias_z;
}

void enable_motion_gc_filter(bool enable) {
    (void)enable;
}

bool is_imu_emergency() {
    return false;
}

} // namespace bsp::imu
