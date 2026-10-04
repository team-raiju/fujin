#include "bsp/imu.hpp"
#include "services/config.hpp"
#include "amaterasu/amaterasu.hpp"
#include <cmath>

namespace bsp::imu {

static float g_bias_z = 0.0f;
static float current_angle = 0.0f;
static float incremental_angle = 0.0f;
static float last_omega = 0.0f;

ImuResult init() {
    reset();
    return OK;
}

ImuResult update() {
    float dt_s = services::Config::CONTROL_PERIOD_S;
    float omega = amaterasu::get_gyro_z_rad_s();
    last_omega = omega;

    current_angle += omega * dt_s;
    incremental_angle += omega * dt_s;

    // Normalize to [-pi, pi]
    while (current_angle > static_cast<float>(M_PI)) {
        current_angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (current_angle < -static_cast<float>(M_PI)) {
        current_angle += 2.0f * static_cast<float>(M_PI);
    }

    return OK;
}

void reset() {
    reset_angle();
    last_omega = 0.0f;
}

float get_angle() {
    return current_angle;
}

float get_incremental_angle() {
    return incremental_angle;
}

void reset_angle() {
    current_angle = 0.0f;
    incremental_angle = 0.0f;
}

float get_rad_per_s() {
    return amaterasu::get_gyro_z_rad_s();
}

float get_raw_rad_per_s() {
    return amaterasu::get_gyro_z_rad_s();
}

float get_z_acceleration() {
    float x, y, z;
    amaterasu::get_accel_m_s2(x, y, z);
    return z;
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
