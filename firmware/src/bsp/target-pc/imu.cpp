#include "bsp/imu.hpp"

namespace bsp::imu {

/// @section Interface implementation

ImuResult init() {
    return OK;
}

ImuResult update() {
    return OK;
}

float get_angle() {
    return 0;
}

void reset_angle() {}

float get_rps() {
    return 0;
}

float get_rad_per_s() {
    return 0;
}

void update_g_bias() {}

void set_g_bias(int32_t) {}

int32_t get_g_bias() {
    return 0;
}

} // namespace
