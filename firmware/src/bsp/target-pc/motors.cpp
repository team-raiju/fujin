#include "bsp/motors.hpp"
#include "amaterasu/amaterasu.hpp"
#include <algorithm>

namespace bsp::motors {

void init() {
    set(0, 0);
}

void set(int16_t speed_left, int16_t speed_right) {
    speed_left = std::clamp<int16_t>(speed_left, -static_cast<int16_t>(MAX_SPEED), static_cast<int16_t>(MAX_SPEED));
    speed_right = std::clamp<int16_t>(speed_right, -static_cast<int16_t>(MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

    float norm_left = speed_left / static_cast<float>(MAX_SPEED);
    float norm_right = speed_right / static_cast<float>(MAX_SPEED);

    amaterasu::set_motor_pwm(norm_left, norm_right);
}

} // namespace bsp::motors
