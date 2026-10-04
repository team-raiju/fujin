#include "bsp/fan.hpp"
#include "amaterasu/amaterasu.hpp"
#include <algorithm>

namespace bsp::fan {

void init() {
    set(0);
}

void set(uint16_t speed) {
    float norm = std::clamp(speed / 1000.0f, 0.0f, 1.0f);
    amaterasu::set_fan_pwm(norm);
}

float get_max_fan_voltage(void) {
    return 7.4f;
}

} // namespace bsp::fan
