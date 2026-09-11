#include "bsp/fan.hpp"

namespace bsp::fan {

void init() {}

void set(uint16_t speed) {
    (void)speed;
}

float get_max_fan_voltage(void) {
    return 7.4f;
}

} // namespace bsp::fan
