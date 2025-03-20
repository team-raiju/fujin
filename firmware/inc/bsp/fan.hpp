#pragma once

#include <cstdint>

namespace bsp::fan {

void init(void);

void set(uint16_t speed);

float get_max_fan_voltage(void);

}
