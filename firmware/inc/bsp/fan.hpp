#pragma once

#include <cstdint>

namespace bsp::fan {

/// @section Constants

static constexpr uint16_t MAX_SPEED = 1000;
static constexpr uint16_t MIN_SPEED = 0;

void init(void);

void set(uint16_t speed);

float get_max_fan_voltage(void);

}
