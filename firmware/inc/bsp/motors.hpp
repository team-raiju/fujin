#pragma once

#include <cstdint>

namespace bsp::motors {

/// @section Constants

constexpr uint16_t COUNTER_PERIOD_MAX = (1000);
constexpr uint16_t MAX_SPEED = (COUNTER_PERIOD_MAX - 1);

/// @section Interface definition

void init(void);
void set(int16_t speed_left, int16_t speed_right);

} // namespace
