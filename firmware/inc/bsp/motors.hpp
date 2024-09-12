#pragma once

#include <cstdint>

namespace bsp::motors {

/// @section Interface definition

void init(void);
void set(int16_t speed_left, int16_t speed_right);

} // namespace
