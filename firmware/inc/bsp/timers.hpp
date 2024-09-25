#pragma once

#include <cstdint>

namespace bsp {

namespace timers {

/// @section Interface definition

void init(void);

} // namespace bsp::timers

// For convenience, the generic delay and ticks can be accessed directly from bsp namespace

uint32_t get_tick_ms(void);
uint32_t get_tick_us(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

} // namespace bsp
