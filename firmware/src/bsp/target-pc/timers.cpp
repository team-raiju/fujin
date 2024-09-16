#include <chrono>
#include <iostream>
#include <thread>

#include "bsp/timers.hpp"

namespace bsp {

using timer = std::chrono::steady_clock;
static std::chrono::time_point start = timer::now();

/// @section Interface implementation

void timers::init(void) {}

uint32_t get_tick_ms(void) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(timer::now() - start).count();
}

uint32_t get_tick_us(void) {
    return std::chrono::duration_cast<std::chrono::microseconds>(timer::now() - start).count();
}

void delay_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delay_us(uint32_t us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} // namespace bsp
