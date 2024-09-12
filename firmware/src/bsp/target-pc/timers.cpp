#include <iostream>

#include "bsp/timers.hpp"

namespace bsp {

/// @section Interface implementation

void timers::init(void) {
    std::cout << "bsp::timers::init called" << std::endl;
}

uint32_t get_tick_ms(void) {
    std::cout << "bsp::get_tick_ms called" << std::endl;
    return 0;
}

uint32_t get_tick_us(void) {
    std::cout << "bsp::get_tick_us called" << std::endl;
    return 0;
}

void delay_ms(uint32_t ms) {
    std::cout << "bsp::delay_ms called" << std::endl;
}

void delay_us(uint32_t us) {
    std::cout << "bsp::delay_us called" << std::endl;
}

} // namespace bsp
