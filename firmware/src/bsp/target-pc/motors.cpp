#include <iostream>

#include "bsp/motors.hpp"

namespace bsp::motors {

/// @section Interface implementation

void init() {
    std::cout << "bsp::motors::init called" << std::endl;
}

void set(int16_t speed_left, int16_t speed_right) {
    std::cout << "bsp::motors::set called" << std::endl;
}

} // namespace
