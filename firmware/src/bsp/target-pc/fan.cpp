#include <iostream>

#include "bsp/fan.hpp"

namespace bsp::fan {

/// @section Interface implementation

void init() {
    std::cout << "bsp::fan::init called" << std::endl;
}

void set(uint8_t speed) {
    std::cout << "bsp::fan::set called" << std::endl;
}

} // namespace
