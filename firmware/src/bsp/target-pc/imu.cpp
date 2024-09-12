#include <iostream>

#include "bsp/imu.hpp"

namespace bsp::imu {

/// @section Interface implementation

void init() {
    std::cout << "bsp::imu::init called" << std::endl;
}

} // namespace
