#include <iostream>

#include "bsp/usb.hpp"

namespace bsp::usb {

/// @section Interface implementation

void init(void) {
    std::cout << "bsp::usb::init called" << std::endl;
}

void reset_on_host(void) {
    std::cout << "bsp::usb::reset_on_host called" << std::endl;
}

} // namespace
