#include <iostream>

#include "bsp/leds.hpp"

namespace bsp::leds {

/// @section Interface implementation

void init(void) {
    std::cout << "bsp::leds::init called" << std::endl;
}

void indication_on(void) {
    std::cout << "bsp::leds::indication_on called" << std::endl;
}

void indication_off(void) {
    std::cout << "bsp::leds::indication_off called" << std::endl;
}

void indication_toggle(void) {
    std::cout << "bsp::leds::indication_toggle called" << std::endl;
}

void stripe_set(uint8_t num, uint8_t r, uint8_t g, uint8_t b) {
    (void)num;
    (void)r;
    (void)g;
    (void)b;

    std::cout << "bsp::leds::stripe_set called" << std::endl;
}

} // namespace
