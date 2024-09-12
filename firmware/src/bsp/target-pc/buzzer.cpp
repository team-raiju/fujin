#include <iostream>

#include "bsp/buzzer.hpp"

namespace bsp::buzzer {

/// @section Interface implementation

void init() {
    std::cout << "bsp::buzzer::init called" << std::endl;
}

void start(void) {
    std::cout << "bsp::buzzer::start called" << std::endl;
}

void stop(void) {
    std::cout << "bsp::buzzer::stop called" << std::endl;
}

void set_volume(uint8_t volume) {
    std::cout << "bsp::buzzer::set_volume called" << std::endl;
}

void set_frequency(uint16_t hz) {
    std::cout << "bsp::buzzer::set_frequency called" << std::endl;
}

} // namespace
