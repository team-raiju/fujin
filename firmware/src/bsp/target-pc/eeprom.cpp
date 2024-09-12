#include <iostream>

#include "bsp/eeprom.hpp"

namespace bsp::eeprom {

/// @section Interface implementation

EepromResult init() {
    std::cout << "bsp::eeprom::init() called" << std::endl;
    return OK;
}

}
