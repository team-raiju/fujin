#include "st/hal.h"

#include "bsp/eeprom.hpp"

namespace bsp::eeprom {

/// @section Interface implementation

EepromResult init() {
    MX_I2C3_Init();
    return OK;
}

}
