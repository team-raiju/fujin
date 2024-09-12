#include "st/hal.h"

#include "bsp/imu.hpp"

namespace bsp::imu {

/// @section Interface implementation

void init() {
    MX_I2C2_Init();
}

} // namespace
