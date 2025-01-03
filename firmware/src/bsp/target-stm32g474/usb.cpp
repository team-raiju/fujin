#include "st/hal.h"

#include "bsp/usb.hpp"

namespace bsp::usb {

/// @section Interface implementation

void init(void) {
    MX_USB_Device_Init();
}

} // namespace
