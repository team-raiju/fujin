#pragma once

namespace bsp::usb {

/// @section Interface definition
void init(void);

void reset_on_host(void);

void clock_config(void);

} // namespace
