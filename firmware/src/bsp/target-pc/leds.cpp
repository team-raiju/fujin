#include <iostream>

#include "bsp/leds.hpp"

namespace bsp::leds {

/// @section Interface implementation

void init(void) {}

void indication_on(void) {}

void indication_off(void) {}

void indication_toggle(void) {}

void stripe_set(uint8_t, uint8_t, uint8_t, uint8_t) {}

} // namespace
