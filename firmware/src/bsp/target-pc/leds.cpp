#include <iostream>

#include "bsp/leds.hpp"

namespace bsp::leds {

/// @section Interface implementation

void init() {}

void indication_on() {}

void indication_off() {}

void indication_toggle() {}

void ir_emitter_on(Emitter) {}
void ir_emitter_off(Emitter) {}
void ir_emitter_all_on() {}
void ir_emitter_all_off() {}

void stripe_set(uint8_t, uint8_t, uint8_t, uint8_t) {}

} // namespace
