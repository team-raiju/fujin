#include <iostream>

#include "bsp/ble.hpp"
#include "bsp/debug.hpp"

namespace bsp::ble {

static BleCallback external_callback = NULL;

void init() {}

void start(void) {}

void stop(void) {}

void transmit(uint8_t*, uint8_t) {}

void register_callback(BleCallback callback) {
    external_callback = callback;
}

void received() {
    if (external_callback) {
        external_callback(NULL, 0);
    }
}

} // namespace
