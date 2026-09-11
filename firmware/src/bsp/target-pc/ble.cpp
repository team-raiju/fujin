#include "bsp/ble.hpp"

namespace bsp::ble {

static BleCallback external_callback = nullptr;
static bool config_locked = false;

void init() {}
void start(void) {}
void stop(void) {}

void transmit(uint8_t*, uint8_t) {}

void register_callback(BleCallback callback) {
    external_callback = callback;
}

void lock_config_rcv() {
    config_locked = true;
}

void unlock_config_rcv() {
    config_locked = false;
}

bool is_config_locked() {
    return config_locked;
}

void received() {
    // Stub for PC simulation
}

} // namespace bsp::ble
