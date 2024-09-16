#include <iostream>

#include "bsp/analog_sensors.hpp"
#include "bsp/debug.hpp"

namespace bsp::analog_sensors {

static bsp_analog_ready_callback_t reading_ready_callback;

static uint32_t dummy_readings[4];

/// @section Interface implementation

void init(void) {}

void start(void) {}

void stop(void) {}

void register_callback(bsp_analog_ready_callback_t callback) {
    reading_ready_callback = callback;
}

uint32_t* ir_latest_reading(void) {
    return dummy_readings;
}

uint32_t battery_latest_reading(void) {
    return 0;
}

uint32_t* current_latest_reading(void) {
    return dummy_readings;
}

} // namespace
