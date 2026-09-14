#include <iostream>

#include "bsp/analog_sensors.hpp"
#include "bsp/debug.hpp"

namespace bsp::analog_sensors {

static bsp_analog_ready_callback_t reading_ready_callback;

static uint32_t dummy_readings[4];
static float dummy_distances[4];

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

float* ir_latest_distance(void) {
    return dummy_distances;
}

uint32_t battery_latest_reading(void) {
    return 0;
}

uint32_t* current_latest_reading(void) {
    return dummy_readings;
}

bool ir_reading_wall(SensingDirection) {
    return true;
}

float ir_distance_mm(SensingDirection direction) {
    return dummy_distances[direction];
}

uint32_t ir_raw_reading(SensingDirection direction) {
    return dummy_readings[direction];
}

void enable_modulation(bool) {}

int32_t ir_side_wall_error() {
    return 0;
}


} // namespace
