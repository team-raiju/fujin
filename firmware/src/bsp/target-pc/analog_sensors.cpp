#include "bsp/analog_sensors.hpp"

namespace bsp::analog_sensors {

static bsp_analog_ready_callback_t reading_ready_callback = nullptr;
static uint32_t dummy_readings[4] = {0, 0, 0, 0};

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
    return 3000;
}

uint32_t* current_latest_reading(void) {
    return dummy_readings;
}

float battery_latest_reading_mv(void) {
    return 3300.0f;
}

float battery_latest_reading_volts(void) {
    return 3.3f;
}

bool battery_low() {
    return false;
}

bool ir_reading_wall(SensingDirection direction) {
    (void)direction;
    return false;
}

uint32_t ir_reading(SensingDirection direction) {
    return dummy_readings[static_cast<size_t>(direction)];
}

SensingStatus ir_get_sensing_status() {
    return SensingStatus{false, false, false};
}

int32_t ir_side_wall_error() {
    return 0;
}

int32_t ir_diagonal_error() {
    return 0;
}

bool ir_wall_control_valid(SensingDirection direction) {
    (void)direction;
    return false;
}

void enable_modulation(bool enable) {
    (void)enable;
}

} // namespace bsp::analog_sensors
