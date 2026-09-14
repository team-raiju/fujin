#include <iostream>

#include "bsp/analog_sensors.hpp"
#include "bsp/debug.hpp"

namespace bsp::analog_sensors {

namespace {
constexpr std::array<SensingPattern, 8> default_ir_wall_patterns = {{
    {1119, 850, 1290, 1460}, // F-L-R
    {960, 820, 1200, 880},   // F-L
    {500, 700, 1160, 1460},  // F-R
    {520, 755, 1200, 870},   // F
    {900, 67, 246, 1460},    // L-R
    {900, 28, 230, 870},     // L
    {530, 35, 261, 1300},    // R
    {180, 66, 238, 800}      // None
}};

static std::array<SensingPattern, 8> ir_wall_patterns = default_ir_wall_patterns;
}

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

IrCalibParams get_calib_params(SensingDirection) {
    return {0, 0, 0};
}

void set_calib_params(SensingDirection, const IrCalibParams&) {}
void reset_calib_params(SensingDirection) {}
void reset_all_calib_params() {}

float raw_to_distance_mm(SensingDirection, uint32_t) {
    return 0.0f;
}

SensingPattern get_wall_pattern(uint8_t index) {
    if (index < 8) return ir_wall_patterns[index];
    return default_ir_wall_patterns[0];
}

void set_wall_pattern(uint8_t index, const SensingPattern& pattern) {
    if (index < 8) ir_wall_patterns[index] = pattern;
}

void reset_wall_pattern(uint8_t index) {
    if (index < 8) ir_wall_patterns[index] = default_ir_wall_patterns[index];
}

void reset_all_wall_patterns() {
    ir_wall_patterns = default_ir_wall_patterns;
}

const std::array<SensingPattern, 8>& get_all_wall_patterns() {
    return ir_wall_patterns;
}

} // namespace
