#include <iostream>

#include "bsp/analog_sensors.hpp"
#include "bsp/debug.hpp"

namespace bsp::analog_sensors {

namespace {
constexpr std::array<SensingPattern, 8> default_ir_wall_patterns = {{
    {156.157f, 144.226f, 129.702f, 129.025f}, // F-L-R
    {168.937f, 146.920f, 135.949f, 192.362f}, // F-L
    {234.924f, 158.840f, 138.945f, 129.025f}, // F-R
    {230.265f, 153.131f, 135.949f, 194.181f}, // F
    {174.572f, 300.000f, 300.000f, 129.025f}, // L-R
    {174.572f, 300.000f, 300.000f, 194.181f}, // L
    {228.042f, 300.000f, 300.000f, 141.274f}, // R
    {300.000f, 300.000f, 300.000f, 208.335f}  // None
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
