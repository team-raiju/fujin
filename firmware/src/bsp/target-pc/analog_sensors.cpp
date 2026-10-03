#include <iostream>
#include <algorithm>
#include <cmath>

#include "bsp/analog_sensors.hpp"
#include "services/config.hpp"
#include "amaterasu/amaterasu.hpp"

namespace bsp::analog_sensors {

namespace {
constexpr std::array<SensingPattern, 8> default_ir_wall_patterns = {{
    {156.157f, 144.226f, 129.702f, 129.025f}, // F-L-R
    {168.937f, 146.920f, 135.949f, 192.362f}, // F-L
    {234.924f, 158.840f, 138.945f, 129.025f}, // F-R
    {230.265f, 153.131f, 135.949f, 194.181f}, // F
    {174.572f, 270.000f, 270.000f, 129.025f}, // L-R
    {174.572f, 270.000f, 270.000f, 194.181f}, // L
    {228.042f, 270.000f, 270.000f, 141.274f}, // R
    {270.000f, 270.000f, 270.000f, 208.335f}  // None
}};

static std::array<SensingPattern, 8> ir_wall_patterns = default_ir_wall_patterns;

static inline size_t direction_to_amaterasu_idx(SensingDirection dir) {
    switch (dir) {
    case SensingDirection::LEFT: return 0;
    case SensingDirection::FRONT_LEFT: return 1;
    case SensingDirection::FRONT_RIGHT: return 2;
    case SensingDirection::RIGHT: return 3;
    default: return 0;
    }
}
}

static bsp_analog_ready_callback_t reading_ready_callback = nullptr;

static uint32_t dummy_readings[4];
static float dummy_distances[4];
static uint32_t cached_readings[4] = {0, 0, 0, 0};

/// @section Interface implementation

void init(void) {}
void start(void) {}
void stop(void) {}

void register_callback(bsp_analog_ready_callback_t callback) {
    reading_ready_callback = callback;
}

static uint32_t ir_reading(SensingDirection direction) {
    size_t idx = direction_to_amaterasu_idx(direction);
    uint16_t adc = amaterasu::get_ir_raw_adc(idx);
    if (adc > 0) {
        return adc;
    }

    // If simulator only provided distance in mm, approximate raw ADC reading
    float dist_mm = amaterasu::get_ir_distance_mm(idx);
    if (dist_mm <= 0.0f || dist_mm > 300.0f) {
        return 0;
    }

    switch (direction) {
    case SensingDirection::LEFT:
        if (dist_mm > 160.0f) return 0;
        return static_cast<uint32_t>(std::clamp(services::Config::ir_wall_dist_ref_left * (135.0f / std::max(dist_mm, 10.0f)), 0.0f, 4095.0f));
    case SensingDirection::RIGHT:
        if (dist_mm > 160.0f) return 0;
        return static_cast<uint32_t>(std::clamp(services::Config::ir_wall_dist_ref_right * (135.0f / std::max(dist_mm, 10.0f)), 0.0f, 4095.0f));
    case SensingDirection::FRONT_LEFT:
        if (dist_mm > 185.0f) return 50;
        return static_cast<uint32_t>(std::clamp(800.0f * (150.0f / std::max(dist_mm, 10.0f)), 0.0f, 4095.0f));
    case SensingDirection::FRONT_RIGHT:
        if (dist_mm > 185.0f) return 200;
        return static_cast<uint32_t>(std::clamp(1200.0f * (150.0f / std::max(dist_mm, 10.0f)), 0.0f, 4095.0f));
    default:
        return 0;
    }
}

uint32_t* ir_latest_reading(void) {
    cached_readings[0] = ir_reading(SensingDirection::RIGHT);
    cached_readings[1] = ir_reading(SensingDirection::FRONT_LEFT);
    cached_readings[2] = ir_reading(SensingDirection::FRONT_RIGHT);
    cached_readings[3] = ir_reading(SensingDirection::LEFT);
    return cached_readings;
}

float* ir_latest_distance(void) {
    return dummy_distances;
}

uint32_t* current_latest_reading(void) {
    return cached_readings;
}

uint32_t battery_latest_reading(void) {
    return static_cast<uint32_t>(battery_latest_reading_mv());
}

float battery_latest_reading_mv(void) {
    return battery_latest_reading_volts() * 1000.0f;
}

float battery_latest_reading_volts(void) {
    return amaterasu::get_battery_voltage();
}

float battery_latest_reading_mv_real(void) {
    return battery_latest_reading_mv();
}

float battery_latest_reading_volts_real(void) {
    return battery_latest_reading_volts();
}

bool battery_low() {
    return battery_latest_reading_volts() < 6.8f;
}

bool ir_is_wall_confirmed(SensingDirection) {
    return true;
}

bool ir_wall_break_condition(SensingDirection) {
    return false;
}

void ir_update_wall_hysteresis(float) {}
void ir_reset_wall_hysteresis(SensingDirection) {}
void ir_reset_all_wall_hysteresis() {}
float ir_slope_value(SensingDirection) {
    return 0.0f;
}

float ir_distance_mm(SensingDirection direction) {
    size_t idx = direction_to_amaterasu_idx(direction);
    float dist = amaterasu::get_ir_distance_mm(idx);
    if (dist > 0.0f) {
        return dist;
    }
    return dummy_distances[direction];
}

uint32_t ir_raw_reading(SensingDirection direction) {
    return ir_reading(direction);
}

bool ir_reading_wall(SensingDirection direction) {
    uint32_t val = ir_reading(direction);
    switch (direction) {
    case SensingDirection::RIGHT:
        return val > services::Config::ir_wall_detect_th_right;
    case SensingDirection::FRONT_LEFT:
        return val > services::Config::ir_wall_detect_th_front_left;
    case SensingDirection::FRONT_RIGHT:
        return val > services::Config::ir_wall_detect_th_front_right;
    case SensingDirection::LEFT:
        return val > services::Config::ir_wall_detect_th_left;
    default:
        return false;
    }
}

bool ir_wall_control_valid(SensingDirection direction) {
    uint32_t val = ir_reading(direction);
    switch (direction) {
    case SensingDirection::LEFT:
        return (val > services::Config::ir_wall_control_th_left);
    case SensingDirection::RIGHT:
        return (val > services::Config::ir_wall_control_th_right);
    default:
        return false;
    }
}

bool ir_diagonal_control_valid(SensingDirection) {
    return false;
}

bool ir_start_condition() {
    return false;
}

int32_t ir_side_wall_error() {
    int32_t left_error = ir_reading(SensingDirection::LEFT) - services::Config::ir_wall_dist_ref_left;
    int32_t right_error = ir_reading(SensingDirection::RIGHT) - services::Config::ir_wall_dist_ref_right;

    int32_t ir_error = 0;
    if (ir_wall_control_valid(SensingDirection::LEFT) && ir_wall_control_valid(SensingDirection::RIGHT)) {
        ir_error = left_error - right_error;
    } else if (ir_wall_control_valid(SensingDirection::LEFT)) {
        ir_error = static_cast<int32_t>(1.5f * left_error);
    } else if (ir_wall_control_valid(SensingDirection::RIGHT)) {
        ir_error = static_cast<int32_t>(-1.5f * right_error);
    } else {
        ir_error = 0;
    }

    return ir_error;
}

int32_t ir_diagonal_error() {
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

SensingStatus ir_get_sensing_status() {
    SensingPattern current_pattern;
    current_pattern.FL = ir_reading(FRONT_LEFT);
    current_pattern.FR = ir_reading(FRONT_RIGHT);
    current_pattern.L = ir_reading(LEFT);
    current_pattern.R = ir_reading(RIGHT);

    // Compares the current value to all references and find the closest match
    uint32_t min_diff = 100000;
    uint8_t pattern_idx = 7;
    for (uint8_t i = 0; i < ir_wall_patterns.size(); i++) {
        uint32_t diff = std::abs((int32_t)current_pattern.FL - (int32_t)ir_wall_patterns[i].FL) +
                        std::abs((int32_t)current_pattern.FR - (int32_t)ir_wall_patterns[i].FR) +
                        std::abs((int32_t)current_pattern.L - (int32_t)ir_wall_patterns[i].L) +
                        std::abs((int32_t)current_pattern.R - (int32_t)ir_wall_patterns[i].R);
        if (diff < min_diff) {
            min_diff = diff;
            pattern_idx = i;
        }
    }

    SensingStatus status = {false, false, false};

    switch (pattern_idx) {
    case 0:
        // F-L-R
        status.front_seeing = true;
        status.left_seeing = true;
        status.right_seeing = true;
        break;
    case 1:
        // F-L
        status.front_seeing = true;
        status.left_seeing = true;
        break;
    case 2:
        // F-R
        status.front_seeing = true;
        status.right_seeing = true;
        break;
    case 3:
        // F
        status.front_seeing = true;
        break;
    case 4:
        // L-R
        status.left_seeing = true;
        status.right_seeing = true;
        break;
    case 5:
        // L
        status.left_seeing = true;
        break;
    case 6:
        // R
        status.right_seeing = true;
        break;
    case 7:
        // None
        break;
    }

    static SensingStatus last_status{false, false, false};
    if (status.front_seeing != last_status.front_seeing ||
        status.right_seeing != last_status.right_seeing ||
        status.left_seeing != last_status.left_seeing) {
        last_status = status;
        std::printf("[sensing] pattern=%u status: f=%d r=%d l=%d (ADCs: FL=%u FR=%u L=%u R=%u)\r\n",
                    pattern_idx, status.front_seeing, status.right_seeing, status.left_seeing,
                    current_pattern.FL, current_pattern.FR, current_pattern.L, current_pattern.R);
    }

    return status;
}

void enable_modulation(bool enable) {
    (void)enable;
}

} // namespace bsp::analog_sensors
