/**
 * @file analog_sensors.hpp
 * @brief Interface for analog sensors, this includes all 4 IR distance sensors
 * a battery sensor and a current sensor
 */

#pragma once

#include <array>
#include <cstdint>

namespace bsp::analog_sensors {

/// @section Custom types

enum SensingDirection {
    RIGHT = 0,
    FRONT_LEFT = 1,
    FRONT_RIGHT = 2,
    LEFT = 3,
};

struct SensingStatus {
    bool front_seeing;
    bool right_seeing;
    bool left_seeing;
};

struct SensingPattern {
    float L;
    float FL;
    float FR;
    float R;
};

SensingPattern get_wall_pattern(uint8_t index);
void set_wall_pattern(uint8_t index, const SensingPattern& pattern);
void reset_wall_pattern(uint8_t index);
void reset_all_wall_patterns();
const std::array<SensingPattern, 8>& get_all_wall_patterns();

typedef void (*bsp_analog_ready_callback_t)(void);

/// @section Interface definition

void init(void);
void start(void);
void stop(void);

/// @brief Register a callback to be called when a new reading is available
void register_callback(bsp_analog_ready_callback_t callback);

uint32_t* ir_latest_reading(void);
float* ir_latest_distance(void);
uint32_t battery_latest_reading(void);
uint32_t* current_latest_reading(void);
float battery_latest_reading_mv(void);
float battery_latest_reading_volts(void);
bool battery_low();

/// @brief Alias for ir_reading: returns sensor reading distance in mm
float ir_distance_mm(SensingDirection direction);
/// @brief Returns raw ADC sensor reading
uint32_t ir_raw_reading(SensingDirection direction);

bool ir_reading_wall(SensingDirection direction);

/// @brief compares the readings to a known pattern and calculates the sensing status
SensingStatus ir_get_sensing_status();

int32_t ir_side_wall_error();
int32_t ir_diagonal_error();
bool ir_wall_control_valid(SensingDirection direction);
void enable_modulation(bool enable = true);

struct IrCalibParams {
    float a;
    float b;
    float c;
};

IrCalibParams get_calib_params(SensingDirection direction);
void set_calib_params(SensingDirection direction, const IrCalibParams& params);
void reset_calib_params(SensingDirection direction);
void reset_all_calib_params();
float raw_to_distance_mm(SensingDirection direction, uint32_t raw);

}
