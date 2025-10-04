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
    uint32_t L;
    uint32_t FL;
    uint32_t FR;
    uint32_t R;
};
/// @brief Sensor raw values in every wall combination
constexpr std::array<SensingPattern, 8> ir_wall_patterns = {{
    {1119, 850, 1290, 1460}, // F-L-R
    {960, 820, 1200, 880},  // F-L
    {500, 700, 1160, 1460}, // F-R
    {520, 755, 1200, 870},  // F
    {900, 67, 246, 1460},   // L-R
    {900, 28, 230, 870},     // L
    {530, 35, 261, 1300},   // R
    {180, 66, 238, 800}     // None
}};

typedef void (*bsp_analog_ready_callback_t)(void);

/// @section Interface definition

void init(void);
void start(void);
void stop(void);

/// @brief Register a callback to be called when a new reading is available
void register_callback(bsp_analog_ready_callback_t callback);

uint32_t* ir_latest_reading(void);
uint32_t battery_latest_reading(void);
uint32_t* current_latest_reading(void);
float battery_latest_reading_mv(void);
bool battery_low();

uint32_t ir_reading(SensingDirection direction);
bool ir_reading_wall(SensingDirection direction);

/// @brief compares the readings to a known pattern and calculates the sensing status
SensingStatus ir_get_sensing_status();

int32_t ir_side_wall_error();
int32_t ir_diagonal_error();
bool ir_wall_control_valid(SensingDirection direction);
void enable_modulation(bool enable = true);

}
