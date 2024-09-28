/**
 * @file analog_sensors.hpp
 * @brief Interface for analog sensors, this includes all 4 IR distance sensors
 * a battery sensor and a current sensor
 */

#pragma once

#include <cstdint>

namespace bsp::analog_sensors {

/// @section Custom types

enum SensingDirection {
    RIGHT = 0,
    FRONT_LEFT = 1,
    FRONT_RIGHT = 2,
    LEFT = 3
};

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

uint32_t ir_reading(SensingDirection direction);
bool ir_reading_wall(SensingDirection direction);

}
