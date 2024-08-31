/**
 * @file analog_sensors.h
 * @brief Interface for analog sensors, this includes all 4 IR distance sensors
 * a battery sensor and a current sensor
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/// @section Custom types

typedef void (*bsp_analog_ready_callback_t)(void);

/// @section Interface definition

void bsp_analog_sensors_init(void);
void bsp_analog_sensors_start(void);
void bsp_analog_sensors_stop(void);

/// @brief Register a callback to be called when a new reading is available
void bsp_analog_sensors_register_callback(bsp_analog_ready_callback_t callback);

uint32_t* bsp_analog_sensors_ir_latest_reading(void);
uint32_t bsp_analog_sensors_battery_latest_reading(void);
uint32_t bsp_analog_sensors_current_latest_reading(void);

#ifdef __cplusplus
}
#endif
