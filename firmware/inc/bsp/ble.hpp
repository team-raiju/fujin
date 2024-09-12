#pragma once

#include <cstdint>

namespace bsp::ble {

/// @section Constants

#define BLE_RECEIVE_PACKET_SIZE 12
#define BLE_MAX_PACKET_SIZE 20

/// @section Custom types

/// @brief callback function for BLE received data
typedef void (*bsp_uart_ble_callback_t)(uint8_t* ble_data, uint8_t rcv_size);

/// @section Interface definition

/// @brief Initializes BLE peripheral
/// @note Doesn't need to be called directly as bsp_init() already calls this
void init(void);

/// @brief Start BLE, listening for received data
void start(void);

/// @brief Stop BLE
void stop(void);

/// @brief Send data through BLE
/// @param data Data to be sent
/// @param size Data length
void transmit(uint8_t* data, uint8_t size);

/// @brief Register a callback to be called when data is received
/// @param callback_function callback function for BLE received data
void register_callback(bsp_uart_ble_callback_t callback);

}
