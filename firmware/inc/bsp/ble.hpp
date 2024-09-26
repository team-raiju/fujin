#pragma once

#include <cstdint>
#include <functional>

namespace bsp::ble {

/// @section Constants

#define BLE_RECEIVE_PACKET_SIZE 12
#define BLE_MAX_PACKET_SIZE 20

/// @section Custom types

enum ble_header {
    BLE_STOP_SIG,
    BLE_REQUEST_DATA,
    BLE_UPDATE_PARAMETERS,
    BLE_BUTTON_1_SHORT,
    BLE_BUTTON_2_SHORT,
    BLE_BUTTON_1_LONG,
    BLE_BUTTON_2_LONG,
};

/// @brief callback function for BLE received data
typedef std::function<void(ble_header)> BleCallback;

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
void register_callback(BleCallback callback);

}
