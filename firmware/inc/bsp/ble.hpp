#pragma once

#include <cstdint>
#include <functional>

namespace bsp::ble {

/// @section Constants

constexpr uint8_t receive_packet_size = 12;
constexpr uint8_t max_packet_size = 20;

/// @section Custom types

enum BleHeader : uint8_t {
    BLE_STOP_SIG = 0x00,
    BLE_REQUEST_DATA = 0x01,
    BLE_UPDATE_PARAMETERS = 0x02,
    BLE_BUTTON_1_SHORT = 0x03,
    BLE_BUTTON_2_SHORT = 0x04,
    BLE_BUTTON_1_LONG = 0x05,
    BLE_BUTTON_2_LONG = 0x06,
};

/// @brief callback function for BLE received data
typedef std::function<void(BleHeader)> BleCallback;

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
