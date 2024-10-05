#pragma once

#include <cstdint>
#include <functional>

namespace bsp::ble {

/// @section Constants

constexpr uint8_t header = 0xFF;
constexpr uint8_t receive_packet_size = 12;
constexpr uint8_t max_packet_size = 20;

/// @section Custom types

enum BlePacketType : uint8_t {
    Command = 0x00,
    RequestData = 0x01,
    UpdateParameters = 0x02,
    MazeData = 0x05,
};

enum BleCommands : uint8_t {
    Stop = 0x00,
    Button1Short = 0x01,
    Button2Short = 0x02,
    Button1Long = 0x03,
    Button2Long = 0x04,
};

/// @brief callback function for BLE received data
typedef std::function<void(uint8_t[receive_packet_size])> BleCallback;

/// @section Interface definition

/// @brief Initializes BLE peripheral
/// @note Doesn't need to be called directly as bsp_init() already calls this
void init();

/// @brief Start BLE, listening for received data
void start();

/// @brief Stop BLE
void stop();

/// @brief Send data through BLE
/// @param data Data to be sent
/// @param size Data length
void transmit(uint8_t* data, uint8_t size);

/// @brief Register a callback to be called when data is received
/// @param callback_function callback function for BLE received data
void register_callback(BleCallback callback);

}
