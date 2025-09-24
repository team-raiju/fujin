#pragma once

#include <cstdint>
#include <functional>

namespace bsp::ble {

/// @section Constants

constexpr uint8_t header = 0xFF;
constexpr uint8_t receive_packet_size = 20;
constexpr uint8_t max_packet_size = 20;

/// @section Custom types

enum BlePacketType : uint8_t {
    Command = 0x00,
    RequestParameters = 0x01,
    UpdateParameters = 0x02,
    SensorData = 0x03,
    BatteryData = 0x04,
    MazeData = 0x05,
    RequestMovementParameters = 0x06,
    UpdateMovementParameters = 0x07,
};

enum BleCommands : uint8_t {
    Stop = 0x00,
    Button1Short = 0x01,
    Button2Short = 0x02,
    Button1Long = 0x03,
    Button2Long = 0x04,
    ButtonMovementParameters = 0x05,
};

enum ForwardParamID : uint8_t {
    MAX_SPEED = 0x00,
    ACCELERATION = 0x01,
    DECELERATION = 0x02,
    TARGET_TRAVEL_CM = 0x03,
};

enum TurnParamID : uint8_t {
    START = 0x00,
    END = 0x01,
    TURN_LINEAR_SPEED = 0x02,
    ANGULAR_ACCEL = 0x03,
    MAX_ANGULAR_SPEED = 0x04,
    ANGLE_TO_TURN = 0x05,
    T_START_DECCEL = 0x06,
    T_STOP = 0x07,
    SIGN = 0x08,
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

void lock_config_rcv();

void unlock_config_rcv();

bool is_config_locked();

}
