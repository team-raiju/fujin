#include <utility>
#include <cstdio>

#include "bsp/timers.hpp"
#include "bsp/eeprom.hpp"
#include "services/config.hpp"
#include "utils/math.hpp"

namespace services {

static bool write_default = false;

float Config::angular_kp = 0.16;
float Config::angular_ki = 0.008;
float Config::angular_kd = 0;

float Config::wall_kp = 0.0035;
float Config::wall_ki = 0;
float Config::wall_kd = 0.006;

float Config::min_move_speed = 0.1;         // [m/s]
float Config::min_turn_speed = 0.1;         // [m/s]
float Config::fix_position_speed = 0.3;     // [m/s]
float Config::search_speed = 0.5;           // [m/s]
float Config::angular_speed = 5.585;         // [rad/s]
float Config::run_speed = 0.75;             // [m/s]

float Config::linear_acceleration = 2.0;     // [m/s^2]
float Config::angular_acceleration = 100.0; // [rad/s^2]

float Config::linear_vel_kp = 8.0;
float Config::linear_vel_ki = 0.11;
float Config::linear_vel_kd = 0;

float Config::diagonal_walls_kp = 0.015;
float Config::diagonal_walls_ki = 0;
float Config::diagonal_walls_kd = 0.015;

// All params
static std::pair<float*, bsp::eeprom::param_addresses_t> params[] = {
    {&Config::angular_kp, bsp::eeprom::ADDR_ANGULAR_KP},
    {&Config::angular_ki, bsp::eeprom::ADDR_ANGULAR_KI},
    {&Config::angular_kd, bsp::eeprom::ADDR_ANGULAR_KD},
    {&Config::wall_kp, bsp::eeprom::ADDR_WALL_KP},
    {&Config::wall_ki, bsp::eeprom::ADDR_WALL_KI},
    {&Config::wall_kd, bsp::eeprom::ADDR_WALL_KD},
    {&Config::min_move_speed, bsp::eeprom::ADDR_MIN_MOVE_SPEED},
    {&Config::min_turn_speed, bsp::eeprom::ADDR_MIN_TURN_SPEED},
    {&Config::fix_position_speed, bsp::eeprom::ADDR_FIX_POSITION_SPEED},
    {&Config::search_speed, bsp::eeprom::ADDR_SEARCH_SPEED},
    {&Config::angular_speed, bsp::eeprom::ADDR_ANGULAR_SPEED},
    {&Config::run_speed, bsp::eeprom::ADDR_RUN_SPEED},
    {&Config::linear_acceleration, bsp::eeprom::ADDR_LINEAR_ACCELERATION},
    {&Config::angular_acceleration, bsp::eeprom::ADDR_ANGULAR_ACCELERATION},
    {&Config::linear_vel_kp, bsp::eeprom::ADDR_LINEAR_VEL_KP},
    {&Config::linear_vel_ki, bsp::eeprom::ADDR_LINEAR_VEL_KI},
    {&Config::linear_vel_kd, bsp::eeprom::ADDR_LINEAR_VEL_KD},
    {&Config::diagonal_walls_kp, bsp::eeprom::ADDR_DIAGONAL_WALLS_KP},
    {&Config::diagonal_walls_ki, bsp::eeprom::ADDR_DIAGONAL_WALLS_KI},
    {&Config::diagonal_walls_kd, bsp::eeprom::ADDR_DIAGONAL_WALLS_KD}};

union _float {
    float value;
    uint8_t raw[sizeof(float)];
    uint32_t u32;
};

void Config::init() {
    if (write_default) {
        write_default_params();
    }

    for (auto& param : params) {
        _float f;
        if (bsp::eeprom::read_u32(param.second, &f.u32) == bsp::eeprom::OK) {
            if (f.u32 == 0xFFFFFFFF) {
                continue;
            }

            *param.first = f.value;

            std::printf("%s: %f\r\n", bsp::eeprom::param_name(param.second), f.value);
            bsp::delay_ms(2);
        }
    }
}

int Config::parse_packet(uint8_t packet[bsp::ble::max_packet_size]) {
    if (packet[0] != bsp::ble::header) {
        return -1;
    }

    if (packet[1] != bsp::ble::BlePacketType::UpdateParameters) {
        return -1;
    }

    const uint8_t parameter = packet[2];

    if (parameter >= len(params)) {
        return -1;
    }

    _float f;
    for (size_t i = 0; i < sizeof(float); i++) {
        f.raw[i] = packet[3 + i];
    }

    *params[parameter].first = f.value;

    bsp::eeprom::write_u32(params[parameter].second, f.u32);

    return 0;
}

int Config::write_default_params() {
    for (auto& param : params) {
        _float f;
        f.value = *param.first;
        if (bsp::eeprom::write_u32(param.second, f.u32) != bsp::eeprom::OK) {
            return -1;
        }
        bsp::delay_ms(5);
    }

    return 0;
}

void Config::send_parameters() {
    uint8_t packet[7] = {0};
    packet[0] = bsp::ble::header;
    packet[1] = bsp::ble::BlePacketType::RequestParameters;

    for (size_t i = 0; i < len(params); i++) {
        _float f;
        f.value = *params[i].first;

        packet[2] = i;
        for (size_t j = 0; j < sizeof(float); j++) {
            packet[3 + j] = f.raw[j];
        }

        bsp::ble::transmit(packet, sizeof(packet));

        bsp::delay_ms(50);
    }
}

}
