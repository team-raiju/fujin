#include "services/config.hpp"
#include "bsp/eeprom.hpp"
#include <utility>

namespace services {

float Config::angular_kp = 0;
float Config::angular_ki = 0;
float Config::angular_kd = 0;

float Config::min_move_speed = 16;
float Config::min_turn_speed = 16;
float Config::fix_position_speed = 20;
float Config::search_speed = 50;
float Config::angular_speed = 30;
float Config::run_speed = 100;

float Config::linear_acceleration = 0.05;
float Config::angular_acceleration = 0.05;

// All params
static std::pair<float*, bsp::eeprom::param_addresses_t> params[] = {
    {&Config::angular_kp, bsp::eeprom::ADDR_ANGULAR_KP},
    {&Config::angular_ki, bsp::eeprom::ADDR_ANGULAR_KI},
    {&Config::angular_kd, bsp::eeprom::ADDR_ANGULAR_KD},
    {&Config::min_move_speed, bsp::eeprom::ADDR_MIN_MOVE_SPEED},
    {&Config::min_turn_speed, bsp::eeprom::ADDR_MIN_TURN_SPEED},
    {&Config::fix_position_speed, bsp::eeprom::ADDR_FIX_POSITION_SPEED},
    {&Config::search_speed, bsp::eeprom::ADDR_SEARCH_SPEED},
    {&Config::angular_speed, bsp::eeprom::ADDR_ANGULAR_SPEED},
    {&Config::run_speed, bsp::eeprom::ADDR_RUN_SPEED},
    {&Config::linear_acceleration, bsp::eeprom::ADDR_LINEAR_ACCELERATION},
    {&Config::angular_acceleration, bsp::eeprom::ADDR_ANGULAR_ACCELERATION}};

union _float {
    float value;
    uint8_t raw[sizeof(float)];
    uint32_t u32;
};

int Config::init() {
    for (auto& param : params) {
        uint8_t data[sizeof(float)];
        if (bsp::eeprom::read_u32(param.second, (uint32_t*)data) == bsp::eeprom::OK) {
            if (data[0] == 0xff && data[1] == 0xff && data[2] == 0xff && data[3] == 0xff) {
                continue;
            }
            _float f;
            for (size_t i = 0; i < sizeof(float); i++) {
                f.raw[i] = data[i];
            }
            *param.first = f.value;
        }
    }
    return 0;
}

void Config::parse_packet(uint8_t packet[bsp::ble::max_packet_size]) {
    if (packet[0] != bsp::ble::header) {
        return;
    }

    if (packet[1] != bsp::ble::BlePacketType::UpdateParameters) {
        return;
    }

    const uint8_t parameter = packet[2];

    if (parameter >= sizeof(params) / sizeof(params[0])) {
        return;
    }

    _float f;
    for (size_t i = 0; i < sizeof(float); i++) {
        f.raw[(sizeof(float) - 1) - i] = packet[3 + i];
    }

    *params[parameter].first = f.value;

    bsp::eeprom::write_u32(params[parameter].second, f.u32);
}

}
