#include "services/config.hpp"

namespace services {

float Config::kp = 0;
float Config::ki = 0;
float Config::kd = 0;

float Config::min_move_speed = 16;
float Config::min_turn_speed = 16;
float Config::fix_position_speed = 20;
float Config::search_speed = 50;
float Config::angular_speed = 30;
float Config::run_speed = 100;

float Config::linear_acceleration = 0.05;
float Config::angular_acceleration = 0.05;

// All params
static float* params[] = {
    &Config::kp,
    &Config::ki,
    &Config::kd,
    &Config::min_move_speed,
    &Config::min_turn_speed,
    &Config::fix_position_speed,
    &Config::search_speed,
    &Config::angular_speed,
    &Config::run_speed,
    &Config::linear_acceleration,
    &Config::angular_acceleration,
};

union _float {
    float value;
    uint8_t raw[sizeof(float)];
};

void Config::parse_packet(uint8_t packet[bsp::ble::max_packet_size]) {
    if (packet[0] != bsp::ble::header) {
        return;
    }

    if (packet[1] != bsp::ble::BlePacketType::UpdateParameters) {
        return;
    }

    const uint8_t parameter = packet[2];

    if (parameter >= 10) {
        return;
    }

    _float f;
    for (size_t i = 0; i < sizeof(float); i++) {
        f.raw[(sizeof(float) - 1) - i] = packet[3 + i];
    }

    *params[parameter] = f.value;
}

}
