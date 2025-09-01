#include <cstdio>
#include <utility>

#include "bsp/eeprom.hpp"
#include "bsp/timers.hpp"
#include "services/config.hpp"
#include "utils/math.hpp"

namespace services {

static bool write_default = false;

float Config::fan_speed = 0.0; //[0-1000]

float Config::angular_kp = 0.10;
float Config::angular_ki = 0.010;
float Config::angular_kd = 0.0;

float Config::wall_kp = 0.0025;
float Config::wall_ki = 0.0;
float Config::wall_kd = 0.0050;

float Config::linear_vel_kp = 8.0;
float Config::linear_vel_ki = 0.11;
float Config::linear_vel_kd = 0.0;

float Config::diagonal_walls_kp = 0.005;
float Config::diagonal_walls_ki = 0;
float Config::diagonal_walls_kd = 0.005;

float Config::min_move_speed = 0.05; // [m/s]

float Config::ir_wall_dist_ref_right = 1360;
float Config::ir_wall_dist_ref_front_left = 150;
float Config::ir_wall_dist_ref_front_right = 150;
float Config::ir_wall_dist_ref_left = 774;

float Config::ir_wall_control_th_right = 1000;
float Config::ir_wall_control_th_front_left = 250;
float Config::ir_wall_control_th_front_right = 250;
float Config::ir_wall_control_th_left = 750;

float Config::ir_wall_detect_th_right = 1200;
float Config::ir_wall_detect_th_front_left = 600;
float Config::ir_wall_detect_th_front_right = 800;
float Config::ir_wall_detect_th_left = 650;

// All params
static std::pair<float*, bsp::eeprom::param_addresses_t> params[] = {
    {&Config::fan_speed, bsp::eeprom::ADDR_FAN_SPEED},
    {&Config::angular_kp, bsp::eeprom::ADDR_ANGULAR_KP},
    {&Config::angular_ki, bsp::eeprom::ADDR_ANGULAR_KI},
    {&Config::angular_kd, bsp::eeprom::ADDR_ANGULAR_KD},
    {&Config::wall_kp, bsp::eeprom::ADDR_WALL_KP},
    {&Config::wall_ki, bsp::eeprom::ADDR_WALL_KI},
    {&Config::wall_kd, bsp::eeprom::ADDR_WALL_KD},
    {&Config::linear_vel_kp, bsp::eeprom::ADDR_LINEAR_VEL_KP},
    {&Config::linear_vel_ki, bsp::eeprom::ADDR_LINEAR_VEL_KI},
    {&Config::linear_vel_kd, bsp::eeprom::ADDR_LINEAR_VEL_KD},
    {&Config::diagonal_walls_kp, bsp::eeprom::ADDR_DIAGONAL_WALLS_KP},
    {&Config::diagonal_walls_ki, bsp::eeprom::ADDR_DIAGONAL_WALLS_KI},
    {&Config::diagonal_walls_kd, bsp::eeprom::ADDR_DIAGONAL_WALLS_KD},
    {&Config::min_move_speed, bsp::eeprom::ADDR_MIN_MOVE_SPEED},
    {&Config::ir_wall_dist_ref_right, bsp::eeprom::ADDR_IR_WALL_DIST_REF_RIGHT},
    {&Config::ir_wall_dist_ref_front_left, bsp::eeprom::ADDR_IR_WALL_DIST_REF_FRONT_LEFT},
    {&Config::ir_wall_dist_ref_front_right, bsp::eeprom::ADDR_IR_WALL_DIST_REF_FRONT_RIGHT},
    {&Config::ir_wall_dist_ref_left, bsp::eeprom::ADDR_IR_WALL_DIST_REF_LEFT},
    {&Config::ir_wall_control_th_right, bsp::eeprom::ADDR_IR_WALL_CONTROL_TH_RIGHT},
    {&Config::ir_wall_control_th_front_left, bsp::eeprom::ADDR_IR_WALL_CONTROL_TH_FRONT_LEFT},
    {&Config::ir_wall_control_th_front_right, bsp::eeprom::ADDR_IR_WALL_CONTROL_TH_FRONT_RIGHT},
    {&Config::ir_wall_control_th_left, bsp::eeprom::ADDR_IR_WALL_CONTROL_TH_LEFT},
    {&Config::ir_wall_detect_th_right, bsp::eeprom::ADDR_IR_WALL_DETECT_TH_RIGHT},
    {&Config::ir_wall_detect_th_front_left, bsp::eeprom::ADDR_IR_WALL_DETECT_TH_FRONT_LEFT},
    {&Config::ir_wall_detect_th_front_right, bsp::eeprom::ADDR_IR_WALL_DETECT_TH_FRONT_RIGHT},
    {&Config::ir_wall_detect_th_left, bsp::eeprom::ADDR_IR_WALL_DETECT_TH_LEFT}};

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
