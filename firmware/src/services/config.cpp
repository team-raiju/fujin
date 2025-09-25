#include <cstdio>
#include <utility>

#include "bsp/eeprom.hpp"
#include "bsp/timers.hpp"
#include "services/config.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"
#include "utils/movement_params.hpp"

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

float Config::z_imu_bias = 0.0;

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
    {&Config::ir_wall_detect_th_left, bsp::eeprom::ADDR_IR_WALL_DETECT_TH_LEFT},
    {&Config::z_imu_bias, bsp::eeprom::ADDR_Z_IMU_BIAS}};

static const std::map<Movement, uint16_t> turn_address_map = {
    {Movement::TURN_RIGHT_45, bsp::eeprom::ADDR_TURN_PARAMS_RIGHT_45},
    {Movement::TURN_LEFT_45, bsp::eeprom::ADDR_TURN_PARAMS_LEFT_45},
    {Movement::TURN_RIGHT_90, bsp::eeprom::ADDR_TURN_PARAMS_RIGHT_90},
    {Movement::TURN_LEFT_90, bsp::eeprom::ADDR_TURN_PARAMS_LEFT_90},
    {Movement::TURN_RIGHT_135, bsp::eeprom::ADDR_TURN_PARAMS_RIGHT_135},
    {Movement::TURN_LEFT_135, bsp::eeprom::ADDR_TURN_PARAMS_LEFT_135},
    {Movement::TURN_RIGHT_180, bsp::eeprom::ADDR_TURN_PARAMS_RIGHT_180},
    {Movement::TURN_LEFT_180, bsp::eeprom::ADDR_TURN_PARAMS_LEFT_180},
    {Movement::TURN_RIGHT_45_FROM_45, bsp::eeprom::ADDR_TURN_PARAMS_RIGHT_45_FROM_45},
    {Movement::TURN_LEFT_45_FROM_45, bsp::eeprom::ADDR_TURN_PARAMS_LEFT_45_FROM_45},
    {Movement::TURN_RIGHT_90_FROM_45, bsp::eeprom::ADDR_TURN_PARAMS_RIGHT_90_FROM_45},
    {Movement::TURN_LEFT_90_FROM_45, bsp::eeprom::ADDR_TURN_PARAMS_LEFT_90_FROM_45},
    {Movement::TURN_RIGHT_135_FROM_45, bsp::eeprom::ADDR_TURN_PARAMS_RIGHT_135_FROM_45},
    {Movement::TURN_LEFT_135_FROM_45, bsp::eeprom::ADDR_TURN_PARAMS_LEFT_135_FROM_45},
    {Movement::TURN_AROUND, bsp::eeprom::ADDR_TURN_PARAMS_TURN_AROUND},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, bsp::eeprom::ADDR_TURN_PARAMS_RIGHT_90_SEARCH_MODE},
    {Movement::TURN_LEFT_90_SEARCH_MODE, bsp::eeprom::ADDR_TURN_PARAMS_LEFT_90_SEARCH_MODE},
    {Movement::TURN_AROUND_INPLACE, bsp::eeprom::ADDR_TURN_PARAMS_TURN_AROUND_INPLACE},
};

static const std::map<Movement, uint16_t> forward_address_map = {
    {Movement::START, bsp::eeprom::ADDR_FORWARD_PARAMS_START},
    {Movement::FORWARD, bsp::eeprom::ADDR_FORWARD_PARAMS_FOWARD},
    {Movement::DIAGONAL, bsp::eeprom::ADDR_FORWARD_PARAMS_DIAGONAL},
    {Movement::STOP, bsp::eeprom::ADDR_FORWARD_PARAMS_STOP},
    {Movement::TURN_AROUND, bsp::eeprom::ADDR_FORWARD_PARAMS_TURN_AROUND},
    {Movement::TURN_RIGHT_45_FROM_45, bsp::eeprom::ADDR_FORWARD_PARAMS_RIGHT_45_FROM_45},
    {Movement::TURN_LEFT_45_FROM_45, bsp::eeprom::ADDR_FORWARD_PARAMS_LEFT_45_FROM_45},
    {Movement::TURN_RIGHT_90_FROM_45, bsp::eeprom::ADDR_FORWARD_PARAMS_RIGHT_90_FROM_45},
    {Movement::TURN_LEFT_90_FROM_45, bsp::eeprom::ADDR_FORWARD_PARAMS_LEFT_90_FROM_45},
    {Movement::TURN_RIGHT_135_FROM_45, bsp::eeprom::ADDR_FORWARD_PARAMS_RIGHT_135_FROM_45},
    {Movement::TURN_LEFT_135_FROM_45, bsp::eeprom::ADDR_FORWARD_PARAMS_LEFT_135_FROM_45},
    {Movement::TURN_AROUND_INPLACE, bsp::eeprom::ADDR_FORWARD_PARAMS_TURN_AROUND_INPLACE},
};

union _float {
    float value;
    uint8_t raw[sizeof(float)];
    uint32_t u32;
};

void Config::init() {
    if (write_default) {
        write_default_params();
        write_all_move_params_to_eeprom();
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

    load_custom_movements_from_eeprom();
    bsp::delay_ms(5);
    load_movement_sequence_from_eeprom();
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

int Config::parse_movement_packet(uint8_t packet[bsp::ble::max_packet_size]) {
    if (packet[0] != bsp::ble::header) {
        return -1;
    }

    if (packet[1] != bsp::ble::BlePacketType::UpdateMovementParameters) {
        return -1;
    }

    uint8_t param_type = packet[2];
    Movement movement_id = static_cast<Movement>(packet[3]);
    uint8_t param_id = packet[4];

    _float f;
    for (size_t i = 0; i < sizeof(float); i++) {
        f.raw[i] = packet[5 + i];
    }
    float value = f.value;

    // Param type 0: ForwardParams
    if (param_type == 0) {
        if (forward_params_custom.find(movement_id) == forward_params_custom.end()) {
            return -1;
        }

        switch (static_cast<bsp::ble::ForwardParamID>(param_id)) {
        case bsp::ble::ForwardParamID::MAX_SPEED:
            forward_params_custom[movement_id].max_speed = value;
            break;
        case bsp::ble::ForwardParamID::ACCELERATION:
            forward_params_custom[movement_id].acceleration = value;
            break;
        case bsp::ble::ForwardParamID::DECELERATION:
            forward_params_custom[movement_id].deceleration = value;
            break;
        case bsp::ble::ForwardParamID::TARGET_TRAVEL_CM:
            forward_params_custom[movement_id].target_travel_cm = value;
            break;
        default:
            return -1;
        }
        write_forward_param_to_eeprom(movement_id);
        return 0;
    }

    // Param type 1: TurnParams
    if (param_type == 1) {
        if (turn_params_custom.find(movement_id) == turn_params_custom.end()) {
            return -1;
        }

        switch (static_cast<bsp::ble::TurnParamID>(param_id)) {
        case bsp::ble::TurnParamID::START:
            turn_params_custom[movement_id].start = value;
            break;
        case bsp::ble::TurnParamID::END:
            turn_params_custom[movement_id].end = value;
            break;
        case bsp::ble::TurnParamID::TURN_LINEAR_SPEED:
            turn_params_custom[movement_id].turn_linear_speed = value;
            break;
        case bsp::ble::TurnParamID::ANGULAR_ACCEL:
            turn_params_custom[movement_id].angular_accel = value;
            break;
        case bsp::ble::TurnParamID::MAX_ANGULAR_SPEED:
            turn_params_custom[movement_id].max_angular_speed = value;
            break;
        case bsp::ble::TurnParamID::ANGLE_TO_TURN:
            turn_params_custom[movement_id].angle_to_turn = value;
            break;
        case bsp::ble::TurnParamID::T_START_DECCEL:
            turn_params_custom[movement_id].t_start_deccel = static_cast<uint16_t>(value);
            break;
        case bsp::ble::TurnParamID::T_STOP:
            turn_params_custom[movement_id].t_stop = static_cast<uint16_t>(value);
            break;
        case bsp::ble::TurnParamID::SIGN:
            turn_params_custom[movement_id].sign = static_cast<int>(value);
            break;
        default:
            return -1;
        }
        write_turn_param_to_eeprom(movement_id);
        return 0;
    }

    return -1;
}

void Config::send_movement_parameters() {
    uint8_t packet[9] = {0};
    packet[0] = bsp::ble::header;
    packet[1] = bsp::ble::BlePacketType::RequestMovementParameters;

    auto send_param = [&](uint8_t param_type, Movement move_id, uint8_t param_id, float value) {
        packet[2] = param_type;
        packet[3] = static_cast<uint8_t>(move_id);
        packet[4] = param_id;

        _float f;
        f.value = value;
        for (size_t j = 0; j < sizeof(float); j++) {
            packet[5 + j] = f.raw[j];
        }

        bsp::ble::transmit(packet, sizeof(packet));
        bsp::delay_ms(20);
    };

    for (const auto& pair : forward_params_custom) {
        const auto& movement_id = pair.first;
        const auto& params = pair.second;
        send_param(0, movement_id, static_cast<uint8_t>(bsp::ble::ForwardParamID::MAX_SPEED), params.max_speed);
        send_param(0, movement_id, static_cast<uint8_t>(bsp::ble::ForwardParamID::ACCELERATION), params.acceleration);
        send_param(0, movement_id, static_cast<uint8_t>(bsp::ble::ForwardParamID::DECELERATION), params.deceleration);
        send_param(0, movement_id, static_cast<uint8_t>(bsp::ble::ForwardParamID::TARGET_TRAVEL_CM),
                   params.target_travel_cm);
    }

    for (const auto& pair : turn_params_custom) {
        const auto& movement_id = pair.first;
        const auto& params = pair.second;
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::START), params.start);
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::END), params.end);
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::TURN_LINEAR_SPEED),
                   params.turn_linear_speed);
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::ANGULAR_ACCEL), params.angular_accel);
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::MAX_ANGULAR_SPEED),
                   params.max_angular_speed);
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::ANGLE_TO_TURN), params.angle_to_turn);
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::T_START_DECCEL),
                   static_cast<float>(params.t_start_deccel));
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::T_STOP),
                   static_cast<float>(params.t_stop));
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::SIGN), static_cast<float>(params.sign));
    }
}

int Config::save_z_bias() {
    _float f;
    f.value = Config::z_imu_bias;

    if (bsp::eeprom::write_u32(bsp::eeprom::ADDR_Z_IMU_BIAS, f.u32) != bsp::eeprom::OK) {
        return -1;
    }

    return 0;
}

void Config::load_custom_movements_from_eeprom() {
    for (const auto& pair : turn_address_map) {
        Movement movement_id = pair.first;
        uint16_t address = pair.second;

        TurnParams params;
        bsp::eeprom::read_array(address, reinterpret_cast<uint8_t*>(&params), sizeof(TurnParams));

        turn_params_custom[movement_id] = params;
        bsp::delay_ms(10);
    }

    for (const auto& pair : forward_address_map) {
        Movement movement_id = pair.first;
        uint16_t address = pair.second;

        ForwardParams params;
        bsp::eeprom::read_array(address, reinterpret_cast<uint8_t*>(&params), sizeof(ForwardParams));
        forward_params_custom[movement_id] = params;
        bsp::delay_ms(10);
    }
}

int Config::write_turn_param_to_eeprom(Movement movement_id) {
    if (turn_address_map.find(movement_id) == turn_address_map.end()) {
        return -1;
    }

    TurnParams params = turn_params_custom[movement_id];
    uint16_t address = turn_address_map.at(movement_id);

    bsp::eeprom::write_array(address, reinterpret_cast<uint8_t*>(&params), sizeof(TurnParams));

    return 0;
}

int Config::write_forward_param_to_eeprom(Movement movement_id) {
    if (forward_address_map.find(movement_id) == forward_address_map.end()) {
        return -1;
    }

    ForwardParams params = forward_params_custom[movement_id];
    uint16_t address = forward_address_map.at(movement_id);

    bsp::eeprom::write_array(address, reinterpret_cast<uint8_t*>(&params), sizeof(ForwardParams));

    return 0;
}

int Config::write_all_move_params_to_eeprom() {

    for (const auto& pair : turn_address_map) {
        write_turn_param_to_eeprom(pair.first);
        bsp::delay_ms(20);
    }

    for (const auto& pair : forward_address_map) {
        Movement movement_id = pair.first;
        write_forward_param_to_eeprom(movement_id);
        bsp::delay_ms(20);
    }

    return 0;
}

int Config::parse_move_sequence_packet(uint8_t packet[bsp::ble::max_packet_size]) {
    if (packet[0] != bsp::ble::header) {
        return -1;
    }

    if (packet[1] != bsp::ble::BlePacketType::UpdateMoveSequence) {
        return -1;
    }

    auto navigation_service = services::Navigation::instance();
    if (!navigation_service) {
        return -1;
    }

    std::vector<std::pair<Movement, uint8_t>> moves;
    uint8_t valid_moves = 0;

    for (int i = 2; i < bsp::ble::max_packet_size; ++i) {
        uint8_t byte = packet[i];

        // Extract the 5-bit movement type and 3-bit count
        auto type = static_cast<Movement>(byte >> 3);
        uint8_t count = byte & 0x07;

        if (count == 0 || type == Movement::STOP) {
            if (type == Movement::STOP) {
                valid_moves++;
                moves.push_back({Movement::STOP, 1});
            }
            break;
        }
        valid_moves++;
        moves.push_back({type, count});
    }

    // Ensure the sequence ends with a STOP command
    if (moves.empty() || moves.back().first != Movement::STOP) {
        valid_moves++;
        moves.push_back({Movement::STOP, 1});
    }

    navigation_service->set_hardcoded_movements(moves);
    bsp::eeprom::write_array(bsp::eeprom::ADDR_MOVE_SEQUENCE_1, &packet[2], valid_moves);

    return 0;
}

void Config::send_move_sequence() {
    auto navigation = services::Navigation::instance();
    if (!navigation) {
        return;
    }

    const auto& moves = navigation->get_hardcoded_movements();

    uint8_t packet[bsp::ble::max_packet_size] = {0};
    packet[0] = bsp::ble::header;
    packet[1] = bsp::ble::BlePacketType::RequestMoveSequence;

    int i = 2;
    for (const auto& move_pair : moves) {
        if (i >= bsp::ble::max_packet_size) {
            break;
        }
        uint8_t type = static_cast<uint8_t>(move_pair.first);
        uint8_t count = move_pair.second;
        packet[i++] = (type << 3) | (count & 0x07);
    }

    // Fill the rest of the packet with STOP commands if there's space
    while (i < bsp::ble::max_packet_size) {
        packet[i++] = (static_cast<uint8_t>(Movement::STOP) << 3) | (1 & 0x07);
    }

    bsp::ble::transmit(packet, sizeof(packet));
}

void Config::load_movement_sequence_from_eeprom() {
    std::vector<std::pair<Movement, uint8_t>> moves;

    for (int i = 0; i < 18; ++i) {
        uint8_t byte;
        if (bsp::eeprom::read_u8(bsp::eeprom::ADDR_MOVE_SEQUENCE_1 + i, &byte) != bsp::eeprom::OK) {
            return;
        }

        if (byte == 0xFF) {
            break;
        }

        uint8_t move_id = byte >> 3;
        if (move_id > static_cast<uint8_t>(Movement::STOP)) {
            break;
        }

        auto type = static_cast<Movement>(move_id);
        uint8_t count = byte & 0x07;

        if (count == 0 || type == Movement::STOP) {
            if (type == Movement::STOP) {
                moves.push_back({Movement::STOP, 1});
            }
            break;
        }
        moves.push_back({type, count});

        bsp::delay_ms(10);
    }

    auto navigation_service = services::Navigation::instance();
    navigation_service->set_hardcoded_movements(moves);
}

}
