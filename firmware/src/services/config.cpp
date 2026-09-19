#include <cmath>
#include <cstdio>
#include <utility>

#include "bsp/analog_sensors.hpp"
#include "bsp/eeprom.hpp"
#include "bsp/timers.hpp"
#include "services/config.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"
#include "utils/movement_params.hpp"

namespace services {

static bool write_default = false;

float Config::fan_speed = 0.0; //[0-1000]

float Config::angular_kp = 0.105;
float Config::angular_ki = 0.010;
float Config::angular_kd = 0.0075;
float Config::angular_acc_feed_forward_k = 0.00037;
float Config::angular_brake_feed_forward_k = 0.00037;
float Config::angular_vel_feed_forward_k = 0.0031;
float Config::linear_vel_acc_feed_forward_k = 0.0;
float Config::linear_vel_brake_feed_forward_k = 0.0;
float Config::linear_vel_feed_forward_k = 0.0;
float Config::max_linear_acc_jerk = 100.0;
float Config::max_linear_brake_jerk = 100.0;
float Config::wheel_radius_mm = 12.75;
float Config::coulomb_ff = 0.13;
float Config::angular_coulomb_ff = 0.0;
float Config::angular_static_ff = 0.0;

float Config::wall_kp = 0.0025;
float Config::wall_ki = 0.0;
float Config::wall_kd = 0.0050;

float Config::linear_vel_kp = 8.0;
float Config::linear_vel_ki = 0.10;
float Config::linear_vel_kd = 0.0;

float Config::diagonal_walls_kp = 0.0025;
float Config::diagonal_walls_ki = 0;
float Config::diagonal_walls_kd = 0.0050;

float Config::min_move_speed = 0.2; // [m/s]

float Config::ir_wall_dist_ref_left = 180;   // Reference distance of when the robot is in the middle of the cell
float Config::ir_wall_dist_ref_right = 165;  // Reference distance of when the robot is in the middle of the cell
float Config::ir_wall_detect_th_left = 220;  // Maximum distance to the wall to still consider we are reading a wall (used on wall brake)
float Config::ir_wall_detect_th_right = 220; // Maximum distance to the wall to still consider we are reading a wall (used on wall brake)

float Config::ir_diagonal_ref_fl = 280;      // Reference distance when the robot is not seeing on a diagonal
float Config::ir_diagonal_ref_fr = 280;      // Reference distance when the robot is not seeing on a diagonal
float Config::ir_diagonal_control_th_fl = 260; // Maximum distance to still enable diagonal control
float Config::ir_diagonal_control_th_fr = 260; // Maximum distance to still enable diagonal control

float Config::sensor_r_slope_max_th = 100.0f;
float Config::sensor_l_slope_max_th = 40.0f;
float Config::right_sensor_angle_deg = 56.5f;
float Config::left_sensor_angle_deg = 56.5f;

float Config::z_imu_bias = -0.4905;

// float Config::start_wall_break_mm_left = 61.0; // 2.0m/s
// float Config::start_wall_break_mm_left = 61.0; // 1.5m/s
// float Config::start_wall_break_mm_right = 75.0;// 1.5m/s

float Config::start_wall_break_mm_left = 65.0;  // 3.0m/s
float Config::start_wall_break_mm_right = 80.0; // 3.0m/s
float Config::enable_wall_break_correction = 1.0;
float Config::enable_lateral_correction_90 = 0.0f;
float Config::enable_lateral_correction_wall = 0.0f;

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
    {&Config::ir_wall_dist_ref_left, bsp::eeprom::ADDR_IR_WALL_DIST_REF_LEFT},
    {&Config::ir_wall_dist_ref_right, bsp::eeprom::ADDR_IR_WALL_DIST_REF_RIGHT},
    {&Config::ir_wall_detect_th_left, bsp::eeprom::ADDR_IR_WALL_DETECT_TH_LEFT},
    {&Config::ir_wall_detect_th_right, bsp::eeprom::ADDR_IR_WALL_DETECT_TH_RIGHT},
    {&Config::ir_diagonal_ref_fl, bsp::eeprom::ADDR_IR_DIAGONAL_REF_FL},
    {&Config::ir_diagonal_ref_fr, bsp::eeprom::ADDR_IR_DIAGONAL_REF_FR},
    {&Config::ir_diagonal_control_th_fl, bsp::eeprom::ADDR_IR_DIAGONAL_CONTROL_TH_FL},
    {&Config::ir_diagonal_control_th_fr, bsp::eeprom::ADDR_IR_DIAGONAL_CONTROL_TH_FR},
    {&Config::sensor_r_slope_max_th, bsp::eeprom::ADDR_SENSOR_R_SLOPE_MAX_TH},
    {&Config::sensor_l_slope_max_th, bsp::eeprom::ADDR_SENSOR_L_SLOPE_MAX_TH},
    {&Config::right_sensor_angle_deg, bsp::eeprom::ADDR_RIGHT_SENSOR_ANGLE_DEG},
    {&Config::left_sensor_angle_deg, bsp::eeprom::ADDR_LEFT_SENSOR_ANGLE_DEG},
    {&Config::z_imu_bias, bsp::eeprom::ADDR_Z_IMU_BIAS},
    {&Config::start_wall_break_mm_left, bsp::eeprom::ADDR_START_WALL_BREAK_MM_LEFT},
    {&Config::start_wall_break_mm_right, bsp::eeprom::ADDR_START_WALL_BREAK_MM_RIGHT},
    {&Config::enable_wall_break_correction, bsp::eeprom::ADDR_ENABLE_WALL_BREAK_CORRECTION},
    {&Config::angular_acc_feed_forward_k, bsp::eeprom::ADDR_ANGULAR_ACC_FEED_FORWARD_K},
    {&Config::angular_brake_feed_forward_k, bsp::eeprom::ADDR_ANGULAR_BRAKE_FEED_FORWARD_K},
    {&Config::angular_vel_feed_forward_k, bsp::eeprom::ADDR_ANGULAR_VEL_FEED_FORWARD_K},
    {&Config::linear_vel_acc_feed_forward_k, bsp::eeprom::ADDR_LINEAR_VEL_ACC_FEED_FORWARD_K},
    {&Config::linear_vel_brake_feed_forward_k, bsp::eeprom::ADDR_LINEAR_VEL_BRAKE_FEED_FORWARD_K},
    {&Config::linear_vel_feed_forward_k, bsp::eeprom::ADDR_LINEAR_VEL_FEED_FORWARD_K},
    {&Config::max_linear_acc_jerk, bsp::eeprom::ADDR_MAX_LINEAR_ACC_JERK},
    {&Config::max_linear_brake_jerk, bsp::eeprom::ADDR_MAX_LINEAR_BRAKE_JERK},
    {&Config::wheel_radius_mm, bsp::eeprom::ADDR_WHEEL_RADIUS_MM},
    {&Config::coulomb_ff, bsp::eeprom::ADDR_COULOMB_FF},
    {&Config::angular_coulomb_ff, bsp::eeprom::ADDR_ANGULAR_COULOMB_FF},
    {&Config::angular_static_ff, bsp::eeprom::ADDR_ANGULAR_STATIC_FF},
    {&Config::enable_lateral_correction_90, bsp::eeprom::ADDR_ENABLE_LATERAL_CORRECTION_90},
    {&Config::enable_lateral_correction_wall, bsp::eeprom::ADDR_ENABLE_LATERAL_CORRECTION_WALL},
};

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
    {Movement::TURN_RIGHT_90_SEARCH_MODE, bsp::eeprom::ADDR_FORWARD_PARAMS_RIGHT_90_SEARCH},
    {Movement::TURN_LEFT_90_SEARCH_MODE, bsp::eeprom::ADDR_FORWARD_PARAMS_LEFT_90_SEARCH},
    {Movement::TURN_RIGHT_45, bsp::eeprom::ADDR_FORWARD_PARAMS_RIGHT_45},
    {Movement::TURN_LEFT_45, bsp::eeprom::ADDR_FORWARD_PARAMS_LEFT_45},
    {Movement::TURN_RIGHT_90, bsp::eeprom::ADDR_FORWARD_PARAMS_RIGHT_90},
    {Movement::TURN_LEFT_90, bsp::eeprom::ADDR_FORWARD_PARAMS_LEFT_90},
    {Movement::TURN_RIGHT_135, bsp::eeprom::ADDR_FORWARD_PARAMS_RIGHT_135},
    {Movement::TURN_LEFT_135, bsp::eeprom::ADDR_FORWARD_PARAMS_LEFT_135},
    {Movement::TURN_RIGHT_180, bsp::eeprom::ADDR_FORWARD_PARAMS_RIGHT_180},
    {Movement::TURN_LEFT_180, bsp::eeprom::ADDR_FORWARD_PARAMS_LEFT_180},
    {Movement::TURN_AROUND_INPLACE, bsp::eeprom::ADDR_FORWARD_PARAMS_TURN_AROUND_INPLACE},
};

static constexpr uint16_t ir_calib_eeprom_addrs[4][3] = {
    {bsp::eeprom::ADDR_IR_CALIB_A_RIGHT, bsp::eeprom::ADDR_IR_CALIB_B_RIGHT, bsp::eeprom::ADDR_IR_CALIB_C_RIGHT},
    {bsp::eeprom::ADDR_IR_CALIB_A_FRONT_LEFT, bsp::eeprom::ADDR_IR_CALIB_B_FRONT_LEFT,
     bsp::eeprom::ADDR_IR_CALIB_C_FRONT_LEFT},
    {bsp::eeprom::ADDR_IR_CALIB_A_FRONT_RIGHT, bsp::eeprom::ADDR_IR_CALIB_B_FRONT_RIGHT,
     bsp::eeprom::ADDR_IR_CALIB_C_FRONT_RIGHT},
    {bsp::eeprom::ADDR_IR_CALIB_A_LEFT, bsp::eeprom::ADDR_IR_CALIB_B_LEFT, bsp::eeprom::ADDR_IR_CALIB_C_LEFT},
};

static const uint16_t ir_wall_patterns_eeprom_addrs[8][4] = {
    {bsp::eeprom::ADDR_IR_WALL_PATTERN_0_L, bsp::eeprom::ADDR_IR_WALL_PATTERN_0_FL,
     bsp::eeprom::ADDR_IR_WALL_PATTERN_0_FR, bsp::eeprom::ADDR_IR_WALL_PATTERN_0_R},
    {bsp::eeprom::ADDR_IR_WALL_PATTERN_1_L, bsp::eeprom::ADDR_IR_WALL_PATTERN_1_FL,
     bsp::eeprom::ADDR_IR_WALL_PATTERN_1_FR, bsp::eeprom::ADDR_IR_WALL_PATTERN_1_R},
    {bsp::eeprom::ADDR_IR_WALL_PATTERN_2_L, bsp::eeprom::ADDR_IR_WALL_PATTERN_2_FL,
     bsp::eeprom::ADDR_IR_WALL_PATTERN_2_FR, bsp::eeprom::ADDR_IR_WALL_PATTERN_2_R},
    {bsp::eeprom::ADDR_IR_WALL_PATTERN_3_L, bsp::eeprom::ADDR_IR_WALL_PATTERN_3_FL,
     bsp::eeprom::ADDR_IR_WALL_PATTERN_3_FR, bsp::eeprom::ADDR_IR_WALL_PATTERN_3_R},
    {bsp::eeprom::ADDR_IR_WALL_PATTERN_4_L, bsp::eeprom::ADDR_IR_WALL_PATTERN_4_FL,
     bsp::eeprom::ADDR_IR_WALL_PATTERN_4_FR, bsp::eeprom::ADDR_IR_WALL_PATTERN_4_R},
    {bsp::eeprom::ADDR_IR_WALL_PATTERN_5_L, bsp::eeprom::ADDR_IR_WALL_PATTERN_5_FL,
     bsp::eeprom::ADDR_IR_WALL_PATTERN_5_FR, bsp::eeprom::ADDR_IR_WALL_PATTERN_5_R},
    {bsp::eeprom::ADDR_IR_WALL_PATTERN_6_L, bsp::eeprom::ADDR_IR_WALL_PATTERN_6_FL,
     bsp::eeprom::ADDR_IR_WALL_PATTERN_6_FR, bsp::eeprom::ADDR_IR_WALL_PATTERN_6_R},
    {bsp::eeprom::ADDR_IR_WALL_PATTERN_7_L, bsp::eeprom::ADDR_IR_WALL_PATTERN_7_FL,
     bsp::eeprom::ADDR_IR_WALL_PATTERN_7_FR, bsp::eeprom::ADDR_IR_WALL_PATTERN_7_R},
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
    bsp::delay_ms(5);
    load_ir_calib_from_eeprom();
    bsp::delay_ms(5);
    load_ir_wall_patterns_from_eeprom();
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

void Config::print_parameters() {
    std::printf("general_params = {\r\n");
    bsp::delay_ms(5);
    std::printf("    fan_speed = %f,\r\n", Config::fan_speed);
    bsp::delay_ms(5);
    std::printf("    angular_kp = %f,\r\n", Config::angular_kp);
    bsp::delay_ms(5);
    std::printf("    angular_ki = %f,\r\n", Config::angular_ki);
    bsp::delay_ms(5);
    std::printf("    angular_kd = %f,\r\n", Config::angular_kd);
    bsp::delay_ms(5);
    std::printf("    angular_acc_feed_forward_k = %f,\r\n", Config::angular_acc_feed_forward_k);
    bsp::delay_ms(5);
    std::printf("    angular_brake_feed_forward_k = %f,\r\n", Config::angular_brake_feed_forward_k);
    bsp::delay_ms(5);
    std::printf("    angular_vel_feed_forward_k = %f,\r\n", Config::angular_vel_feed_forward_k);
    bsp::delay_ms(5);
    std::printf("    linear_vel_acc_feed_forward_k = %f,\r\n", Config::linear_vel_acc_feed_forward_k);
    bsp::delay_ms(5);
    std::printf("    linear_vel_brake_feed_forward_k = %f,\r\n", Config::linear_vel_brake_feed_forward_k);
    bsp::delay_ms(5);
    std::printf("    linear_vel_feed_forward_k = %f,\r\n", Config::linear_vel_feed_forward_k);
    bsp::delay_ms(5);
    std::printf("    wall_kp = %f,\r\n", Config::wall_kp);
    bsp::delay_ms(5);
    std::printf("    wall_ki = %f,\r\n", Config::wall_ki);
    bsp::delay_ms(5);
    std::printf("    wall_kd = %f,\r\n", Config::wall_kd);
    bsp::delay_ms(5);
    std::printf("    linear_vel_kp = %f,\r\n", Config::linear_vel_kp);
    bsp::delay_ms(5);
    std::printf("    linear_vel_ki = %f,\r\n", Config::linear_vel_ki);
    bsp::delay_ms(5);
    std::printf("    linear_vel_kd = %f,\r\n", Config::linear_vel_kd);
    bsp::delay_ms(5);
    std::printf("    diagonal_walls_kp = %f,\r\n", Config::diagonal_walls_kp);
    bsp::delay_ms(5);
    std::printf("    diagonal_walls_ki = %f,\r\n", Config::diagonal_walls_ki);
    bsp::delay_ms(5);
    std::printf("    diagonal_walls_kd = %f,\r\n", Config::diagonal_walls_kd);
    bsp::delay_ms(5);
    std::printf("    start_wall_break_mm_left = %f,\r\n", Config::start_wall_break_mm_left);
    bsp::delay_ms(5);
    std::printf("    start_wall_break_mm_right = %f,\r\n", Config::start_wall_break_mm_right);
    bsp::delay_ms(5);
    std::printf("    enable_wall_break_correction = %f,\r\n", Config::enable_wall_break_correction);
    bsp::delay_ms(5);
    std::printf("    max_linear_acc_jerk = %f,\r\n", Config::max_linear_acc_jerk);
    bsp::delay_ms(5);
    std::printf("    max_linear_brake_jerk = %f,\r\n", Config::max_linear_brake_jerk);
    bsp::delay_ms(5);
    std::printf("    wheel_radius_mm = %f,\r\n", Config::wheel_radius_mm);
    bsp::delay_ms(5);
    std::printf("    coulomb_ff = %f,\r\n", Config::coulomb_ff);
    bsp::delay_ms(5);
    std::printf("    angular_coulomb_ff = %f,\r\n", Config::angular_coulomb_ff);
    bsp::delay_ms(5);
    std::printf("    angular_static_ff = %f,\r\n", Config::angular_static_ff);
    bsp::delay_ms(5);
    std::printf("};\r\n");
    bsp::delay_ms(5);
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
        case bsp::ble::ForwardParamID::TARGET_TRAVEL_MM:
            forward_params_custom[movement_id].target_travel_mm = value;
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
        case bsp::ble::TurnParamID::T_START_DECCEL:
            turn_params_custom[movement_id].t_start_deccel = Config::ms_to_ticks(value);
            break;
        case bsp::ble::TurnParamID::T_STOP:
            turn_params_custom[movement_id].t_stop = Config::ms_to_ticks(value);
            break;
        case bsp::ble::TurnParamID::SIGN:
            turn_params_custom[movement_id].sign = static_cast<int>(value);
            break;
        case bsp::ble::TurnParamID::TIME_TO_DECREASE_JERK_1:
            turn_params_custom[movement_id].time_to_decrease_jerk_1 = Config::ms_to_ticks(value);
            break;
        case bsp::ble::TurnParamID::TIME_TO_DECREASE_JERK_2:
            turn_params_custom[movement_id].time_to_decrease_jerk_2 = Config::ms_to_ticks(value);
            break;
        case bsp::ble::TurnParamID::ACCEL_RAMP_UP_JERK:
            turn_params_custom[movement_id].accel_ramp_up_jerk = value;
            break;
        case bsp::ble::TurnParamID::ACCEL_RAMP_DOWN_JERK:
            turn_params_custom[movement_id].accel_ramp_down_jerk = value;
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
        send_param(0, movement_id, static_cast<uint8_t>(bsp::ble::ForwardParamID::TARGET_TRAVEL_MM),
                   params.target_travel_mm);
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
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::T_START_DECCEL),
                   Config::ticks_to_ms(params.t_start_deccel));
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::T_STOP),
                   Config::ticks_to_ms(params.t_stop));
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::SIGN), static_cast<float>(params.sign));
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::TIME_TO_DECREASE_JERK_1),
                   Config::ticks_to_ms(params.time_to_decrease_jerk_1));
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::TIME_TO_DECREASE_JERK_2),
                   Config::ticks_to_ms(params.time_to_decrease_jerk_2));
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::ACCEL_RAMP_UP_JERK),
                   params.accel_ramp_up_jerk);
        send_param(1, movement_id, static_cast<uint8_t>(bsp::ble::TurnParamID::ACCEL_RAMP_DOWN_JERK),
                   params.accel_ramp_down_jerk);
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

void Config::load_ir_calib_from_eeprom() {
    for (int i = 0; i < 4; i++) {
        _float fa, fb, fc;
        if (bsp::eeprom::read_u32(ir_calib_eeprom_addrs[i][0], &fa.u32) == bsp::eeprom::OK &&
            bsp::eeprom::read_u32(ir_calib_eeprom_addrs[i][1], &fb.u32) == bsp::eeprom::OK &&
            bsp::eeprom::read_u32(ir_calib_eeprom_addrs[i][2], &fc.u32) == bsp::eeprom::OK) {

            if (fa.u32 != 0xFFFFFFFF && fb.u32 != 0xFFFFFFFF && fc.u32 != 0xFFFFFFFF) {
                if (!std::isnan(fa.value) && !std::isinf(fa.value) && fa.value > 0.0f && !std::isnan(fb.value) &&
                    !std::isinf(fb.value) && !std::isnan(fc.value) && !std::isinf(fc.value)) {
                    bsp::analog_sensors::set_calib_params(static_cast<bsp::analog_sensors::SensingDirection>(i),
                                                          {fa.value, fb.value, fc.value});

                    std::printf("%s: %f\r\n", bsp::eeprom::param_name(ir_calib_eeprom_addrs[i][0]), fa.value);
                    bsp::delay_ms(2);
                    std::printf("%s: %f\r\n", bsp::eeprom::param_name(ir_calib_eeprom_addrs[i][1]), fb.value);
                    bsp::delay_ms(2);
                    std::printf("%s: %f\r\n", bsp::eeprom::param_name(ir_calib_eeprom_addrs[i][2]), fc.value);
                    bsp::delay_ms(2);
                }
            }
        }
        bsp::delay_ms(2);
    }
}

int Config::save_ir_calib_to_eeprom(bsp::analog_sensors::SensingDirection direction) {
    uint8_t idx = static_cast<uint8_t>(direction);
    if (idx >= 4) {
        return -1;
    }
    auto params = bsp::analog_sensors::get_calib_params(direction);
    _float fa, fb, fc;
    fa.value = params.a;
    fb.value = params.b;
    fc.value = params.c;

    if (bsp::eeprom::write_u32(ir_calib_eeprom_addrs[idx][0], fa.u32) != bsp::eeprom::OK) {
        return -1;
    }
    bsp::delay_ms(5);
    if (bsp::eeprom::write_u32(ir_calib_eeprom_addrs[idx][1], fb.u32) != bsp::eeprom::OK) {
        return -1;
    }
    bsp::delay_ms(5);
    if (bsp::eeprom::write_u32(ir_calib_eeprom_addrs[idx][2], fc.u32) != bsp::eeprom::OK) {
        return -1;
    }
    bsp::delay_ms(5);

    return 0;
}

int Config::save_all_ir_calib_to_eeprom() {
    for (int i = 0; i < 4; i++) {
        if (save_ir_calib_to_eeprom(static_cast<bsp::analog_sensors::SensingDirection>(i)) != 0) {
            return -1;
        }
    }
    return 0;
}

void Config::load_ir_wall_patterns_from_eeprom() {
    for (int i = 0; i < 8; i++) {
        _float l, fl, fr, r;
        if (bsp::eeprom::read_u32(ir_wall_patterns_eeprom_addrs[i][0], &l.u32) == bsp::eeprom::OK &&
            bsp::eeprom::read_u32(ir_wall_patterns_eeprom_addrs[i][1], &fl.u32) == bsp::eeprom::OK &&
            bsp::eeprom::read_u32(ir_wall_patterns_eeprom_addrs[i][2], &fr.u32) == bsp::eeprom::OK &&
            bsp::eeprom::read_u32(ir_wall_patterns_eeprom_addrs[i][3], &r.u32) == bsp::eeprom::OK) {

            if (l.u32 != 0xFFFFFFFF && fl.u32 != 0xFFFFFFFF && fr.u32 != 0xFFFFFFFF && r.u32 != 0xFFFFFFFF) {
                const bool legacy_raw_pattern = l.u32 <= 4095 && fl.u32 <= 4095 && fr.u32 <= 4095 && r.u32 <= 4095;
                if (legacy_raw_pattern) {
                    bsp::analog_sensors::set_wall_pattern(
                        i,
                        {bsp::analog_sensors::raw_to_distance_mm(bsp::analog_sensors::LEFT, l.u32),
                         bsp::analog_sensors::raw_to_distance_mm(bsp::analog_sensors::FRONT_LEFT, fl.u32),
                         bsp::analog_sensors::raw_to_distance_mm(bsp::analog_sensors::FRONT_RIGHT, fr.u32),
                         bsp::analog_sensors::raw_to_distance_mm(bsp::analog_sensors::RIGHT, r.u32)});
                } else {
                    bsp::analog_sensors::set_wall_pattern(i, {l.value, fl.value, fr.value, r.value});
                }

                auto pattern = bsp::analog_sensors::get_wall_pattern(i);
                std::printf("%s: %f\r\n", bsp::eeprom::param_name(ir_wall_patterns_eeprom_addrs[i][0]), pattern.L);
                bsp::delay_ms(2);
                std::printf("%s: %f\r\n", bsp::eeprom::param_name(ir_wall_patterns_eeprom_addrs[i][1]), pattern.FL);
                bsp::delay_ms(2);
                std::printf("%s: %f\r\n", bsp::eeprom::param_name(ir_wall_patterns_eeprom_addrs[i][2]), pattern.FR);
                bsp::delay_ms(2);
                std::printf("%s: %f\r\n", bsp::eeprom::param_name(ir_wall_patterns_eeprom_addrs[i][3]), pattern.R);
                bsp::delay_ms(2);
            }
        }
        bsp::delay_ms(2);
    }
}

int Config::save_ir_wall_pattern_to_eeprom(uint8_t pattern_idx) {
    if (pattern_idx >= 8) {
        return -1;
    }
    auto pattern = bsp::analog_sensors::get_wall_pattern(pattern_idx);
    _float value;

    value.value = pattern.L;
    if (bsp::eeprom::write_u32(ir_wall_patterns_eeprom_addrs[pattern_idx][0], value.u32) != bsp::eeprom::OK){
        return -1;
    }
    bsp::delay_ms(5);
    value.value = pattern.FL;
    if (bsp::eeprom::write_u32(ir_wall_patterns_eeprom_addrs[pattern_idx][1], value.u32) != bsp::eeprom::OK) {
        return -1;
    }
    bsp::delay_ms(5);
    value.value = pattern.FR;
    if (bsp::eeprom::write_u32(ir_wall_patterns_eeprom_addrs[pattern_idx][2], value.u32) != bsp::eeprom::OK){
        return -1;
    }
    bsp::delay_ms(5);
    value.value = pattern.R;
    if (bsp::eeprom::write_u32(ir_wall_patterns_eeprom_addrs[pattern_idx][3], value.u32) != bsp::eeprom::OK){
        return -1;
    }
    bsp::delay_ms(5);

    return 0;
}

int Config::save_all_ir_wall_patterns_to_eeprom() {
    for (uint8_t i = 0; i < 8; i++) {
        if (save_ir_wall_pattern_to_eeprom(i) != 0) {
            return -1;
        }
    }
    return 0;
}

int Config::reset_ir_wall_patterns_in_eeprom() {
    bsp::analog_sensors::reset_all_wall_patterns();
    return save_all_ir_wall_patterns_to_eeprom();
}

}
