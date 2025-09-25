#pragma once

#include "bsp/ble.hpp"

namespace services {

class Config {
public:

    static float fan_speed;

    static float angular_kp;
    static float angular_ki;
    static float angular_kd;

    static float wall_kp;
    static float wall_ki;
    static float wall_kd;

    static float linear_vel_kp;
    static float linear_vel_ki;
    static float linear_vel_kd;

    static float diagonal_walls_kp;
    static float diagonal_walls_ki;
    static float diagonal_walls_kd;

    static float min_move_speed;

    static float ir_wall_dist_ref_right;
    static float ir_wall_dist_ref_front_left;
    static float ir_wall_dist_ref_front_right;
    static float ir_wall_dist_ref_left;

    static float ir_wall_control_th_right;
    static float ir_wall_control_th_front_left;
    static float ir_wall_control_th_front_right;
    static float ir_wall_control_th_left;

    static float ir_wall_detect_th_right;
    static float ir_wall_detect_th_front_left;
    static float ir_wall_detect_th_front_right;
    static float ir_wall_detect_th_left;

    static float z_imu_bias;

    static void init();
    static int parse_packet(uint8_t packet[bsp::ble::max_packet_size]);
    static int parse_movement_packet(uint8_t packet[bsp::ble::max_packet_size]);
    static int parse_move_sequence_packet(uint8_t packet[bsp::ble::max_packet_size]);
    static int write_default_params();
    static int write_all_move_params_to_eeprom();
    static int write_turn_param_to_eeprom(Movement movement_id);
    static int write_forward_param_to_eeprom(Movement movement_id);
    static void send_parameters();
    static void send_movement_parameters();
    static void send_move_sequence();
    static int save_z_bias();
    static void load_custom_movements_from_eeprom();
    static void load_movement_sequence_from_eeprom();
};

}
