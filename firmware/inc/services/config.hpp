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

    static void init();
    static int parse_packet(uint8_t packet[bsp::ble::max_packet_size]);
    static int write_default_params();
    static void send_parameters();
};

}
