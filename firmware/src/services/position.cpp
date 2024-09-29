#include "services/position.hpp"
#include "bsp/encoders.hpp"
#include "services/navigation.hpp"
#include <cstdio>

#define WHEEL_RADIUS_CM (1.27)
#define WHEEL_PERIMETER_CM (M_TWOPI * WHEEL_RADIUS_CM)

#define WHEEL_TO_ENCODER_RATIO                                                                                         \
    (1.0)                    // 1 rotation of the wheel equals to WHEEL_TO_ENCODER_RATIO rotations of the encoder
#define ENCODER_PPR (1024.0) // Pulses in one rotation of the encoder
#define PULSES_PER_WHEEL_ROTATION (WHEEL_TO_ENCODER_RATIO * ENCODER_PPR)

#define WHEELS_DIST_CM (7.0)
#define ENCODER_DIST_CM_PULSE (WHEEL_PERIMETER_CM / PULSES_PER_WHEEL_ROTATION)

namespace services {

Position* Position::instance() {
    static Position p;
    return &p;
}

void Position::init(void) {
    reset();

    bsp::encoders::register_callback_encoder_left(
        [this](auto type) { encoder_left_counter += type == bsp::encoders::DirectionType::CCW ? 1 : -1; });

    bsp::encoders::register_callback_encoder_right(
        [this](auto type) { encoder_right_counter += type == bsp::encoders::DirectionType::CCW ? 1 : -1; });
}

void Position::reset(void) {
    traveled_dist = 0;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    current_position = {0, 0};
    current_direction = NORTH;
}

void Position::update(void) {
    if (encoder_left_counter == 0 && encoder_right_counter == 0) {
        return;
    }

    float delta_x_cm;
    float estimated_delta_l_cm = (encoder_left_counter * ENCODER_DIST_CM_PULSE);
    float estimated_delta_r_cm = (encoder_right_counter * ENCODER_DIST_CM_PULSE);

    delta_x_cm = (estimated_delta_l_cm + estimated_delta_r_cm) / 2.0;
    traveled_dist += delta_x_cm;
    encoder_left_counter = 0;
    encoder_right_counter = 0;

    // static int cnt = 0;
    // if (cnt++ % 100 == 0){
    //     std::printf("Travelled distance: (%f) cm\r\n", traveled_dist);
    // }
}

float Position::get_traveled_cm(void) {
    return traveled_dist;
}

Direction Position::get_robot_direction(void) {
    return current_direction;
}

void Position::set_robot_direction(Direction dir) {
    current_direction = dir;
}

Point Position::get_robot_position(void) {
    return current_position;
}

void Position::increase_robot_position(Direction dir) {
    switch (dir) {
    case NORTH:
        current_position.y++;
        break;
    case EAST:
        current_position.x++;
        break;
    case SOUTH:
        current_position.y--;
        break;
    case WEST:
        current_position.x--;
        break;
    default:
        break;
    }
}

}
