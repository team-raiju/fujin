#include "services/navigation.hpp"
#include "bsp/encoders.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"
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

Navigation* Navigation::instance() {
    static Navigation p;
    return &p;
}

void Navigation::init(void) {
    reset();

    bsp::encoders::register_callback_encoder_left(
        [this](auto type) { encoder_left_counter += type == bsp::encoders::DirectionType::CCW ? 1 : -1; });

    bsp::encoders::register_callback_encoder_right(
        [this](auto type) { encoder_right_counter += type == bsp::encoders::DirectionType::CCW ? 1 : -1; });
}

void Navigation::reset(void) {
    traveled_dist = 0;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    current_position = {0, 0};
    current_direction = NORTH;
}

void Navigation::update(void) {
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

float Navigation::get_traveled_cm(void) {
    return traveled_dist;
}

Direction Navigation::get_robot_direction(void) {
    return current_direction;
}

void Navigation::set_robot_direction(Direction dir) {
    current_direction = dir;
}

Point Navigation::get_robot_position(void) {
    return current_position;
}

void Navigation::move(Movement) {
    // Actually move the robot here and update its position
}

}
