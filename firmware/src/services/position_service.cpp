#include "services/position_service.hpp"
#include "bsp/encoders.hpp"
#include <cstdio>


namespace services::position {

/// @section Private variables

static float travelled_dist = 0;
static int32_t encoder_right_counter;
static int32_t encoder_left_counter;
static Position current_position;
static Direction current_direction;


/// @section Private Functions

static void cb_encoder_l(bsp::encoders::DirectionType type) {
    if (type == bsp::encoders::DirectionType::CCW) {
        encoder_left_counter++;
    } else {
        encoder_left_counter--;
    }
}

static void cb_encoder_r(bsp::encoders::DirectionType type) {
    if (type == bsp::encoders::DirectionType::CCW) {
        encoder_right_counter++;
    } else {
        encoder_right_counter--;
    } 
}

/// @section Interface implementation

void init(void) {
    reset();
    bsp::encoders::register_callback_encoder_left(cb_encoder_l);
    bsp::encoders::register_callback_encoder_right(cb_encoder_r);
}

void reset(void) {
    travelled_dist = 0;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    current_position = {0, 0};
    current_direction = NORTH;
}

void update(void) {

    if (encoder_left_counter == 0 && encoder_right_counter == 0) {
        return;
    }

    float delta_x_cm;
    float estimated_delta_l_cm = (encoder_left_counter * ENCODER_DIST_CM_PULSE);
    float estimated_delta_r_cm = (encoder_right_counter * ENCODER_DIST_CM_PULSE);

    delta_x_cm = (estimated_delta_l_cm + estimated_delta_r_cm) / 2.0;
    travelled_dist += delta_x_cm;
    encoder_left_counter = 0;
    encoder_right_counter = 0;

    // static int cnt = 0;
    // if (cnt++ % 100 == 0){
    //     std::printf("Travelled distance: (%f) cm\r\n", travelled_dist);
    // }

}

float get_travelled_dist_cm(void) {
    return travelled_dist;
}

Direction get_robot_direction(void) {
    return current_direction;
}

void set_robot_direction(Direction dir) {
    current_direction = dir;
}

Position get_robot_position(void) {
    return current_position;
}

void increase_robot_position(Direction dir) {
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
