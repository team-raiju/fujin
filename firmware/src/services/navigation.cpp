#include <cstdio>

#include "bsp/analog_sensors.hpp"
#include "bsp/encoders.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"

/// @section Constants

static constexpr float WHEEL_RADIUS_CM = (1.30);
static constexpr float WHEEL_PERIMETER_CM = (M_TWOPI * WHEEL_RADIUS_CM);

static constexpr float WHEEL_TO_ENCODER_RATIO = (1.0);
static constexpr float ENCODER_PPR = (1024.0);
static constexpr float PULSES_PER_WHEEL_ROTATION = (WHEEL_TO_ENCODER_RATIO * ENCODER_PPR);

static constexpr float WHEELS_DIST_CM = (7.0);
static constexpr float ENCODER_DIST_CM_PULSE = (WHEEL_PERIMETER_CM / PULSES_PER_WHEEL_ROTATION);

static constexpr float CELL_SIZE_CM = 18.0;
static constexpr float HALF_CELL_SIZE_CM = 9.0;

/// @section Service implementation

namespace services {

Navigation* Navigation::instance() {
    static Navigation p;
    return &p;
}

void Navigation::init(void) {
    reset();

    if (!is_initialized) {
        bsp::encoders::register_callback_encoder_left(
            [this](auto type) { encoder_left_counter += type == bsp::encoders::DirectionType::CCW ? 1 : -1; });

        bsp::encoders::register_callback_encoder_right(
            [this](auto type) { encoder_right_counter += type == bsp::encoders::DirectionType::CCW ? 1 : -1; });

        is_initialized = true;
    }
}

void Navigation::reset(void) {
    bsp::imu::reset_angle();

    traveled_dist = 0;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    current_position = {0, 0};
    current_direction = Direction::NORTH;

    state = 0;

    angular_vel_pid.reset();
    angular_vel_pid.kp = 10.0;
    angular_vel_pid.ki = 1;
    angular_vel_pid.kd = 0;
    angular_vel_pid.integral_limit = 1000;

    walls_pid.reset();
    walls_pid.kp = 10.0;
    walls_pid.ki = 0;
    walls_pid.kd = 0;
    walls_pid.integral_limit = 0;

    target_speed = 0;
    rotation_ratio = 0;
    target_travel = HALF_CELL_SIZE_CM;
    current_movement = Movement::FORWARD;
}

void Navigation::update(void) {
    bsp::imu::update();

    // We didn't move, do nothing
    if (encoder_left_counter == 0 && encoder_right_counter == 0) {
        return;
    }

    float estimated_delta_l_cm = (encoder_left_counter * ENCODER_DIST_CM_PULSE);
    float estimated_delta_r_cm = (encoder_right_counter * ENCODER_DIST_CM_PULSE);

    float delta_x_cm = (estimated_delta_l_cm + estimated_delta_r_cm) / 2.0;
    traveled_dist += delta_x_cm;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
}

bool Navigation::step() {
    using bsp::analog_sensors::ir_reading;
    using bsp::analog_sensors::ir_reading_wall;
    using bsp::analog_sensors::SensingDirection;

    auto left_ir = ir_reading(SensingDirection::LEFT);
    auto right_ir = ir_reading(SensingDirection::RIGHT);

    switch (current_movement) {
    case Movement::STOP:
        target_speed = 0;
        is_finished = true;
        break;

    case Movement::FORWARD:
        target_speed = 50;
        rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
        //rotation_ratio += walls_pid.calculate(0.0, (left_ir - right_ir));
        if (traveled_dist >= target_travel) {
            update_position();
            current_movement = STOP;
        }
        break;

    case Movement::RIGHT:
        // Move half a cell, turn right, move half a cell
        if (state == 0) {
            target_speed = std::max(target_speed - 0.05, 0.0);
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (traveled_dist > HALF_CELL_SIZE_CM) {
                state = 1;
            }
        } else if (state == 1) {
            target_speed = 0;
            rotation_ratio = 30;
            if (std::abs(bsp::imu::get_angle()) > (M_PI_2 - 0.2)) {
                traveled_dist = 0;
                state = 2;
            }
        } else if (state == 2) {
            target_speed = std::min(target_speed + 0.05, 50.0);
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (traveled_dist > HALF_CELL_SIZE_CM) {
                update_position();
                current_movement = STOP;
            }
        }

        break;

    case Movement::LEFT:
        // Move half a cell, turn left, move half a cell
        if (state == 0) {
            target_speed = std::max(target_speed - 0.05, 0.0);
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (traveled_dist > HALF_CELL_SIZE_CM) {
                state = 1;
            }
        } else if (state == 1) {
            target_speed = 0;
            rotation_ratio = -30;
            if (std::abs(bsp::imu::get_angle()) > (M_PI_2 - 0.2)) {
                traveled_dist = 0;
                state = 2;
            }
        } else if (state == 2) {
            target_speed = std::min(target_speed + 0.05, 50.0);
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (traveled_dist > HALF_CELL_SIZE_CM) {
                update_position();
                current_movement = STOP;
            }
        }
        break;

    case Movement::TURN_AROUND:
        target_speed = 0;
        rotation_ratio = -30;

        if (std::abs(bsp::imu::get_angle()) > (M_PI - 0.4)) {
            update_position();
            current_movement = STOP;
        }

        break;
    }

    float speed_l = target_speed + rotation_ratio;
    float speed_r = target_speed - rotation_ratio;
    bsp::motors::set(speed_l, speed_r);

    return is_finished;
}

Point Navigation::get_robot_position(void) {
    return current_position;
}

Direction Navigation::get_robot_direction(void) {
    return current_direction;
}

void Navigation::move(Direction dir) {
    bsp::imu::reset_angle();

    current_movement = get_movement(dir);
    traveled_dist = 0;
    state = 0;
    target_travel = CELL_SIZE_CM;
}

void Navigation::update_position() {
    switch (current_direction) {
    case Direction::NORTH:
        current_position.y++;
        break;
    case Direction::EAST:
        current_position.x++;
        break;
    case Direction::SOUTH:
        current_position.y--;
        break;
    case Direction::WEST:
        current_position.x--;
        break;
    default:
        break;
    }
}

Movement Navigation::get_movement(Direction target_dir) {
    using enum Direction;

    if (target_dir == current_direction) {
        return FORWARD;
    } else if ((target_dir == NORTH && current_direction == WEST) ||
               (target_dir == EAST && current_direction == NORTH) ||
               (target_dir == SOUTH && current_direction == EAST) ||
               (target_dir == WEST && current_direction == SOUTH)) {
        return RIGHT;
    } else if ((target_dir == NORTH && current_direction == SOUTH) ||
               (target_dir == EAST && current_direction == WEST) ||
               (target_dir == SOUTH && current_direction == NORTH) ||
               (target_dir == WEST && current_direction == EAST)) {
        return TURN_AROUND;
    } else {
        return LEFT;
    }
}

}
