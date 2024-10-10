#include <cstdio>

#include "bsp/analog_sensors.hpp"
#include "bsp/encoders.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "services/config.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"

/// @section Constants

static constexpr float WHEEL_RADIUS_CM = (1.31);
static constexpr float WHEEL_PERIMETER_CM = (M_TWOPI * WHEEL_RADIUS_CM);

static constexpr float WHEEL_TO_ENCODER_RATIO = (1.0);
static constexpr float ENCODER_PPR = (1024.0);
static constexpr float PULSES_PER_WHEEL_ROTATION = (WHEEL_TO_ENCODER_RATIO * ENCODER_PPR);

static constexpr float WHEELS_DIST_CM = (7.0);
static constexpr float ENCODER_DIST_CM_PULSE = (WHEEL_PERIMETER_CM / PULSES_PER_WHEEL_ROTATION);

static constexpr float CELL_SIZE_CM = 18.0;
static constexpr float HALF_CELL_SIZE_CM = 9.0;

static constexpr float ROBOT_DIST_FROM_CENTER_START = 2.0;

using bsp::leds::Color;

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

    is_finished = false;

    angular_vel_pid.reset();
    angular_vel_pid.kp = Config::angular_kp;
    angular_vel_pid.ki = Config::angular_ki;
    angular_vel_pid.kd = Config::angular_kd;
    angular_vel_pid.integral_limit = 1000;

    walls_pid.reset();
    walls_pid.kp = Config::wall_kp;
    walls_pid.ki = Config::wall_ki;
    walls_pid.kd = Config::wall_kd;
    walls_pid.integral_limit = 0;

    target_speed = 0;
    rotation_ratio = 0;
    target_travel = HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START;
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
    using bsp::analog_sensors::ir_side_wall_error;
    using bsp::analog_sensors::SensingDirection;

    switch (current_movement) {
    case Movement::STOP: {

        // Move half a cell, stop, turn 180, stop, move half a cell
        static int half_turn = false;
        if (state == 0) {
            target_speed = std::max((target_speed - Config::linear_acceleration), Config::min_move_speed);
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (std::abs(traveled_dist) >= HALF_CELL_SIZE_CM - 0.5) {
                state = 1;
                reference_time = bsp::get_tick_ms();
            }
        } else if (state == 1) {
            target_speed = 0.0;
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (bsp::get_tick_ms() - reference_time > 50) {
                state = 2;
                half_turn = false;
            }
        } else if (state == 2) {
            target_speed = 0;
            if (std::abs(bsp::imu::get_angle()) < M_PI_2) {
                rotation_ratio = Config::angular_speed;
            } else {
                rotation_ratio = std::max(float(rotation_ratio - Config::angular_acceleration), Config::min_turn_speed);
            }

            if (!half_turn && std::abs(bsp::imu::get_angle()) > M_PI_2) {
                half_turn = true;
            }

            if (std::abs(bsp::imu::get_angle()) > (M_PI - 0.03) ||
                (std::abs(bsp::imu::get_angle()) < (0.15) && half_turn)) {
                traveled_dist = 0;
                state = 3;
            }
        } else if (state == 3) {
            target_speed = 0;
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());

            if (std::abs(bsp::imu::get_rad_per_s()) < 0.05) {
                state = 4;
                reference_time = bsp::get_tick_ms();
            }

        } else if (state == 4) {
            // target_speed = -Config::fix_position_speed;
            // rotation_ratio = 0;
            // if (bsp::get_tick_ms() - reference_time > 300) {
            bsp::imu::reset_angle();
            state = 5;
            traveled_dist = 0;
            // }
        } else if (state == 5) {
            target_speed = std::min((target_speed + Config::linear_acceleration), Config::min_move_speed);
            float angle_error = bsp::imu::get_angle();

            rotation_ratio = -angular_vel_pid.calculate(-angle_error * 0.5, bsp::imu::get_rad_per_s());
            if (std::abs(traveled_dist) >= HALF_CELL_SIZE_CM) {
                state = 6;
                reference_time = bsp::get_tick_ms();
            }
        } else if (state == 6) {
            target_speed = 0.0;
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (bsp::get_tick_ms() - reference_time > 2500) {
                bsp::leds::stripe_set(Color::Red);
                update_position();
                is_finished = true;
            }
        }

        break;
    }
    case Movement::FORWARD: {
        using bsp::analog_sensors::ir_reading;

        bool front_emergency = ir_reading(SensingDirection::FRONT_LEFT) > 2850 &&
                               ir_reading(SensingDirection::FRONT_RIGHT) > 2850 &&
                               ir_reading(SensingDirection::LEFT) > 2470 && ir_reading(SensingDirection::RIGHT) > 2850;

        if (std::abs(traveled_dist) >= (target_travel - CELL_SIZE_CM)) {
            target_speed = std::max((target_speed - Config::linear_acceleration), Config::search_speed);
        } else {
            target_speed = std::min((target_speed + Config::linear_acceleration), Config::run_speed);
        }

        float target_rad_s = walls_pid.calculate(0.0, ir_side_wall_error());
        float angle_error = bsp::imu::get_angle();

        rotation_ratio = -angular_vel_pid.calculate(target_rad_s - (angle_error * 0.5), bsp::imu::get_rad_per_s());

        if (std::abs(traveled_dist) >= target_travel || front_emergency) {
            bsp::leds::stripe_set(Color::Red);
            // bsp::leds::stripe_send();
            update_position();
            // uint8_t cells_travelled = std::abs(traveled_dist) / CELL_SIZE_CM;

            // for (int i = 0; i < cells_travelled - 1; i++) {
            //     update_position();
            // }
            is_finished = true;
        }
        break;
    }

    case Movement::RIGHT: {
        // Move half a cell, stop, turn right, stop, move half a cell
        if (state == 0) {
            float elapsed_time = bsp::get_tick_ms() - reference_time;
            if (elapsed_time < 500) {
                target_speed = std::max((target_speed - 1.75f * Config::linear_acceleration), 0.0f);
            } else {
                target_speed = std::min((target_speed + Config::linear_acceleration), Config::min_move_speed);
            }

            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (std::abs(traveled_dist) >= HALF_CELL_SIZE_CM - 0.5) {
                state = 1;
                reference_time = bsp::get_tick_ms();
            }
        } else if (state == 1) {
            target_speed = 0.0;
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (bsp::get_tick_ms() - reference_time > 50) {
                state = 2;
            }
        } else if (state == 2) {
            target_speed = 0;
            if (std::abs(bsp::imu::get_angle()) < M_PI / 4) {
                rotation_ratio = Config::angular_speed;
            } else {
                rotation_ratio = std::max((rotation_ratio - Config::angular_acceleration), Config::min_turn_speed);
            }

            if (std::abs(bsp::imu::get_angle()) > M_PI_2 - 0.05) {
                traveled_dist = 0;
                state = 3;
            }
        } else if (state == 3) {
            target_speed = 0;
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());

            if (std::abs(bsp::imu::get_rad_per_s()) < 0.05) {
                state = 4;
            }

        } else if (state == 4) {
            target_speed = std::min((target_speed + Config::linear_acceleration), Config::search_speed);
            float target_rad_s = walls_pid.calculate(0.0, ir_side_wall_error());

            rotation_ratio = -angular_vel_pid.calculate(target_rad_s, bsp::imu::get_rad_per_s());
            if (std::abs(traveled_dist) >= HALF_CELL_SIZE_CM - 0.5) {
                update_position();
                is_finished = true;
            }
        }

        break;
    }

    case Movement::LEFT: {
        // Move half a cell, stop, turn left, stop, move half a cell
        if (state == 0) {
            float elapsed_time = bsp::get_tick_ms() - reference_time;
            if (elapsed_time < 500) {
                target_speed = std::max((target_speed - 1.75f * Config::linear_acceleration), 0.0f);
            } else {
                target_speed = std::min((target_speed + Config::linear_acceleration), Config::min_move_speed);
            }


            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if ((std::abs(traveled_dist) >= HALF_CELL_SIZE_CM - 0.5)) {
                state = 1;
                reference_time = bsp::get_tick_ms();
            }
        } else if (state == 1) {
            target_speed = 0.0;
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (bsp::get_tick_ms() - reference_time > 50) {
                state = 2;
            }
        } else if (state == 2) {
            target_speed = 0;
            if (std::abs(bsp::imu::get_angle()) < M_PI / 4) {
                rotation_ratio = -Config::angular_speed;
            } else {
                rotation_ratio =
                    std::min(float(rotation_ratio + Config::angular_acceleration), -Config::min_turn_speed);
            }

            if (std::abs(bsp::imu::get_angle()) > M_PI_2 - 0.1) {
                traveled_dist = 0;
                state = 3;
            }
        } else if (state == 3) {
            target_speed = 0;
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());

            if (std::abs(bsp::imu::get_rad_per_s()) < 0.05) {
                state = 4;
            }

        } else if (state == 4) {
            target_speed = std::min((target_speed + Config::linear_acceleration), Config::search_speed);
            float target_rad_s = walls_pid.calculate(0.0, ir_side_wall_error());

            rotation_ratio = -angular_vel_pid.calculate(target_rad_s, bsp::imu::get_rad_per_s());
            if (std::abs(traveled_dist) >= HALF_CELL_SIZE_CM - 0.5) {
                update_position();
                is_finished = true;
            }
        }

        break;
    }

    case Movement::TURN_AROUND: {
        // Move half a cell, stop, turn 180, stop, move half a cell
        static int half_turn = false;
        if (state == 0) {
            float elapsed_time = bsp::get_tick_ms() - reference_time;
            if (elapsed_time < 500) {
                target_speed = std::max((target_speed - 1.75f * Config::linear_acceleration), 0.0f);
            } else {
                target_speed = std::min((target_speed + Config::linear_acceleration), Config::min_move_speed);
            }
            
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());

            if ((std::abs(traveled_dist) >= HALF_CELL_SIZE_CM - 0.5)) {
                state = 1;
                reference_time = bsp::get_tick_ms();
            }
        } else if (state == 1) {
            target_speed = 0.0;
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (bsp::get_tick_ms() - reference_time > 50) {
                state = 2;
                half_turn = false;
            }
        } else if (state == 2) {
            target_speed = 0;
            if (std::abs(bsp::imu::get_angle()) < M_PI_2) {
                rotation_ratio = Config::angular_speed;
            } else {
                rotation_ratio = std::max(float(rotation_ratio - Config::angular_acceleration), Config::min_turn_speed);
            }

            if (!half_turn && std::abs(bsp::imu::get_angle()) > M_PI_2) {
                half_turn = true;
            }

            if (std::abs(bsp::imu::get_angle()) > (M_PI - 0.03) ||
                (std::abs(bsp::imu::get_angle()) < (0.15) && half_turn)) {
                traveled_dist = 0;
                state = 3;
            }
        } else if (state == 3) {
            target_speed = 0;
            rotation_ratio = -angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());

            if (std::abs(bsp::imu::get_rad_per_s()) < 0.05) {
                state = 4;
                reference_time = bsp::get_tick_ms();
            }

        } else if (state == 4) {
            // target_speed = -Config::fix_position_speed;
            // rotation_ratio = 0;
            // if (bsp::get_tick_ms() - reference_time > 250) {
                bsp::imu::reset_angle();
                state = 5;
                traveled_dist = 0;
            // }

        } else if (state == 5) {
            target_speed = std::min((target_speed + Config::linear_acceleration), Config::search_speed);
            float angle_error = bsp::imu::get_angle();

            rotation_ratio = -angular_vel_pid.calculate(-angle_error * 0.5, bsp::imu::get_rad_per_s());
            if (std::abs(traveled_dist) >= HALF_CELL_SIZE_CM) {
                update_position();
                is_finished = true;
            }
        }

        break;
    }
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

void Navigation::move(Direction dir, uint8_t cells) {
    bsp::imu::reset_angle();

    if (cells > 1) {
        bsp::leds::stripe_set(Color::Blue);
    }

    current_movement = get_movement(dir);
    traveled_dist = 0;
    state = 0;
    reference_time = bsp::get_tick_ms();
    target_travel = (cells * CELL_SIZE_CM) - 0.5;
    is_finished = false;
}

void Navigation::stop() {
    bsp::imu::reset_angle();

    if (current_direction == Direction::NORTH) {
        target_direction = Direction::SOUTH;
    } else if (current_direction == Direction::EAST) {
        target_direction = Direction::WEST;
    } else if (current_direction == Direction::SOUTH) {
        target_direction = Direction::NORTH;
    } else if (current_direction == Direction::WEST) {
        target_direction = Direction::EAST;
    }

    bsp::leds::stripe_set(Color::Blue);

    current_movement = STOP;
    traveled_dist = 0;
    state = 0;
    reference_time = bsp::get_tick_ms();
    target_travel = HALF_CELL_SIZE_CM;
    is_finished = false;
}

void Navigation::stop_run_mode() {
    bsp::imu::reset_angle();

    bsp::leds::stripe_set(Color::Blue);

    current_movement = FORWARD;
    traveled_dist = 0;
    state = 0;
    reference_time = bsp::get_tick_ms();
    target_travel = HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START;
    is_finished = false;
}

void Navigation::update_position() {
    current_direction = target_direction;
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

    target_direction = target_dir;

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
