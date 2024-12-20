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

static constexpr float WHEEL_RADIUS_CM = (1.275);
static constexpr float WHEEL_PERIMETER_CM = (M_TWOPI * WHEEL_RADIUS_CM);

static constexpr float WHEEL_TO_ENCODER_RATIO = (1.0);
static constexpr float ENCODER_PPR = (1024.0);
static constexpr float PULSES_PER_WHEEL_ROTATION = (WHEEL_TO_ENCODER_RATIO * ENCODER_PPR);

static constexpr float WHEELS_DIST_CM = (7.0);
static constexpr float ENCODER_DIST_CM_PULSE = (WHEEL_PERIMETER_CM / PULSES_PER_WHEEL_ROTATION);
static constexpr float ENCODER_DIST_MM_PULSE = (ENCODER_DIST_CM_PULSE * 10.0);

static constexpr float CELL_SIZE_CM = 18.0;
static constexpr float HALF_CELL_SIZE_CM = 9.0;

static constexpr float ROBOT_DIST_FROM_CENTER_START = 2.0;

static constexpr float MM_PER_US_TO_M_PER_S = 1000.0; // 1mm/us = 1000m/s

static constexpr float CONTROL_FREQUENCY_HZ = 1000.0;

using bsp::leds::Color;

/// @section Service implementation

namespace services {

Navigation* Navigation::instance() {
    static Navigation p;
    return &p;
}

void Navigation::init(void) {
    reset();
    control = Control::instance();

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

    traveled_dist_cm = 0;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    current_position = {0, 0};
    current_direction = Direction::NORTH;
    optimize_turns = false;

    state = 0;

    is_finished = false;
    control->reset();

    target_travel = HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START;
    current_movement = Movement::FORWARD;
    reference_time = bsp::get_tick_ms();
    bsp::encoders::set_linear_velocity_m_s(0);
}

void Navigation::optimize(bool opt) {
    optimize_turns = opt;
}

float Navigation::get_torricelli_distance(float final_speed, float initial_speed, float acceleration) {
    return (final_speed * final_speed - initial_speed * initial_speed) / (2 * acceleration);
}

void Navigation::update(void) {
    static uint32_t last_update_vel_time_us = 0;

    uint32_t current_time_us = bsp::get_tick_us();
    uint32_t delta_time_us = current_time_us - last_update_vel_time_us;

    bsp::imu::update();

    if (encoder_left_counter == 0 && encoder_right_counter == 0) {
        // 50ms without movement
        // min meaasured vel is ENCODER_DIST_MM_PULSE / 50 = 0.00156 m/s
        if (delta_time_us > 50000) {
            bsp::encoders::set_linear_velocity_m_s(0);
        } else {
            float velocity_m_s = (ENCODER_DIST_MM_PULSE / delta_time_us) * MM_PER_US_TO_M_PER_S;
            bsp::encoders::set_linear_velocity_m_s(velocity_m_s);
        }
        return;
    }

    float estimated_delta_l_mm = (encoder_left_counter * ENCODER_DIST_MM_PULSE);
    float estimated_delta_r_mm = (encoder_right_counter * ENCODER_DIST_MM_PULSE);
    float delta_x_mm = (estimated_delta_l_mm + estimated_delta_r_mm) / 2.0;
    traveled_dist_cm += delta_x_mm / 10.0;

    float velocity_right_m_s = (estimated_delta_r_mm / delta_time_us) * MM_PER_US_TO_M_PER_S;
    float velocity_left_m_s = (estimated_delta_l_mm / delta_time_us) * MM_PER_US_TO_M_PER_S;
    float velocity_m_s = (velocity_right_m_s + velocity_left_m_s) / 2.0;
    bsp::encoders::set_linear_velocity_m_s(velocity_m_s);
    last_update_vel_time_us = current_time_us;

    encoder_left_counter = 0;
    encoder_right_counter = 0;
}

// TODO: Improve the mini FSM with state names and better transitions
bool Navigation::step() {
    using bsp::analog_sensors::ir_side_wall_error;
    using bsp::analog_sensors::SensingDirection;

    switch (current_movement) {
    case Movement::STOP: {

        // Move half a cell, stop, turn 180, stop, move half a cell
        break;
    }

    case Movement::FORWARD: {
        using bsp::analog_sensors::ir_reading;

        bool front_emergency = ir_reading(SensingDirection::FRONT_LEFT) > 2850 &&
                               ir_reading(SensingDirection::FRONT_RIGHT) > 2850 &&
                               ir_reading(SensingDirection::LEFT) > 2470 && ir_reading(SensingDirection::RIGHT) > 2850;

        float control_linear_speed = control->get_target_linear_speed();
        float final_speed = Config::min_move_speed;

        if (std::abs(traveled_dist_cm) <
            (target_travel -
             (100 * get_torricelli_distance(final_speed, Config::search_speed, -Config::linear_acceleration)))) {
            if (control_linear_speed < Config::search_speed) {
                control_linear_speed += Config::linear_acceleration / CONTROL_FREQUENCY_HZ;
                if (control_linear_speed > Config::search_speed) {
                    control_linear_speed = Config::search_speed;
                }
            }
        } else {
            if (control_linear_speed > final_speed) {
                control_linear_speed -= Config::linear_acceleration / CONTROL_FREQUENCY_HZ;
                if (control_linear_speed < final_speed) {
                    control_linear_speed = final_speed;
                }
            }
        }

        control->set_target_linear_speed(control_linear_speed);
        control->set_target_angular_speed(0);
        control->set_wall_pid_enabled(true);

        if (std::abs(traveled_dist_cm) >= target_travel || front_emergency) {
            bsp::leds::stripe_set(Color::Red);
            is_finished = true;
        }

        break;
    }

    case Movement::RIGHT: {
        // Move 70ms foward
        // Angular accel 16ms  (total time 86ms)
        // max angular vel 145ms (total time 231ms)
        // Angular -accel 16ms (total time 247ms)
        // Move 70ms foward (total time 317ms)

        float elapsed_time = bsp::get_tick_ms() - reference_time;
        control->set_target_linear_speed(Config::search_speed);
        control->set_wall_pid_enabled(false);

        if (elapsed_time <= 70) {
            control->set_target_angular_speed(0);
        } else if (elapsed_time <= 86) {
            float ideal_rad_s = Config::angular_acceleration * elapsed_time / 1000.0;
            ideal_rad_s = std::min(ideal_rad_s, Config::angular_speed);
            control->set_target_angular_speed(-ideal_rad_s);
        } else if (elapsed_time <= 231) {
            float ideal_rad_s = Config::angular_speed;
            control->set_target_angular_speed(-ideal_rad_s);
        } else if (elapsed_time <= 247) {
            float ideal_rad_s =
                Config::angular_speed - (Config::angular_acceleration * (elapsed_time - (247)) / 1000.0);
            ideal_rad_s = std::max(ideal_rad_s, 0.0f);
            control->set_target_angular_speed(-ideal_rad_s);
        } else if (elapsed_time <= 317) {
            control->set_target_angular_speed(0);
        } else {
            is_finished = true;
        }

        break;
    }

    case Movement::LEFT: {
        // Move half a cell, stop, turn right, stop, move half a cell

        float elapsed_time = bsp::get_tick_ms() - reference_time;
        control->set_target_linear_speed(Config::search_speed);
        control->set_wall_pid_enabled(false);

        if (elapsed_time <= 16) {
            float ideal_rad_s = Config::angular_acceleration * elapsed_time / 1000.0;
            ideal_rad_s = std::min(ideal_rad_s, Config::angular_speed);
            control->set_target_angular_speed(ideal_rad_s);
        } else if (elapsed_time <= 161) {
            float ideal_rad_s = Config::angular_speed;
            control->set_target_angular_speed(ideal_rad_s);
        } else if (elapsed_time <= 177) {
            float ideal_rad_s = Config::angular_speed - (Config::angular_acceleration * (elapsed_time - 161) / 1000.0);
            ideal_rad_s = std::max(ideal_rad_s, 0.0f);
            control->set_target_angular_speed(ideal_rad_s);
        } else {
            is_finished = true;
        }

        break;
    }

    case Movement::TURN_AROUND: {
        // Move half a cell, stop, turn 180, stop, move half a cell
        break;
    }
    }

    if (is_finished) {
        update_position();
    }

    control->update();

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

    traveled_dist_cm = 0;
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
    traveled_dist_cm = 0;
    state = 0;
    reference_time = bsp::get_tick_ms();
    target_travel = HALF_CELL_SIZE_CM;
    is_finished = false;
}

void Navigation::stop_run_mode() {
    bsp::imu::reset_angle();

    bsp::leds::stripe_set(Color::Blue);

    current_movement = FORWARD;
    traveled_dist_cm = 0;
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
