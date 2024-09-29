#include "algorithms/pid.hpp"
#include "bsp/analog_sensors.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/core.hpp"
#include "bsp/debug.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "fsm/state.hpp"
#include "services/maze.hpp"
#include "services/position.hpp"
#include "utils/math.hpp"
#include "utils/soft_timer.hpp"
#include <cstdio>

using services::Maze;
using services::Movement;
using services::Position;

namespace fsm {

void PreSearch::enter() {
    bsp::debug::print("state:PreSearch");

    bsp::leds::stripe_set(0, 0, 255, 0);
    bsp::leds::stripe_set(1, 0, 0, 0);

    bsp::motors::set(0, 0);
}

State* PreSearch::react(BleCommand const&) {
    return nullptr;
}

State* PreSearch::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<Idle>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreRun>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<Search>();
    }

    return nullptr;
}

void Search::enter() {
    bsp::debug::print("state:Search");
    bsp::leds::indication_on();

    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    bsp::leds::ir_emitter_all_on();
    bsp::imu::reset_angle();

    soft_timer::start(1, soft_timer::CONTINUOUS);

    stop_counter = 0;

    angular_vel_pid.reset();
    angular_vel_pid.kp = 10.0;
    angular_vel_pid.ki = 0;
    angular_vel_pid.kd = 0;
    angular_vel_pid.integral_limit = 0;

    auto maze = Maze::instance();
    maze->init_step_map({Maze::GOAL_X_POS, Maze::GOAL_Y_POS});
    maze->calculate_step_map({Maze::GOAL_X_POS, Maze::GOAL_Y_POS}, Maze::SEARCH);
}

State* Search::react(BleCommand const&) {
    return nullptr;
}

State* Search::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return nullptr;
    }

    if (event.button == ButtonPressed::SHORT2) {
        return nullptr;
    }

    return nullptr;
}

State* Search::react(Timeout const&) {
    auto position = Position::instance();

    float target_speed = 0;
    float rotation_ratio = 0;
    position->update();
    bsp::imu::update();

    switch (current_movement) {
    case Movement::FORWARD:
        target_speed = 50;
        rotation_ratio = angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
        if (position->get_traveled_cm() > Maze::CELL_SIZE_CM) {
            // dispatch(UpdateMaze());
        }
        break;

    case Movement::RIGHT:
        target_speed = 0;
        rotation_ratio = 30;
        if (std::abs(bsp::imu::get_angle()) > (M_PI_2 - 0.2)) {
            // dispatch(UpdateMaze());
        }
        break;

    case Movement::LEFT:
        target_speed = 0;
        rotation_ratio = -30;
        if (std::abs(bsp::imu::get_angle()) > (M_PI_2 - 0.2)) {
            // dispatch(UpdateMaze());
        }
        break;

    case Movement::BACKWARD:
        target_speed = 0;
        rotation_ratio = -30;
        if (std::abs(bsp::imu::get_angle()) > (M_PI - 0.4)) {
            // dispatch(UpdateMaze());
        }
        break;

    default:
        return &State::get<PreSearch>();
        break;
    }

    float speed_l = target_speed + rotation_ratio;
    float speed_r = target_speed - rotation_ratio;
    bsp::motors::set(speed_l, speed_r);

    // if (stop_counter++ > 300) {
    //     return &State::get<PreSearch>();
    // }

    return nullptr;
}

State* Search::react(UpdateMaze const&) {
    using bsp::analog_sensors::ir_reading_wall;
    using bsp::analog_sensors::SensingDirection;

    bool front_seeing = ir_reading_wall(SensingDirection::FRONT_LEFT) || ir_reading_wall(SensingDirection::FRONT_RIGHT);
    bool right_seeing = ir_reading_wall(SensingDirection::RIGHT);
    bool left_seeing = ir_reading_wall(SensingDirection::LEFT);

    auto maze = Maze::instance();
    auto position = Position::instance();

    auto robot_pos = position->get_robot_position();
    auto robot_dir = position->get_robot_direction();

    // Goal reached
    if (robot_pos.x == Maze::GOAL_X_POS && robot_pos.y == Maze::GOAL_Y_POS) {
        return &State::get<PreSearch>();
    }

    maze->update_wall_map(robot_pos, robot_dir, front_seeing, right_seeing, left_seeing);
    maze->init_step_map({Maze::GOAL_X_POS, Maze::GOAL_Y_POS});
    maze->calculate_step_map({Maze::GOAL_X_POS, Maze::GOAL_Y_POS}, Maze::SEARCH);

    auto next_direction = maze->get_target_cell_dir(robot_pos, robot_dir, Maze::SEARCH);
    current_movement = maze->get_target_movement(robot_dir, next_direction);

    // 7. prepare velocity and params for next movement

    return nullptr;
}

void Search::exit() {
    bsp::motors::set(0, 0);
    bsp::leds::ir_emitter_all_off();
    bsp::leds::indication_off();
}

}
