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
#include "services/position_service.hpp"
#include "utils/math.h"
#include "utils/pid.hpp"
#include "utils/soft_timer.hpp"
#include <cstdio>

using namespace services::position;
using namespace bsp;

namespace fsm {

void PreSearch::enter() {
    debug::print("state:PreSearch");

    leds::stripe_set(0, 0, 255, 0);
    leds::stripe_set(1, 0, 0, 0);

    motors::set(0, 0);
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
    debug::print("state:Search");
    leds::indication_on();

    buzzer::start();
    delay_ms(2000);
    buzzer::stop();

    leds::ir_emmiter_all_on();
    imu::reset_angle();
    soft_timer::start(1, soft_timer::CONTINUOUS);
    stop_counter = 0;
    current_movement = FORWARD;
    angular_vel_pid.reset();
    angular_vel_pid.update_parameters(10.0, 0.0, 0.0, 0.0);
    maze.init_step_map({Maze::GOAL_X_POS, Maze::GOAL_Y_POS});
    maze.calculate_step_map({Maze::GOAL_X_POS, Maze::GOAL_Y_POS}, Maze::SEARCH);
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

    float target_speed = 0;
    float rotation_ratio = 0;
    services::position::update();
    imu::update();

    switch (current_movement) {
    case FORWARD:
        target_speed = 50;
        rotation_ratio = angular_vel_pid.calculate(0.0, imu::get_rad_per_s());
        if (get_travelled_dist_cm() > Maze::CELL_SIZE_CM) {
            // dispatch(UpdateMaze());
        }
        break;
    
    case RIGHT:
        target_speed = 0;
        rotation_ratio = 30;
        if (utils_abs(bsp::imu::get_angle()) > (M_PI_2 - 0.2)) {
            // dispatch(UpdateMaze());
        }
        break;
    
    case LEFT:
        target_speed = 0;
        rotation_ratio = -30;
        if (utils_abs(bsp::imu::get_angle()) > (M_PI_2 - 0.2)) {
            // dispatch(UpdateMaze());
        }
        break;
    
    case BACKWARD:
        target_speed = 0;
        rotation_ratio = -30;
        if (utils_abs(bsp::imu::get_angle()) > (M_PI - 0.4)) {
            // dispatch(UpdateMaze());
        }
        break;
    default:
        return &State::get<PreSearch>();
        break;
    }

    float speed_l = target_speed + rotation_ratio;
    float speed_r = target_speed - rotation_ratio;
    motors::set(speed_l, speed_r);

    // if (stop_counter++ > 300) {
    //     return &State::get<PreSearch>();
    // }

    return nullptr;
}

State* Search::react(UpdateMaze const&) {
    bool front_seeing = analog_sensors::ir_reading_wall(analog_sensors::FRONT_LEFT) ||
                        analog_sensors::ir_reading_wall(analog_sensors::FRONT_RIGHT);
    bool right_seeing = analog_sensors::ir_reading_wall(analog_sensors::RIGHT);
    bool left_seeing = analog_sensors::ir_reading_wall(analog_sensors::LEFT);

    Position robot_pos = get_robot_position();
    Direction robot_dir = get_robot_direction();

    // Goal reached
    if (robot_pos.x == Maze::GOAL_X_POS && robot_pos.y == Maze::GOAL_Y_POS) {
        return &State::get<PreSearch>();
    }

    maze.update_wall_map(robot_pos, robot_dir, front_seeing, right_seeing, left_seeing);
    maze.init_step_map({Maze::GOAL_X_POS, Maze::GOAL_Y_POS});
    maze.calculate_step_map({Maze::GOAL_X_POS, Maze::GOAL_Y_POS}, Maze::SEARCH);

    Direction next_direction = maze.get_target_cell_dir(robot_pos, robot_dir, Maze::SEARCH);
    current_movement = maze.get_target_movement(robot_dir, next_direction);

    // 7. prepare velocity and params for next movement

    return nullptr;
}

void Search::exit() {
    motors::set(0, 0);
    leds::ir_emmiter_all_off();
    leds::indication_off();
}

}
