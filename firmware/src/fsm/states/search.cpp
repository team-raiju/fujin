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
#include "services/navigation.hpp"
#include "utils/math.hpp"
#include "utils/soft_timer.hpp"
#include <cstdio>

using services::Maze;
using services::Navigation;

static uint8_t cell_wall(Direction robot_dir, bool front_seeing, bool right_seeing, bool left_seeing) {

    uint8_t walls = 0;
    switch (robot_dir) {
    case NORTH:
        walls = front_seeing * N | right_seeing * E | left_seeing * W;
        break;
    case EAST:
        walls = front_seeing * E | right_seeing * S | left_seeing * N;
        break;
    case SOUTH:
        walls = front_seeing * S | right_seeing * W | left_seeing * E;
        break;
    case WEST:
        walls = front_seeing * W | right_seeing * N | left_seeing * S;
        break;
    }
    return walls;
}

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
    movement_state = 0;

    angular_vel_pid.reset();
    angular_vel_pid.kp = 10.0;
    angular_vel_pid.ki = 0;
    angular_vel_pid.kd = 0;
    angular_vel_pid.integral_limit = 0;

    walls_pid.reset();
    walls_pid.kp = 10.0;
    walls_pid.ki = 0;
    walls_pid.kd = 0;
    walls_pid.integral_limit = 0;

    target_speed = 0;
    rotation_ratio = 0;
    target_travel = Navigation::HALF_CELL_SIZE_CM;

    // auto maze = Maze::instance();
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
    using bsp::analog_sensors::ir_reading;
    using bsp::analog_sensors::SensingDirection;

    auto navigation = Navigation::instance();

    bool update_maze = false;
    navigation->update();
    bsp::imu::update();

    auto left_ir = ir_reading(SensingDirection::LEFT);
    auto right_ir = ir_reading(SensingDirection::RIGHT);

    switch (current_movement) {
    case Movement::FORWARD:
        target_speed = 50;
        rotation_ratio = angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
        rotation_ratio += walls_pid.calculate(0.0, (left_ir - right_ir));
        if (navigation->get_traveled_cm() > target_travel) {
            navigation->move(current_movement);
            update_maze = true;
        }
        break;

    case Movement::RIGHT:
        // Move half a cell, turn right, move half a cell
        if (movement_state == 0) {
            target_speed = std::min(target_speed - 0.05, 0.0);
            rotation_ratio = angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (navigation->get_traveled_cm() > Navigation::HALF_CELL_SIZE_CM) {
                movement_state = 1;
            }
        } else if (movement_state == 1) {
            target_speed = 0;
            rotation_ratio = 30;
            if (std::abs(bsp::imu::get_angle()) > (M_PI_2 - 0.2)) {
                navigation->set_traveled_cm(0);
                movement_state = 2;
            }
        } else if (movement_state == 2) {
            target_speed = std::max(target_speed + 0.05, 50.0);
            rotation_ratio = angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (navigation->get_traveled_cm() > Navigation::HALF_CELL_SIZE_CM) {
                navigation->move(current_movement);
                update_maze = true;
            }
        }

        break;

    case Movement::LEFT:
        // Move half a cell, turn left, move half a cell
        if (movement_state == 0) {
            target_speed = std::min(target_speed - 0.05, 0.0);
            rotation_ratio = angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (navigation->get_traveled_cm() > Navigation::HALF_CELL_SIZE_CM) {
                movement_state = 1;
            }
        } else if (movement_state == 1) {
            target_speed = 0;
            rotation_ratio = -30;
            if (std::abs(bsp::imu::get_angle()) > (M_PI_2 - 0.2)) {
                navigation->set_traveled_cm(0);
                movement_state = 2;
            }
        } else if (movement_state == 2) {
            target_speed = std::max(target_speed + 0.05, 50.0);
            rotation_ratio = angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());
            if (navigation->get_traveled_cm() > Navigation::HALF_CELL_SIZE_CM) {
                navigation->move(current_movement);
                update_maze = true;
            }
        }
        break;

    case Movement::TURN_AROUND:
        target_speed = 0;
        rotation_ratio = -30;
        if (std::abs(bsp::imu::get_angle()) > (M_PI - 0.4)) {
            navigation->move(current_movement);
            update_maze = true;
        }
        break;
    }

    float speed_l = target_speed + rotation_ratio;
    float speed_r = target_speed - rotation_ratio;
    bsp::motors::set(speed_l, speed_r);

    if (update_maze) {
        using bsp::analog_sensors::ir_reading_wall;
        using bsp::analog_sensors::SensingDirection;
        bool front_seeing =
            ir_reading_wall(SensingDirection::FRONT_LEFT) || ir_reading_wall(SensingDirection::FRONT_RIGHT);
        bool right_seeing = ir_reading_wall(SensingDirection::RIGHT);
        bool left_seeing = ir_reading_wall(SensingDirection::LEFT);
        auto maze = Maze::instance();
        auto navigation = Navigation::instance();

        auto robot_pos = navigation->get_robot_position();
        auto robot_dir = navigation->get_robot_direction();

        uint8_t walls = cell_wall(robot_dir, front_seeing, right_seeing, left_seeing);

        auto dir = maze->next_step(robot_pos, walls, false);
        current_movement = navigation->get_target_movement(robot_dir, dir);
        navigation->set_traveled_cm(0);
        bsp::imu::reset_angle();
        movement_state = 0;
        target_travel = Navigation::CELL_SIZE_CM;

        if (robot_pos == services::Maze::GOAL_POS) {
            return &State::get<PreSearch>();
        }
    }

    // if (stop_counter++ > 300) {
    //     return &State::get<PreSearch>();
    // }

    return nullptr;
}

void Search::exit() {
    bsp::motors::set(0, 0);
    bsp::leds::ir_emitter_all_off();
    bsp::leds::indication_off();
    soft_timer::stop();
}

}
