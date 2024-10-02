#include <cstdio>

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

using services::Maze;
using services::Navigation;

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

Search::Search() {
    navigation = services::Navigation::instance();
    maze = services::Maze::instance();
}

void Search::enter() {
    bsp::debug::print("state:Search");
    bsp::leds::indication_on();
    bsp::leds::ir_emitter_all_on();

    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    soft_timer::start(1, soft_timer::CONTINUOUS);

    navigation->init();
}

State* Search::react(BleCommand const&) {
    return nullptr;
}

State* Search::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreSearch>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreSearch>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<PreSearch>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreSearch>();
    }

    return nullptr;
}

State* Search::react(Timeout const&) {
    using bsp::analog_sensors::ir_reading_wall;
    using bsp::analog_sensors::SensingDirection;

    navigation->update();
    bool done = navigation->step();

    if (done) {
        bool front_seeing =
            ir_reading_wall(SensingDirection::FRONT_LEFT) || ir_reading_wall(SensingDirection::FRONT_RIGHT);
        bool right_seeing = ir_reading_wall(SensingDirection::RIGHT);
        bool left_seeing = ir_reading_wall(SensingDirection::LEFT);

        auto robot_pos = navigation->get_robot_position();
        auto robot_dir = navigation->get_robot_direction();

        uint8_t walls = (front_seeing * N | right_seeing * E | left_seeing * W) << robot_dir;

        auto dir = maze->next_step(robot_pos, walls, false);
        navigation->move(dir);

        if (robot_pos == services::Maze::GOAL_POS) {
            return &State::get<PreSearch>();
        }
    }

    return nullptr;
}

void Search::exit() {
    bsp::motors::set(0, 0);
    bsp::leds::ir_emitter_all_off();
    bsp::leds::indication_off();
    soft_timer::stop();
}

}
