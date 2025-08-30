#include <cstdio>

#include "algorithms/pid.hpp"
#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
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
#include "services/notification.hpp"
#include "utils/math.hpp"
#include "utils/soft_timer.hpp"

using bsp::leds::Color;
using services::Maze;
using services::Navigation;

namespace fsm {

void PreSearch::enter() {

    bsp::debug::print("state:PreSearch");

    bsp::leds::stripe_set(0, Color::Green);
    bsp::leds::stripe_set(1, Color::Black);
    bsp::leds::stripe_send();

    bsp::motors::set(0, 0);

    /* Only start IR if powered by the battery */
    if (bsp::analog_sensors::battery_latest_reading_mv() > 7000) {
        soft_timer::start(100, soft_timer::SINGLE);
    }
}

State* PreSearch::react(Timeout const&) {
    using bsp::analog_sensors::ir_reading_wall;
    using bsp::analog_sensors::SensingDirection;

    bsp::leds::ir_emitter_on(bsp::leds::LEFT_FRONT);
    bsp::leds::ir_emitter_on(bsp::leds::RIGHT_FRONT);
    bsp::analog_sensors::enable_modulation();
    bsp::delay_ms(5);

    for (int i = 0; i < 400; i++) {
        if (!ir_reading_wall(SensingDirection::FRONT_LEFT) || !ir_reading_wall(SensingDirection::FRONT_RIGHT)) {
            soft_timer::start(100, soft_timer::SINGLE);
            bsp::leds::ir_emitter_all_off();
            bsp::analog_sensors::enable_modulation(false);
            return &State::get<PreSearch>();
        }
        bsp::delay_ms(1);
    }

    return &State::get<Search>();
}

State* PreSearch::react(BleCommand const&) {
    return nullptr;
}

State* PreSearch::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreCalib>();
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
    notification = services::Notification::instance();
}

void Search::enter() {
    bsp::debug::print("state:Search");
    bsp::leds::indication_on();
    bsp::leds::stripe_set(Color::Red);
    bsp::leds::ir_emitter_all_on();
    bsp::analog_sensors::enable_modulation();

    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    soft_timer::start(1, soft_timer::CONTINUOUS);

    navigation->reset(true);

    returning = false;
    save_maze = false;
    stop_next_move = false;
}

State* Search::react(BleCommand const&) {
    return nullptr;
}

State* Search::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<Idle>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<Idle>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<Idle>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<Idle>();
    }

    return nullptr;
}

static bool indicate_read = false;
static uint32_t last_indication = 0;

State* Search::react(Timeout const&) {
    using bsp::analog_sensors::ir_reading_wall;
    using bsp::analog_sensors::SensingDirection;
    using bsp::analog_sensors::SensingStatus;

    if (indicate_read && bsp::get_tick_ms() - last_indication > 150) {
        bsp::leds::stripe_set(Color::Black);
        indicate_read = false;
        bsp::buzzer::stop();
    }

    notification->update();
    navigation->update();
    bool done = navigation->step();

    if (done) {
        if (stop_next_move) {
            return &State::get<Idle>();
        }

        SensingStatus sensingStatus = bsp::analog_sensors::ir_get_sensing_status();

        last_indication = bsp::get_tick_ms();
        indicate_read = true;
        if (sensingStatus.front_seeing) {
            bsp::leds::stripe_set(0, sensingStatus.left_seeing ? Color::Blue : Color::Red);
            bsp::leds::stripe_set(1, sensingStatus.right_seeing ? Color::Blue : Color::Red);
        } else {
            bsp::leds::stripe_set(0, sensingStatus.left_seeing ? Color::Green : Color::Black);
            bsp::leds::stripe_set(1, sensingStatus.right_seeing ? Color::Green : Color::Black);
        }

        bsp::leds::stripe_send();

        auto robot_pos = navigation->get_robot_position();
        auto robot_dir = navigation->get_robot_direction();

        uint8_t walls = (sensingStatus.front_seeing * N | sensingStatus.right_seeing * E | sensingStatus.left_seeing * W) << robot_dir;

        bool goal_reached =
            std::any_of(std::begin(services::Maze::GOAL_POSITIONS), std::end(services::Maze::GOAL_POSITIONS),
                        [&](const Point& goal) { return goal == robot_pos; });

        if (goal_reached && !returning) {
            bsp::buzzer::start();
            save_maze = true;
            returning = true;
        }

        if (robot_pos == services::Maze::ORIGIN && returning) {
            navigation->set_movement(Movement::STOP, Movement::STOP, Movement::STOP, 1);
            stop_next_move = true;
        } else {
            auto dir = maze->next_step(robot_pos, walls, returning, true);
            navigation->move(dir);
        }
    }

    return nullptr;
}

void Search::exit() {
    bsp::motors::set(0, 0);
    bsp::analog_sensors::enable_modulation(false);
    bsp::leds::ir_emitter_all_off();
    bsp::leds::indication_off();
    soft_timer::stop();
    if (save_maze) {
        bsp::buzzer::start();
        maze->save_maze_to_memory();
        bsp::delay_ms(500);
        bsp::buzzer::stop();
    }
}

}
