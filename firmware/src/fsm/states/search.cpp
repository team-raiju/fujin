#include <cstdio>

#include "algorithms/pid.hpp"
#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/core.hpp"
#include "bsp/debug.hpp"
#include "bsp/fan.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "fsm/state.hpp"
#include "services/control.hpp"
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

    bsp::leds::stripe_set(Color::Green, Color::Black);
    bsp::leds::stripe_send();

    bsp::motors::set(0, 0);
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
        return &State::get<SearchParamSelect>();
    }

    return nullptr;
}

SearchParamSelect::SearchParamSelect() {
    navigation = services::Navigation::instance();
    param_type = PARAM_CUSTOM;
}

void SearchParamSelect::enter() {
    bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
    param_type = PARAM_CUSTOM;
    bsp::debug::print("state:SearchParamSelect");
}

State* SearchParamSelect::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT2) {
        if (param_type == PARAM_CUSTOM) {
            param_type = PARAM_SLOW;
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Blue);
        } else if (param_type == PARAM_SLOW) {
            param_type = PARAM_MEDIUM;
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Black);
        } else if (param_type == PARAM_MEDIUM) {
            param_type = PARAM_FAST;
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Orange);
        } else { // param_type == PARAM_FAST
            param_type = PARAM_CUSTOM;
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
        }
        return nullptr;
    }

    if (event.button == ButtonPressed::SHORT1) {
        if (param_type == PARAM_CUSTOM) {
            param_type = PARAM_FAST;
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Orange);
        } else if (param_type == PARAM_FAST) {
            param_type = PARAM_MEDIUM;
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Black);
        } else if (param_type == PARAM_MEDIUM) {
            param_type = PARAM_SLOW;
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Blue);
        } else { // param_type == PARAM_SLOW
            param_type = PARAM_CUSTOM;
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
        }
        return nullptr;
    }

    if (event.button == ButtonPressed::LONG1) {
        switch (param_type) {
        case PARAM_CUSTOM:
            navigation->reset(services::Navigation::CUSTOM);
            std::printf("Custom params\r\n");
            break;
        case PARAM_SLOW:
            navigation->reset(services::Navigation::SEARCH_SLOW);
            std::printf("Slow params\r\n");
            break;
        case PARAM_MEDIUM:
            navigation->reset(services::Navigation::SEARCH_MEDIUM);
            std::printf("Medium params\r\n");
            break;
        case PARAM_FAST:
            navigation->reset(services::Navigation::SEARCH_FAST);
            std::printf("Fast params\r\n");
            break;
        }
        return &State::get<SearchWaitStart>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreSearch>();
    }
    return nullptr;
}

void SearchWaitStart::enter() {

    bsp::debug::print("state:SearchWaitStart");

    bsp::leds::stripe_set(Color::White);

    bsp::motors::set(0, 0);
    bsp::fan::set(0);

    /* Only start IR if powered by the battery */
    if (bsp::analog_sensors::battery_latest_reading_mv() > 7000) {
        soft_timer::start(100, soft_timer::SINGLE);
    }
}

State* SearchWaitStart::react(Timeout const&) {
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
            return &State::get<SearchWaitStart>();
        }
        bsp::delay_ms(1);
    }

    return &State::get<Search>();
}

State* SearchWaitStart::react(BleCommand const&) {
    return nullptr;
}

State* SearchWaitStart::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::LONG1) {
        return &State::get<Search>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreSearch>();
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

    returning = false;
    save_maze = false;
    stop_next_move = false;
    emergency = false;
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

    if (indicate_read && ((bsp::get_tick_ms() - last_indication) > 150)) {
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
            // stop_next_move = false;
            // returning = false;
            // maze->reset();
        }

        SensingStatus sensingStatus = bsp::analog_sensors::ir_get_sensing_status();

        last_indication = bsp::get_tick_ms();
        indicate_read = true;
        bsp::leds::stripe_set(Color::Black);
        bsp::leds::stripe_set(Color::Green);

        auto robot_cell_pos = navigation->get_robot_cell_position();
        auto robot_dir = navigation->get_robot_direction();

        uint8_t walls =
            (sensingStatus.front_seeing * N | sensingStatus.right_seeing * E | sensingStatus.left_seeing * W)
            << robot_dir;

        bool goal_reached =
            std::any_of(std::begin(services::Maze::GOAL_POSITIONS), std::end(services::Maze::GOAL_POSITIONS),
                        [&](const Point& goal) { return goal == robot_cell_pos; });

        if (goal_reached && !returning) {
            bsp::buzzer::start();
            save_maze = true;
            returning = true;
        }

        if (robot_cell_pos == services::Maze::ORIGIN && returning) {
            navigation->set_movement(Movement::TURN_AROUND_INPLACE, Movement::FORWARD, Movement::STOP, 1);
            // navigation->set_movement(Direction::NORTH);
            stop_next_move = true;
        } else {
            auto dir = maze->next_step(robot_cell_pos, walls, returning, true);
            navigation->set_movement(dir);
        }
    }

    if ((bsp::imu::is_imu_emergency() || services::Control::instance()->is_emergency())) {
        emergency = true;
        soft_timer::stop();
        bsp::motors::set(0, 0);
        bsp::fan::set(0);
        bsp::leds::stripe_set(Color::Orange);
        bsp::buzzer::start();
        bsp::delay_ms(500);
        bsp::buzzer::stop();
        return &State::get<Idle>();
    }

    return nullptr;
}

void Search::exit() {
    bsp::motors::set(0, 0);
    if (!emergency) {
        services::Control::instance()->stop_fan();
    }
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
