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
#include "services/config.hpp"
#include "services/control.hpp"
#include "services/logger.hpp"
#include "services/maze.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"
#include "utils/soft_timer.hpp"

using bsp::leds::Color;

namespace fsm {

static bool indicate_read = false;
static uint32_t last_indication = 0;
static services::Navigation::target_movement_mode_t move_mode = services::Navigation::SMOOTH;

void PreRun::enter() {

    bsp::debug::print("state:PreRun");

    bsp::leds::stripe_set(Color::Green);

    bsp::motors::set(0, 0);
    bsp::fan::set(0);
}

State* PreRun::react(BleCommand const&) {
    return nullptr;
}

State* PreRun::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreSearch>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<Idle>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<RunParamSelect>();
    }

    return nullptr;
}

RunParamSelect::RunParamSelect() {
    navigation = services::Navigation::instance();
    param_type = PARAM_CUSTOM;
}

void RunParamSelect::enter() {
    bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
    param_type = PARAM_CUSTOM;
    bsp::debug::print("state:RunParamSelect");
}

State* RunParamSelect::react(ButtonPressed const& event) {
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
            navigation->reset(services::Navigation::SLOW);
            std::printf("Slow params\r\n");
            break;
        case PARAM_MEDIUM:
            navigation->reset(services::Navigation::MEDIUM);
            std::printf("Medium params\r\n");
            break;
        case PARAM_FAST:
            navigation->reset(services::Navigation::FAST);
            std::printf("Fast params\r\n");
            break;
        }
        return &State::get<RunMoveModeSelect>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreRun>();
    }
    return nullptr;
}

RunMoveModeSelect::RunMoveModeSelect() {
    move_mode = services::Navigation::SMOOTH;
}

void RunMoveModeSelect::enter() {
    // When entering the state, set the LED for the default selection: SMOOTH (Green, Black)
    bsp::leds::stripe_set(bsp::leds::Color::Pink, bsp::leds::Color::Black);
    move_mode = services::Navigation::SMOOTH;
    std::printf("state:RunMoveModeSelect\r\n");
}

State* RunMoveModeSelect::react(ButtonPressed const& event) {

    if (event.button == ButtonPressed::SHORT2) {
        if (move_mode == services::Navigation::SMOOTH) {
            move_mode = services::Navigation::DIAGONALS;
            bsp::leds::stripe_set(bsp::leds::Color::Pink, bsp::leds::Color::Pink);
        } else if (move_mode == services::Navigation::DIAGONALS) {
            move_mode = services::Navigation::HARD_CODED;
            bsp::leds::stripe_set(bsp::leds::Color::Red, bsp::leds::Color::Black);
        } else { // move_mode == services::Navigation::HARD_CODED
            move_mode = services::Navigation::SMOOTH;
            bsp::leds::stripe_set(bsp::leds::Color::Pink, bsp::leds::Color::Black);
        }
        return nullptr;
    }

    if (event.button == ButtonPressed::SHORT1) {
        if (move_mode == services::Navigation::SMOOTH) {
            move_mode = services::Navigation::HARD_CODED;
            bsp::leds::stripe_set(bsp::leds::Color::Red, bsp::leds::Color::Black);
        } else if (move_mode == services::Navigation::HARD_CODED) {
            move_mode = services::Navigation::DIAGONALS;
            bsp::leds::stripe_set(bsp::leds::Color::Pink, bsp::leds::Color::Pink);
        } else { // move_mode == services::Navigation::DIAGONALS
            move_mode = services::Navigation::SMOOTH;
            bsp::leds::stripe_set(bsp::leds::Color::Pink, bsp::leds::Color::Black);
        }
        return nullptr;
    }

    if (event.button == ButtonPressed::LONG1) {
        std::printf("Selected move mode: ");
        switch (move_mode) {
        case services::Navigation::SMOOTH:
            std::printf("SMOOTH\r\n");
            break;
        case services::Navigation::DIAGONALS:
            std::printf("DIAGONALS\r\n");
            break;
        case services::Navigation::HARD_CODED:
            std::printf("HARD_CODED\r\n");
            break;
        default:
            break;
        }
        return &State::get<RunWaitStart>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<RunParamSelect>();
    }

    return nullptr;
}

void RunWaitStart::enter() {

    bsp::debug::print("state:RunWaitStart");

    bsp::leds::stripe_set(Color::Green);

    bsp::motors::set(0, 0);
    bsp::fan::set(0);

    /* Only start IR if powered by the battery */
    if (bsp::analog_sensors::battery_latest_reading_mv() > 7000) {
        soft_timer::start(100, soft_timer::SINGLE);
    }
}

State* RunWaitStart::react(Timeout const&) {
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
            return &State::get<RunWaitStart>();
        }
        bsp::delay_ms(1);
    }

    return &State::get<Run>();
}

State* RunWaitStart::react(BleCommand const&) {
    return nullptr;
}

State* RunWaitStart::react(ButtonPressed const& event) {

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<Run>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<RunMoveModeSelect>();
    }

    return nullptr;
}

Run::Run() {
    navigation = services::Navigation::instance();
    maze = services::Maze::instance();
    logger = services::Logger::instance();
}

void Run::enter() {
    bsp::debug::print("state:Run");
    bsp::leds::indication_on();
    bsp::leds::stripe_set(Color::Red);
    bsp::leds::ir_emitter_all_on();
    bsp::analog_sensors::enable_modulation();

    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    services::Control::instance()->start_fan();
    bsp::delay_ms(250);

    soft_timer::start(1, soft_timer::CONTINUOUS);

    logger->init();

    maze->read_maze_from_memory();
    maze->print(maze->ORIGIN);
    target_directions = maze->directions_to_goal();
    maze->print(maze->ORIGIN);

    target_movements.clear();

    // services::Navigation::instance()->set_hardcoded_movements(
    //     {{Movement::START, 1}, {Movement::FORWARD, 1}, {Movement::STOP, 1}});

    target_movements = navigation->get_movements_to_goal(target_directions, move_mode);

    move_count = 0;
    emergency = false;
}

State* Run::react(ButtonPressed const& event) {
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

State* Run::react(BleCommand const&) {
    return nullptr;
}

State* Run::react(Timeout const&) {
    using bsp::analog_sensors::ir_reading_wall;
    using bsp::analog_sensors::SensingDirection;

    if (indicate_read && bsp::get_tick_ms() - last_indication > 75) {
        bsp::leds::stripe_set(Color::Black);
        indicate_read = false;
    }

    navigation->update();
    bool done = navigation->step();

    logger->update();

    if (done) {

        move_count++;
        if (move_count >= target_movements.size()) {
            return &State::get<Idle>();
        }

        last_indication = bsp::get_tick_ms();
        indicate_read = true;
        bsp::leds::stripe_set(Color::Green);

        auto movement = target_movements[move_count].first;

        auto prev_movement = ((move_count) >= 1) ? target_movements[move_count - 1].first : Movement::STOP;

        auto next_movement =
            ((move_count + 1) < target_movements.size()) ? target_movements[move_count + 1].first : Movement::STOP;

        auto cells = target_movements[move_count].second;

        navigation->set_movement(movement, prev_movement, next_movement, cells);
    }

    if ((bsp::imu::is_imu_emergency() || services::Control::instance()->is_emergency()) && move_count > 1) {
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

void Run::exit() {
    soft_timer::stop();
    bsp::motors::set(0, 0);
    if (!emergency) {
        services::Control::instance()->stop_fan();
    }
    bsp::fan::set(0);
    bsp::leds::stripe_set(Color::Black);
    bsp::analog_sensors::enable_modulation(false);
    bsp::leds::ir_emitter_all_off();
    bsp::leds::indication_off();
    logger->save_size();
    bsp::buzzer::stop();
}

}
