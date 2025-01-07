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
#include "services/logger.hpp"
#include "services/maze.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"
#include "utils/soft_timer.hpp"
#include "services/config.hpp"


using bsp::leds::Color;

namespace fsm {

void PreRun::enter() {

    bsp::debug::print("state:PreRun");

    bsp::leds::stripe_set(Color::Green);

    bsp::motors::set(0, 0);
    bsp::fan::set(0);

    /* Only start IR if powered by the battery */
    if (bsp::analog_sensors::battery_latest_reading_mv() > 7000) {
        soft_timer::start(100, soft_timer::SINGLE);
    }
}

State* PreRun::react(Timeout const&) {
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
            return &State::get<PreRun>();
        }
        bsp::delay_ms(1);
    }

    return &State::get<Run>();
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
        return &State::get<Run>();
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

    uint8_t fan_speed = 0;
    for (int i = 0; i < fan_speed; i++) {
        bsp::fan::set(i);
        bsp::delay_ms(10);
    }
    bsp::fan::set(fan_speed);
    bsp::delay_ms(100);
                                                                                               
    soft_timer::start(1, soft_timer::CONTINUOUS);

    navigation->reset(false);

    logger->init();

    maze->read_maze_from_memory();
    maze->print(maze->ORIGIN);
    target_directions = maze->directions_to_goal();
    maze->print(maze->ORIGIN);

    std::vector<std::pair<Movement, uint8_t>> default_target_movements =
        navigation->get_default_target_movements(target_directions);

    target_movements.clear();
    bool use_diagonal = true;

    if (use_diagonal) {
        target_movements = navigation->get_diagonal_movements(default_target_movements);
    } else {
        target_movements = navigation->get_smooth_movements(default_target_movements);
    }


    // target_movements.push_back({Movement::START, 1});
    // target_movements.push_back({Movement::FORWARD, 1});
    // target_movements.push_back({Movement::TURN_RIGHT_180, 1});
    // target_movements.push_back({Movement::FORWARD, 1});
    // target_movements.push_back({Movement::TURN_LEFT_90, 1});
    // target_movements.push_back({Movement::FORWARD, 1});
    // target_movements.push_back({Movement::TURN_LEFT_90, 1});
    // target_movements.push_back({Movement::FORWARD, 5});
    // target_movements.push_back({Movement::TURN_LEFT_180, 1});
    // target_movements.push_back({Movement::FORWARD, 1});
    // target_movements.push_back({Movement::TURN_RIGHT_135, 1});
    // target_movements.push_back({Movement::DIAGONAL, 1});
    // target_movements.push_back({Movement::TURN_RIGHT_45_FROM_45, 1});
    // target_movements.push_back({Movement::STOP, 1});

    move_count = 0;
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

static bool indicate_read = false;
static uint32_t last_indication = 0;

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

        last_indication = bsp::get_tick_ms();
        indicate_read = true;
        bsp::leds::stripe_set(Color::Green);

        move_count++;
        if (move_count >= target_movements.size()) {
            return &State::get<Idle>();
        }

        auto movement = target_movements[move_count].first;
        auto cells = target_movements[move_count].second;

        navigation->set_movement(movement, cells);
    }

    bool imu_emergency = (bsp::imu::get_z_acceleration() > 4.5 || bsp::imu::get_z_acceleration() < -2.5);
    if (imu_emergency) {
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
    bsp::motors::set(0, 0);
    bsp::fan::set(0);
    bsp::analog_sensors::enable_modulation(false);
    bsp::leds::ir_emitter_all_off();
    bsp::leds::indication_off();
    soft_timer::stop();
    logger->save_size();
}

}
