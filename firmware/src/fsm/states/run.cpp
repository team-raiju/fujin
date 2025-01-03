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
#include "services/maze.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"
#include "utils/soft_timer.hpp"
#include "services/logger.hpp"

using bsp::leds::Color;

namespace fsm {

void PreRun::enter() {

    bsp::debug::print("state:PreRun");

    bsp::leds::stripe_set(Color::Green);

    bsp::motors::set(0, 0);

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

    soft_timer::start(1, soft_timer::CONTINUOUS);

    navigation->init();
    navigation->optimize();
    
    logger->init();

    // maze->read_maze_from_memory();
    target_movements[0] = {Movement::FORWARD, 1};
    target_movements[1] = {Movement::TURN_RIGHT_90, 1};
    // target_movements[2] = {Movement::FORWARD, 1};
    // target_movements[3] = {Movement::TURN_LEFT_90, 1};
    // target_movements[4] = {Movement::FORWARD, 1};
    // target_movements[5] = {Movement::TURN_LEFT_90, 1};
    // target_movements[6] = {Movement::FORWARD, 1};
    // target_movements[7] = {Movement::FORWARD, 1};
    // target_movements[8] = {Movement::FORWARD, 1};
    // target_movements[9] = {Movement::FORWARD, 1};
    // target_movements[10] = {Movement::FORWARD, 1};
    // target_movements[11] = {Movement::TURN_LEFT_180, 1};
    // target_movements[12] = {Movement::FORWARD, 1};
    // target_movements[13] = {Movement::TURN_RIGHT_180, 1};
    // target_movements[14] = {Movement::FORWARD, 1};
    // target_movements[16] = {Movement::STOP, 1};
    // target_movements[17] = {Movement::STOP, 0};

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

State* Run::react(Timeout const&) {
    using bsp::analog_sensors::ir_reading_wall;
    using bsp::analog_sensors::SensingDirection;

    navigation->update();
    bool done = navigation->step();

    logger->update();

    if (done) {
        auto movement = target_movements[move_count].first;
        auto cells = target_movements[move_count].second;
        move_count++;
        if (move_count > 1 || cells == 0){
            return &State::get<Idle>();
        }

        navigation->set_movement(movement);
        
    }

    return nullptr;
}

void Run::exit() {
    bsp::motors::set(0, 0);
    bsp::analog_sensors::enable_modulation(false);
    bsp::leds::ir_emitter_all_off();
    bsp::leds::indication_off();
    soft_timer::stop();
    logger->save_size();
}

}
