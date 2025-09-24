#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/debug.hpp"
#include "bsp/fan.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "fsm/state.hpp"
#include "services/config.hpp"
#include "services/logger.hpp"
#include "utils/soft_timer.hpp"

namespace fsm {

void Idle::enter() {
    using bsp::leds::Color;

    bsp::debug::print("state:Idle");
    bsp::buzzer::stop();
    if (bsp::analog_sensors::battery_low()) {
        bsp::leds::stripe_set(Color::Red);
    } else {
        bsp::leds::stripe_set(Color::Black);
    }

    bsp::motors::set(0, 0);
    bsp::fan::set(0);
    bsp::ble::unlock_config_rcv();
}

State* Idle::react(BleCommand const&) {
    bsp::buzzer::start();
    bsp::delay_ms(300);
    bsp::buzzer::stop();

    return nullptr;
}

State* Idle::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreRun>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::LONG1) {
        services::Logger::instance()->print_log();
    }

    if (event.button == ButtonPressed::LONG2) {
        services::Config::send_parameters();
        auto maze = services::Maze::instance();
        maze->read_maze_from_memory();
        maze->print(maze->ORIGIN);
        services::Notification::instance()->send_maze();
    }

    if (event.button == ButtonPressed::LONG3) {
        services::Config::send_movement_parameters();
    }

    if (event.button == ButtonPressed::LONG4){
        services::Logger::instance()->send_log_ble();
    }

    return nullptr;
}

State* Idle::react(Timeout const&) {
    return nullptr;
}

void Idle::exit() {
    bsp::ble::lock_config_rcv();
}

}
