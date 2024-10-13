#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/debug.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "fsm/state.hpp"
#include "utils/soft_timer.hpp"

namespace fsm {

PreCalib::PreCalib() {}

void PreCalib::enter() {
    using bsp::leds::Color;

    bsp::debug::print("state:PreCalib");

    bsp::leds::stripe_set(Color::Purple);

    bsp::motors::set(0, 0);
}

State* PreCalib::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<Idle>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreSearch>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<Calib>();
    }

    return nullptr;
}

void PreCalib::exit() {}

Calib::Calib() {
    notification = services::Notification::instance();
}

void Calib::enter() {
    using bsp::leds::Color;

    bsp::debug::print("state:Calib");

    bsp::leds::stripe_set(Color::Black);

    bsp::leds::ir_emitter_all_on();
    bsp::analog_sensors::enable_modulation();

    bsp::motors::set(0, 0);

    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    soft_timer::start(1, soft_timer::CONTINUOUS);
}

State* Calib::react(ButtonPressed const&) {
    return &State::get<PreCalib>();
}

State* Calib::react(Timeout const&) {
    notification->update(true);
    return nullptr;
}

void Calib::exit() {}

}
