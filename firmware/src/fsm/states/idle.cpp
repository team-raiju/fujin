#include "bsp/debug.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "fsm/state.hpp"
#include "utils/soft_timer.hpp"

namespace fsm {

void Idle::enter() {
    using bsp::leds::Color;

    bsp::debug::print("state:Idle");

    bsp::leds::stripe_set(0, Color::Red);
    bsp::leds::stripe_set(1, Color::Red);
    bsp::leds::stripe_send();

    bsp::motors::set(0, 0);
}

State* Idle::react(BleCommand const&) {
    return nullptr;
}

State* Idle::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreRun>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreSearch>();
    }

    return nullptr;
}

State* Idle::react(Timeout const&) {
    return nullptr;
}

}
