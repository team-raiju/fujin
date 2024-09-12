#include "fsm/state.hpp"

namespace fsm {

void Idle::enter() {}

State* Idle::react(BleCommand const&) {
    return nullptr;
}

State* Idle::react(ButtonPressed const&) {
    return nullptr;
}

State* Idle::react(Timeout const&) {
    return nullptr;
}

}
