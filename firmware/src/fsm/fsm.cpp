#include "fsm/fsm.hpp"

namespace fsm {

FSM::FSM() {}

Event x;

void FSM::start() {}

void FSM::spin() {
    // Consume event queue
    auto state = current_state->react(x);

    if (state != nullptr) {
        current_state->exit();
        current_state = state;
        current_state->enter();
    }
}

}
