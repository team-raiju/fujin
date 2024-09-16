#include "fsm/fsm.hpp"
#include "bsp/ble.hpp"
#include "bsp/buttons.hpp"

#include <variant>

namespace fsm {

FSM::FSM() {
    current_state = &State::get<Idle>();
}

void FSM::start() {
    // TODO: Most of these will probably move to their respective services when needed
    bsp::buttons::register_callback_button1([this](auto type) {
        dispatch(ButtonPressed{
            .button = type == bsp::buttons::PressType::SHORT ? ButtonPressed::SHORT1 : ButtonPressed::LONG1,
        });
    });

    bsp::buttons::register_callback_button1([this](auto type) {
        dispatch(ButtonPressed{
            .button = type == bsp::buttons::PressType::SHORT ? ButtonPressed::SHORT2 : ButtonPressed::LONG2,
        });
    });

    bsp::ble::register_callback([this](uint8_t*, uint8_t) { dispatch(BleCommand()); });
}

void FSM::spin() {
    Event event;
    if (!event_queue.get(&event)) {
        return;
    }

    auto state = std::visit([this](auto&& evt) { return current_state->react(evt); }, event);

    if (state != nullptr) {
        current_state->exit();
        current_state = state;
        current_state->enter();
    }
}

void FSM::dispatch(Event const& event) {
    event_queue.put(event);
}

}
