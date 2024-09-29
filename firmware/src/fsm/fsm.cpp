#include "fsm/fsm.hpp"
#include "bsp/ble.hpp"
#include "bsp/buttons.hpp"
#include "services/position.hpp"
#include "utils/soft_timer.hpp"

#include <map>
#include <variant>

namespace fsm {

FSM::FSM() {
    current_state = &State::get<Idle>();

    position_service = services::Position::instance();
    maze_service = services::Maze::instance();

    position_service->init();
}

void FSM::start() {
    // TODO: Most of these will probably move to their respective services when needed
    bsp::buttons::register_callback_button1([this](auto type) {
        dispatch(ButtonPressed{
            .button = type == bsp::buttons::PressType::SHORT ? ButtonPressed::SHORT1 : ButtonPressed::LONG1,
        });
    });

    bsp::buttons::register_callback_button2([this](auto type) {
        dispatch(ButtonPressed{
            .button = type == bsp::buttons::PressType::SHORT ? ButtonPressed::SHORT2 : ButtonPressed::LONG2,
        });
    });

    bsp::ble::register_callback([this](bsp::ble::BleHeader packet) {
        static std::map<bsp::ble::BleHeader, ButtonPressed::Type> b{
            {bsp::ble::BLE_BUTTON_1_SHORT, ButtonPressed::SHORT1},
            {bsp::ble::BLE_BUTTON_2_SHORT, ButtonPressed::SHORT2},
            {bsp::ble::BLE_BUTTON_1_LONG, ButtonPressed::LONG1},
            {bsp::ble::BLE_BUTTON_2_LONG, ButtonPressed::LONG2},
        };

        dispatch(ButtonPressed{.button = b[packet]});
    });

    soft_timer::register_callback([this]() { dispatch(Timeout()); });

    current_state->enter();
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
