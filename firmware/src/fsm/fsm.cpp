#include <variant>

#include "bsp/ble.hpp"
#include "bsp/buttons.hpp"
#include "fsm/fsm.hpp"
#include "services/config.hpp"
#include "services/navigation.hpp"
#include "utils/soft_timer.hpp"

namespace fsm {

FSM::FSM() {
    current_state = &State::get<Idle>();

    navigation_service = services::Navigation::instance();
    maze_service = services::Maze::instance();

    navigation_service->init();
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

    bsp::ble::register_callback([this](auto packet) {
        if (packet[0] != bsp::ble::header) {
            return;
        }

        if (packet[1] == bsp::ble::BlePacketType::UpdateParameters && !bsp::ble::is_config_locked()) {
            services::Config::parse_packet(packet);
            dispatch(BleCommand());
        }

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
