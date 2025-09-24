#include <variant>
#include <map>

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


        if (packet[1] == bsp::ble::BlePacketType::UpdateMovementParameters && !bsp::ble::is_config_locked()) {
            services::Config::parse_movement_packet(packet);
            dispatch(BleCommand());
        }

        if (packet[1] == bsp::ble::BlePacketType::Command) {
            static std::map<uint8_t, ButtonPressed::Type> b{
                {bsp::ble::BleCommands::Stop, ButtonPressed::LONG2},
                {bsp::ble::BleCommands::Button1Short, ButtonPressed::SHORT1},
                {bsp::ble::BleCommands::Button2Short, ButtonPressed::SHORT2},
                {bsp::ble::BleCommands::Button1Long, ButtonPressed::LONG1},
                {bsp::ble::BleCommands::Button2Long, ButtonPressed::LONG2},
                {bsp::ble::BleCommands::ButtonMovementParameters, ButtonPressed::LONG3},
                {bsp::ble::BleCommands::ButtonLogDump, ButtonPressed::LONG4},
            };

            dispatch(ButtonPressed{.button = b[packet[2]]});   
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
