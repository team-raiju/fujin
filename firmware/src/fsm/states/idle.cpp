#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buttons.hpp"
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

static void send_maze_and_movements(bool use_backup) {
    auto maze = services::Maze::instance();
    maze->read_maze_from_memory(use_backup);

    const auto& fwd_super = get_forward_params(services::Navigation::SUPER);
    const auto& trn_super = get_turn_params(services::Navigation::SUPER);
    auto target_directions_time = maze->directions_to_goal(true, nullptr, &fwd_super, &trn_super);
    auto target_movements_time = services::Navigation::instance()->get_movements_to_goal(
        target_directions_time, services::Navigation::target_movement_mode_t::DIAGONALS);
    services::Notification::instance()->send_target_movements(target_movements_time, true);
    bsp::delay_ms(10);

    auto target_directions_classic = maze->directions_to_goal(false);
    auto target_movements_classic = services::Navigation::instance()->get_movements_to_goal(
        target_directions_classic, services::Navigation::target_movement_mode_t::DIAGONALS);
    services::Notification::instance()->send_target_movements(target_movements_classic, false);
    bsp::delay_ms(10);

    // maze->print(maze->ORIGIN);
    // bsp::delay_ms(5);

    services::Notification::instance()->send_maze();
}

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
    bsp::buttons::enable(true);
}

State* Idle::react(BleCommand const& event) {
    if (event.packet[1] == bsp::ble::BlePacketType::RequestIrCalibParams) {
        CalibrationIRSensors::send_calib_params();
        return nullptr;
    }
    if (event.packet[1] == bsp::ble::BlePacketType::RequestParameters) {
        services::Config::send_parameters();
        return nullptr;
    }
    if (event.packet[1] == bsp::ble::BlePacketType::LoadMovementPreset) {
        bsp::buzzer::start();
        bsp::delay_ms(100);
        bsp::buzzer::stop();
        services::Config::load_movement_preset(event.packet[2]);
        return nullptr;
    }
    if (event.packet[1] == bsp::ble::BlePacketType::LoadGeneralPreset) {
        bsp::buzzer::start();
        bsp::delay_ms(100);
        bsp::buzzer::stop();
        services::Config::load_general_preset(event.packet[2]);
        return nullptr;
    }

    bsp::buzzer::start();
    bsp::delay_ms(100);
    bsp::buzzer::stop();

    return nullptr;
}

State* Idle::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreSearch>();
    }

    if (event.button == ButtonPressed::LONG1) {
        services::Logger::instance()->print_log();
    }

    if (event.button == ButtonPressed::LONG2) {
        services::Config::send_parameters();
    }
    
    if (event.button == ButtonPressed::LONG7) {
        send_maze_and_movements(false);
    }

    if (event.button == ButtonPressed::LONG8) {
        send_maze_and_movements(true);
    }

    if (event.button == ButtonPressed::LONG3) {
        services::Config::send_movement_parameters();
    }

    if (event.button == ButtonPressed::LONG4) {
        services::Logger::instance()->send_log_ble();
    }

    if (event.button == ButtonPressed::LONG5) {
        services::Config::send_move_sequence();
    }

    if (event.button == ButtonPressed::LONG6) {
        return &State::get<CalibrationIRSensors>();
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
