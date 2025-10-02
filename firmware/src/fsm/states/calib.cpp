#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/debug.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "bsp/fan.hpp"
#include "fsm/state.hpp"
#include "services/config.hpp"
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
        return &State::get<CalibrationModeSelect>();
    }

    return nullptr;
}

void PreCalib::exit() {}

CalibrationModeSelect::CalibrationModeSelect() {
    calibration_mode = IR_CALIBRATION;
}

void CalibrationModeSelect::enter() {
    bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
    calibration_mode = IR_CALIBRATION;
}

State* CalibrationModeSelect::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT2) {
        if (calibration_mode == IR_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Blue);
            calibration_mode = IMU_CALIBRATION;
        } else if (calibration_mode == IMU_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Black);
            calibration_mode = FAN_CALIBRATION;
        } else {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
            calibration_mode = IR_CALIBRATION;
        }
        return nullptr;
    }

    if (event.button == ButtonPressed::SHORT1) {
        if (calibration_mode == IR_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Black);
            calibration_mode = FAN_CALIBRATION;
        } else if (calibration_mode == FAN_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Blue);
            calibration_mode = IMU_CALIBRATION;
        } else {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
            calibration_mode = IR_CALIBRATION;
        }
    }

    if (event.button == ButtonPressed::LONG1) {
        if (calibration_mode == IR_CALIBRATION) {
            return &State::get<CalibrationIRSensors>();
        } else if (calibration_mode == IMU_CALIBRATION) {
            return &State::get<CalibrationIMU>();
        } else {
            return &State::get<CalibrationFan>();
        }
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreCalib>();
    }

    return nullptr;
}

CalibrationIRSensors::CalibrationIRSensors() {
    notification = services::Notification::instance();
}

void CalibrationIRSensors::enter() {
    bsp::debug::print("state:CalibrationIRSensors");

    bsp::leds::stripe_set(bsp::leds::Color::Black);

    bsp::leds::ir_emitter_all_on();
    bsp::analog_sensors::enable_modulation();

    bsp::motors::set(0, 0);

    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    soft_timer::start(1, soft_timer::CONTINUOUS);
}

State* CalibrationIRSensors::react(ButtonPressed const&) {
    return &State::get<PreCalib>();
}

State* CalibrationIRSensors::react(Timeout const&) {
    notification->update(true);
    return nullptr;
}

void CalibrationIRSensors::exit() {}

CalibrationIMU::CalibrationIMU() {}

void CalibrationIMU::enter() {
    bsp::leds::stripe_set(bsp::leds::Color::Red, bsp::leds::Color::Red);
    bsp::imu::enable_motion_gc_filter(true);
    soft_timer::start(1, soft_timer::CONTINUOUS);
    bsp::imu::reset_angle();
    loop_counter = 0;
}

State* CalibrationIMU::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreCalib>();
    }

    return nullptr;
}

State* CalibrationIMU::react(Timeout const&) {
    if (loop_counter > 10000) {
        soft_timer::stop();
        return &State::get<PreCalib>();
    }

    loop_counter++;
    if (loop_counter % 200 == 0) {
        std::printf("bias: %f; Angle: %f; Loop: %d\r\n", bsp::imu::get_g_bias_z(), bsp::imu::get_angle(), loop_counter);
    }

    bsp::imu::update();

    return nullptr;
}

void CalibrationIMU::exit() {
    soft_timer::stop();
    bsp::motors::set(0, 0);

    services::Config::z_imu_bias = bsp::imu::get_g_bias_z();
    std::printf("saving z_imu_bias: %f\r\n", services::Config::z_imu_bias);
    services::Config::save_z_bias();
    bsp::imu::enable_motion_gc_filter(false);
}

CalibrationFan::CalibrationFan() {}

void CalibrationFan::enter() {
    bsp::leds::stripe_set(bsp::leds::Color::Red, bsp::leds::Color::Red);
    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    services::Control::instance()->init();
    services::Control::instance()->start_fan();
    bsp::motors::set(0, 0);

    soft_timer::start(1, soft_timer::CONTINUOUS);
    loop_counter = 0;
}

State* CalibrationFan::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreCalib>();
    }

    return nullptr;
}

State* CalibrationFan::react(Timeout const&) {
    if (loop_counter++ > 3000) {
        soft_timer::stop();
        return &State::get<PreCalib>();
    }

    return nullptr;
}

void CalibrationFan::exit() {
    soft_timer::stop();
    bsp::motors::set(0, 0);

    services::Control::instance()->stop_fan();
    bsp::fan::set(0);
}

}
