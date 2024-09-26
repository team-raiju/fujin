#include <cstdio>
#include "bsp/debug.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "fsm/state.hpp"
#include "bsp/core.hpp"
#include "bsp/timers.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/analog_sensors.hpp"
#include "utils/math.h"
#include "bsp/imu.hpp"
#include "utils/soft_timer.hpp"
#include "services/position_service.hpp"

namespace fsm {

void PreSearch::enter() {
    bsp::debug::print("state:PreSearch");

    bsp::leds::stripe_set(0, 0, 255, 0);
    bsp::leds::stripe_set(1, 0, 0, 0);

    bsp::motors::set(0, 0);
}

State* PreSearch::react(BleCommand const&) {
    return nullptr;
}

State* PreSearch::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<Idle>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreRun>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<Search>();
    }

    return nullptr;

}

void Search::enter() {
    bsp::debug::print("state:Search");
    bsp::leds::indication_on();

    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    bsp::leds::ir_emmiter_all_on();
    bsp::imu::reset_angle();
    soft_timer::start(1, soft_timer::CONTINUOUS);
    stop_counter = 0;
    angular_vel_pid.reset();
    angular_vel_pid.update_parameters(10.0, 0.0, 0.0, 0.0);
    
}

State* Search::react(BleCommand const&) {
    return nullptr;
}

State* Search::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return nullptr;
    }

    if (event.button == ButtonPressed::SHORT2) {
        return nullptr;
    }

    return nullptr;
}

State* Search::react(Timeout const&) {

    services::position::update();
    bsp::imu::update();

    float target_speed = 50;
    float rotation_ratio = angular_vel_pid.calculate(0.0, bsp::imu::get_rad_per_s());

    float speed_l = target_speed + rotation_ratio;
    float speed_r = target_speed - rotation_ratio;

    bsp::motors::set(speed_l, speed_r);
    // static int cnt = 0;
    // if (cnt++ % 100 == 0){
    //     std::printf("(%f, %f, %f) \r\n", speed_l, speed_r, bsp::imu::get_rad_per_s());
    // }

    // if (services::position::get_travelled_dist_cm() > 9) {
    //     return &State::get<Idle>();
    // }

    if (stop_counter++ > 300) {
        return &State::get<PreSearch>();
    }

    return nullptr;
}

void Search::exit() {
    bsp::motors::set(0, 0);
    bsp::leds::ir_emmiter_all_off();
    bsp::leds::indication_off();
}


}
