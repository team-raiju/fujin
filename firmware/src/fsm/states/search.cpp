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
#include <cstdio>

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

    last_error = 0;
    search_state = STRAIGHT;
    counter = 0;
    stop_counter = 0;
    last_empty_wall_right = true;
    bsp::leds::ir_emmiter_all_on();
    bsp::imu::reset_angle();
    soft_timer::start(1, soft_timer::CONTINUOUS);
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

    int32_t sensor_side_left = bsp::analog_sensors::ir_latest_reading()[3];
    int32_t sensor_side_right = bsp::analog_sensors::ir_latest_reading()[0];
    int32_t sensor_front_left = bsp::analog_sensors::ir_latest_reading()[1];
    int32_t sensor_front_right = bsp::analog_sensors::ir_latest_reading()[2];
    bsp::imu::update();
    float rotation_ratio = 0;
    float target_speed = 0;

    switch (search_state) {
        case STRAIGHT:{
            float error;
            target_speed = 50;
            if (sensor_side_left < 1475 && sensor_side_right < 1200) {
                last_empty_wall_right = true;
                error = 0;
            } else if (sensor_side_left < 1475){
                last_empty_wall_right = false;
                error = 1650 - sensor_side_right;
            } else if (sensor_side_right < 1200){
                last_empty_wall_right = true;
                error = sensor_side_left - 1650;
            } else if (sensor_side_right > 2800 || sensor_side_left > 2800) {
                error = 0;
            } else {
                error = sensor_side_left - sensor_side_right;
            }

            float derivative = error - last_error;
            float kp = 0.02;
            float kd = 0.08;

            rotation_ratio = kp * error + kd * derivative;
            last_error = error;

            if (sensor_front_left > 3500 || sensor_front_right > 3500) {
                bsp::leds::indication_off();
                search_state = START_TURN;
                counter = 0;
            }

            break;
        }
        case START_TURN: {
            float error = sensor_front_right - sensor_front_left;
            rotation_ratio = 0.05 * error;
            target_speed = 0;

            if (counter++ > 500){
                if (last_empty_wall_right) {
                    search_state = TURN_RIGHT;
                } else {
                    search_state = TURN_LEFT;
                }
                bsp::leds::indication_on();
            }
            break;
        }
        case TURN_RIGHT:{
            target_speed = 0;
            float angle_diff = M_PI_2 - utils_abs(bsp::imu::get_angle());
            rotation_ratio = 60 * angle_diff;
            if (utils_abs(bsp::imu::get_angle()) > M_PI_2 - 0.4) {
                bsp::imu::reset_angle();
                search_state = STRAIGHT;
            }
            break;
        }
        case TURN_LEFT:{
            target_speed = 0;
            float angle_diff = M_PI_2 - utils_abs(bsp::imu::get_angle());
            rotation_ratio = -60 * angle_diff;
            if (utils_abs(bsp::imu::get_angle()) > M_PI_2 - 0.4) {
                bsp::imu::reset_angle();
                search_state = STRAIGHT;
            }
            break;
        }
    }


    int16_t left_speed = target_speed + rotation_ratio;
    int16_t right_speed = target_speed - rotation_ratio;

    //static int my_cnt = 0;
    //if (my_cnt++ % 100 == 0) {
        //std::printf("(%ld, %ld) - (%d, %d)\r\n", sensor_side_left, sensor_side_right, left_speed, right_speed);
        // std::printf("(%ld, %ld)\r\n", sensor_front_left, sensor_front_right);
        // std::printf("%ld - %ld: %d\r\n", sensor_side_left, sensor_side_right, last_empty_wall_right);
    //}

    bsp::motors::set(left_speed, right_speed);

    if (stop_counter++ > 5000) {
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
