#include <cstdio>

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/core.hpp"
#include "bsp/leds.hpp"
#include "bsp/timers.hpp"
#include "fsm/fsm.hpp"
#include "fsm/state.hpp"
#include "services/config.hpp"
#include "bsp/imu.hpp"

void startup() {
    bsp::buzzer::set_frequency(2000);
    bsp::buzzer::set_volume(1);
    bsp::buzzer::start();

    for (int i = 0; i < 10; i++) {
        bsp::leds::indication_toggle();
        bsp::delay_ms(100);
    }

    bsp::leds::indication_off();
    bsp::buzzer::stop();
    bsp::analog_sensors::start();
    bsp::delay_ms(30);
    services::Config::init();
    bsp::ble::init();
    bsp::ble::start();
    bsp::imu::set_g_bias_z(services::Config::z_imu_bias);
}

int main() {

    bsp::init();

    startup();

    fsm::FSM fsm;

    fsm.start();

    for (;;) {
        fsm.spin();
    }
}
