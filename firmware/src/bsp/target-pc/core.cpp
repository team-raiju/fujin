#include <iostream>

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buttons.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/core.hpp"
#include "bsp/eeprom.hpp"
#include "bsp/encoders.hpp"
#include "bsp/fan.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "bsp/usb.hpp"

namespace bsp {

/// @section Interface implementation

void init() {
    std::cout << "bsp::init() called" << std::endl;

    analog_sensors::init();
    ble::init();
    buttons::init();
    buzzer::init();
    encoders::init();
    fan::init();
    imu::init();
    leds::init();
    motors::init();
    timers::init();
    usb::init();

    if (eeprom::init() != eeprom::EepromResult::OK) {}

    delay_ms(20);
}

}
