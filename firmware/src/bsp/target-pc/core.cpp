#include <iostream>
#include <thread>
#include <variant>

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buttons.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/core.hpp"
#include "bsp/debug.hpp"
#include "bsp/eeprom.hpp"
#include "bsp/encoders.hpp"
#include "bsp/fan.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "bsp/usb.hpp"

#include "fsm/event.hpp"
#include "fsm/fsm.hpp"
#include "utils/RingBuffer.hpp"

using bsp::eeprom::EepromResult;
using bsp::imu::ImuResult;

namespace bsp {

namespace buttons {
void button_1_pressed();
}

namespace ble {
void received();
}

/// @section Interface implementation

void init() {
    analog_sensors::init();
    ble::init();
    buttons::init();
    buzzer::init();
    encoders::init();
    fan::init();
    leds::init();
    motors::init();
    timers::init();
    usb::init();

    if (imu::init() != ImuResult::OK) {
        std::cerr << "Failed to initialize imu" << std::endl;
    }

    if (eeprom::init() != EepromResult::OK) {
        std::cerr << "Failed to initialize eeprom" << std::endl;
    }

    // Testing
    delay_ms(20);

    std::thread([] {
        for (;;) {
            delay_ms(2000);
            buttons::button_1_pressed();
        }
    }).detach();

    std::thread([] {
        for (;;) {
            delay_ms(5000);
            ble::received();
        }
    }).detach();
}

void debug::print(const char* s) {
    std::cout << s << std::endl;
}

}
