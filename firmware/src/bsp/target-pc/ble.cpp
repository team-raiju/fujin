#include <iostream>

#include "bsp/ble.hpp"

namespace bsp::ble {

static bsp_uart_ble_callback_t external_callback;

void init() {
    std::cout << "bsp::ble::init called" << std::endl;
}

void start(void) {
    std::cout << "bsp::ble::start called" << std::endl;
}

void stop(void) {
    std::cout << "bsp::ble::stop called" << std::endl;
}

void transmit(uint8_t* data, uint8_t size) {
    std::cout << "bsp::ble::transmit called" << std::endl;
}

void register_callback(bsp_uart_ble_callback_t callback) {
    external_callback = callback;
}

} // namespace
