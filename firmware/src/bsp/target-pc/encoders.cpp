#include <iostream>

#include "bsp/encoders.hpp"

namespace bsp::encoders {

/// @section Interface implementation

void init() {
    std::cout << "bsp::encoders::init called" << std::endl;
}

void register_callback_encoder1(bsp_gpio_encoder_callback_t callback) {
    (void)callback;
}

void register_callback_encoder2(bsp_gpio_encoder_callback_t callback) {
    (void)callback;
}

}
