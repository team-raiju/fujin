#include <iostream>

#include "bsp/buttons.hpp"

namespace bsp::buttons {

/// @section Private variables

static bsp_button_callback_t button_1_callback = NULL;
static bsp_button_callback_t button_2_callback = NULL;

/// @section Interface implementation

void init() {
    std::cout << "bsp::buttons::init called" << std::endl;
}

void register_callback_button1(bsp_button_callback_t callback) {
    button_1_callback = callback;
}

void register_callback_button2(bsp_button_callback_t callback) {
    button_2_callback = callback;
}

} // namespace
