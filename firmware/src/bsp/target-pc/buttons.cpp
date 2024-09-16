#include <iostream>

#include "bsp/buttons.hpp"
#include "bsp/debug.hpp"

namespace bsp::buttons {

/// @section Private variables

static ButtonCallback button_1_callback = NULL;
static ButtonCallback button_2_callback = NULL;

/// @section Interface implementation

void init() {}

void register_callback_button1(ButtonCallback callback) {
    button_1_callback = callback;
}

void register_callback_button2(ButtonCallback callback) {
    button_2_callback = callback;
}

void button_1_pressed() {
    if (button_1_callback) {
        button_1_callback(PressType::SHORT);
    }
}

} // namespace
