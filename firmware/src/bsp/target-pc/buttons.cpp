#include "bsp/buttons.hpp"

namespace bsp::buttons {

static ButtonCallback button_1_callback = nullptr;
static ButtonCallback button_2_callback = nullptr;

void init() {}

void register_callback_button1(ButtonCallback callback) {
    button_1_callback = callback;
}

void register_callback_button2(ButtonCallback callback) {
    button_2_callback = callback;
}

void button_1_pressed(PressType type) {
    if (button_1_callback) {
        button_1_callback(type);
    }
}

void button_2_pressed(PressType type) {
    if (button_2_callback) {
        button_2_callback(type);
    }
}

} // namespace bsp::buttons
