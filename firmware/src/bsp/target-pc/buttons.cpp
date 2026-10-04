#include "bsp/buttons.hpp"

namespace bsp::buttons {

static ButtonCallback button_1_callback = nullptr;
static ButtonCallback button_2_callback = nullptr;
static bool is_button_enabled = true;

void init() {
    is_button_enabled = true;
}

void register_callback_button1(ButtonCallback callback) {
    button_1_callback = callback;
}

void register_callback_button2(ButtonCallback callback) {
    button_2_callback = callback;
}

void enable(bool enable) {
    is_button_enabled = enable;
}

void disable() {
    enable(false);
}

void set_enabled(bool enabled) {
    enable(enabled);
}

bool is_enabled() {
    return is_button_enabled;
}

void button_1_pressed(PressType type) {
    if (!is_button_enabled) {
        return;
    }
    if (button_1_callback) {
        button_1_callback(type);
    }
}

void button_2_pressed(PressType type) {
    if (!is_button_enabled) {
        return;
    }
    if (button_2_callback) {
        button_2_callback(type);
    }
}

} // namespace bsp::buttons
