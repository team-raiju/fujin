#include "st/hal.h"

#include "bsp/buttons.hpp"
#include "bsp/timers.hpp"
#include "pin_mapping.h"

namespace bsp::buttons {

#define SHORT_PRESS_MS 50
#define LONG_PRESS_MS 700

/// @section Private variables

static ButtonCallback button_1_callback = NULL;
static ButtonCallback button_2_callback = NULL;
static bool is_button_enabled = true;
static uint32_t b1_timer = 0;
static uint32_t b2_timer = 0;

/// @section Interface implementation

void init() {
    // MX_GPIO_Init is shared and called in core init
    is_button_enabled = true;
    b1_timer = 0;
    b2_timer = 0;
}

void register_callback_button1(ButtonCallback callback) {
    button_1_callback = callback;
}

void register_callback_button2(ButtonCallback callback) {
    button_2_callback = callback;
}

void enable(bool enable) {
    is_button_enabled = enable;
    if (!enable) {
        b1_timer = 0;
        b2_timer = 0;
    }
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

void gpio_exti_callback(uint16_t GPIO_Pin) {
    if (!is_button_enabled) {
        return;
    }

    // Button 1
    if (GPIO_Pin == GPIO_BUTTON_1_PIN) {
        if (!button_1_callback) {
            return;
        }

        if (HAL_GPIO_ReadPin(GPIO_BUTTON_1_PORT, GPIO_BUTTON_1_PIN) == GPIO_PIN_RESET) {
            b1_timer = bsp::get_tick_ms();
        } else {
            if (b1_timer == 0) {
                return;
            }
            uint32_t passed = bsp::get_tick_ms() - b1_timer;
            b1_timer = 0;
            if (passed > LONG_PRESS_MS) {
                button_1_callback(PressType::LONG);
            } else if (passed > SHORT_PRESS_MS) {
                button_1_callback(PressType::SHORT);
            }
        }
    }

    // Button 2
    if (GPIO_Pin == GPIO_BUTTON_2_PIN) {
        if (!button_2_callback) {
            return;
        }

        if (HAL_GPIO_ReadPin(GPIO_BUTTON_2_PORT, GPIO_BUTTON_2_PIN) == GPIO_PIN_RESET) {
            b2_timer = bsp::get_tick_ms();
        } else {
            if (b2_timer == 0) {
                return;
            }
            uint32_t passed = bsp::get_tick_ms() - b2_timer;
            b2_timer = 0;
            if (passed > LONG_PRESS_MS) {
                button_2_callback(PressType::LONG);
            } else if (passed > SHORT_PRESS_MS) {
                button_2_callback(PressType::SHORT);
            }
        }
    }
}

} // namespace
