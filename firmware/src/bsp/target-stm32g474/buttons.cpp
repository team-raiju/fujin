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

/// @section Interface implementation

void init() {
    // MX_GPIO_Init is shared and called in core init
}

void register_callback_button1(ButtonCallback callback) {
    button_1_callback = callback;
}

void register_callback_button2(ButtonCallback callback) {
    button_2_callback = callback;
}

static uint32_t b1_timer = 0;
static uint32_t b2_timer = 0;

void gpio_exti_callback(uint16_t GPIO_Pin) {
    // Button 1
    if (GPIO_Pin == GPIO_BUTTON_1_PIN) {
        if (!button_1_callback) {
            return;
        }

        if (HAL_GPIO_ReadPin(GPIO_BUTTON_1_PORT, GPIO_BUTTON_1_PIN) == GPIO_PIN_RESET) {
            b1_timer = bsp::get_tick_ms();
        } else {
            uint32_t passed = bsp::get_tick_ms() - b1_timer;
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
            uint32_t passed = bsp::get_tick_ms() - b2_timer;
            if (passed > LONG_PRESS_MS) {
                button_2_callback(PressType::LONG);
            } else if (passed > SHORT_PRESS_MS) {
                button_2_callback(PressType::SHORT);
            }
        }
    }
}

} // namespace
