#include "st/hal.h"

#include "bsp/buttons.hpp"
#include "pin_mapping.h"

namespace bsp::buttons {

/// @section Private variables

static bsp_button_callback_t button_1_callback = NULL;
static bsp_button_callback_t button_2_callback = NULL;

/// @section Interface implementation

void init() {
    // MX_GPIO_Init is shared and called in core init
}

void register_callback_button1(bsp_button_callback_t callback) {
    button_1_callback = callback;
}

void register_callback_button2(bsp_button_callback_t callback) {
    button_2_callback = callback;
}

} // namespace

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Button 1
    if (GPIO_Pin == GPIO_BUTTON_1_PIN && HAL_GPIO_ReadPin(GPIO_BUTTON_1_PORT, GPIO_BUTTON_1_PIN) == GPIO_PIN_SET) {
        if (bsp::buttons::button_1_callback != NULL) {
            bsp::buttons::button_1_callback();
        }
        return;
    }

    // Button 2
    if (GPIO_Pin == GPIO_BUTTON_2_PIN && HAL_GPIO_ReadPin(GPIO_BUTTON_2_PORT, GPIO_BUTTON_2_PIN) == GPIO_PIN_SET) {
        if (bsp::buttons::button_2_callback != NULL) {
            bsp::buttons::button_2_callback();
        }
        return;
    }
}
