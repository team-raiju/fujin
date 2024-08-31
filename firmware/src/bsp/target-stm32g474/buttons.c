#include "st/hal.h"

#include "bsp/buttons.h"
#include "pin_mapping.h"

/// @section Private variables

static bsp_button_callback_t button_1_callback = NULL;
static bsp_button_callback_t button_2_callback = NULL;

/// @section Interface implementation

void bsp_buttons_init() {
    // MX_GPIO_Init is shared and called in core init
}

void bsp_buttons_register_callback_button1(bsp_button_callback_t callback) {
    button_1_callback = callback;
}

void bsp_buttons_register_callback_button2(bsp_button_callback_t callback) {
    button_2_callback = callback;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Button 1
    if (GPIO_Pin == GPIO_BUTTON_1_PIN && HAL_GPIO_ReadPin(GPIO_BUTTON_1_PORT, GPIO_BUTTON_1_PIN) == GPIO_PIN_SET) {
        if (button_1_callback != NULL) {
            button_1_callback();
        }
        return;
    }

    // Button 2
    if (GPIO_Pin == GPIO_BUTTON_2_PIN && HAL_GPIO_ReadPin(GPIO_BUTTON_2_PORT, GPIO_BUTTON_2_PIN) == GPIO_PIN_SET) {
        if (button_2_callback != NULL) {
            button_2_callback();
        }
        return;
    }
}
