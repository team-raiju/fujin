#include <stdint.h>

#include "st/hal.h"

#include "bsp/leds.hpp"
#include "pin_mapping.h"

namespace bsp::leds {

/// @section Interface implementation

void init(void) {
    MX_TIM2_Init();
}

void indication_on(void) {
    HAL_GPIO_WritePin(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_PIN_SET);
}

void indication_off(void) {
    HAL_GPIO_WritePin(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_PIN_RESET);
}

void indication_toggle(void) {
    HAL_GPIO_TogglePin(GPIO_LED_PORT, GPIO_LED_PIN);
}

void stripe_set(uint8_t num, uint8_t r, uint8_t g, uint8_t b) {
    (void)num;
    (void)r;
    (void)g;
    (void)b;
}

} // namespace
