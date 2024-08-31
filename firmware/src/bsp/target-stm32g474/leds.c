#include <stdint.h>

#include "st/hal.h"

#include "bsp/leds.h"
#include "pin_mapping.h"

/// @section Interface implementation

void bsp_leds_init(void) {
    MX_TIM2_Init();
}

void bsp_leds_indication_on(void) {
    HAL_GPIO_WritePin(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_PIN_SET);
}

void bsp_leds_indication_off(void) {
    HAL_GPIO_WritePin(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_PIN_RESET);
}

void bsp_leds_indication_toggle(void) {
    HAL_GPIO_TogglePin(GPIO_LED_PORT, GPIO_LED_PIN);
}

void bsp_leds_stripe_set(uint8_t num, uint8_t r, uint8_t g, uint8_t b) {
    (void)num;
    (void)r;
    (void)g;
    (void)b;
}
