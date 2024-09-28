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

void ir_emitter_on(Emitter led) {
    switch (led) {
    case LEFT_SIDE:
        HAL_GPIO_WritePin(GPIO_LED_IR_L_S_PORT, GPIO_LED_IR_L_S_PIN, GPIO_PIN_SET);
        break;
    case LEFT_FRONT:
        HAL_GPIO_WritePin(GPIO_LED_IR_L_F_PORT, GPIO_LED_IR_L_F_PIN, GPIO_PIN_SET);
        break;
    case RIGHT_FRONT:
        HAL_GPIO_WritePin(GPIO_LED_IR_R_F_PORT, GPIO_LED_IR_R_F_PIN, GPIO_PIN_SET);
        break;
    case RIGHT_SIDE:
        HAL_GPIO_WritePin(GPIO_LED_IR_R_S_PORT, GPIO_LED_IR_R_S_PIN, GPIO_PIN_SET);
        break;
    }
}

void ir_emitter_off(Emitter led) {
    switch (led) {
    case LEFT_SIDE:
        HAL_GPIO_WritePin(GPIO_LED_IR_L_S_PORT, GPIO_LED_IR_L_S_PIN, GPIO_PIN_RESET);
        break;
    case LEFT_FRONT:
        HAL_GPIO_WritePin(GPIO_LED_IR_L_F_PORT, GPIO_LED_IR_L_F_PIN, GPIO_PIN_RESET);
        break;
    case RIGHT_FRONT:
        HAL_GPIO_WritePin(GPIO_LED_IR_R_F_PORT, GPIO_LED_IR_R_F_PIN, GPIO_PIN_RESET);
        break;
    case RIGHT_SIDE:
        HAL_GPIO_WritePin(GPIO_LED_IR_R_S_PORT, GPIO_LED_IR_R_S_PIN, GPIO_PIN_RESET);
        break;
    }
}

void ir_emitter_all_on(void) {
    ir_emitter_on(LEFT_FRONT);
    ir_emitter_on(LEFT_SIDE);
    ir_emitter_on(RIGHT_FRONT);
    ir_emitter_on(RIGHT_SIDE);
}

void ir_emitter_all_off(void) {
    ir_emitter_off(LEFT_FRONT);
    ir_emitter_off(LEFT_SIDE);
    ir_emitter_off(RIGHT_FRONT);
    ir_emitter_off(RIGHT_SIDE);
}

void stripe_set(uint8_t num, uint8_t r, uint8_t g, uint8_t b) {
    (void)num;
    (void)r;
    (void)g;
    (void)b;
}

} // namespace
