#include <cstdint>
#include <cstdio>

#include "st/hal.h"

#include "bsp/leds.hpp"
#include "devices/WS2812.hpp"
#include "pin_mapping.h"
#include "utils/math.hpp"

namespace bsp::leds {

Color Color::Red = Color(0x7F, 0x00, 0x00);
Color Color::Green = Color(0x00, 0x3F, 0x00);
Color Color::Blue = Color(0x00, 0x00, 0x7F);
Color Color::Purple = Color(0x7F, 0x00, 0x7F);
Color Color::Yellow = Color(0x3F, 0x3F, 0x00);
Color Color::Orange = Color(0x64, 0xC8, 0x00);
Color Color::White = Color(0x3F, 0x3F, 0x3F);
Color Color::Black = Color(0x00, 0x00, 0x00);
Color Color::Pink = Color(0x1F, 0xC8, 0x64);
Color Color::Cyan = Color(0xD2, 0x32, 0x12);

uint32_t Color::encode() const {
    return (bit_reverse(b) << 16) | (bit_reverse(r) << 8) | bit_reverse(g);
}

/// @section Private variables

static devices::WS2812<2> ws(&htim2, TIM_CHANNEL_1);

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

void stripe_set(Color const& color_1, Color const& color_2) {
    ws.set(0, color_1.encode());
    ws.set(1, color_2.encode());
    stripe_send();
}


void stripe_set(Color const& color) {
    uint32_t encoded = color.encode();
    for (size_t i = 0; i < ws.amount; i++) {
        ws.set(i, encoded);
    }
    stripe_send();
}

void stripe_send() {
    ws.send();
}

} // namespace

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    bsp::leds::ws.pulse_finished_callback(htim);
}
