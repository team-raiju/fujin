#pragma once

#include <cstdint>

namespace bsp::encoders {

/// @section Custom types

typedef void (*bsp_gpio_encoder_callback_t)(uint8_t type);

/// @section Interface definitions

void init(void);

void register_callback_encoder1(bsp_gpio_encoder_callback_t callback);
void register_callback_encoder2(bsp_gpio_encoder_callback_t callback);

}
