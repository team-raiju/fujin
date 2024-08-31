#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/// @section Custom types

typedef void (*bsp_gpio_encoder_callback_t)(uint8_t type);

/// @section Interface definitions

void bsp_encoders_init(void);

void bsp_encoders_register_callback_encoder1(bsp_gpio_encoder_callback_t callback);
void bsp_encoders_register_callback_encoder2(bsp_gpio_encoder_callback_t callback);

#ifdef __cplusplus
}
#endif
