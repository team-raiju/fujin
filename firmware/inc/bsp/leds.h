#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/// @section Constants

#define LED_STRIPE_AMOUNT 2 ///< Number of LEDs in the stripe

/// @section Interface definition

/// @brief Initializes LEDs peripherals
/// @note Doesn't need to be called directly as bsp_init() already calls this
void bsp_leds_init(void);

void bsp_leds_indication_on(void);
void bsp_leds_indication_off(void);
void bsp_leds_indication_toggle(void);

/// @brief Sets the color of a LED in the stripe
/// @param num LED number [0, LED_STRIPE_AMOUNT)
/// @param r Red value. [0, 255]
/// @param g Green value. [0, 255]
/// @param b Blue value. [0, 255]
void bsp_leds_stripe_set(uint8_t num, uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif
