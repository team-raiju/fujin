#include "st/hal.h"

#include "bsp/encoders.h"

void bsp_encoders_init() {
    MX_SPI1_Init();
}

void bsp_encoders_register_callback_encoder1(bsp_gpio_encoder_callback_t callback) {
    (void)callback;
}

void bsp_encoders_register_callback_encoder2(bsp_gpio_encoder_callback_t callback) {
    (void)callback;
}
