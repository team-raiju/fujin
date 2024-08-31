#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// @section Custom types

typedef void (*bsp_button_callback_t)(void);

/// @section Interface definition

void bsp_buttons_init(void);

void bsp_buttons_register_callback_button1(bsp_button_callback_t callback);
void bsp_buttons_register_callback_button2(bsp_button_callback_t callback);

#ifdef __cplusplus
}
#endif
