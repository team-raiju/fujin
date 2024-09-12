#pragma once

namespace bsp::buttons {

/// @section Custom types

typedef void (*bsp_button_callback_t)(void);

/// @section Interface definition

void init(void);

void register_callback_button1(bsp_button_callback_t callback);
void register_callback_button2(bsp_button_callback_t callback);

} // namespace
