#pragma once

#include <functional>

namespace bsp::buttons {

/// @section Custom types

enum PressType { SHORT, LONG };

typedef std::function<void(PressType)> ButtonCallback;

/// @section Interface definition

void init(void);

void register_callback_button1(ButtonCallback callback);
void register_callback_button2(ButtonCallback callback);

} // namespace
