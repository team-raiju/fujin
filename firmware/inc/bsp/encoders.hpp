#pragma once

#include <cstdint>
#include <functional>

namespace bsp::encoders {

/// @section Custom types

// TODO: Enum the type
enum DirectionType { CW, CCW };

typedef std::function<void(DirectionType type)> EncoderCallback;

/// @section Interface definitions

void init();

void register_callback_encoder_left(EncoderCallback callback);
void register_callback_encoder_right(EncoderCallback callback);

}
