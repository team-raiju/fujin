#pragma once

#include <cstdint>

namespace bsp::buzzer {

/// @section Interface definition

/// @brief Initializes Buzzer peripheral
/// @note Doesn't need to be called directly as bsp_init() already calls this
void init(void);

/// @brief Starts the buzzer
void start(void);

/// @brief Stops the buzzer
void stop(void);

void set_volume(uint8_t volume);

void set_frequency(uint16_t hz);

}
