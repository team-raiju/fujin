#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/// @section Interface definition

/// @brief Initializes Buzzer peripheral
/// @note Doesn't need to be called directly as bsp_init() already calls this
void bsp_buzzer_init(void);

/// @brief Starts the buzzer
void bsp_buzzer_start(void);

/// @brief Stops the buzzer
void bsp_buzzer_stop(void);

void bsp_buzzer_set_volume(uint8_t volume);

void bsp_buzzer_set_frequency(uint16_t hz);

#ifdef __cplusplus
}
#endif
