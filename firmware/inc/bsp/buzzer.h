#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// @section Interface definition

/// @brief Initializes Buzzer peripheral
/// @note Doesn't need to be called directly as bsp_init() already calls this
void bsp_buzzer_init(void);

/// @brief Starts the buzzer
void bsp_buzzer_start(void);

/// @brief Stops the buzzer
void bsp_buzzer_stop(void);

#ifdef __cplusplus
}
#endif
