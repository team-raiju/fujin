#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// @section Interface definition

/// @brief Initializes all peripherals
/// @note Must call all other bsp_*_init functions
void bsp_init(void);

#ifdef __cplusplus
}
#endif
