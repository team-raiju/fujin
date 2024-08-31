#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/// @section Interface definition

void bsp_motors_init(void);
void bsp_motors_set(int16_t speed_left, int16_t speed_right);

#ifdef __cplusplus
}
#endif
