#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/// @section Interface definition

void bsp_timers_init(void);

uint32_t bsp_get_tick_ms(void);
uint32_t bsp_get_tick_us(void);
void bsp_delay_ms(uint32_t ms);
void bsp_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif
