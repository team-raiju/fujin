#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void bsp_fan_init(void);

void bsp_fan_set(uint8_t speed);

#ifdef __cplusplus
}
#endif
