#ifndef BSP_FAN_H
#define BSP_FAN_H

#include <stdint.h>

void bsp_fan_init();
void bsp_fan_set(uint8_t speed);

void bsp_fan_init_routine(uint8_t speed);
void bsp_fan_stop_routine();

void bsp_fan_set_with_bat(uint16_t bat_mv, uint8_t speed);

#endif /* BSP_FAN_H */
