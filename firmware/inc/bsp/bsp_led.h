#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stdint.h>

void BSP_ledInit(void);
void BSP_ledOff(void);
void BSP_ledOn(void);
void BSP_ledToggle(void);
#endif /* BSP_LED_H */
