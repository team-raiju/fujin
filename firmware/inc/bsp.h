#ifndef BSP_H
#define BSP_H

#include <stdint.h>

/* number of clock ticks in a second */
#define BSP_TICKS_PER_SEC (1000)

/* number of clock ticks in a millisecond */
#define BSP_TICKS_PER_MILISSEC (BSP_TICKS_PER_SEC/1000.0)

void BSP_init(void);
uint32_t BSP_GetTick();
uint32_t BSP_Get1usTick();
uint32_t BSP_Get10usTick();
void BSP_delay_us(uint32_t delay_us);
void BSP_Assert();
void BSP_delay();


#endif /* BSP_H */
