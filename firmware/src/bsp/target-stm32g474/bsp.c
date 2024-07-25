#include <stdlib.h> /* for exit() */

#include "bsp.h" /* Board Support Package interface */
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include "adc.h"
// #include "bsp_i2c.h"
// #include "driving_service.h"
// #include "bsp_brushless.h"
// #include "buzzer_service.h"
// #include "bsp_vcp.h"

__weak void SystemClock_Config(void);

void BSP_init(void)
{
    HAL_Init();

    SystemClock_Config();
    SystemCoreClockUpdate();

    MX_GPIO_Init();
    // MX_DMA_Init();

    // bsp_i2c_init(IO_I2C_2);
    // bsp_i2c_init(IO_I2C_3);

    // MX_TIM5_Init();
    // HAL_TIM_Base_Start(&htim5);

    BSP_delay(20);
}

uint32_t BSP_GetTick()
{
    return HAL_GetTick();
}

uint32_t BSP_Get1usTick()
{
    return __HAL_TIM_GET_COUNTER(&htim5);
}

uint32_t BSP_Get10usTick()
{
    return BSP_Get1usTick() / 10;
}

void BSP_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

void BSP_Assert()
{
    // drive_m_s(0, 0);
    // bsp_brushless_set(0);
    // buzzer_start();

    volatile uint32_t timeout = 10000000;
    while (timeout > 0) {
        timeout--;
    }

    NVIC_SystemReset();
}