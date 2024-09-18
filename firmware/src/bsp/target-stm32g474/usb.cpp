#include "st/hal.h"

#include "bsp/usb.hpp"

namespace bsp::usb {

/// @section Interface implementation

void init(void) {
    reset_on_host();
    clock_config();
    MX_USB_Device_Init();
}

void reset_on_host(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    /*Configure GPIO pin : PA12, a.k.a. USB_DP */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_Delay(5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
}

void clock_config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_CRSInitTypeDef RCC_CRSInitStruct;

    /* Enable HSI48 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /*Configure the clock recovery system (CRS)**********************************/

    /*Enable CRS Clock*/
    __HAL_RCC_CRS_CLK_ENABLE();

    /* Default Synchro Signal division factor (not divided) */
    RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;

    /* Set the SYNCSRC[1:0] bits according to CRS_Source value */
    RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;

    /* HSI48 is synchronized with USB SOF at 1KHz rate */
    RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;

    /* Set the TRIM[5:0] to the default value */
    RCC_CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

    /* Start automatic synchronization */
    HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

} // namespace
