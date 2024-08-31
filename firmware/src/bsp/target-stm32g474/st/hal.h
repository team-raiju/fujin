#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32g4xx_hal.h>
#include <usbd_def.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

extern CRC_HandleTypeDef hcrc;

extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim2_ch1;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern USBD_HandleTypeDef hUsbDeviceFS;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void Error_Handler(void);

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_TIM2_Init(void);
void MX_ADC1_Init(void);
void MX_I2C2_Init(void);
void MX_I2C3_Init(void);
void MX_SPI1_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void MX_ADC2_Init(void);
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_CRC_Init(void);
void MX_USB_Device_Init(void);

#ifdef __cplusplus
}
#endif
