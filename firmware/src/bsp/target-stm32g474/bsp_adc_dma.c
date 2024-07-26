

#include <stdbool.h>

#include "bsp_adc_dma.h"
#include "adc.h"
#include "dma.h"
#include "utils.h"

uint32_t adc_1_dma_buffer[ADC_1_DMA_BUFFER_SIZE];
uint32_t adc_2_dma_buffer[ADC_2_DMA_BUFFER_SIZE];

bsp_adc_dma_callback_t adc_1_dma_callback_function;
bsp_adc_dma_callback_t adc_2_dma_callback_function;

static void default_adc_dma_callback(uint32_t *out_data)
{
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_1_dma_callback_function(&adc_1_dma_buffer[0]);
    } else if (hadc->Instance == ADC2) {
        adc_2_dma_callback_function(&adc_2_dma_buffer[0]);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_1_dma_callback_function(&adc_1_dma_buffer[ADC_1_DMA_HALF_BUFFER_SIZE]);
    } else if (hadc->Instance == ADC2) {
        adc_2_dma_callback_function(&adc_2_dma_buffer[ADC_2_DMA_HALF_BUFFER_SIZE]);
    }
}

void BSP_ADC_DMA_Init(bsp_adc_num_t adc_num)
{
    if (adc_num == BSP_ADC_1) {
        MX_ADC1_Init();
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
        BSP_ADC_DMA_Register_Callback(BSP_ADC_1, default_adc_dma_callback);
    } else if (adc_num == BSP_ADC_2) {
        MX_ADC2_Init();
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
        BSP_ADC_DMA_Register_Callback(BSP_ADC_2, default_adc_dma_callback);
    }
}

void BSP_ADC_DMA_Start(bsp_adc_num_t adc_num)
{
    if (adc_num == BSP_ADC_1) {
        HAL_ADC_Start_DMA(&hadc1, adc_1_dma_buffer, ADC_1_DMA_BUFFER_SIZE);
    } else if (adc_num == BSP_ADC_2) {
        HAL_ADC_Start_DMA(&hadc2, adc_2_dma_buffer, ADC_2_DMA_BUFFER_SIZE);
    }
}

void BSP_ADC_DMA_Stop(bsp_adc_num_t adc_num)
{
    if (adc_num == BSP_ADC_1) {
        HAL_ADC_Stop_DMA(&hadc1);
    } else if (adc_num == BSP_ADC_2) {
        HAL_ADC_Stop_DMA(&hadc2);
    }
}

void BSP_ADC_DMA_Register_Callback(bsp_adc_num_t adc_num, bsp_adc_dma_callback_t callback_function)
{
    if (adc_num == BSP_ADC_1) {
        adc_1_dma_callback_function = callback_function;
    } else if (adc_num == BSP_ADC_2) {
        adc_2_dma_callback_function = callback_function;
    }
}
