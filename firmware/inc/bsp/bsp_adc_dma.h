#ifndef BSP_ADC_DMA_H
#define BSP_ADC_DMA_H

#include <stdint.h>

/* With these settings and ADC clock =  CLK/4 and sample cyles = 47.5*/
/* We have 1 sample per 450us*/
#define ADC_1_DMA_CHANNELS         5
#define READINGS_PER_ADC_1         128
#define ADC_1_DMA_BUFFER_SIZE      ADC_1_DMA_CHANNELS *READINGS_PER_ADC_1
#define ADC_1_DMA_HALF_BUFFER_SIZE (ADC_1_DMA_BUFFER_SIZE / 2)

/* With these settings and ADC clock =  CLK/4 and sample cyles = 47.5*/
/* We have 1 sample per 45us*/
#define ADC_2_DMA_CHANNELS         2
#define READINGS_PER_ADC_2         32
#define ADC_2_DMA_BUFFER_SIZE      ADC_2_DMA_CHANNELS *READINGS_PER_ADC_2
#define ADC_2_DMA_HALF_BUFFER_SIZE (ADC_2_DMA_BUFFER_SIZE / 2)

typedef void (*bsp_adc_dma_callback_t)(uint32_t *adc_data);

typedef enum {
    BSP_ADC_1,
    BSP_ADC_2,
} bsp_adc_num_t;

void BSP_ADC_DMA_Init(bsp_adc_num_t adc_num);
void BSP_ADC_DMA_Start(bsp_adc_num_t adc_num);
void BSP_ADC_DMA_Stop(bsp_adc_num_t adc_num);
void BSP_ADC_DMA_Register_Callback(bsp_adc_num_t adc_num, bsp_adc_dma_callback_t callback_function);

#endif /* BSP_ADC_DMA_H */
