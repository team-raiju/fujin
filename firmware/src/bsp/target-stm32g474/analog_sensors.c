#include "st/hal.h"

#include "bsp/analog_sensors.h"

/*
 * Battery Sensor - ADC1 CH15
 * Phototransistors - ADC1 CH 1, 2, 11, 14
 * Current Sensors - ADC2 CH 12, 17
 */

/// @section Constants

/* With these settings and ADC clock =  CLK/4 and sample cycles = 47.5*/
/* We have 1 sample per 450us*/
#define ADC_1_DMA_CHANNELS 5
#define READINGS_PER_ADC_1 128
#define ADC_1_DMA_BUFFER_SIZE ADC_1_DMA_CHANNELS* READINGS_PER_ADC_1
#define ADC_1_DMA_HALF_BUFFER_SIZE (ADC_1_DMA_BUFFER_SIZE / 2)

/* With these settings and ADC clock =  CLK/4 and sample cycles = 47.5*/
/* We have 1 sample per 45us*/
#define ADC_2_DMA_CHANNELS 2
#define READINGS_PER_ADC_2 32
#define ADC_2_DMA_BUFFER_SIZE ADC_2_DMA_CHANNELS* READINGS_PER_ADC_2
#define ADC_2_DMA_HALF_BUFFER_SIZE (ADC_2_DMA_BUFFER_SIZE / 2)

/// @section Private variables

static uint32_t adc_1_dma_buffer[ADC_1_DMA_BUFFER_SIZE];
static uint32_t adc_2_dma_buffer[ADC_2_DMA_BUFFER_SIZE];
static bsp_analog_ready_callback_t reading_ready_callback;

static uint32_t ir_readings[4];
static uint32_t battery_reading;
static uint32_t current_reading = -1;

/// @section Interface implementation

void bsp_analog_sensors_init(void) {
    MX_ADC1_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // Current sensor is not being used for now
    // MAX_ADC2_Init();
    // HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
}

void bsp_analog_sensors_start(void) {
    HAL_ADC_Start_DMA(&hadc1, adc_1_dma_buffer, ADC_1_DMA_BUFFER_SIZE);
    HAL_ADC_Start_DMA(&hadc2, adc_2_dma_buffer, ADC_2_DMA_BUFFER_SIZE);
}

void bsp_analog_sensors_stop(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
}

void bsp_analog_sensors_register_callback(bsp_analog_ready_callback_t callback) {
    reading_ready_callback = callback;
}

uint32_t* bsp_analog_sensors_ir_latest_reading(void) {
    return ir_readings;
}

uint32_t bsp_analog_sensors_battery_latest_reading(void) {
    return battery_reading;
}

uint32_t bsp_analog_sensors_current_latest_reading(void) {
    return current_reading;
}
