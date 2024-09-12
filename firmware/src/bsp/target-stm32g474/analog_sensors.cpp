#include "st/hal.h"

#include "bsp/analog_sensors.hpp"

namespace bsp::analog_sensors {

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
static uint32_t current_reading[2];

/// @section Interface implementation

void init(void) {
    MX_ADC1_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // Current sensor is not being used for now
    // MAX_ADC2_Init();
    // HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
}

void start(void) {
    HAL_ADC_Start_DMA(&hadc1, adc_1_dma_buffer, ADC_1_DMA_BUFFER_SIZE);
    HAL_ADC_Start_DMA(&hadc2, adc_2_dma_buffer, ADC_2_DMA_BUFFER_SIZE);
}

void stop(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
}

void register_callback(bsp_analog_ready_callback_t callback) {
    reading_ready_callback = callback;
}

uint32_t* ir_latest_reading(void) {
    return ir_readings;
}

uint32_t battery_latest_reading(void) {
    return battery_reading;
}

uint32_t* current_latest_reading(void) {
    return current_reading;
}

/// @section Private functions

void adc1_callback(uint32_t* data) {
    uint32_t aux_readings[ADC_1_DMA_CHANNELS] = {0};

    for (uint16_t i = 0; i < (ADC_1_DMA_HALF_BUFFER_SIZE - 1); i += ADC_1_DMA_CHANNELS) {
        for (uint16_t j = 0; j < ADC_1_DMA_CHANNELS; j++) {
            aux_readings[j] += data[i + j];
        }
    }

    for (uint16_t j = 0; j < ADC_1_DMA_CHANNELS; j++) {
        aux_readings[j] /= (ADC_1_DMA_HALF_BUFFER_SIZE / ADC_1_DMA_CHANNELS);
    }

    ir_readings[0] = aux_readings[0];
    ir_readings[1] = aux_readings[1];
    ir_readings[2] = aux_readings[2];
    ir_readings[3] = aux_readings[3];
    battery_reading = aux_readings[4];
}

void adc2_callback(uint32_t* data) {
    uint32_t aux_readings[ADC_2_DMA_CHANNELS] = {0};

    for (uint16_t i = 0; i < (ADC_2_DMA_HALF_BUFFER_SIZE - 1); i += ADC_2_DMA_CHANNELS) {
        for (uint16_t j = 0; j < ADC_2_DMA_CHANNELS; j++) {
            aux_readings[j] += data[i + j];
        }
    }

    for (uint16_t j = 0; j < ADC_2_DMA_CHANNELS; j++) {
        aux_readings[j] /= (ADC_2_DMA_HALF_BUFFER_SIZE / ADC_2_DMA_CHANNELS);
    }

    current_reading[0] = aux_readings[0];
    current_reading[1] = aux_readings[1];
}

}

/// @section HAL callbacks

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        bsp::analog_sensors::adc1_callback(&bsp::analog_sensors::adc_1_dma_buffer[0]);
    } else if (hadc->Instance == ADC2) {
        bsp::analog_sensors::adc2_callback(&bsp::analog_sensors::adc_2_dma_buffer[0]);
    }

    if (bsp::analog_sensors::reading_ready_callback != NULL) {
        bsp::analog_sensors::reading_ready_callback();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        bsp::analog_sensors::adc1_callback(&bsp::analog_sensors::adc_1_dma_buffer[ADC_1_DMA_HALF_BUFFER_SIZE]);
    } else if (hadc->Instance == ADC2) {
        bsp::analog_sensors::adc2_callback(&bsp::analog_sensors::adc_2_dma_buffer[ADC_2_DMA_HALF_BUFFER_SIZE]);
    }

    if (bsp::analog_sensors::reading_ready_callback != NULL) {
        bsp::analog_sensors::reading_ready_callback();
    }
}
