#include "st/hal.h"

#include "bsp/analog_sensors.hpp"
#include "bsp/leds.hpp"
#include "bsp/timers.hpp"
#include "utils/math.hpp"

namespace bsp::analog_sensors {

/*
 * Battery Sensor - ADC1 CH15
 * Phototransistors - ADC1 CH 1, 2, 11, 14
 * Current Sensors - ADC2 CH 12, 17
 */

/// @section Constants

/* With these settings and ADC clock =  CLK/4 and sample cycles = 47.5*/
/* We have 1 sample per 900us*/
#define ADC_1_DMA_CHANNELS 5
#define READINGS_PER_ADC_1 128
#define ADC_1_DMA_BUFFER_SIZE (ADC_1_DMA_CHANNELS * READINGS_PER_ADC_1)
#define ADC_1_DMA_HALF_BUFFER_SIZE (ADC_1_DMA_BUFFER_SIZE / 2)

/* With these settings and ADC clock =  CLK/4 and sample cycles = 47.5*/
/* We have 1 sample per 45us*/
#define ADC_2_DMA_CHANNELS 2
#define READINGS_PER_ADC_2 32
#define ADC_2_DMA_BUFFER_SIZE (ADC_2_DMA_CHANNELS * READINGS_PER_ADC_2)
#define ADC_2_DMA_HALF_BUFFER_SIZE (ADC_2_DMA_BUFFER_SIZE / 2)

#define ADC_MAX_VALUE 4095.0
#define ADC_MAX_VOLTAGE_MV 3300.0
#define PWR_BATTERY_THRESHOLD_MV 10700.0
#define PWR_BAT_VOLTAGE_DIV_R1 100.0
#define PWR_BAT_VOLTAGE_DIV_R2 33.0
#define PWR_BAT_VOLTAGE_MULTIPLIER ((PWR_BAT_VOLTAGE_DIV_R1 + PWR_BAT_VOLTAGE_DIV_R2) / PWR_BAT_VOLTAGE_DIV_R2)
#define PWR_BAT_POSITION_IN_ADC 4

#define IR_AVG_WINDOW 20

/// @section Private variables

static uint32_t adc_1_dma_buffer[ADC_1_DMA_BUFFER_SIZE];
static uint32_t adc_2_dma_buffer[ADC_2_DMA_BUFFER_SIZE];
static bsp_analog_ready_callback_t reading_ready_callback;

static uint32_t ir_readings[4];
static int32_t ir_readings_on[4];
static int32_t ir_readings_off[4];
static uint32_t battery_reading;
static uint32_t current_reading[2];
static bool modulation_enabled;
static uint32_t ir_window[4][IR_AVG_WINDOW];
static size_t window_idx[4];

/* Reading value when robot is in the middle of the cell */
static uint32_t ir_wall_dist_reference[4] = {
    820, // RIGHT
    150,  // FRONT_LEFT
    150,  // FRONT_RIGHT
    570  // LEFT
};

/* Threshold to calculate PID error */
static uint32_t ir_threshold_control[4] = {
    600,  // RIGHT
    250, // FRONT_LEFT
    250, // FRONT_RIGHT
    400   // LEFT
};

/* Reading on the cell start for considering wall on the next cell */
static uint32_t ir_wall_threshold[4] = {
    420, // RIGHT
    900, // FRONT_LEFT
    900, // FRONT_RIGHT
    420  // LEFT
};

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
    // HAL_ADC_Start_DMA(&hadc2, adc_2_dma_buffer, ADC_2_DMA_BUFFER_SIZE);
}

void stop(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    // HAL_ADC_Stop_DMA(&hadc2);
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

float battery_latest_reading_mv(void) {
    float measured_adc_voltage = (battery_reading / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE_MV;
    return measured_adc_voltage * PWR_BAT_VOLTAGE_MULTIPLIER;
}

bool battery_low() {
    return battery_latest_reading_mv() <= PWR_BATTERY_THRESHOLD_MV;
}

uint32_t* current_latest_reading(void) {
    return current_reading;
}

uint32_t ir_reading(SensingDirection direction) {
    return ir_readings[direction];
}

bool ir_reading_wall(SensingDirection direction) {
    return (ir_readings[direction] > ir_wall_threshold[direction]);
}

int32_t ir_side_wall_error() {
    int32_t left_error = ir_readings[SensingDirection::LEFT] - ir_wall_dist_reference[SensingDirection::LEFT];
    int32_t right_error = ir_readings[SensingDirection::RIGHT] - ir_wall_dist_reference[SensingDirection::RIGHT];

    int32_t ir_error;
    if (ir_wall_control_valid(SensingDirection::LEFT) && ir_wall_control_valid(SensingDirection::RIGHT)) {
        ir_error = left_error - right_error;
    } else if (ir_wall_control_valid(SensingDirection::LEFT)) {
        ir_error = 2.0 * left_error;
    } else if (ir_wall_control_valid(SensingDirection::RIGHT)) {
        ir_error = -2.0 * right_error;
    } else {
        ir_error = 0;
    }

    return ir_error;
}

int32_t ir_diagonal_error() {
    bool greater_error_left = ir_readings[SensingDirection::FRONT_LEFT] > ir_readings[SensingDirection::FRONT_RIGHT];

    int32_t ir_error;

    if (greater_error_left && ir_wall_control_valid(SensingDirection::FRONT_LEFT)) {
        ir_error = ir_readings[SensingDirection::FRONT_LEFT] - ir_wall_dist_reference[SensingDirection::FRONT_LEFT];
    } else if (!greater_error_left && ir_wall_control_valid(SensingDirection::FRONT_RIGHT)) {
        ir_error = -(ir_readings[SensingDirection::FRONT_RIGHT] - ir_wall_dist_reference[SensingDirection::FRONT_RIGHT]);
    } else {
        ir_error = 0;
    }

    return ir_error;
}

bool ir_wall_control_valid(SensingDirection direction) {
    return (ir_readings[direction] > ir_threshold_control[direction]);
}

void enable_modulation(bool enable) {
    modulation_enabled = enable;
}

/// @section Private functions

void adc1_callback(uint32_t* data) {
    static bool read_ir_off = true;
    uint32_t aux_readings[ADC_1_DMA_CHANNELS] = {0};

    if (modulation_enabled) {
        if (read_ir_off) {
            bsp::leds::ir_emitter_all_on();
        } else {
            bsp::leds::ir_emitter_all_off();
        }
    }

    for (uint16_t i = 0; i < (ADC_1_DMA_HALF_BUFFER_SIZE - 1); i += ADC_1_DMA_CHANNELS) {
        for (uint16_t j = 0; j < ADC_1_DMA_CHANNELS; j++) {
            aux_readings[j] += data[i + j];
        }
    }

    for (uint16_t j = 0; j < ADC_1_DMA_CHANNELS; j++) {
        aux_readings[j] /= (ADC_1_DMA_HALF_BUFFER_SIZE / ADC_1_DMA_CHANNELS);
    }

    for (int i = 0; i < 4; i++) {
        if (modulation_enabled) {
            if (read_ir_off) {
                ir_readings_off[i] = aux_readings[i];
            } else {
                ir_readings_on[i] = aux_readings[i];
                uint32_t reading = std::max(ir_readings_on[i] - ir_readings_off[i], 0L);
                ir_readings[i] = moving_average(ir_window[i], IR_AVG_WINDOW, &window_idx[i], reading);
            }
        } else {
            ir_readings[0] = aux_readings[0];
        }
    }

    read_ir_off = !read_ir_off;
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
