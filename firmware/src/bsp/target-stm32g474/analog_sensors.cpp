#include "st/hal.h"

#include "bsp/analog_sensors.hpp"
#include "bsp/leds.hpp"
#include "bsp/timers.hpp"
#include "services/config.hpp"
#include "utils/math.hpp"

namespace bsp::analog_sensors {

/*
 * Battery Sensor - ADC1 CH15
 * Phototransistors - ADC1 CH 1, 2, 11, 14
 * Current Sensors - ADC2 CH 12, 17
 */

/// @section Constants

/* With ADC clock = PCLK/4 (42.5 MHz), 5 channels, and 24.5 sample cycles (37 cycles/conv):
 * - 1 conversion: 37 / 42.5 MHz = 0.87 us
 * - 1 scan (5 ch): 185 / 42.5 MHz = 4.35 us
 * - Half-buffer (22 scans): 22 * 4.35 us = ~95.8 us per DMA interrupt (phase duration)
 * - Full 5-phase modulation cycle (OFF + 4 individual ON): 5 * 95.8 us = ~479 us (~2.09 kHz <= 500 us)
 * - EMA filter (alpha = 0.5): group delay = 1 * 479 us = ~479 us (90% settling: ~1.59 ms)
 */
#define ADC_1_DMA_CHANNELS 5
#define READINGS_PER_ADC_1 44
#define ADC_1_DMA_BUFFER_SIZE (ADC_1_DMA_CHANNELS * READINGS_PER_ADC_1)
#define ADC_1_DMA_HALF_BUFFER_SIZE (ADC_1_DMA_BUFFER_SIZE / 2)

/* With these settings and ADC clock = CLK/4 and sample cycles = 24.5*/
/* We have 1 sample per 28us*/
#define ADC_2_DMA_CHANNELS 2
#define READINGS_PER_ADC_2 32
#define ADC_2_DMA_BUFFER_SIZE (ADC_2_DMA_CHANNELS * READINGS_PER_ADC_2)
#define ADC_2_DMA_HALF_BUFFER_SIZE (ADC_2_DMA_BUFFER_SIZE / 2)

#define ADC_MAX_VALUE 4095.0
#define ADC_MAX_VOLTAGE_MV 3300.0
#define ADC_MAX_VOLTAGE_VOLTS 3.3f

#define PWR_BATTERY_THRESHOLD_MV 10700.0
#define PWR_BAT_VOLTAGE_DIV_R1 100.0
#define PWR_BAT_VOLTAGE_DIV_R2 33.0
// #define PWR_BAT_VOLTAGE_MULTIPLIER ((PWR_BAT_VOLTAGE_DIV_R1 + PWR_BAT_VOLTAGE_DIV_R2) / PWR_BAT_VOLTAGE_DIV_R2)
#define PWR_BAT_VOLTAGE_MULTIPLIER (4.19) // Experimentaly set
#define PWR_BAT_POSITION_IN_ADC 4

#define IR_EMA_ALPHA 0.5f

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
static uint8_t mod_step = 0;
static constexpr bsp::leds::Emitter sensor_emitters[4] = {
    bsp::leds::LEFT_SIDE,
    bsp::leds::LEFT_FRONT,
    bsp::leds::RIGHT_FRONT,
    bsp::leds::RIGHT_SIDE,
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
    bsp::leds::ir_emitter_all_off();
    mod_step = 0;
    HAL_ADC_Start_DMA(&hadc1, adc_1_dma_buffer, ADC_1_DMA_BUFFER_SIZE);
    // HAL_ADC_Start_DMA(&hadc2, adc_2_dma_buffer, ADC_2_DMA_BUFFER_SIZE);
}

void stop(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    bsp::leds::ir_emitter_all_off();
    mod_step = 0;
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

float battery_latest_reading_volts(void) {
    float measured_adc_voltage = (battery_reading / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE_VOLTS;
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
    switch (direction) {
    case SensingDirection::RIGHT:
        return ir_readings[direction] > services::Config::ir_wall_detect_th_right;
    case SensingDirection::FRONT_LEFT:
        return ir_readings[direction] > services::Config::ir_wall_detect_th_front_left;
    case SensingDirection::FRONT_RIGHT:
        return ir_readings[direction] > services::Config::ir_wall_detect_th_front_right;
    case SensingDirection::LEFT:
        return ir_readings[direction] > services::Config::ir_wall_detect_th_left;
    default:
        return false;
    }
}

int32_t ir_side_wall_error() {
    int32_t left_error = ir_readings[SensingDirection::LEFT] - services::Config::ir_wall_dist_ref_left;
    int32_t right_error = ir_readings[SensingDirection::RIGHT] - services::Config::ir_wall_dist_ref_right;

    int32_t ir_error;
    if (ir_wall_control_valid(SensingDirection::LEFT) && ir_wall_control_valid(SensingDirection::RIGHT)) {
        ir_error = left_error - right_error;
    } else if (ir_wall_control_valid(SensingDirection::LEFT)) {
        ir_error = 1.5 * left_error;
    } else if (ir_wall_control_valid(SensingDirection::RIGHT)) {
        ir_error = -1.5 * right_error;
    } else {
        ir_error = 0;
    }

    return ir_error;
}

SensingStatus ir_get_sensing_status() {
    SensingPattern current_pattern;
    current_pattern.FL = ir_reading(FRONT_LEFT);
    current_pattern.FR = ir_reading(FRONT_RIGHT);
    current_pattern.L = ir_reading(LEFT);
    current_pattern.R = ir_reading(RIGHT);

    // Compares the current value to all references and find the closest match
    uint32_t min_diff = 10000;
    uint8_t pattern_idx = 0;
    for (uint8_t i = 0; i < ir_wall_patterns.size(); i++) {
        uint32_t diff = std::abs((int32_t)current_pattern.FL - (int32_t)ir_wall_patterns[i].FL) +
                        std::abs((int32_t)current_pattern.FR - (int32_t)ir_wall_patterns[i].FR) +
                        std::abs((int32_t)current_pattern.L - (int32_t)ir_wall_patterns[i].L) +
                        std::abs((int32_t)current_pattern.R - (int32_t)ir_wall_patterns[i].R);
        if (diff < min_diff) {
            min_diff = diff;
            pattern_idx = i;
        }
    }

    SensingStatus status = {false, false, false};

    switch (pattern_idx) {
    case 0:
        // F-L-R
        status.front_seeing = true;
        status.left_seeing = true;
        status.right_seeing = true;
        break;
    case 1:
        // F-L
        status.front_seeing = true;
        status.left_seeing = true;
        break;
    case 2:
        // F-R
        status.front_seeing = true;
        status.right_seeing = true;
        break;
    case 3:
        // F
        status.front_seeing = true;
        break;
    case 4:
        // L-R
        status.left_seeing = true;
        status.right_seeing = true;
        break;
    case 5:
        // L
        status.left_seeing = true;
        break;
    case 6:
        // R
        status.right_seeing = true;
        break;
    case 7:
        // None
        break;
    }

    return status;
}

int32_t ir_diagonal_error() {
    bool greater_error_left = ir_readings[SensingDirection::FRONT_LEFT] > ir_readings[SensingDirection::FRONT_RIGHT];

    int32_t ir_error;

    if (greater_error_left && ir_wall_control_valid(SensingDirection::FRONT_LEFT)) {
        ir_error = ir_readings[SensingDirection::FRONT_LEFT] - services::Config::ir_wall_dist_ref_front_left;
    } else if (!greater_error_left && ir_wall_control_valid(SensingDirection::FRONT_RIGHT)) {
        ir_error = -(ir_readings[SensingDirection::FRONT_RIGHT] - services::Config::ir_wall_dist_ref_front_right);
    } else {
        ir_error = 0;
    }

    return ir_error;
}

bool ir_wall_control_valid(SensingDirection direction) {
    switch (direction) {
    case SensingDirection::RIGHT:
        return ir_readings[direction] > services::Config::ir_wall_control_th_right;
    case SensingDirection::FRONT_LEFT:
        return ir_readings[direction] > services::Config::ir_wall_control_th_front_left;
    case SensingDirection::FRONT_RIGHT:
        return ir_readings[direction] > services::Config::ir_wall_control_th_front_right;
    case SensingDirection::LEFT:
        return ir_readings[direction] > services::Config::ir_wall_control_th_left;
    default:
        return false;
    }
}

void enable_modulation(bool enable) {
    modulation_enabled = enable;
    mod_step = 0;
    bsp::leds::ir_emitter_all_off();
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

    if (!modulation_enabled) {
        bsp::leds::ir_emitter_all_off();
        mod_step = 0;
        for (int i = 0; i < 4; i++) {
            ir_readings[i] = aux_readings[i];
        }
    } else {
        if (mod_step == 0) {
            // Ambient light reading (all emitters were off during conversion)
            for (int i = 0; i < 4; i++) {
                ir_readings_off[i] = aux_readings[i];
            }
            // Turn ON emitter for sensor 0 for the upcoming buffer conversion
            bsp::leds::ir_emitter_on(sensor_emitters[0]);
            mod_step = 1;
        } else {
            uint8_t sensor_idx = mod_step - 1; // 0, 1, 2, 3

            // Record reading with only this sensor's emitter turned on
            ir_readings_on[sensor_idx] = aux_readings[sensor_idx];
            uint32_t reading = std::max(ir_readings_on[sensor_idx] - ir_readings_off[sensor_idx], 0L);
            ir_readings[sensor_idx] = static_cast<uint32_t>(
                IR_EMA_ALPHA * static_cast<float>(reading) + (1.0f - IR_EMA_ALPHA) * static_cast<float>(ir_readings[sensor_idx]));

            // Turn OFF current emitter
            bsp::leds::ir_emitter_off(sensor_emitters[sensor_idx]);

            if (mod_step < 4) {
                // Turn ON next sensor's emitter
                bsp::leds::ir_emitter_on(sensor_emitters[mod_step]);
                mod_step++;
            } else {
                // All emitters are now OFF; next cycle will sample ambient light (step 0)
                mod_step = 0;
            }
        }
    }

    battery_reading = 0.5 * aux_readings[4] + battery_reading * 0.5;
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
