/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include "bsp.h"

#include "adc_service.h"
#include "bsp_adc_dma.h"
#include "bsp_motors.h"
#include "bsp_vcp.h"
#include "driving_service.h"
#include "utils.h"

#include <string.h>
/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/
#define ADC_MAX_VALUE      4095.0
#define ADC_MAX_VOLTAGE_MV 3300

/* Vcc ---- R1 --- R2 --- GND*/
// 3S
#define PWR_BATTERY_THRESHOLD_MV   11250.0
#define PWR_BAT_VOLTAGE_DIV_R1     100.0
#define PWR_BAT_VOLTAGE_DIV_R2     33.0
#define PWR_BAT_VOLTAGE_MULTIPLIER ((PWR_BAT_VOLTAGE_DIV_R1 + PWR_BAT_VOLTAGE_DIV_R2) / PWR_BAT_VOLTAGE_DIV_R2)
#define PWR_BAT_POSITION_IN_ADC    4

/* Motor current */
// #define R_IPROPI               2000     // Measured in ohms
// #define CURRENT_SCALING_FACTOR 0.000450 // Measured in uA/A
// #define CURRENT_TO_M_S         175.0f

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/

typedef enum dist_sensor_pos_in_adc {
    DIST_POS_L_S = 0,
    DIST_POS_L_F = 1,
    DIST_POS_R_F = 2,
    DIST_POS_R_S = 3,
} dist_sensor_pos_in_adc_t;

// typedef enum motor_current_pos_in_adc {
//     MOTOR_CURRENT_POS_R = 0,
//     MOTOR_CURRENT_POS_L = 1
// } motor_current_pos_in_adc_t;
/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/
static void adc_1_data_interrupt(uint32_t *out_data);
static void adc_2_data_interrupt(uint32_t *out_data);

/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/

static volatile bool dist_sensor_is_seeing[NUM_OF_IR_SENSORS];
static volatile bool dist_sensor_is_seeing_last[NUM_OF_IR_SENSORS];
static volatile float pwr_bat_voltage_mv = 12600;

static volatile float last_motor_current_l_ma = 0; // Measured in mA
static volatile float last_motor_current_r_ma = 0; // Measured in mA

static uint8_t dist_sensors_mask = 0xff;
static uint16_t seeing_threshold = 1000;

static volatile uint32_t last_adc_1_raw[NUM_OF_IR_SENSORS];
// static volatile uint32_t last_adc_2_raw[NUM_OF_CURRENT_SENSORS];

/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/
static void pwr_battery_value_update(uint16_t bat_raw_adc)
{
    float measured_voltage = (bat_raw_adc / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE_MV;

    pwr_bat_voltage_mv = (measured_voltage * PWR_BAT_VOLTAGE_MULTIPLIER);
}

static void dist_sensor_update(uint32_t *adc_1_raw_data)
{
    last_adc_1_raw[DIST_SENS_L_S] = adc_1_raw_data[DIST_POS_L_S];
    last_adc_1_raw[DIST_SENS_L_F] = adc_1_raw_data[DIST_POS_L_F];
    last_adc_1_raw[DIST_SENS_R_F] = adc_1_raw_data[DIST_POS_R_F];
    last_adc_1_raw[DIST_SENS_R_S] = adc_1_raw_data[DIST_POS_R_S];

    dist_sensor_is_seeing[DIST_SENS_L_S] = (last_adc_1_raw[DIST_SENS_L_S] < seeing_threshold);
    dist_sensor_is_seeing[DIST_SENS_L_F] = (last_adc_1_raw[DIST_SENS_L_F] < seeing_threshold);
    dist_sensor_is_seeing[DIST_SENS_R_F] = (last_adc_1_raw[DIST_SENS_R_F] < seeing_threshold);
    dist_sensor_is_seeing[DIST_SENS_R_S] = (last_adc_1_raw[DIST_SENS_R_S] < seeing_threshold);

    for (int i = 0; i < NUM_OF_IR_SENSORS; i++) {
        dist_sensor_is_seeing_last[i] = dist_sensor_is_seeing[i];
    }
}

// This interrupt is currently called every 45.2us due to ADC configurations
// Aproximately 22kHz
static void adc_1_data_interrupt(uint32_t *out_data)
{
    uint32_t aux_readings[ADC_1_DMA_CHANNELS] = { 0 };

    for (uint16_t i = 0; i < (ADC_1_DMA_HALF_BUFFER_SIZE - 1); i += ADC_1_DMA_CHANNELS) {
        for (uint16_t j = 0; j < ADC_1_DMA_CHANNELS; j++) {
            aux_readings[j] += out_data[i + j];
        }
    }

    for (uint16_t j = 0; j < ADC_1_DMA_CHANNELS; j++) {
        aux_readings[j] /= (ADC_1_DMA_HALF_BUFFER_SIZE / ADC_1_DMA_CHANNELS);
    }

    dist_sensor_update(aux_readings);
    pwr_battery_value_update(aux_readings[PWR_BAT_POSITION_IN_ADC]);
}

static void adc_2_data_interrupt(uint32_t *out_data)
{
    uint32_t aux_readings[ADC_2_DMA_CHANNELS] = { 0 };

    for (uint16_t i = 0; i < (ADC_2_DMA_HALF_BUFFER_SIZE - 1); i += ADC_2_DMA_CHANNELS) {
        for (uint16_t j = 0; j < ADC_2_DMA_CHANNELS; j++) {
            aux_readings[j] += out_data[i + j];
        }
    }

    for (uint16_t j = 0; j < ADC_2_DMA_CHANNELS; j++) {
        aux_readings[j] /= (ADC_2_DMA_HALF_BUFFER_SIZE / ADC_2_DMA_CHANNELS);
    }

    //     motor_current_update(aux_readings);
}

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/

void adc_service_init()
{
    BSP_ADC_DMA_Init(BSP_ADC_1);
    BSP_ADC_DMA_Start(BSP_ADC_1);

    // BSP_ADC_DMA_Init(BSP_ADC_2);
    // BSP_ADC_DMA_Start(BSP_ADC_2);
}

void adc_service_start_callback()
{
    BSP_ADC_DMA_Register_Callback(BSP_ADC_1, adc_1_data_interrupt);
    BSP_ADC_DMA_Register_Callback(BSP_ADC_2, adc_2_data_interrupt);
}

bool adc_dist_sensor_is_seeing(dist_sensor_t position)
{
    if (position > NUM_OF_IR_SENSORS) {
        return 0;
    }

    bool line_sensor_enable = dist_sensors_mask & (1 << position);

    return (dist_sensor_is_seeing[position] & line_sensor_enable);
}

uint8_t adc_dist_sensors_get_all()
{
    uint8_t mask = 0;
    for (int i = 0; i < NUM_OF_IR_SENSORS; i++) {
        if (adc_dist_sensor_is_seeing(i)) {
            mask |= (1 << i);
        }
    }

    return mask;
}

bool adc_get_low_pwr_bat()
{
    return (pwr_bat_voltage_mv <= PWR_BATTERY_THRESHOLD_MV);
}

uint16_t adc_get_pwr_bat_mv()
{
    return (uint16_t)pwr_bat_voltage_mv;
}

void adc_dist_sensor_set_mask(uint8_t mask)
{
    dist_sensors_mask = mask;
}

uint32_t adc_get_raw_dist(dist_sensor_t position)
{
    if (position > NUM_OF_IR_SENSORS) {
        return 0;
    }

    return last_adc_1_raw[position];
}
