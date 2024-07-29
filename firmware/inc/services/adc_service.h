#ifndef ADC_SERVICE_H
#define ADC_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

#define NUM_OF_IR_SENSORS 4
#define NUM_OF_CURRENT_SENSORS 2

// Position of sensor on ADC DMA buffer
typedef enum dist_sensor {
    DIST_SENS_L_S,
    DIST_SENS_L_F,
    DIST_SENS_R_F,
    DIST_SENS_R_S,
} dist_sensor_t;

typedef enum motor_current_sensor {
    MOTOR_CURRENT_R,
    MOTOR_CURRENT_L,
} motor_current_sensor_t;

void adc_service_init(void);
void adc_service_start_callback();

uint8_t dist_sensors_get_all();
bool adc_get_low_pwr_bat(void);
uint16_t adc_get_pwr_bat_mv(void);
void adc_dist_sensor_set_mask(uint8_t mask);
uint32_t adc_get_raw_dist(dist_sensor_t position);


#endif /* ADC_SERVICE_H */
