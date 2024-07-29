#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include <stdint.h>

void imu_service_init();

int8_t imu_update();

float get_imu_angle_z();

void reset_imu_angle_z();

void print_g_bias();

void stop_g_bias_calculation();

void set_imu_z_bias(uint32_t set_z_bias, uint32_t signal_negative);

uint32_t get_imu_z_bias();

uint32_t get_imu_z_bias_signal_negative();

float get_dps_angle_z();

float get_rad_s_angle_z();

#endif /* IMU_SERVICE_H */
