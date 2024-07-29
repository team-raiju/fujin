/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include "imu_service.h"
#include "bsp.h"
#include "bsp_gpio_mapping.h"
#include "bsp_i2c.h"
#include "lsm6dsr.h"
#include "lsm6dsr_reg.h"
#include "bsp_crc.h"
#include "bsp_vcp.h"
#include "motion_gc.h"

/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/
// When using frequencies higher than 415hz, gyro error is so big that it is not possible to calibrate
#define OUTPUT_DATA_RATE_HZ 415

#define LSM6DSR_I2C_ADDR    0xD4
#define LSM6DSR_I2C_CHANNEL IO_I2C_2
#define WHO_I_AM_VAL        0x6B

#define INTIAL_VALUE_FLAG 0xffffffff

#define SAMPLE_FREQ_HZ 1000

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/

typedef struct {
    float x;
    float y;
    float z;
} axis_3x_data_t;

/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/
static LSM6DSR_Object_t lsm6dsr_ctx;
static LSM6DSR_IO_t lsm6dsr_io_ctx;
static uint8_t who_i_am_val = 0;
static axis_3x_data_t angles = { 0.0, 0.0, 0.0 };

static float x_gbias = 0.0;
static float y_gbias = 0.0;
static float z_gbias = 0.0;

static uint32_t last_time_imu = INTIAL_VALUE_FLAG;

static axis_3x_data_t angular_rate_dps;

static float sample_frequency = SAMPLE_FREQ_HZ;

static const bool use_motion_gc = true;
/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/

static int32_t imu_init_peripheral()
{
    return 0;
}

static int32_t imu_deinit_peripheral()
{
    return 0;
}

static int32_t get_tick_adapter()
{
    int32_t ticks = (int32_t)BSP_GetTick();
    return ticks;
}

static int32_t imu_read_i2c(uint16_t addr, uint16_t mem_address, uint8_t *data, uint16_t len)
{
    return bsp_i2c_read(LSM6DSR_I2C_CHANNEL, addr, mem_address, IO_I2C_MEM_SIZE_8BIT, data, len);
}

static int32_t imu_write_i2c(uint16_t addr, uint16_t mem_address, uint8_t *data, uint16_t len)
{
    return bsp_i2c_write(LSM6DSR_I2C_CHANNEL, addr, mem_address, IO_I2C_MEM_SIZE_8BIT, data, len);
}

static void transform_mdps_to_dps(LSM6DSR_Axes_t *mdps, axis_3x_data_t *dps)
{
    dps->x = mdps->x / 1000.0f;
    dps->y = mdps->y / 1000.0f;
    dps->z = mdps->z / 1000.0f;
}

// static void transform_mg_to_g(LSM6DSR_Axes_t *mg, axis_3x_data_t *g)
// {
//     g->x = mg->x / 1000.0f;
//     g->y = mg->y / 1000.0f;
//     g->z = mg->z / 1000.0f;
// }

static float calculate_delta_time_s()
{
    uint32_t current_time = BSP_Get1usTick();

    if (last_time_imu == INTIAL_VALUE_FLAG) {
        last_time_imu = current_time;
    }

    uint32_t delta_time_1us = (current_time - last_time_imu);

    float delta_time_s = delta_time_1us / 1000000.0;
    last_time_imu = current_time;

    return delta_time_s;
}

static int8_t imu_init_motion_gc()
{
    MotionGC_Initialize(MGC_MCU_STM32, &sample_frequency);
    MGC_knobs_t knobs;
    MotionGC_GetKnobs(&knobs);

    /* Experimentaly set */
    knobs.AccThr = 0.05f;
    knobs.GyroThr = 0.4f;
    knobs.MaxGyro = 15.0f;
    knobs.MaxAcc = 1.4f;
    MotionGC_SetKnobs(&knobs);

    MGC_output_t initial_calib = {
        .GyroBiasX = x_gbias,
        .GyroBiasY = y_gbias,
        .GyroBiasZ = z_gbias,
    };

    MotionGC_SetCalParams(&initial_calib);

    return 0;
}

static int imu_sensor_init()
{
    lsm6dsr_io_ctx.Init = imu_init_peripheral;
    lsm6dsr_io_ctx.DeInit = imu_deinit_peripheral;
    lsm6dsr_io_ctx.Address = LSM6DSR_I2C_ADDR;
    lsm6dsr_io_ctx.BusType = LSM6DSR_I2C_BUS;
    lsm6dsr_io_ctx.GetTick = get_tick_adapter;
    lsm6dsr_io_ctx.ReadReg = imu_read_i2c;
    lsm6dsr_io_ctx.WriteReg = imu_write_i2c;

    if (LSM6DSR_RegisterBusIO(&lsm6dsr_ctx, &lsm6dsr_io_ctx) != LSM6DSR_OK) {
        return -1;
    }

    if (LSM6DSR_Init(&lsm6dsr_ctx) != LSM6DSR_OK) {
        return -1;
    }

    if (LSM6DSR_ReadID(&lsm6dsr_ctx, &who_i_am_val) != LSM6DSR_OK) {
        return -1;
    }

    if (who_i_am_val != WHO_I_AM_VAL) {
        return -1;
    }

    if (LSM6DSR_ACC_Enable(&lsm6dsr_ctx) != LSM6DSR_OK) {
        return -1;
    }

    if (LSM6DSR_GYRO_Enable(&lsm6dsr_ctx) != LSM6DSR_OK) {
        return -1;
    }

    if (LSM6DSR_GYRO_SetFullScale(&lsm6dsr_ctx, 4000) != LSM6DSR_OK) {
        return -1;
    }

    if (LSM6DSR_ACC_SetFullScale(&lsm6dsr_ctx, 2) != LSM6DSR_OK) {
        return -1;
    }

    if (LSM6DSR_ACC_SetOutputDataRate(&lsm6dsr_ctx, OUTPUT_DATA_RATE_HZ) != LSM6DSR_OK) {
        return -1;
    }

    if (LSM6DSR_GYRO_SetOutputDataRate(&lsm6dsr_ctx, OUTPUT_DATA_RATE_HZ) != LSM6DSR_OK) {
        return -1;
    }

    if (use_motion_gc) {
        /* Gyroscope calibration */
        if (imu_init_motion_gc() != 0) {
            return -1;
        }
    }

    return 0;
}

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/

void imu_service_init()
{
    if (use_motion_gc) {
        bsp_crc_init();
    }

    imu_sensor_init();
}

int8_t imu_update()
{
    LSM6DSR_Axes_t angular_rate_mdps;
    LSM6DSR_Axes_t acceleration_mg;
    MGC_input_t data_in_gc;
    MGC_output_t data_out_gc;
    int bias_update;

    /* Get raw data */
    if (use_motion_gc) {
        if (LSM6DSR_ACC_GetAxes(&lsm6dsr_ctx, &acceleration_mg) != LSM6DSR_OK) {
            return -1;
        }
    }

    if (LSM6DSR_GYRO_GetAxes(&lsm6dsr_ctx, &angular_rate_mdps) != LSM6DSR_OK) {
        return -1;
    }

    /* Transform to SI units */
    transform_mdps_to_dps(&angular_rate_mdps, &angular_rate_dps);
    float delta_time = calculate_delta_time_s();

    /* Apply calibration */
    if (use_motion_gc) {
        float new_frequency = 1.0f / delta_time;
        MotionGC_SetFrequency(&new_frequency);

        data_in_gc.Acc[0] = (acceleration_mg.x / 1000.0f);
        data_in_gc.Acc[1] = (acceleration_mg.y / 1000.0f);
        data_in_gc.Acc[2] = (acceleration_mg.z / 1000.0f);

        data_in_gc.Gyro[0] = angular_rate_dps.x;
        data_in_gc.Gyro[1] = angular_rate_dps.y;
        data_in_gc.Gyro[2] = angular_rate_dps.z;
        MotionGC_Update(&data_in_gc, &data_out_gc, &bias_update);

        angular_rate_dps.x = (data_in_gc.Gyro[0] - data_out_gc.GyroBiasX);
        angular_rate_dps.y = (data_in_gc.Gyro[1] - data_out_gc.GyroBiasY);
        angular_rate_dps.z = (data_in_gc.Gyro[2] - data_out_gc.GyroBiasZ);
    }

    /* Update angles */
    float temp_angle_z = angles.z + angular_rate_dps.z * delta_time;

    /* Filter angle Z*/
    if (temp_angle_z > 360.0f) {
        temp_angle_z -= 360.0f;
    } else if (temp_angle_z < 0.0f) {
        temp_angle_z += 360.0f;
    }

    angles.z = temp_angle_z;                     // yaw
    angles.x += angular_rate_dps.x * delta_time; // pitch
    angles.y += angular_rate_dps.y * delta_time; // roll

    return 0;
}

float get_imu_angle_z()
{
    return angles.z;
}

void reset_imu_angle_z()
{
    last_time_imu = INTIAL_VALUE_FLAG;
    angles.z = 0;
}

void print_g_bias()
{
    MGC_output_t gyro_bias;
    MotionGC_GetCalParams(&gyro_bias);

    DEBUG_PRINT("Gyro bias (X, Y, Z): %f %f %f\r\n", gyro_bias.GyroBiasX, gyro_bias.GyroBiasY, gyro_bias.GyroBiasZ);
}

void stop_g_bias_calculation()
{
    MGC_output_t gyro_bias;
    MotionGC_GetCalParams(&gyro_bias);

    DEBUG_PRINT("Gyro bias: %f %f %f\r\n", gyro_bias.GyroBiasX, gyro_bias.GyroBiasY, gyro_bias.GyroBiasZ);

    x_gbias = gyro_bias.GyroBiasX;
    y_gbias = gyro_bias.GyroBiasY;
    z_gbias = gyro_bias.GyroBiasZ;
}

void set_imu_z_bias(uint32_t set_z_bias, uint32_t signal_negative)
{
    if (signal_negative) {
        z_gbias = -1.0 * (float)set_z_bias / 1000.0f;
    } else {
        z_gbias = set_z_bias / 1000.0f;
    }

    MGC_output_t gyro_bias = {
        .GyroBiasX = x_gbias,
        .GyroBiasY = y_gbias,
        .GyroBiasZ = z_gbias,
    };
    MotionGC_SetCalParams(&gyro_bias);
}

uint32_t get_imu_z_bias()
{
    if (z_gbias < 0) {
        return -z_gbias * 1000;
    } else {
        return z_gbias * 1000;
    }
}

uint32_t get_imu_z_bias_signal_negative()
{
    return (z_gbias < 0);
}

float get_dps_angle_z()
{
    return angular_rate_dps.z;
}

float get_rad_s_angle_z()
{
    return angular_rate_dps.z * 0.01745329252;
}