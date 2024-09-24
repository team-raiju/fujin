#include <lsm6dsr.h>
#include <motion_gc.h>

#include "st/hal.h"

#include "bsp/imu.hpp"
#include "bsp/timers.hpp"
#include "pin_mapping.h"
#include "utils/math.h"

namespace bsp::imu {

/// @section Constants

#define USE_MOTION_GC true

#define OUTPUT_DATA_RATE_HZ 415
#define WHO_I_AM_EXPECTED 0x6B
#define INITIAL_VALUE_FLAG 0xffffffff
#define SAMPLE_FREQ_HZ 1000
#define I2C_TIMEOUT 1000

/// @section Private variables

static LSM6DSR_Object_t lsm6dsr_ctx;
static LSM6DSR_IO_t lsm6dsr_io;
static uint8_t who_i_am_val = 0;

static float x_gbias = 0.0;
static float y_gbias = 0.0;
static float z_gbias = 0.0;

// Angular Velocity in rad/s
static float ω;

// Angular Displacement in rad
static float φ;

static float sample_frequency = SAMPLE_FREQ_HZ;

/// @section Private functions

void init_motion_gc() {
    MotionGC_Initialize(MGC_MCU_STM32, &sample_frequency);
    MGC_knobs_t knobs;
    MotionGC_GetKnobs(&knobs);

    // Empiric
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
}

static uint32_t last_time_imu = 0;
/// @brief Get time passed since last time this function was called, in seconds
/// @return delta time since last call in seconds
float Δt() {
    uint32_t current_tick = bsp::get_tick_us();

    // If we just started or reset the angle, Δt will be 0
    if (last_time_imu == 0) {
        last_time_imu = current_tick;
    }

    uint32_t δt = (current_tick - last_time_imu);

    float delta_time_s = δt / 1000000.0;
    last_time_imu = current_tick;

    return delta_time_s;
}

/// @section Interface implementation

ImuResult init() {
    MX_I2C2_Init();

    lsm6dsr_io.Init = [] { return 0L; };
    lsm6dsr_io.DeInit = [] { return 0L; };
    lsm6dsr_io.Address = LSM6DSR_I2C_ADDR;
    lsm6dsr_io.BusType = LSM6DSR_I2C_BUS;
    lsm6dsr_io.GetTick = [] { return (long)bsp::get_tick_ms(); };
    lsm6dsr_io.ReadReg = [](uint16_t addr, uint16_t mem_address, uint8_t* data, uint16_t len) {
        return (long)HAL_I2C_Mem_Read(&hi2c2, addr, mem_address, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
    };
    lsm6dsr_io.WriteReg = [](uint16_t addr, uint16_t mem_address, uint8_t* data, uint16_t len) {
        return (long)HAL_I2C_Mem_Write(&hi2c2, addr, mem_address, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
    };

    if (LSM6DSR_RegisterBusIO(&lsm6dsr_ctx, &lsm6dsr_io) != LSM6DSR_OK) {
        return ERROR;
    }

    if (LSM6DSR_Init(&lsm6dsr_ctx) != LSM6DSR_OK) {
        return ERROR;
    }

    if (LSM6DSR_ReadID(&lsm6dsr_ctx, &who_i_am_val) != LSM6DSR_OK) {
        return ERROR;
    }

    if (who_i_am_val != WHO_I_AM_EXPECTED) {
        return ERROR;
    }

    if (LSM6DSR_ACC_Enable(&lsm6dsr_ctx) != LSM6DSR_OK) {
        return ERROR;
    }

    if (LSM6DSR_GYRO_Enable(&lsm6dsr_ctx) != LSM6DSR_OK) {
        return ERROR;
    }

    if (LSM6DSR_GYRO_SetFullScale(&lsm6dsr_ctx, 4000) != LSM6DSR_OK) {
        return ERROR;
    }

    if (LSM6DSR_ACC_SetFullScale(&lsm6dsr_ctx, 2) != LSM6DSR_OK) {
        return ERROR;
    }

    if (LSM6DSR_ACC_SetOutputDataRate(&lsm6dsr_ctx, OUTPUT_DATA_RATE_HZ) != LSM6DSR_OK) {
        return ERROR;
    }

    if (LSM6DSR_GYRO_SetOutputDataRate(&lsm6dsr_ctx, OUTPUT_DATA_RATE_HZ) != LSM6DSR_OK) {
        return ERROR;
    }

#if USE_MOTION_GC
    init_motion_gc();
#endif

    return OK;
}

ImuResult update() {
    // Angular Velocity in milidegrees per second
    LSM6DSR_Axes_t mω;

    // Gravity acceleration in mm/s²
    LSM6DSR_Axes_t mg;

    MGC_input_t data_in_gc;
    MGC_output_t data_out_gc;
    int bias_update;

#if USE_MOTION_GC
    if (LSM6DSR_ACC_GetAxes(&lsm6dsr_ctx, &mg) != LSM6DSR_OK) {
        return ERROR;
    }
#endif

    if (LSM6DSR_GYRO_GetAxes(&lsm6dsr_ctx, &mω) != LSM6DSR_OK) {
        return ERROR;
    }

    float δt = Δt();

    ω = deg2rad(mω.z / 1000.0f);

#if USE_MOTION_GC
    float new_frequency = 1.0f / δt;
    MotionGC_SetFrequency(&new_frequency);

    data_in_gc.Acc[0] = (mg.x / 1000.0f);
    data_in_gc.Acc[1] = (mg.y / 1000.0f);
    data_in_gc.Acc[2] = (mg.z / 1000.0f);

    data_in_gc.Gyro[0] = mω.x / 1000.0f;
    data_in_gc.Gyro[1] = mω.y / 1000.0f;
    data_in_gc.Gyro[2] = mω.z / 1000.0f;
    MotionGC_Update(&data_in_gc, &data_out_gc, &bias_update);

    ω = deg2rad(data_in_gc.Gyro[2] - data_out_gc.GyroBiasZ);
#endif

    φ += ω * δt;

    /* Filter angle Z*/
    while (φ > 360.0f) {
        φ -= 360.0f;
    }

    while (φ < 0.0f) {
        φ += 360.0f;
    }

    return OK;
}

float get_angle() {
    return φ;
}

void reset_angle() {
    last_time_imu = 0;
    φ = 0;
}

float get_rps() {
    return ω;
}

void update_g_bias() {
    MGC_output_t gyro_bias;
    MotionGC_GetCalParams(&gyro_bias);

    x_gbias = gyro_bias.GyroBiasX;
    y_gbias = gyro_bias.GyroBiasY;
    z_gbias = gyro_bias.GyroBiasZ;
}

void set_g_bias(int32_t bias) {
    z_gbias = bias / 1000.0f;

    MGC_output_t gyro_bias = {
        .GyroBiasX = x_gbias,
        .GyroBiasY = y_gbias,
        .GyroBiasZ = z_gbias,
    };

    MotionGC_SetCalParams(&gyro_bias);
}

int32_t get_g_bias() {
    return z_gbias;
}

} // namespace
