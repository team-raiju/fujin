#include <lsm6dsr.h>
#include <motion_gc.h>
#include <cstdio>

#include "st/hal.h"

#include "bsp/imu.hpp"
#include "bsp/timers.hpp"
#include "pin_mapping.h"
#include "utils/math.hpp"

namespace bsp::imu {

/// @section Constants

#define WHO_I_AM_EXPECTED 0x6B
#define INITIAL_VALUE_FLAG 0xffffffff
#define SAMPLE_FREQ_HZ 1000
#define I2C_TIMEOUT 1000

/// @section Private variables
// If OUTPUT_DATA_RATE_HZ > 415, motion gc will not work. bias is not updated
static bool enable_motion_gc = false;
static uint16_t output_data_rate = 1200;
static uint16_t motion_gc_data_rate = 415;

static LSM6DSR_Object_t lsm6dsr_ctx;
static LSM6DSR_IO_t lsm6dsr_io;
static uint8_t who_i_am_val = 0;

static float init_x_gbias = 0.33;
static float init_y_gbias = -0.426;
static float init_z_gbias = -0.290;

// Angular Velocity in rad/s
static float ω;

// Last Angular Velocity in rad/s
static float last_ω;

// Angular acceleration in rad/s²
static float α;

// Angular Displacement in rad [-pi to pi]
static float φ;

// Incremental angular Displacement in rad [-inf to inf]
static float incremental_φ;

// Vertical acceleration in m/s²
static float a_z;

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
        .GyroBiasX = init_x_gbias,
        .GyroBiasY = init_y_gbias,
        .GyroBiasZ = init_z_gbias,
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

    if (LSM6DSR_ACC_SetOutputDataRate(&lsm6dsr_ctx, output_data_rate) != LSM6DSR_OK) {
        return ERROR;
    }

    if (LSM6DSR_GYRO_SetOutputDataRate(&lsm6dsr_ctx, output_data_rate) != LSM6DSR_OK) {
        return ERROR;
    }

    init_motion_gc();

    return OK;
}

ImuResult update() {
    // Angular Velocity in milidegrees per second
    LSM6DSR_Axes_t mω;

    // Gravity acceleration in mm/s²
    LSM6DSR_Axes_t mg = {0, 0, 0};

    MGC_input_t data_in_gc;
    MGC_output_t data_out_gc;
    int bias_update;

    if (enable_motion_gc) {
        if (LSM6DSR_ACC_GetAxes(&lsm6dsr_ctx, &mg) != LSM6DSR_OK) {
            return ERROR;
        }
    }

    if (LSM6DSR_GYRO_GetAxes(&lsm6dsr_ctx, &mω) != LSM6DSR_OK) {
        return ERROR;
    }

    float δt = Δt();

    ω = deg2rad(mω.z / 1000.0f);
    a_z = mg.z / 1000.0f;

    if (enable_motion_gc) {
        if (δt > 0.000001) {
            float new_frequency = 1.0f / δt;
            MotionGC_SetFrequency(&new_frequency);
        }

        data_in_gc.Acc[0] = (mg.x / 1000.0f);
        data_in_gc.Acc[1] = (mg.y / 1000.0f);
        data_in_gc.Acc[2] = (mg.z / 1000.0f);

        data_in_gc.Gyro[0] = mω.x / 1000.0f;
        data_in_gc.Gyro[1] = mω.y / 1000.0f;
        data_in_gc.Gyro[2] = mω.z / 1000.0f;
        MotionGC_Update(&data_in_gc, &data_out_gc, &bias_update);

        ω = deg2rad(data_in_gc.Gyro[2] - data_out_gc.GyroBiasZ);
    } else {
        ω = deg2rad((mω.z / 1000.0f) - get_g_bias_z());
    }

    if (δt > 0.000001) {
        α = (ω - last_ω) / δt;
    }

    last_ω = ω;

    φ += ω * δt;
    incremental_φ += ω * δt;

    // Return to first revolution
    while (φ > M_TWOPI) {
        φ -= M_TWOPI;
    }

    while (φ < 0.0f) {
        φ += M_TWOPI;
    }

    // Convert from 0-360 to -180-180
    if (φ > M_PI) {
        φ -= M_TWOPI;
    }

    return OK;
}

float get_angle() {
    return φ;
}

float get_incremental_angle() {
    return incremental_φ;
}

void reset_angle() {
    last_time_imu = 0;
    φ = 0;
    incremental_φ = 0;
}

float get_rad_per_s() {
    return ω;
}

float get_z_acceleration() {
    return a_z;
}

void set_g_bias_z(float z_gbias) {

    MGC_output_t gyro_bias = {
        .GyroBiasX = init_x_gbias,
        .GyroBiasY = init_y_gbias,
        .GyroBiasZ = z_gbias,
    };

    MotionGC_SetCalParams(&gyro_bias);
}

float get_g_bias_z() {
    MGC_output_t gyro_bias;
    MotionGC_GetCalParams(&gyro_bias);

    float z_gbias = gyro_bias.GyroBiasZ;
    return z_gbias;
}

void enable_motion_gc_filter(bool enable) {
    enable_motion_gc = enable;

    uint16_t new_data_rate = enable ? motion_gc_data_rate : output_data_rate;

    if (LSM6DSR_ACC_SetOutputDataRate(&lsm6dsr_ctx, new_data_rate) != LSM6DSR_OK) {
        std::printf("Error setting ACC output data rate\n");
    }

    if (LSM6DSR_GYRO_SetOutputDataRate(&lsm6dsr_ctx, new_data_rate) != LSM6DSR_OK) {
        std::printf("Error setting Gyro output data rate\n");
    }
}

bool is_imu_emergency() {
    bool emergency_z_angular_accel = std::abs(α) > 6000.0f;

    return emergency_z_angular_accel || (std::abs(ω) > 35.0);
}

} // namespace
