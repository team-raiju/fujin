#include "st/hal.h"

#include "bsp/imu.h"

void bsp_imu_init() {
    MX_I2C2_Init();
}
