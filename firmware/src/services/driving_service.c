#include "driving_service.h"
#include "adc_service.h"
#include "bsp_motors.h"
#include "utils.h"

#define M_S_TO_DUTY_CYCLE 175.0f

void driving_init()
{
    BSP_motorsInit();
}

void drive_m_s(float left_m_s, float righ_m_s)
{
    int16_t speed_l = constrain(left_m_s * M_S_TO_DUTY_CYCLE, -1000, 1000);
    int16_t speed_r = constrain(righ_m_s * M_S_TO_DUTY_CYCLE, -1000, 1000);

    BSP_motors(speed_l, speed_r);
}