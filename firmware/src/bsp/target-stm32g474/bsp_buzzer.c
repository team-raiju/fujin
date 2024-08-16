#include "bsp_buzzer.h"
#include "tim.h"
#include <stdint.h>

#define CPU_FREQUENCY  (170000000UL) // SystemCoreClock
#define COUNTER_PERIOD (100)

void BSP_buzzerInit()
{
    MX_TIM8_Init();
}

void BSP_buzzerStart()
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}

void BSP_buzzerStop()
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
}

void BSP_buzzerSetPwmDutyCycle(uint8_t duty_cycle)
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty_cycle);
}

void BSP_buzzerSetFrequency(uint16_t frequency_hz)
{
    // frequency_hz = (CPU_FREQUENCY/ COUNTER_PERIOD * (preescaler_value + 1)) ->

    uint16_t preescaler_value = ((CPU_FREQUENCY / COUNTER_PERIOD) / frequency_hz) - 1;
    __HAL_TIM_SET_PRESCALER(&htim8, preescaler_value);
}
