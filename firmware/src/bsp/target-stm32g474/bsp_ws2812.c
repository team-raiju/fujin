/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include "bsp_ws2812.h"
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include <stdbool.h>

/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/
#define WS2812_PACKET_SIZE 24
#define FINAL_PADDING_SIZE \
    2 /* Used to guarantee that no extra bits will be sent at the final of the   \
       data stream */

#define WS2812_PWM_SIZE ((WS2812_PACKET_SIZE * WS2812_MAX_LED_AMOUNT) + FINAL_PADDING_SIZE)

/* PWM BIT1 must be a number so that the pwm has 580ns to 1000ns high time */
/* PWM BIT0 must be a number so that the pwm has 220ns to 380ns high time */

/* Here we are using BIT1 approximately 625ns and BIT0 approximately 305ns */
/* Considering timer frequency = 85MHz */

#define PWM_BIT_1 53 // 85 * 0.625 = 53.125
#define PWM_BIT_0 26 // 85 * 0.305 = 25.925

/* The maximum value of the counter is 105. So the PWM period is 1250ns */

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/
static uint32_t color_to_bits(uint8_t r, uint8_t g, uint8_t b);

/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/

uint32_t led_data[WS2812_MAX_LED_AMOUNT];
uint32_t pwm_data[WS2812_PWM_SIZE];

/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/
static uint32_t color_to_bits(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t bits = 0;

    // blue
    for (int8_t i = 7; i >= 0; i--) {
        if (b & (1 << i)) {
            bits |= (1 << (23 - (7 - i)));
        }
    }

    // red
    for (int8_t i = 7; i >= 0; i--) {
        if (r & (1 << i)) {
            bits |= (1 << (15 - (7 - i)));
        }
    }

    // green
    for (int8_t i = 7; i >= 0; i--) {
        if (g & (1 << i)) {
            bits |= (1 << (7 - (7 - i)));
        }
    }

    return bits;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    }
}

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/

void BSP_ws2812_init()
{
    MX_TIM2_Init(); // Led Stripe Timer
}

void BSP_ws2812_set(uint8_t num, uint8_t r, uint8_t g, uint8_t b)
{
    if (num >= WS2812_MAX_LED_AMOUNT) {
        return;
    }

    led_data[num] = color_to_bits(r, g, b);
}

void BSP_ws2812_send()
{
    for (uint8_t i = 0; i < WS2812_MAX_LED_AMOUNT; i++) {
        for (int8_t j = (WS2812_PACKET_SIZE - 1); j >= 0; j--) {
            if (led_data[i] & (1 << j)) {
                pwm_data[(i * WS2812_PACKET_SIZE) + j] = PWM_BIT_1;
            } else {
                pwm_data[(i * WS2812_PACKET_SIZE) + j] = PWM_BIT_0;
            }
        }
    }

    for (int i = 0; i < FINAL_PADDING_SIZE; i++) {
        pwm_data[(WS2812_PACKET_SIZE * WS2812_MAX_LED_AMOUNT) + i] = 0;
    }

    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, pwm_data, WS2812_PWM_SIZE);
}
