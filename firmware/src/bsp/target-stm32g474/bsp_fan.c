/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include "bsp_fan.h"
#include "adc_service.h"
#include "utils.h"
#include "tim.h"
#include "bsp.h"
/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/

#define MAX_SPEED (100)
#define MIN_SPEED (0)

#define COUNTER_PERIOD_MAX (999)

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/

typedef void (*timer_init_function)(void);

typedef struct pwm {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    timer_init_function init_function;
} pwm_t;

typedef struct motor {
    pwm_t pwm_pin;
} fan_motor_t;
/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/
static fan_motor_t fan_motor = {
    .pwm_pin = { .htim = &htim3, .channel = TIM_CHANNEL_2, .init_function = MX_TIM3_Init },
};

static uint8_t current_speed = 0;
/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/
static void pwm_set(pwm_t *pwm, uint32_t value)
{
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, value);
}

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/

void bsp_fan_init()
{
    fan_motor.pwm_pin.init_function();

    HAL_TIM_PWM_Start(fan_motor.pwm_pin.htim, fan_motor.pwm_pin.channel);
    pwm_set(&fan_motor.pwm_pin, 0);

    current_speed = 0;
}

void bsp_fan_set(uint8_t speed)
{
    speed = min(speed, MAX_SPEED);

    uint32_t counter_value = map(speed, MIN_SPEED, MAX_SPEED, 0, COUNTER_PERIOD_MAX);

    pwm_set(&fan_motor.pwm_pin, counter_value);

    current_speed = speed;
}

void bsp_fan_init_routine(uint8_t speed)
{
    
}

void bsp_fan_set_with_bat(uint16_t bat_mv, uint8_t speed)
{
    
}

void bsp_fan_stop_routine()
{
    
}