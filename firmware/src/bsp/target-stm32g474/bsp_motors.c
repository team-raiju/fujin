/**
 ***************************************************************************************************
 * @brief   brief
 *
 * @author  author
 **************************************************************************************************/
/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include <stdint.h>

#include "bsp_motors.h"
#include "bsp_gpio.h"
#include "tim.h"
#include "utils.h"

/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/
#define COUNTER_PERIOD_MAX (1000)
#define MAX_SPEED          (COUNTER_PERIOD_MAX - 1)

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/
typedef struct pwm {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} pwm_t;

typedef struct motor_dir_pin {
    io_port_t port;
    io_pin_t pin;
} motor_dir_pin_t;

typedef struct motor {
    pwm_t pwm_pin;
    motor_dir_pin_t dir_gpio;
} motor_t;

/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/
static motor_t left_motor = {
    .pwm_pin = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_4,
    },
    .dir_gpio = {
        .port = IO_PORTC,
        .pin = IO_PIN_2,
    },
};

static motor_t right_motor = {
    .pwm_pin = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_2,
    },
    .dir_gpio = {
        .port = IO_PORTC,
        .pin = IO_PIN_0,
    },
};
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

static void set_motor_pwm(motor_t *motor, int32_t speed)
{
    if (speed < 0) {
        BSP_GPIO_Write_Pin(motor->dir_gpio.port, motor->dir_gpio.pin, IO_HIGH);
        pwm_set(&motor->pwm_pin, -speed);
    } else {
        BSP_GPIO_Write_Pin(motor->dir_gpio.port, motor->dir_gpio.pin, IO_LOW);
        pwm_set(&motor->pwm_pin, speed);
    }
}

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/

void BSP_motorsInit(void)
{
    MX_TIM1_Init(); // Motors Timer

    /* Motor Right */
    HAL_TIM_PWM_Start(right_motor.pwm_pin.htim, right_motor.pwm_pin.channel);
    pwm_set(&right_motor.pwm_pin, 0);

    /* Motor Left */
    HAL_TIM_PWM_Start(left_motor.pwm_pin.htim, left_motor.pwm_pin.channel);
    pwm_set(&left_motor.pwm_pin, 0);
}

void BSP_motors(int16_t vel_left, int16_t vel_right)
{
    vel_right = constrain(vel_right, -MAX_SPEED, MAX_SPEED);
    vel_left = constrain(vel_left, -MAX_SPEED, MAX_SPEED);

    set_motor_pwm(&left_motor, vel_left);
    set_motor_pwm(&right_motor, vel_right);
}

motor_rot_dir_t BSP_motor_left_dir_pin()
{
    if (BSP_GPIO_Read_Pin(left_motor.dir_gpio.port, left_motor.dir_gpio.pin) == IO_LOW) {
        return MOT_ROT_CW;
    } else {
        return MOT_ROT_CCW;
    }
}

motor_rot_dir_t BSP_motor_right_dir_pin()
{
    if (BSP_GPIO_Read_Pin(right_motor.dir_gpio.port, right_motor.dir_gpio.pin) == IO_LOW) {
        return MOT_ROT_CW;
    } else {
        return MOT_ROT_CCW;
    }
}