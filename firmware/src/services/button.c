/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include "bsp.h"

#include "button.h"
#include "bsp_gpio.h"
#include "bsp_gpio_mapping.h"

// #include "robot_fsm.h"

static void button_1_interrupt()
{
    static uint32_t button_1_start_val;

    if (BSP_GPIO_Read_Pin(GPIO_BUTTON_1_PORT, GPIO_BUTTON_1_PIN) == IO_LOW) {
        button_1_start_val = BSP_GetTick();
    } else {
        uint16_t time_diff_ms = (BSP_GetTick() - button_1_start_val);
        if (time_diff_ms > BSP_TICKS_PER_MILISSEC * 750) {
            // fsm_ring_buf_put(BUTTON_1_LONG_SIG);
        } else if (time_diff_ms > BSP_TICKS_PER_MILISSEC * 20) {
            // fsm_ring_buf_put(BUTTON_1_SHORT_SIG);
        }
    }
}

static void button_2_interrupt()
{
    static uint32_t button_2_start_val;

    if (BSP_GPIO_Read_Pin(GPIO_BUTTON_2_PORT, GPIO_BUTTON_2_PIN) == IO_LOW) {
        button_2_start_val = BSP_GetTick();
    } else {
        uint16_t time_diff_ms = (BSP_GetTick() - button_2_start_val);
        if (time_diff_ms > BSP_TICKS_PER_MILISSEC * 750) {
            // fsm_ring_buf_put(BUTTON_2_LONG_SIG);
        } else if (time_diff_ms > BSP_TICKS_PER_MILISSEC * 20) {
            // fsm_ring_buf_put(BUTTON_2_SHORT_SIG);
        }
    }
}

static void button_1_init()
{
    BSP_GPIO_Register_Button_1_Callback(button_1_interrupt);
}

static void button_2_init()
{
    BSP_GPIO_Register_Button_2_Callback(button_2_interrupt);
}

void buttons_init()
{
    button_1_init();
    button_2_init();
}