#include "bsp_led_ir.h"
#include "bsp_gpio.h"
#include "bsp_gpio_mapping.h"
#include <stdbool.h>

void bsp_leds_ir_init(void)
{
}

void bsp_led_ir_on(bsp_led_ir_t led)
{
    switch (led) {
    case BSP_LED_IR_L_S:
        BSP_GPIO_Write_Pin(GPIO_LED_IR_L_S_PORT, GPIO_LED_IR_L_S_PIN, IO_HIGH);
        break;
    case BSP_LED_IR_L_F:
        BSP_GPIO_Write_Pin(GPIO_LED_IR_L_F_PORT, GPIO_LED_IR_L_F_PIN, IO_HIGH);
        break;
    case BSP_LED_IR_R_F:
        BSP_GPIO_Write_Pin(GPIO_LED_IR_R_F_PORT, GPIO_LED_IR_R_F_PIN, IO_HIGH);
        break;
    case BSP_LED_IR_R_S:
        BSP_GPIO_Write_Pin(GPIO_LED_IR_R_S_PORT, GPIO_LED_IR_R_S_PIN, IO_HIGH);
        break;
    }
}

void bsp_led_ir_off(bsp_led_ir_t led)
{
    switch (led) {
    case BSP_LED_IR_L_S:
        BSP_GPIO_Write_Pin(GPIO_LED_IR_L_S_PORT, GPIO_LED_IR_L_S_PIN, IO_LOW);
        break;
    case BSP_LED_IR_L_F:
        BSP_GPIO_Write_Pin(GPIO_LED_IR_L_F_PORT, GPIO_LED_IR_L_F_PIN, IO_LOW);
        break;
    case BSP_LED_IR_R_F:
        BSP_GPIO_Write_Pin(GPIO_LED_IR_R_F_PORT, GPIO_LED_IR_R_F_PIN, IO_LOW);
        break;
    case BSP_LED_IR_R_S:
        BSP_GPIO_Write_Pin(GPIO_LED_IR_R_S_PORT, GPIO_LED_IR_R_S_PIN, IO_LOW);
        break;
    }
}

void bsp_led_ir_all_off(void)
{
    bsp_led_ir_off(BSP_LED_IR_L_S);
    bsp_led_ir_off(BSP_LED_IR_L_F);
    bsp_led_ir_off(BSP_LED_IR_R_F);
    bsp_led_ir_off(BSP_LED_IR_R_S);
}

void bsp_led_ir_all_on(void)
{
    bsp_led_ir_on(BSP_LED_IR_L_S);
    bsp_led_ir_on(BSP_LED_IR_L_F);
    bsp_led_ir_on(BSP_LED_IR_R_F);
    bsp_led_ir_on(BSP_LED_IR_R_S);
}
