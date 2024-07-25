#include <stdbool.h>
#include "bsp_led.h"
#include "bsp_gpio.h"
#include "bsp_gpio_mapping.h"

void BSP_ledInit(void)
{
}

void BSP_ledOff(void)
{
    BSP_GPIO_Write_Pin(GPIO_LED_PORT, GPIO_LED_PIN, IO_LOW);
}

void BSP_ledOn(void)
{
    BSP_GPIO_Write_Pin(GPIO_LED_PORT, GPIO_LED_PIN, IO_HIGH);
}

void BSP_ledToggle(void)
{
    BSP_GPIO_Toggle_Pin(GPIO_LED_PORT, GPIO_LED_PIN);
}
