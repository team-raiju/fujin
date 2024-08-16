/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include "bsp_gpio.h"
#include "bsp_gpio_mapping.h"
#include "gpio.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/

static bsp_button_callback_t button_1_callback_function = NULL;
static bsp_button_callback_t button_2_callback_function = NULL;
static bsp_gpio_encoder_callback_t encoder_l_callback_function = NULL;
static bsp_gpio_encoder_callback_t encoder_r_callback_function = NULL;
static ir_callback_t ir_callback_function = NULL;

/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/
static uint16_t BSP_GPIO_Pin_Mapping(io_pin_t io_pin)
{
    uint16_t hardware_pin_num[] = { GPIO_PIN_0,  GPIO_PIN_1,  GPIO_PIN_2,  GPIO_PIN_3, GPIO_PIN_4,  GPIO_PIN_5,
                                    GPIO_PIN_6,  GPIO_PIN_7,  GPIO_PIN_8,  GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11,
                                    GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 };

    if (io_pin < sizeof(hardware_pin_num) / sizeof(uint16_t)) {
        return hardware_pin_num[io_pin];
    }

    return GPIO_PIN_0;
}

static GPIO_TypeDef *BSP_GPIO_Port_Mapping(io_port_t io_port)
{
    GPIO_TypeDef *hardware_port_num[] = {
        GPIOA, // IO_PORTA
        GPIOB, // IO_PORTB
        GPIOC, // IO_PORTC
        GPIOD, // IO_PORTD
    };

    if (io_port < sizeof(hardware_port_num) / sizeof(GPIO_TypeDef *)) {
        return hardware_port_num[io_port];
    }

    return GPIOA;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Button 1
    if (GPIO_Pin == BSP_GPIO_Pin_Mapping(GPIO_BUTTON_1_PIN)) {
        if (button_1_callback_function != NULL) {
            button_1_callback_function();
        }
        return;
    }

    // Button 2
    if (GPIO_Pin == BSP_GPIO_Pin_Mapping(GPIO_BUTTON_2_PIN)) {
        if (button_2_callback_function != NULL) {
            button_2_callback_function();
        }
        return;
    }

    // Encoders
    if (GPIO_Pin == BSP_GPIO_Pin_Mapping(ENCODER_LEFT_A_PIN)) {
        if (encoder_l_callback_function != NULL) {
            encoder_l_callback_function(0);
        }
        return;
    }

    if (GPIO_Pin == BSP_GPIO_Pin_Mapping(ENCODER_LEFT_B_PIN)) {
        if (encoder_l_callback_function != NULL) {
            if (BSP_GPIO_Read_Pin(ENCODER_LEFT_B_PORT, ENCODER_LEFT_B_PIN) == IO_HIGH) {
                encoder_l_callback_function(1);
            } else {
                encoder_l_callback_function(2);
            }
        }
        return;
    }

    if (GPIO_Pin == BSP_GPIO_Pin_Mapping(ENCODER_RIGHT_A_PIN)) {
        if (encoder_r_callback_function != NULL) {
            encoder_r_callback_function(0);
        }
        return;
    }

    if (GPIO_Pin == BSP_GPIO_Pin_Mapping(ENCODER_RIGHT_B_PIN)) {
        if (encoder_r_callback_function != NULL) {
            if (BSP_GPIO_Read_Pin(ENCODER_RIGHT_B_PORT, ENCODER_RIGHT_B_PIN) == IO_HIGH) {
                encoder_r_callback_function(1);
            } else {
                encoder_r_callback_function(2);
            }
        }
        return;
    }
}

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/
io_level_t BSP_GPIO_Read_Pin(io_port_t port, io_pin_t gpio_pin)
{
    return (io_level_t)HAL_GPIO_ReadPin(BSP_GPIO_Port_Mapping(port), BSP_GPIO_Pin_Mapping(gpio_pin));
}

void BSP_GPIO_Write_Pin(io_port_t port, io_pin_t gpio_pin, io_level_t level)
{
    HAL_GPIO_WritePin(BSP_GPIO_Port_Mapping(port), BSP_GPIO_Pin_Mapping(gpio_pin), (uint16_t)level);
}

void BSP_GPIO_Toggle_Pin(io_port_t port, io_pin_t gpio_pin)
{
    HAL_GPIO_TogglePin(BSP_GPIO_Port_Mapping(port), BSP_GPIO_Pin_Mapping(gpio_pin));
}

void BSP_GPIO_Register_Button_1_Callback(bsp_button_callback_t callback_function)
{
    button_1_callback_function = callback_function;
}

void BSP_GPIO_Register_Button_2_Callback(bsp_button_callback_t callback_function)
{
    button_2_callback_function = callback_function;
}

void BSP_GPIO_Register_Encoder_L_Callback(bsp_gpio_encoder_callback_t callback_function)
{
    encoder_l_callback_function = callback_function;
}
void BSP_GPIO_Register_Encoder_R_Callback(bsp_gpio_encoder_callback_t callback_function)
{
    encoder_r_callback_function = callback_function;
}

void BSP_GPIO_Register_IR_Callback(ir_callback_t callback_function)
{
    ir_callback_function = callback_function;
}

void BSP_GPIO_USBD_Reset_On_Host(void)
{
    // By pulling the USB DP pin down for few milliseconds the host computer
    // disconnects the USB device if it was connected before reset

    /* Rendering hardware reset harmless (no need to replug USB cable): */
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    /*Configure GPIO pin : PA12, a.k.a. USB_DP */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_Delay(5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
    /* Hardware reset rendered harmless! */
}