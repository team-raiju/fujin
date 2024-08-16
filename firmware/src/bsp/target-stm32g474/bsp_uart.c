#include "bsp_uart.h"
#include "usart.h"
#include <stdlib.h>

static bsp_uart_callback_t uart_custom_callbacks[MAX_UART_NUM];
static bsp_uart_callback_t uart_custom_error_callbacks[MAX_UART_NUM];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance) {
        if (uart_custom_callbacks[UART_NUM_1] != NULL) {
            uart_custom_callbacks[UART_NUM_1](NULL);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance) {
        if (uart_custom_error_callbacks[UART_NUM_1] != NULL) {
            uart_custom_error_callbacks[UART_NUM_1](NULL);
        }
    }
}

void BSP_UART_Register_Callback(uart_num_t uart_num, bsp_uart_callback_t callback_function)
{
    if (uart_num >= MAX_UART_NUM) {
        return;
    }

    uart_custom_callbacks[uart_num] = callback_function;
}

void BSP_UART_Register_Error_Callback(uart_num_t uart_num, bsp_uart_callback_t callback_function)
{
    if (uart_num >= MAX_UART_NUM) {
        return;
    }

    uart_custom_error_callbacks[uart_num] = callback_function;
}