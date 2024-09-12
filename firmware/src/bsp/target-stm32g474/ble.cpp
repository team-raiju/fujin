#include "st/hal.h"

#include "bsp/ble.hpp"
#include "utils/math.h"

namespace bsp::ble {

/// @section Constants

#define UART_BLE_TIMEOUT 100

/// @section Private variables

static bool is_running = false;
static uint8_t raw_data[BLE_RECEIVE_PACKET_SIZE];
static bsp_uart_ble_callback_t external_callback;

/// @section Interface implementation

void init() {
    MX_USART1_UART_Init();
}

void start(void) {
    if (is_running) {
        return;
    }

    HAL_UART_Receive_DMA(&huart1, raw_data, BLE_RECEIVE_PACKET_SIZE);
    is_running = true;
}

void stop(void) {
    HAL_UART_DMAStop(&huart1);
    is_running = false;
}

void transmit(uint8_t* data, uint8_t size) {
    uint8_t size_to_send = min(size, BLE_MAX_PACKET_SIZE);
    HAL_UART_Transmit(&huart1, data, size_to_send, UART_BLE_TIMEOUT);
}

void register_callback(bsp_uart_ble_callback_t callback) {
    external_callback = callback;
}

} // namespace

/// @section HAL callbacks

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == huart1.Instance) {
        if (bsp::ble::external_callback != NULL) {
            bsp::ble::external_callback(bsp::ble::raw_data, BLE_RECEIVE_PACKET_SIZE);
        }
    }
}
