#include "st/hal.h"

#include "bsp/ble.hpp"
#include "utils/math.h"

namespace bsp::ble {

/// @section Constants

static constexpr uint32_t ble_timeout = 100;

/// @section Private variables

static bool is_running = false;
static uint8_t raw_data[receive_packet_size];
static BleCallback external_callback;

/// @section Interface implementation

void init() {
    MX_USART1_UART_Init();
}

void start(void) {
    if (is_running) {
        return;
    }

    HAL_UART_Receive_DMA(&huart1, raw_data, receive_packet_size);
    is_running = true;
}

void stop(void) {
    HAL_UART_DMAStop(&huart1);
    is_running = false;
}

void transmit(uint8_t* data, uint8_t size) {
    uint8_t size_to_send = std::min(size, max_packet_size);
    HAL_UART_Transmit(&huart1, data, size_to_send, ble_timeout);
}

void register_callback(BleCallback callback) {
    external_callback = callback;
}

} // namespace

/// @section HAL callbacks

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == huart1.Instance) {
        if (bsp::ble::external_callback == NULL) {
            return;
        }

        if (bsp::ble::raw_data[0] > bsp::ble::BLE_BUTTON_2_LONG) {
            return;
        }

        bsp::ble::BleHeader header = static_cast<bsp::ble::BleHeader>(bsp::ble::raw_data[0]);
        bsp::ble::external_callback(header);
    }
}
