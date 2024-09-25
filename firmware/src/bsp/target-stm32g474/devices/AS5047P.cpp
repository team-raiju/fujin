#include <cstdint>
#include <functional>

#include "AS5047P.hpp"

#include "../st/hal.h"
#include "bsp/timers.hpp"

#define AS5047_COMMAND_READ 0x4000

#define AS5047P_FRAME_PARD (1 << 15)
#define AS5047P_FRAME_EF (1 << 14)
#define AS5047P_FRAME_DATA (0x3FFF)

#define AS5047P_ACCESS_WRITE 0
#define AS5047P_ACCESS_READ 1

static uint16_t calculate_parity_bit(uint32_t val) {
    val ^= val >> 1;
    val ^= val >> 2;
    val = (val & 0x11111111U) * 0x11111111U;
    return (val >> 28) & 1;
}

static bool is_received_parity_ok(uint16_t frameRx) {
    uint32_t parityReceived;
    uint32_t parityCalculated;

    parityReceived = (frameRx & AS5047P_FRAME_PARD) >> 15;
    parityCalculated = calculate_parity_bit(frameRx & (AS5047P_FRAME_DATA | AS5047P_FRAME_EF));

    return parityCalculated == parityReceived;
}

namespace devices {

AS5047::AS5047(GPIO_TypeDef* cs_port, uint16_t cs_pin,
               std::function<Result(uint16_t* tx_data, uint16_t* rx_data, uint16_t size)> transmit_receive) {
    this->cs_port = cs_port;
    this->cs_pin = cs_pin;
    this->transmit_receive = transmit_receive;
}

void AS5047::select() {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
}

void AS5047::deselect() {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

AS5047::Result AS5047::init() {
    MX_SPI1_Init();
    deselect();
    return OK;
}

AS5047::Result AS5047::read_write_raw(bool rw, uint16_t data_tx, uint16_t* data_rx) {
    int16_t frame_tx;

    //--- Set RW bit and calculate parity bit
    frame_tx = data_tx & AS5047P_FRAME_DATA;
    frame_tx |= rw << 14;
    frame_tx |= calculate_parity_bit(frame_tx) << 15;

    uint16_t buff_tx = frame_tx;
    uint16_t buff_rx = 0;

    select();
    auto ret = transmit_receive(&buff_tx, &buff_rx, 1);
    deselect();

    bsp::delay_us(1);

    //--- Low level SPI access returned error
    if (ret != OK) {
        return ERROR;
    }

    *data_rx = (buff_rx & AS5047P_FRAME_DATA);

    //--- Parity bit error occurred in Rx frame
    if (!is_received_parity_ok(buff_rx)) {
        return ERROR;
    }

    return OK;
}

AS5047::Result AS5047::read_register(Register reg_addr, uint16_t* data) {
    read_write_raw(AS5047P_ACCESS_READ, reg_addr, data);
    return read_write_raw(AS5047P_ACCESS_READ, VOL_DIAAGC_ADDR, data);
}

AS5047::Result AS5047::write_register(Register reg_addr, uint16_t data) {
    uint16_t data_received;

    read_write_raw(AS5047P_ACCESS_WRITE, reg_addr, &data_received);

    return read_write_raw(AS5047P_ACCESS_WRITE, data, &data_received);
}

} // namespace
