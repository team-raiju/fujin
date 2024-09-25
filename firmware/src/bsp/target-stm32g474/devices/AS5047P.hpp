#pragma once

#include <cstdint>
#include <functional>

#include "../st/hal.h"

namespace devices {

/// @section Custom Types

class AS5047 {
public:
    enum Result {
        OK,
        ERROR,
    };

    enum Register : uint16_t {
        VOL_NOP_ADDR = 0x0000,
        VOL_ERRFL_ADDR = 0x0001,
        VOL_PROG_ADDR = 0x0003,
        VOL_DIAAGC_ADDR = 0x3FFC,
        VOL_MAG_ADDR = 0x3FFD,
        VOL_ANGLEUNC_ADDR = 0x3FFE,
        VOL_ANGLECOM_ADDR = 0x3FFF,
        nVOL_ZPOSM_ADDR = 0x0016,
        nVOL_ZPOSL_ADDR = 0x0017,
        nVOL_SETTINGS1_ADDR = 0x0018,
        nVOL_SETTINGS2_ADDR = 0x0019,
    };

    AS5047(GPIO_TypeDef* cs_port, uint16_t cs_pin,
           std::function<Result(uint16_t* tx_data, uint16_t* rx_data, uint16_t size)> transmit_receive);

    void select();
    void deselect();

    Result init();
    Result write_register(Register reg_addr, uint16_t data);
    Result read_register(Register reg_addr, uint16_t* data);

private:
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    // as5047_spi_transmit_receive must be a function that transmits and receives
    // data over SPI. SPI data should be 16bit frame, MSB first and CPOL=0, CPHA=1
    // (2nd edge)
    std::function<Result(uint16_t* tx_data, uint16_t* rx_data, uint16_t size)> transmit_receive;

    Result read_write_raw(bool rw, uint16_t data_tx, uint16_t* data_rx);
};

}
