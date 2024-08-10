#ifndef AS5047P_H
#define AS5047P_H

#include "bsp_gpio.h"


typedef enum {
    AS5047_RES_OK,
    AS5047_RES_ERROR,
} as5047_result_t;

typedef as5047_result_t (*as5047_spi_transmit_receive)(uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

typedef struct {
    io_port_t port;
    io_pin_t pin;
} cs_pin_t;

typedef struct {
    cs_pin_t cs_pin;
    as5047_spi_transmit_receive spi_transmit_receive;
} as5047_obj_t;


#define AS5047_COMMAND_READ         0x4000

typedef enum {
    AS5047P_VOL_NOP_ADDR = 0x0000,
    AS5047P_VOL_ERRFL_ADDR = 0x0001,
    AS5047P_VOL_PROG_ADDR = 0x0003,
    AS5047P_VOL_DIAAGC_ADDR = 0x3FFC,
    AS5047P_VOL_MAG_ADDR = 0x3FFD,
    AS5047P_VOL_ANGLEUNC_ADDR = 0x3FFE,
    AS5047P_VOL_ANGLECOM_ADDR = 0x3FFF,
    
    AS5047P_nVOL_ZPOSM_ADDR = 0x0016,
    AS5047P_nVOL_ZPOSL_ADDR = 0x0017,
    AS5047P_nVOL_SETTINGS1_ADDR = 0x0018,
    AS5047P_nVOL_SETTINGS2_ADDR = 0x0019
} as5047p_registers_t;


#endif /* AS5047P_H */
