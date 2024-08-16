#ifndef AS5047P_H
#define AS5047P_H

#include "bsp_gpio.h"
#include <stdint.h>

#define AS5047P_FRAME_PARD (1 << 15)
#define AS5047P_FRAME_EF   (1 << 14)
#define AS5047P_FRAME_DATA (0x3FFF)

#define AS5047P_ACCESS_WRITE 0
#define AS5047P_ACCESS_READ  1

typedef enum {
    AS5047_RES_OK,
    AS5047_RES_ERROR,
} as5047_result_t;

// as5047_spi_transmit_receive must be a function that transmits and receives
// data over SPI. SPI data should be 16bit frame, MSB first and CPOL=0, CPHA=1
// (2nd edge)
typedef as5047_result_t (*as5047_spi_transmit_receive)(uint16_t *tx_data, uint16_t *rx_data, uint16_t size);
typedef void (*as5047_delay_us)(uint32_t delay_us);

typedef struct {
    io_port_t port;
    io_pin_t pin;
} cs_pin_t;

typedef struct {
    cs_pin_t cs_pin;
    as5047_spi_transmit_receive spi_transmit_receive;
    as5047_delay_us delay_us;
} as5047_obj_t;

#define AS5047_COMMAND_READ 0x4000

#define AS5047P_VOL_NOP_ADDR        0x0000
#define AS5047P_VOL_ERRFL_ADDR      0x0001
#define AS5047P_VOL_PROG_ADDR       0x0003
#define AS5047P_VOL_DIAAGC_ADDR     0x3FFC
#define AS5047P_VOL_MAG_ADDR        0x3FFD
#define AS5047P_VOL_ANGLEUNC_ADDR   0x3FFE
#define AS5047P_VOL_ANGLECOM_ADDR   0x3FFF
#define AS5047P_nVOL_ZPOSM_ADDR     0x0016
#define AS5047P_nVOL_ZPOSL_ADDR     0x0017
#define AS5047P_nVOL_SETTINGS1_ADDR 0x0018
#define AS5047P_nVOL_SETTINGS2_ADDR 0x0019

void as5047_select(as5047_obj_t *as5047_instance);
void as5047_unselect(as5047_obj_t *as5047_instance);
as5047_result_t as5047_init(as5047_obj_t *as5047_instance);
as5047_result_t as5047_write_register(as5047_obj_t *as5047_instance, uint16_t reg_addr, uint16_t data);
as5047_result_t as5047_read_register(as5047_obj_t *as5047_instance, uint16_t reg_addr, uint16_t *data);

#endif /* AS5047P_H */
