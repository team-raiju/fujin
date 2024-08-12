/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include <stdint.h>
#include "AS5047P.h"
#include "bsp_spi.h"
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

/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/

static uint16_t calculate_parity_bit(uint16_t data_2_cal)
{
	uint16_t parity_bit_value = 0;
	while(data_2_cal != 0) {
		parity_bit_value ^= data_2_cal; 
		data_2_cal >>=1;
	}

	return (parity_bit_value & 0x1);
}

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/

void as5047_select(as5047_obj_t *as5047_instance)
{
    BSP_GPIO_Write_Pin(as5047_instance->cs_pin.port, as5047_instance->cs_pin.pin, IO_LOW);
}

void as5047_unselect(as5047_obj_t *as5047_instance)
{
    BSP_GPIO_Write_Pin(as5047_instance->cs_pin.port, as5047_instance->cs_pin.pin, IO_HIGH);
}


as5047_result_t as5047_init(as5047_obj_t *as5047_instance)
{
    as5047_unselect(as5047_instance);

    return AS5047_RES_OK;
}

as5047_result_t as5047_write_register(as5047_obj_t *as5047_instance, as5047p_registers_t reg_addr, uint16_t data)
{

    as5047_result_t ret;
    uint16_t write_tx_frame = reg_addr;
    uint16_t rx_frame = 0;

    if (calculate_parity_bit(write_tx_frame & 0x3FFF) == 1) {
        write_tx_frame |= 0x8000;
    }

    as5047_select(as5047_instance); 
    ret = as5047_instance->spi_transmit_receive((uint8_t *)&write_tx_frame, (uint8_t *)&rx_frame, 1);
    as5047_unselect(as5047_instance);  
    as5047_instance->delay_us(1); 

    if (ret != AS5047_RES_OK) {
        return AS5047_RES_ERROR;
    }

    write_tx_frame = data;

    if (calculate_parity_bit(write_tx_frame & 0x3FFF) == 1) {
        write_tx_frame |= 0x8000;
    }

    rx_frame = 0;
    as5047_select(as5047_instance); 
    ret = as5047_instance->spi_transmit_receive((uint8_t *)&write_tx_frame, (uint8_t *)&rx_frame, 1);
    as5047_unselect(as5047_instance);  

    if (ret != AS5047_RES_OK) {
        return AS5047_RES_ERROR;
    }

    return AS5047_RES_OK; 
}

as5047_result_t as5047_read_register(as5047_obj_t *as5047_instance, as5047p_registers_t reg_addr, uint16_t *data)
{
    uint16_t rx_frame = 0;
    as5047_result_t ret;
    uint16_t read_tx_frame = reg_addr;


    if (calculate_parity_bit(read_tx_frame) == 0) {
        read_tx_frame |= 0x8000;
    }

    read_tx_frame |= AS5047_COMMAND_READ;

    as5047_select(as5047_instance); 
    ret = as5047_instance->spi_transmit_receive((uint8_t *)&read_tx_frame, (uint8_t *)&rx_frame, 1);
    as5047_unselect(as5047_instance);   
    as5047_instance->delay_us(1); 

    if (ret != AS5047_RES_OK) {
        return AS5047_RES_ERROR;
    }

    rx_frame = 0;
    read_tx_frame = AS5047P_VOL_NOP_ADDR;
    as5047_select(as5047_instance); 
    ret = as5047_instance->spi_transmit_receive((uint8_t *)&read_tx_frame, (uint8_t *)&rx_frame, 1);
    as5047_unselect(as5047_instance);   

    if (ret != AS5047_RES_OK) {
        return AS5047_RES_ERROR;
    }
    
    // uint16_t pard = rx_frame & 0x8000;
    // uint16_t ef = rx_frame & 0x4000;
    *data = rx_frame & 0x3FFF;

    return AS5047_RES_OK;
}