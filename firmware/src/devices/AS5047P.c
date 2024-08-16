/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include "AS5047P.h"
#include "bsp_spi.h"
#include <stdint.h>
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

static uint16_t calculate_parity_bit(uint32_t val)
{
    val ^= val >> 1;
    val ^= val >> 2;
    val = (val & 0x11111111U) * 0x11111111U;
    return (val >> 28) & 1;
}

static bool is_received_parity_ok(uint16_t frameRx)
{
    uint32_t parityReceived;
    uint32_t parityCalculated;

    //--- Calculate parity for Rx frame
    parityReceived = (frameRx & AS5047P_FRAME_PARD) >> 15;
    parityCalculated = calculate_parity_bit(frameRx & (AS5047P_FRAME_DATA | AS5047P_FRAME_EF));

    //--- Parity check
    if (parityCalculated == parityReceived) {
        return true;
    } else {
        return false;
    }
}

static as5047_result_t as5047_read_write_raw(as5047_obj_t *as5047_instance, uint16_t dataTx, bool rw, uint16_t *out_data_rx)
{
    as5047_result_t ret;
    int16_t frameTx;

    //--- Set RW bit and calculate parity bit
    frameTx = dataTx & AS5047P_FRAME_DATA;
    frameTx |= rw << 14;
    frameTx |= calculate_parity_bit(frameTx) << 15;

    uint16_t buffTx = frameTx;
    uint16_t buffRx = 0;

    as5047_select(as5047_instance);
    ret = as5047_instance->spi_transmit_receive(&buffTx, &buffRx, 1);
    as5047_unselect(as5047_instance);

    as5047_instance->delay_us(1);

    //--- Low level SPI access returned error
    if (ret != AS5047_RES_OK) {
        return AS5047_RES_ERROR;
    }

    //--- Parity bit error occurred in Rx frame
    if (!is_received_parity_ok(buffRx)) {
        *out_data_rx = (buffRx & AS5047P_FRAME_DATA);
        return AS5047_RES_ERROR;
    }

    *out_data_rx = (buffRx & AS5047P_FRAME_DATA);

    return AS5047_RES_OK;
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

as5047_result_t as5047_read_register(as5047_obj_t *as5047_instance, uint16_t reg_addr, uint16_t *data)
{
    as5047_result_t ret;
    uint16_t data_received;

    as5047_read_write_raw(as5047_instance, reg_addr, AS5047P_ACCESS_READ, &data_received);

    ret = as5047_read_write_raw(as5047_instance, AS5047P_VOL_DIAAGC_ADDR, AS5047P_ACCESS_READ, &data_received);

    if (ret != AS5047_RES_OK) {
        return AS5047_RES_ERROR;
    }

    *data = data_received;

    return AS5047_RES_OK;
}

as5047_result_t as5047_write_register(as5047_obj_t *as5047_instance, uint16_t reg_addr, uint16_t data)
{
    as5047_result_t ret;
    uint16_t data_received;

    as5047_read_write_raw(as5047_instance, reg_addr, AS5047P_ACCESS_WRITE, &data_received);

    ret = as5047_read_write_raw(as5047_instance, data, AS5047P_ACCESS_WRITE, &data_received);

    if (ret != AS5047_RES_OK) {
        return AS5047_RES_ERROR;
    }

    return AS5047_RES_OK;
}