/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include "bsp_spi.h"
#include "spi.h"
#include <stdbool.h>

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
static SPI_HandleTypeDef *spi_handle_ptr = &hspi1;
static bsp_spi_callback_t spi_callback_function = NULL;
static uint8_t spi_rx[SPI_MAX_RX_IT];

static bool spi_busy = false;
/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_busy = false;
    if (hspi == spi_handle_ptr) {
        if (spi_callback_function != NULL) {
            spi_callback_function(spi_rx);
        }
    }
}

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/
void bsp_bus_spi_init()
{
    MX_SPI1_Init();
}

void bsp_bus_spi_deinit()
{
}

bsp_spi_result_t bsp_bus_spi_transmit(uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (HAL_SPI_Transmit(spi_handle_ptr, data, size, timeout) != HAL_OK) {
        return SPI_RES_ERROR;
    }
    return SPI_RES_OK;
}

bsp_spi_result_t bsp_bus_spi_receive(uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (HAL_SPI_Receive(spi_handle_ptr, data, size, timeout) != HAL_OK) {
        return SPI_RES_ERROR;
    }
    return SPI_RES_OK;
}

bsp_spi_result_t bsp_bus_spi_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    if (HAL_SPI_TransmitReceive(spi_handle_ptr, tx_data, rx_data, size, timeout) != HAL_OK) {
        return SPI_RES_ERROR;
    }
    return SPI_RES_OK;
}

bsp_spi_result_t bsp_bus_spi_transmit_receive_it(uint8_t *tx_data, uint16_t size, bsp_spi_callback_t rcv_function)
{
    if (size > SPI_MAX_RX_IT) {
        return SPI_RES_ERROR;
    }

    if (spi_busy) {
        return SPI_RES_ERROR;
    }

    if (HAL_SPI_TransmitReceive_IT(spi_handle_ptr, tx_data, spi_rx, size) != HAL_OK) {
        return SPI_RES_ERROR;
    }

    spi_busy = true;
    spi_callback_function = rcv_function;

    return SPI_RES_OK;
}

bsp_spi_result_t bsp_bus_spi_transmit_receive_dma(uint8_t *tx_data, uint16_t size, bsp_spi_callback_t rcv_function)
{
    if (size > SPI_MAX_RX_IT) {
        return SPI_RES_ERROR;
    }

    if (spi_busy) {
        return SPI_RES_ERROR;
    }

    // memcpy(spi_rx, tx_data, size);

    if (HAL_SPI_TransmitReceive_DMA(spi_handle_ptr, tx_data, spi_rx, size) != HAL_OK) {
        return SPI_RES_ERROR;
    }

    spi_busy = true;
    spi_callback_function = rcv_function;

    return SPI_RES_OK;
}

void bsp_spi_dma_stop()
{
    HAL_SPI_DMAStop(spi_handle_ptr);
}