#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdint.h>

#define SPI_MAX_RX_IT 32

typedef enum {
    SPI_RES_OK,
    SPI_RES_ERROR,
} bsp_spi_result_t;

typedef void (*bsp_spi_callback_t)(uint8_t *rx_data);

void bsp_bus_spi_init();

void bsp_bus_spi_deinit();

bsp_spi_result_t bsp_bus_spi_transmit(uint8_t *data, uint16_t size, uint32_t timeout);

bsp_spi_result_t bsp_bus_spi_receive(uint8_t *data, uint16_t size, uint32_t timeout);

bsp_spi_result_t bsp_bus_spi_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout);

bsp_spi_result_t bsp_bus_spi_transmit_receive_it(uint8_t *tx_data, uint16_t size, bsp_spi_callback_t rcv_function);

bsp_spi_result_t bsp_bus_spi_transmit_receive_dma(uint8_t *tx_data, uint16_t size, bsp_spi_callback_t rcv_function);

void bsp_spi_dma_stop();

#endif /* BSP_SPI_H */
