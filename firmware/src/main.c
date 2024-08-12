#include "bsp.h"
#include "bsp_led.h"

#include "driving_service.h"
#include "led_service.h"
#include "adc_service.h"
#include "button.h"
#include "buzzer_service.h"
#include "ble_service.h"
#include "bsp_vcp.h"
#include "imu_service.h"
#include "position_service.h"
#include "bsp_eeprom.h"
#include "bsp_motors.h"
#include "bsp_fan.h"
#include "AS5047P.h"
#include "bsp_spi.h"
#include "bsp_gpio_mapping.h"

static as5047_result_t spi_receive_transmit_adapter(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    bsp_spi_result_t ret;
    ret = bsp_bus_spi_transmit_receive(tx_data, rx_data, size, 100);
    if (ret != SPI_RES_OK) {
        return AS5047_RES_ERROR;
    }

    return AS5047_RES_OK;
}


as5047_obj_t as5047_instance_1 = {
    .cs_pin = {
        .port = SPI_CS_1_PORT,
        .pin = SPI_CS_1_PIN,
    },
    .spi_transmit_receive = spi_receive_transmit_adapter,
    .delay_us = BSP_delay_us,
};

uint16_t my_data;

int main() {

    BSP_init(); 
   
    bsp_bus_spi_init();
    as5047_init(&as5047_instance_1);

    // driving_init();
    // bsp_fan_init();
    // led_stripe_init();
    // adc_service_init();
    // buttons_init();
    // buzzer_service_init();
    // bsp_vcp_init();
    // BSP_eeprom_init();
    // position_service_init();
    // imu_service_init();


    as5047_read_register(&as5047_instance_1, AS5047P_nVOL_SETTINGS1_ADDR, &my_data);
    BSP_delay(500);
    DEBUG_PRINT("Data: %d\n", my_data);

    as5047_write_register(&as5047_instance_1, AS5047P_nVOL_SETTINGS1_ADDR, 5);

    BSP_delay(500);

    as5047_read_register(&as5047_instance_1, AS5047P_nVOL_SETTINGS1_ADDR, &my_data);

    DEBUG_PRINT("Data: %d\n", my_data);

    BSP_delay(500);

    as5047_read_register(&as5047_instance_1, AS5047P_nVOL_SETTINGS1_ADDR, &my_data);



     DEBUG_PRINT("Data: %d\n", my_data);

    while(1) {
        BSP_ledToggle();
        BSP_delay_us(500000);

    }

}