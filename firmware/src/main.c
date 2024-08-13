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
#include "bsp_gpio_mapping.h"



int main() {

    BSP_init(); 
   

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



    while(1) {
        BSP_ledToggle();
        BSP_delay_us(500000);
    }

}