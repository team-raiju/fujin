#include "st/hal.h"

#include "bsp/analog_sensors.h"
#include "bsp/ble.h"
#include "bsp/buttons.h"
#include "bsp/buzzer.h"
#include "bsp/core.h"
#include "bsp/eeprom.h"
#include "bsp/encoders.h"
#include "bsp/fan.h"
#include "bsp/imu.h"
#include "bsp/leds.h"
#include "bsp/motors.h"
#include "bsp/timers.h"
#include "bsp/usb.h"
#include "pin_mapping.h"

/// @section Interface implementation'

void bsp_init() {
    SystemClock_Config();
    SystemCoreClockUpdate();

    // Shared peripheral initializations
    MX_CRC_Init();
    MX_GPIO_Init();
    MX_DMA_Init();

    bsp_analog_sensors_init();
    bsp_ble_init();
    bsp_buttons_init();
    bsp_buzzer_init();
    bsp_encoders_init();
    bsp_fan_init();
    bsp_imu_init();
    bsp_leds_init();
    bsp_motors_init();
    bsp_timers_init();
    bsp_usb_init();

    if (bsp_eeprom_init() != EEPROM_OK) {
        Error_Handler();
    }
}
