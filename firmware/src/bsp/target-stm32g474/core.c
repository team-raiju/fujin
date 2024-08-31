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

/// @section Private variables

static uint8_t dfu_mode = 0;

/// @section Interface implementation

void bsp_init() {
    HAL_Init();

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

    bsp_delay_ms(20);
}

void prepare_dfu() {
    if (dfu_mode == 1) {
        // DEBUG_PRINT("\r\n");
        return;
    }

    dfu_mode = 1;

    FLASH_OBProgramInitTypeDef flash_option_bytes;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_StatusTypeDef ret;
    ret = HAL_FLASH_Unlock();
    if (ret != HAL_OK) {
        // DEBUG_PRINT(" - HAL_FLASH_Unlock ret = %d\r\n", ret);
    }

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

    /* Unlock the Options Bytes *************************************************/
    ret = HAL_FLASH_OB_Unlock();
    if (ret != HAL_OK) {
        // DEBUG_PRINT(" - HAL_FLASH_OB_Unlock ret = %d\r\n", ret);
        return;
    }

    HAL_FLASHEx_OBGetConfig(&flash_option_bytes);

    /* Check/Apply nBoot0 = 0
     * **********************************************************/

    /* Boot from system memory in next boot when nBoot0 = 0; nSWBOOT0 = 0 and
     * nBoot1 = 1*/
    flash_option_bytes.OptionType = OPTIONBYTE_USER;
    flash_option_bytes.USERType = OB_USER_nBOOT0;
    flash_option_bytes.USERConfig = OB_nBOOT0_RESET;

    ret = HAL_FLASHEx_OBProgram(&flash_option_bytes);
    if (ret != HAL_OK) {
        // DEBUG_PRINT(" - HAL_FLASHEx_OBProgram ret = %d\r\n", ret);
        return;
    }

    /* Lock the Options Bytes ***************************************************/
    ret = HAL_FLASH_OB_Lock();
    if (ret != HAL_OK) {
        // DEBUG_PRINT(" - HAL_FLASH_OB_Lock ret = %d\r\n", ret);
        return;
    }

    /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
    ret = HAL_FLASH_Lock();
    if (ret != HAL_OK) {
        // DEBUG_PRINT(" - HAL_FLASH_Lock ret = %d\r\n", ret);
        return;
    }

    // DEBUG_PRINT("\r\n");
}
