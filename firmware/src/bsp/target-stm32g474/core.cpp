#include <cstdio>

#include "st/hal.h"

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buttons.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/core.hpp"
#include "bsp/debug.hpp"
#include "bsp/eeprom.hpp"
#include "bsp/encoders.hpp"
#include "bsp/fan.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "bsp/usb.hpp"
#include "pin_mapping.h"

namespace bsp {

namespace buttons {
void gpio_exti_callback(uint16_t GPIO_Pin);
}

namespace encoders {
void gpio_exti_callback(uint16_t GPIO_Pin);
}

/// @section Private variables

static uint8_t dfu_mode = 0;

/// @section Interface implementation

void init() {
    HAL_Init();

    SystemClock_Config();
    SystemCoreClockUpdate();

    // Shared peripheral initializations
    MX_CRC_Init();
    MX_GPIO_Init();
    MX_DMA_Init();

    analog_sensors::init();
    ble::init();
    buttons::init();
    buzzer::init();
    encoders::init();
    fan::init();
    leds::init();
    motors::init();
    timers::init();
    usb::init();

    if (imu::init() != imu::ImuResult::OK) {
        Error_Handler();
    }

    if (eeprom::init() != eeprom::EepromResult::OK) {
        Error_Handler();
    }

    delay_ms(20);
}

void debug::print(const char* s) {
    std::printf("%s\r\n", s);
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

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    bsp::buttons::gpio_exti_callback(GPIO_Pin);
    bsp::encoders::gpio_exti_callback(GPIO_Pin);
}
