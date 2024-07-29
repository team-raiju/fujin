/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include "dfu.h"
#include "main.h"
#include "bsp_gpio.h"
#include "bsp_gpio_mapping.h"
#include "bsp_vcp.h"
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
static uint8_t dfu_mode = 0;
/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/

void prepare_dfu()
{
    if (BSP_GPIO_Read_Pin(ENCODER_RIGHT_B_PORT, ENCODER_RIGHT_B_PIN)) {
        DEBUG_PRINT("BUZZER OK");
    } else {
        DEBUG_PRINT("BUZZER ALERT");
    }

    if (dfu_mode == 1) {
        DEBUG_PRINT("\r\n");
        return;
    }

    dfu_mode = 1;

    FLASH_OBProgramInitTypeDef flash_option_bytes;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_StatusTypeDef ret;
    ret = HAL_FLASH_Unlock();
    if (ret != HAL_OK) {
        DEBUG_PRINT(" - HAL_FLASH_Unlock ret = %d\r\n", ret);
    }

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

    /* Unlock the Options Bytes *************************************************/
    ret = HAL_FLASH_OB_Unlock();
    if (ret != HAL_OK) {
        DEBUG_PRINT(" - HAL_FLASH_OB_Unlock ret = %d\r\n", ret);
        return;
    }

    HAL_FLASHEx_OBGetConfig(&flash_option_bytes);

    /* Check/Apply nBoot0 = 0  **********************************************************/

    /* Boot from system memory in next boot when nBoot0 = 0; nSWBOOT0 = 0 and nBoot1 = 1*/
    flash_option_bytes.OptionType = OPTIONBYTE_USER;
    flash_option_bytes.USERType = OB_USER_nBOOT0;
    flash_option_bytes.USERConfig = OB_nBOOT0_RESET;

    ret = HAL_FLASHEx_OBProgram(&flash_option_bytes);
    if (ret != HAL_OK) {
        DEBUG_PRINT(" - HAL_FLASHEx_OBProgram ret = %d\r\n", ret);
        return;
    }

    /* Lock the Options Bytes ***************************************************/
    ret = HAL_FLASH_OB_Lock();
    if (ret != HAL_OK) {
        DEBUG_PRINT(" - HAL_FLASH_OB_Lock ret = %d\r\n", ret);
        return;
    }

    /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
    ret = HAL_FLASH_Lock();
    if (ret != HAL_OK) {
        DEBUG_PRINT(" - HAL_FLASH_Lock ret = %d\r\n", ret);
        return;
    }

    DEBUG_PRINT("\r\n");
}