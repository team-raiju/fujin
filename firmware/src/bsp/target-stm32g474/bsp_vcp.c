/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include "bsp_vcp.h"

#include "bsp_gpio.h"

#include "bsp.h"
#include "dfu.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/
static void USBD_Clock_Config(void);
static int8_t vcp_receive_interrupt(uint8_t *data, uint32_t *len);
/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/

static int8_t vcp_receive_interrupt(uint8_t *data, uint32_t *len)
{
    return 0;
}

static void USBD_Clock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_CRSInitTypeDef RCC_CRSInitStruct = { 0 };

    /* Enable HSI48 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /*Configure the clock recovery system (CRS)**********************************/

    /*Enable CRS Clock*/
    __HAL_RCC_CRS_CLK_ENABLE();

    /* Default Synchro Signal division factor (not divided) */
    RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;

    /* Set the SYNCSRC[1:0] bits according to CRS_Source value */
    RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;

    /* HSI48 is synchronized with USB SOF at 1KHz rate */
    RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;

    /* Set the TRIM[5:0] to the default value */
    RCC_CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

    /* Start automatic synchronization */
    HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/
int __io_putchar(int ch)
{
    uint8_t c[1];
    c[0] = ch & 0x00FF;
    uint8_t ret = CDC_Transmit_FS(c, 1);
    if (ret != 0) {
        return -1;
    }
    return ch;
}

int _write(int file, char *ptr, int len)
{
    UNUSED(file);
    uint8_t ret = CDC_Transmit_FS((uint8_t *)(ptr), len);

    int timeout = 0;
    while (ret != 0 && timeout < 100) {
        timeout++;
        ret = CDC_Transmit_FS((uint8_t *)(ptr), len);
    }

    if (timeout >= 100) {
        return -1;
    }

    return len;
}

void bsp_vcp_init()
{
    USBD_Interface_fops_FS.Receive = vcp_receive_interrupt;
    BSP_GPIO_USBD_Reset_On_Host();
    USBD_Clock_Config();
    MX_USB_Device_Init();
}
