#include "st/hal.h"

#include "bsp/eeprom.h"

eeprom_result_t bsp_eeprom_init() {
    MX_I2C3_Init();
    return EEPROM_OK;
}
