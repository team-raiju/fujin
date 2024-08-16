/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include "eeprom_24lc512.h"
#include "utils.h"
#include <stdint.h>

/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/

#define HIGHEST_ADDR 0xFFFF
#define WRITE_SIZE   64
#define READ_SIZE    128

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/

/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/

ext_eeprom_result_t eeprom_24lc512_write(eeprom_24lc512_obj_t *eeprom_instance, uint16_t write_addr, uint8_t *data,
                                         uint16_t size)
{
    uint32_t retries = 0;
    int ret;
    const uint32_t I2C_RETRIES = 100;

    if (write_addr + size > HIGHEST_ADDR + 1) {
        return EXT_EEPROM_ERROR;
    }

    while (size > 0) {
        int bytes_to_write = min(WRITE_SIZE - (write_addr % WRITE_SIZE), (int)size);

        do {
            ret = eeprom_instance->write_i2c(eeprom_instance->addr, write_addr, data, bytes_to_write);
            retries++;
        } while (ret != 0 && retries < I2C_RETRIES);

        if (ret != 0) {
            return EXT_EEPROM_ERROR;
        }

        data += bytes_to_write;
        write_addr += bytes_to_write;
        size -= bytes_to_write;
    }

    return EXT_EEPROM_OK;
}

ext_eeprom_result_t eeprom_24lc512_read(eeprom_24lc512_obj_t *eeprom_instance, uint16_t addr, uint8_t *read_data,
                                        uint16_t size)
{
    uint32_t retries = 0;
    int ret;
    const uint32_t I2C_RETRIES = 10;

    if (addr + size > HIGHEST_ADDR + 1) {
        return EXT_EEPROM_ERROR;
    }

    int offset = 0;

    while (size > 0) {
        int bytes_to_read = min((int)READ_SIZE, (int)size);

        do {
            ret = eeprom_instance->read_i2c(eeprom_instance->addr, addr, (read_data + offset), bytes_to_read);
            retries++;
        } while (ret != 0 && retries < I2C_RETRIES);

        if (ret != 0) {
            return EXT_EEPROM_ERROR;
        }

        size -= bytes_to_read;
        offset += bytes_to_read;
        addr += bytes_to_read;
    }

    return EXT_EEPROM_OK;
}