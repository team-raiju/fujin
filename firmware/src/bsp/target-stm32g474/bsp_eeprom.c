/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include <string.h>
#include "bsp_eeprom.h"
#include "eeprom_24lc512.h"
#include "bsp_i2c.h"
#include "bsp_vcp.h"
#include "bsp.h"
/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/
static int eeprom_read_i2c(uint16_t device_address, uint16_t mem_address, uint8_t *data, uint16_t size);
static int eeprom_write_i2c(uint16_t device_address, uint16_t mem_address, uint8_t *data, uint16_t size);
/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/
static eeprom_24lc512_obj_t eeprom_2 = { .addr = 0xA0, .read_i2c = eeprom_read_i2c, .write_i2c = eeprom_write_i2c };

/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/
static int eeprom_read_i2c(uint16_t device_address, uint16_t mem_address, uint8_t *data, uint16_t size)
{
    return bsp_i2c_read(IO_I2C_3, device_address, mem_address, IO_I2C_MEM_SIZE_16BIT, data, size);
}

static int eeprom_write_i2c(uint16_t device_address, uint16_t mem_address, uint8_t *data, uint16_t size)
{
    return bsp_i2c_write(IO_I2C_3, device_address, mem_address, IO_I2C_MEM_SIZE_16BIT, data, size);
}
/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/

eeprom_result_t BSP_eeprom_init()
{
    return EEPROM_OK;
}

eeprom_result_t BSP_eeprom_read_u8(uint16_t address, uint8_t *data)
{
    if (address >= EE_MAX_ADDR) {
        *data = 0;
        return EEPROM_ERROR;
    }

    ext_eeprom_result_t ret = eeprom_24lc512_read(&eeprom_2, address, data, 1);

    if (ret == EXT_EEPROM_OK) {
        return EEPROM_OK;
    }

    *data = 0;
    return EEPROM_ERROR;
}

eeprom_result_t BSP_eeprom_write_u8(uint16_t address, uint8_t data)
{
    if (address >= EE_MAX_ADDR) {
        return EEPROM_ERROR;
    }

    ext_eeprom_result_t ret = eeprom_24lc512_write(&eeprom_2, address, &data, 1);

    if (ret == EXT_EEPROM_OK) {
        return EEPROM_OK;
    }

    return EEPROM_ERROR;
}

eeprom_result_t BSP_eeprom_read_u16(uint16_t address, uint16_t *data)
{
    if (address >= EE_MAX_ADDR) {
        *data = 0;
        return EEPROM_ERROR;
    }

    uint8_t read_data[2];
    ext_eeprom_result_t ret = eeprom_24lc512_read(&eeprom_2, address, read_data, 2);

    if (ret == EXT_EEPROM_OK) {
        *data = ((uint16_t)read_data[0] << 8 | read_data[1]);
        return EEPROM_OK;
    }

    *data = 0;
    return EEPROM_ERROR;
}

eeprom_result_t BSP_eeprom_write_u16(uint16_t address, uint16_t data)
{
    if (address >= EE_MAX_ADDR) {
        return EEPROM_ERROR;
    }

    uint8_t write_data[2];
    write_data[0] = (data >> 8) & 0xff;
    write_data[1] = data & 0xff;

    ext_eeprom_result_t ret = eeprom_24lc512_write(&eeprom_2, address, write_data, 2);

    if (ret == EXT_EEPROM_OK) {
        return EEPROM_OK;
    }

    return EEPROM_ERROR;
}

eeprom_result_t BSP_eeprom_read_u32(uint16_t address, uint32_t *data)
{
    if (address >= EE_MAX_ADDR) {
        *data = 0;
        return EEPROM_ERROR;
    }

    uint8_t read_data[4];
    ext_eeprom_result_t ret = eeprom_24lc512_read(&eeprom_2, address, read_data, 4);

    if (ret == EXT_EEPROM_OK) {
        *data = ((uint32_t)read_data[0] << 24 | (uint32_t)read_data[1] << 16 | (uint32_t)read_data[2] << 8 | read_data[3]);
        return EEPROM_OK;
    }

    *data = 0;
    return EEPROM_ERROR;
}

eeprom_result_t BSP_eeprom_write_u32(uint16_t address, uint32_t data)
{
    if (address >= EE_MAX_ADDR) {
        return EEPROM_ERROR;
    }

    uint8_t write_data[4];
    write_data[0] = (data >> 24) & 0xff;
    write_data[1] = (data >> 16) & 0xff;
    write_data[2] = (data >> 8) & 0xff;
    write_data[3] = data & 0xff;

    ext_eeprom_result_t ret = eeprom_24lc512_write(&eeprom_2, address, write_data, 4);

    if (ret == EXT_EEPROM_OK) {
        return EEPROM_OK;
    }

    return EEPROM_ERROR;
}

eeprom_result_t BSP_eeprom_read(uint16_t addr, uint8_t *read_data, uint16_t size)
{
    if (addr + size > EE_MAX_ADDR) {
        return EEPROM_ERROR;
    }

    ext_eeprom_result_t ret = eeprom_24lc512_read(&eeprom_2, addr, read_data, size);

    if (ret == EXT_EEPROM_OK) {
        return EEPROM_OK;
    }

    return EEPROM_ERROR;
}

eeprom_result_t BSP_eeprom_write(uint16_t addr, uint8_t *write_data, uint16_t size)
{
    if (addr + size > EE_MAX_ADDR) {
        return EEPROM_ERROR;
    }

    ext_eeprom_result_t ret = eeprom_24lc512_write(&eeprom_2, addr, write_data, size);

    if (ret == EXT_EEPROM_OK) {
        return EEPROM_OK;
    }

    return EEPROM_ERROR;
}

eeprom_result_t BSP_eeprom_clean_all()
{
    uint32_t addr_to_clean = 0;

    while (addr_to_clean < 0xffff) {
        uint8_t data_write[64];
        memset(data_write, 0xff, sizeof(data_write));

        int8_t ret = eeprom_24lc512_write(&eeprom_2, addr_to_clean, data_write, sizeof(data_write));

        if (ret != EXT_EEPROM_OK) {
            DEBUG_PRINT("Error cleaning eeprom\r\n");
            return EEPROM_ERROR;
        }

        DEBUG_PRINT("Addr cleaned: %ld\r\n", addr_to_clean);
        addr_to_clean += 64;

        BSP_delay(5);
    }

    return EEPROM_OK;
}

eeprom_result_t BSP_eeprom_print_all()
{
    for (int i = 0; i < 0xffff; i += 64) {
        uint8_t data_read[64];
        ext_eeprom_result_t ret;

        ret = eeprom_24lc512_read(&eeprom_2, i, data_read, sizeof(data_read));

        if (ret == EXT_EEPROM_OK) {
            DEBUG_PRINT("%05d: ", i);
            for (uint16_t j = 0; j < sizeof(data_read); j++) {
                DEBUG_PRINT("0x%02x, ", data_read[j]);
            }
            DEBUG_PRINT("\r\n");
        } else {
            DEBUG_PRINT("%05d: ERROR READING\r\n", i);
        }
        BSP_delay(5);
    }

    return EEPROM_OK;
}