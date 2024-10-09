#include "eeprom_24lc512.hpp"
#include "utils/math.hpp"
#include <cstdint>

#include "../st/hal.h"

constexpr uint16_t HIGHEST_ADDR = 0xFFFF;
constexpr uint16_t WRITE_SIZE = 64;
constexpr uint16_t READ_SIZE = 128;
constexpr uint16_t I2C_ADDRESS = 0xA0;
constexpr uint32_t I2C_TIMEOUT = 500;

namespace devices {

EEPROM_24LC512::Result EEPROM_24LC512::write(uint16_t write_addr, uint8_t* data, uint16_t size) {
    uint32_t retries = 0;
    int ret;
    const uint32_t I2C_WRITE_RETRIES = 100;

    if (write_addr + size > HIGHEST_ADDR + 1) {
        return EEPROM_24LC512::Result::ERROR;
    }

    while (size > 0) {
        int bytes_to_write = std::min(WRITE_SIZE - (write_addr % WRITE_SIZE), (int)size);

        do {
            ret = HAL_I2C_Mem_Write(&hi2c3, I2C_ADDRESS, write_addr, I2C_MEMADD_SIZE_16BIT, data, bytes_to_write,
                           I2C_TIMEOUT);
            retries++;
        } while (ret != HAL_OK && retries < I2C_WRITE_RETRIES);

        if (ret != HAL_OK) {
            return EEPROM_24LC512::Result::ERROR;
        }

        data += bytes_to_write;
        write_addr += bytes_to_write;
        size -= bytes_to_write;
    }

    return EEPROM_24LC512::Result::OK;
}

EEPROM_24LC512::Result EEPROM_24LC512::read(uint16_t addr, uint8_t* read_data, uint16_t size) {

    uint32_t retries = 0;
    int ret;
    const uint32_t I2C_READ_RETRIES = 10;

    if (addr + size > HIGHEST_ADDR + 1) {
        return EEPROM_24LC512::Result::ERROR;
    }

    int offset = 0;

    while (size > 0) {
        int bytes_to_read = std::min(READ_SIZE, size);

        do {
            ret = HAL_I2C_Mem_Read(&hi2c3, I2C_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT, (read_data + offset), bytes_to_read,
                           I2C_TIMEOUT);
            retries++;
        } while (ret != HAL_OK && retries < I2C_READ_RETRIES);

        if (ret != HAL_OK) {
            return EEPROM_24LC512::Result::ERROR;
        }

        size -= bytes_to_read;
        offset += bytes_to_read;
        addr += bytes_to_read;
    }

    return EEPROM_24LC512::Result::OK;
}

}
