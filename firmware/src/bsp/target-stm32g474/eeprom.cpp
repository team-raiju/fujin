#include "st/hal.h"
#include <cstdio>

#include "bsp/eeprom.hpp"
#include "bsp/timers.hpp"
#include "devices/eeprom_24lc512.hpp"

namespace bsp::eeprom {

bool address_is_valid(uint16_t address) {
    return address <= ADDR_MAX;
}

/// @section Interface implementation

EepromResult init() {
    MX_I2C3_Init();
    return OK;
}

EepromResult read_u8(uint16_t address, uint8_t* data) {
    if (!address_is_valid(address)) {
        return ERROR;
    }

    devices::EEPROM_24LC512 eeprom;
    auto result = eeprom.read(address, data, sizeof(uint8_t));
    if (result == devices::EEPROM_24LC512::Result::OK) {
        return OK;
    }

    *data = 0;
    return ERROR;
}

EepromResult write_u8(uint16_t address, uint8_t data) {
    if (!address_is_valid(address)) {
        return ERROR;
    }

    devices::EEPROM_24LC512 eeprom;
    auto result = eeprom.write(address, &data, sizeof(uint8_t));
    if (result == devices::EEPROM_24LC512::Result::OK) {
        return OK;
    }

    return ERROR;
}

EepromResult read_u16(uint16_t address, uint16_t* data) {
    if (!address_is_valid(address)) {
        return ERROR;
    }

    uint8_t read_data[2];

    devices::EEPROM_24LC512 eeprom;
    auto result = eeprom.read(address, read_data, sizeof(read_data));

    if (result == devices::EEPROM_24LC512::Result::OK) {
        *data = ((uint16_t)read_data[0] << 8) | read_data[1];
        return OK;
    }

    *data = 0;
    return ERROR;
}

EepromResult write_u16(uint16_t address, uint16_t data) {
    if (!address_is_valid(address)) {
        return ERROR;
    }

    uint8_t write_data[2] = {(uint8_t)((data >> 8) & 0xff), (uint8_t)(data & 0xff)};

    devices::EEPROM_24LC512 eeprom;
    auto result = eeprom.write(address, write_data, sizeof(write_data));
    if (result == devices::EEPROM_24LC512::Result::OK) {
        return OK;
    }

    return ERROR;
}

EepromResult read_u32(uint16_t address, uint32_t* data) {
    if (!address_is_valid(address)) {
        return ERROR;
    }

    uint8_t read_data[4];

    devices::EEPROM_24LC512 eeprom;
    auto result = eeprom.read(address, read_data, sizeof(read_data));

    if (result == devices::EEPROM_24LC512::Result::OK) {
        *data = ((uint32_t)read_data[0] << 24) | ((uint32_t)read_data[1] << 16) | ((uint32_t)read_data[2] << 8) |
                read_data[3];
        return OK;
    }

    *data = 0;
    return ERROR;
}

EepromResult write_u32(uint16_t address, uint32_t data) {
    if (!address_is_valid(address)) {
        return ERROR;
    }

    uint8_t write_data[4] = {(uint8_t)((data >> 24) & 0xff), (uint8_t)((data >> 16) & 0xff),
                             (uint8_t)((data >> 8) & 0xff), (uint8_t)(data & 0xff)};

    devices::EEPROM_24LC512 eeprom;
    auto result = eeprom.write(address, write_data, sizeof(write_data));
    if (result == devices::EEPROM_24LC512::Result::OK) {
        return OK;
    }
    return ERROR;
}

void clear(void) {
    uint32_t addr_to_clean = 0;
    while (addr_to_clean < 0xffff) {
        uint8_t data_write[64];
        memset(data_write, 0xff, sizeof(data_write));

        devices::EEPROM_24LC512 eeprom;
        auto ret = eeprom.write(addr_to_clean, data_write, sizeof(data_write));

        if (ret != devices::EEPROM_24LC512::Result::OK) {
            std::printf("Error clearing address 0x%04lx\r\n", addr_to_clean);
            return;
        }

        std::printf("Cleared 0x%04lx\r\n", addr_to_clean);

        addr_to_clean += 64;
        bsp::delay_ms(5);

    }



}

void print_all(void) {
    for (int i = 0; i < 0xffff; i += 64) {
        uint8_t data_read[64];
        devices::EEPROM_24LC512 eeprom;
        auto result = eeprom.read(i, data_read, sizeof(data_read));
        if (result == devices::EEPROM_24LC512::Result::OK) {
            std::printf("0x%04x: ", i);
            for (int j = 0; j < 64; j++) {
                std::printf("%02x ", data_read[j]);
                if ((j + 1) % 16 == 0) {
                    std::printf("\r\n        ");
                }
            }
            bsp::delay_ms(5);
            std::printf("\r\n");
        }
        bsp::delay_ms(5);
    }
}

const char* param_name(uint16_t address) {
    for (const auto& param : paramInfoArray) {
        if (param.address == address) {
            return param.name;
        }
    }

    return "UNKNOWN";
}

}
