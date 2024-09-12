#pragma once

#include <cstdint>

namespace bsp::eeprom {

/// @section Custom types

enum EepromResult {
    OK,
    ERROR,
    DATA_NOT_FOUND,
};

typedef enum : uint16_t {
    EEPROM_ADDR_MEMORY_CLEAR = 0x0000,
    EEPROM_ADDR_BASE_SPEED = 0x0004,
    EEPROM_ADDR_KP = 0x0008,
    EEPROM_ADDR_KD = 0x000C,
    EEPROM_ADDR_KI = 0x0010,

    EEPROM_ADDR_MAX = 0xFFFF,
} eeprom_param_addresses_t;

/// @section Interface definition

EepromResult init(void);

EepromResult read_u8(eeprom_param_addresses_t address, uint8_t* data);
EepromResult write_u8(eeprom_param_addresses_t address, uint8_t data);
EepromResult read_u16(eeprom_param_addresses_t address, uint16_t* data);
EepromResult write_u16(eeprom_param_addresses_t address, uint16_t data);
EepromResult read_u32(eeprom_param_addresses_t address, uint32_t* data);
EepromResult write_u32(eeprom_param_addresses_t address, uint32_t data);

EepromResult clear();
EepromResult print_all();

}
