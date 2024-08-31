#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/// @section Custom types

typedef enum {
    EEPROM_OK,
    EEPROM_ERROR,
    EEPROM_DATA_NOT_FOUND,
} eeprom_result_t;

typedef enum : uint16_t {
    EEPROM_ADDR_MEMORY_CLEAR = 0x0000,
    EEPROM_ADDR_BASE_SPEED = 0x0004,
    EEPROM_ADDR_KP = 0x0008,
    EEPROM_ADDR_KD = 0x000C,
    EEPROM_ADDR_KI = 0x0010,

    EEPROM_ADDR_MAX = 0xFFFF,
} eeprom_param_addresses_t;

/// @section Interface definition

eeprom_result_t bsp_eeprom_init(void);

eeprom_result_t bsp_eeprom_read_u8(eeprom_param_addresses_t address, uint8_t* data);
eeprom_result_t bsp_eeprom_write_u8(eeprom_param_addresses_t address, uint8_t data);
eeprom_result_t bsp_eeprom_read_u16(eeprom_param_addresses_t address, uint16_t* data);
eeprom_result_t bsp_eeprom_write_u16(eeprom_param_addresses_t address, uint16_t data);
eeprom_result_t bsp_eeprom_read_u32(eeprom_param_addresses_t address, uint32_t* data);
eeprom_result_t bsp_eeprom_write_u32(eeprom_param_addresses_t address, uint32_t data);

eeprom_result_t bsp_eeprom_clear();
eeprom_result_t bsp_eeprom_print_all();

#ifdef __cplusplus
}
#endif
