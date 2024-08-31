#pragma once

#include <stdint.h>

typedef enum {
    EXT_EEPROM_OK,
    EXT_EEPROM_ERROR,
} ext_eeprom_result_t;

/* Expect that the return value is 0 when success and -1 if not success */
typedef int (*eeprom_24lc512_read_i2c)(uint16_t device_address, uint16_t mem_address, uint8_t* data, uint16_t size);
typedef int (*eeprom_24lc512_write_i2c)(uint16_t device_address, uint16_t mem_address, uint8_t* data, uint16_t size);

typedef struct {
    uint8_t addr;
    eeprom_24lc512_read_i2c read_i2c;
    eeprom_24lc512_write_i2c write_i2c;
} eeprom_24lc512_obj_t;

ext_eeprom_result_t eeprom_24lc512_write(eeprom_24lc512_obj_t* eeprom_instance, uint16_t write_addr, uint8_t* data,
                                         uint16_t len);
ext_eeprom_result_t eeprom_24lc512_read(eeprom_24lc512_obj_t* eeprom_instance, uint16_t addr, uint8_t* read_data,
                                        uint16_t len);
