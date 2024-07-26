#ifndef BSP_EEPROM_H
#define BSP_EEPROM_H

#include <stdint.h>

#define FIRST_ADDR_PARAM 57600
#define LAST_ADDR_PARAM  58623
#define EE_MAX_ADDR      65535

typedef enum eeprom_result {
    EEPROM_OK,
    EEPROM_ERROR,
    EEPROM_DATA_NOT_FOUND,
} eeprom_result_t;

typedef enum eeprom_param_addresses {
    EE_ADDR_MEMORY_CLEAR = FIRST_ADDR_PARAM,
    EE_ADDR_BASE_SPEED_1,
    EE_ADDR_BASE_SPEED_2,
    EE_ADDR_KP_1,
    EE_ADDR_KP_2,
    EE_ADDR_KD_1,
    EE_ADDR_KD_2,
    EE_ADDR_KI_1,
    EE_ADDR_KI_2,
} eeprom_param_addresses_t;

typedef enum eeprom_sensor_param_addr {
    EE_ADDR_SENSOR_1_COEFFICIENTS_1 = (LAST_ADDR_PARAM + 1),
    EE_ADDR_SENSOR_1_COEFFICIENTS_2,
    EE_ADDR_SENSOR_1_OFFSETS_1,
    EE_ADDR_SENSOR_1_OFFSETS_2,

    EE_MAX_SENSOR_PARAM_ADDR,

} eeprom_sensor_param_addr_t;

eeprom_result_t BSP_eeprom_init();
eeprom_result_t BSP_eeprom_read_u8(uint16_t address, uint8_t *data);
eeprom_result_t BSP_eeprom_write_u8(uint16_t address, uint8_t data);
eeprom_result_t BSP_eeprom_read_u16(uint16_t address, uint16_t *data);
eeprom_result_t BSP_eeprom_write_u16(uint16_t address, uint16_t data);
eeprom_result_t BSP_eeprom_read_u32(uint16_t address, uint32_t *data);
eeprom_result_t BSP_eeprom_write_u32(uint16_t address, uint32_t data);

eeprom_result_t BSP_eeprom_read(uint16_t addr, uint8_t *read_data, uint16_t size);
eeprom_result_t BSP_eeprom_write(uint16_t addr, uint8_t *write_data, uint16_t size);

eeprom_result_t BSP_eeprom_clean_all();
eeprom_result_t BSP_eeprom_print_all();

#endif /* BSP_EEPROM_H */
