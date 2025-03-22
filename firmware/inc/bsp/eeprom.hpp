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
    ADDR_MEMORY_CLEAR = 0x0000,

    // PARAMS 0x0004 ~ 0x0FFF
    ADDR_FAN_SPEED = 0x0004,
    ADDR_ANGULAR_KP = 0x0008,
    ADDR_ANGULAR_KI = 0x000C,
    ADDR_ANGULAR_KD = 0x0010,
    ADDR_WALL_KP = 0x0014,
    ADDR_WALL_KI = 0x0018,
    ADDR_WALL_KD = 0x001C,
    ADDR_LINEAR_VEL_KP = 0x0020,
    ADDR_LINEAR_VEL_KI = 0x0024,
    ADDR_LINEAR_VEL_KD = 0x0028,
    ADDR_DIAGONAL_WALLS_KP = 0x002C,
    ADDR_DIAGONAL_WALLS_KI = 0x0030,
    ADDR_DIAGONAL_WALLS_KD = 0x0034,
    ADDR_MIN_MOVE_SPEED = 0x0038,

    ADDR_IR_WALL_DIST_REF_RIGHT = 0x003C,
    ADDR_IR_WALL_DIST_REF_FRONT_LEFT = 0x0040,
    ADDR_IR_WALL_DIST_REF_FRONT_RIGHT = 0x0044,
    ADDR_IR_WALL_DIST_REF_LEFT = 0x0048,
    ADDR_IR_WALL_CONTROL_TH_RIGHT = 0x004C,
    ADDR_IR_WALL_CONTROL_TH_FRONT_LEFT = 0x0050,
    ADDR_IR_WALL_CONTROL_TH_FRONT_RIGHT = 0x0054,
    ADDR_IR_WALL_CONTROL_TH_LEFT = 0x0058,
    ADDR_IR_WALL_DETECT_TH_RIGHT = 0x005C,
    ADDR_IR_WALL_DETECT_TH_FRONT_LEFT = 0x0060,
    ADDR_IR_WALL_DETECT_TH_FRONT_RIGHT = 0x0064,
    ADDR_IR_WALL_DETECT_TH_LEFT = 0x0068,
    
    // MAZE 0x1000 ~ 0x13FF
    ADDR_MAZE_START = 0x1000,

    // LOGGER 0x1400 ~ 0xFFFF
    ADDR_LOGGER_SIZE = 0x1400,
    ADDR_LOGGER_START = 0x1404,

    ADDR_MAX = 0xFFFF,
} param_addresses_t;

struct ParamInfo {
    uint16_t address;
    const char* name;
};

// Initialize the array with address-name pairs
const ParamInfo paramInfoArray[] = {
    {ADDR_MEMORY_CLEAR, "ADDR_MEMORY_CLEAR"},
    {ADDR_FAN_SPEED, "ADDR_FAN_SPEED"},
    {ADDR_ANGULAR_KP, "ADDR_ANGULAR_KP"},
    {ADDR_ANGULAR_KI, "ADDR_ANGULAR_KI"},
    {ADDR_ANGULAR_KD, "ADDR_ANGULAR_KD"},
    {ADDR_WALL_KP, "ADDR_WALL_KP"},
    {ADDR_WALL_KI, "ADDR_WALL_KI"},
    {ADDR_WALL_KD, "ADDR_WALL_KD"},
    {ADDR_LINEAR_VEL_KP, "ADDR_LINEAR_VEL_KP"},
    {ADDR_LINEAR_VEL_KI, "ADDR_LINEAR_VEL_KI"},
    {ADDR_LINEAR_VEL_KD, "ADDR_LINEAR_VEL_KD"},
    {ADDR_DIAGONAL_WALLS_KP, "ADDR_DIAGONAL_WALLS_KP"},
    {ADDR_DIAGONAL_WALLS_KI, "ADDR_DIAGONAL_WALLS_KI"},
    {ADDR_DIAGONAL_WALLS_KD, "ADDR_DIAGONAL_WALLS_KD"},
    {ADDR_MIN_MOVE_SPEED, "ADDR_MIN_MOVE_SPEED"},
};

/// @section Interface definition

EepromResult init(void);

EepromResult read_u8(uint16_t address, uint8_t* data);
EepromResult write_u8(uint16_t address, uint8_t data);
EepromResult read_u16(uint16_t address, uint16_t* data);
EepromResult write_u16(uint16_t address, uint16_t data);
EepromResult read_u32(uint16_t address, uint32_t* data);
EepromResult read_array(uint16_t address, uint8_t* data, uint16_t size);
EepromResult write_u32(uint16_t address, uint32_t data);
EepromResult write_array(uint16_t address, uint8_t* data, uint16_t size);

void clear(void);
void print_all(void);
const char* param_name(uint16_t address);

}
