#include <cstring>
#include <cstdio>
#include "bsp/eeprom.hpp"
#include "services/config.hpp"
#include "utils/movement_params.hpp"

namespace bsp::eeprom {

static uint8_t memory[0x10000];

static void write_float_param(uint16_t addr, float val) {
    uint32_t raw;
    std::memcpy(&raw, &val, sizeof(float));
    std::memcpy(&memory[addr], &raw, sizeof(uint32_t));
}

EepromResult init(void) {
    std::memset(memory, 0xFF, sizeof(memory));

    // Calibrated configuration parameters from params.txt (Calibration Test Map)
    write_float_param(ADDR_IR_WALL_DIST_REF_RIGHT, 1391.0f);
    write_float_param(ADDR_IR_WALL_DIST_REF_FRONT_LEFT, 200.0f);
    write_float_param(ADDR_IR_WALL_DIST_REF_FRONT_RIGHT, 200.0f);
    write_float_param(ADDR_IR_WALL_DIST_REF_LEFT, 800.0f);

    write_float_param(ADDR_IR_WALL_CONTROL_TH_RIGHT, 1100.0f);
    write_float_param(ADDR_IR_WALL_CONTROL_TH_FRONT_LEFT, 500.0f);
    write_float_param(ADDR_IR_WALL_CONTROL_TH_FRONT_RIGHT, 500.0f);
    write_float_param(ADDR_IR_WALL_CONTROL_TH_LEFT, 650.0f);

    write_float_param(ADDR_IR_WALL_DETECT_TH_RIGHT, 1200.0f);
    write_float_param(ADDR_IR_WALL_DETECT_TH_FRONT_LEFT, 500.0f);
    write_float_param(ADDR_IR_WALL_DETECT_TH_FRONT_RIGHT, 800.0f);
    write_float_param(ADDR_IR_WALL_DETECT_TH_LEFT, 700.0f);

    write_float_param(ADDR_Z_IMU_BIAS, -0.4905f);
    write_float_param(ADDR_MIN_MOVE_SPEED, 0.2f);

    // Search PID params from params.txt
    write_float_param(ADDR_ANGULAR_KP, 0.085f);
    write_float_param(ADDR_ANGULAR_KI, 0.011f);
    write_float_param(ADDR_ANGULAR_KD, 0.000000f);
    write_float_param(ADDR_WALL_KP, 0.0020000f);
    write_float_param(ADDR_WALL_KI, 0.000000f);
    write_float_param(ADDR_WALL_KD, 0.0040000f);
    write_float_param(ADDR_LINEAR_VEL_KP, 8.000000f);
    write_float_param(ADDR_LINEAR_VEL_KI, 0.10000f);
    write_float_param(ADDR_LINEAR_VEL_KD, 0.000000f);
    write_float_param(ADDR_DIAGONAL_WALLS_KP, 0.000000f);
    write_float_param(ADDR_DIAGONAL_WALLS_KI, 0.000000f);
    write_float_param(ADDR_DIAGONAL_WALLS_KD, 0.000000f);
    write_float_param(ADDR_FAN_SPEED, 0.0f);

    // Wall break calibration
    write_float_param(ADDR_START_WALL_BREAK_MM_LEFT, 65.0f);
    write_float_param(ADDR_START_WALL_BREAK_MM_RIGHT, 80.0f);
    write_float_param(ADDR_ENABLE_WALL_BREAK_CORRECTION, 1.0f);

    // Calibrated Turn Params for Custom Movements (matching physical robot EEPROM)
    auto write_turn = [](uint16_t addr, const TurnParams& tp) {
        std::memcpy(&memory[addr], &tp, sizeof(TurnParams));
    };

    // T(ms) = ms * 2 in 2000 Hz ticks
    write_turn(ADDR_TURN_PARAMS_RIGHT_45, {-50.0f, -86.0f, 0.5f, 100.00f, 7.854f, 0, 0, -1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_LEFT_45, {-50.0f, -86.0f, 0.5f, 100.00f, 7.854f, 0, 0, 1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_RIGHT_90, {0.0f, -41.0f, 0.5f, 150.0f, 10.47f, 300, 500, -1, 140, 440, 0, 0});
    write_turn(ADDR_TURN_PARAMS_LEFT_90, {0.0f, -41.0f, 0.5f, 150.0f, 10.47f, 300, 500, 1, 140, 440, 0, 0});
    write_turn(ADDR_TURN_PARAMS_RIGHT_135, {-15.0f, -76.0f, 0.5f, 100.00f, 7.5049f, 0, 0, -1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_LEFT_135, {0.0f, -86.0f, 0.5f, 100.00f, 7.5049f, 0, 0, 1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_RIGHT_180, {0.0f, 0.0f, 0.5f, 122.17f, 5.65f, 0, 0, -1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_LEFT_180, {0.0f, 0.0f, 0.5f, 122.17f, 5.45f, 0, 0, 1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_RIGHT_45_FROM_45, {0.0f, 37.0f, 0.5f, 100.0f, 7.8539f, 0, 0, -1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_LEFT_45_FROM_45, {0.0f, 37.0f, 0.5f, 100.0f, 7.8539f, 0, 0, 1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_RIGHT_90_FROM_45, {0.0f, 35.0f, 0.5f, 100.0f, 7.8539f, 0, 0, -1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_LEFT_90_FROM_45, {0.0f, 35.0f, 0.5f, 100.0f, 7.8539f, 0, 0, 1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_RIGHT_135_FROM_45, {0.0f, 38.0f, 1.5f, 436.33f, 20.07f, 232, 334, -1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_LEFT_135_FROM_45, {0.0f, 39.0f, 1.5f, 436.33f, 20.07f, 232, 334, 1, 0, 0, 0, 0});

    // Calibrated Search Turns (stored in physical robot EEPROM)
    write_turn(ADDR_TURN_PARAMS_RIGHT_90_SEARCH_MODE, {0.0f, 0.0f, 0.3f, 43.633f, 4.014f, 782, 966, -1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_LEFT_90_SEARCH_MODE, {0.0f, 0.0f, 0.3f, 43.633f, 4.014f, 782, 966, 1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_TURN_AROUND, {0.0f, 0.0f, 0.3f, 52.36f, 3.49f, 1802, 1936, -1, 0, 0, 0, 0});
    write_turn(ADDR_TURN_PARAMS_TURN_AROUND_INPLACE, {0.0f, 0.0f, 0.3f, 52.36f, 3.49f, 1802, 1936, -1, 0, 0, 0, 0});

    // Calibrated Forward Params for Custom Movements
    auto write_fwd = [](uint16_t addr, const ForwardParams& fp) {
        std::memcpy(&memory[addr], &fp, sizeof(ForwardParams));
    };
    write_fwd(ADDR_FORWARD_PARAMS_START, {0.3f, 0.85f, 0.85f, 109.0f});
    write_fwd(ADDR_FORWARD_PARAMS_FOWARD, {0.3f, 0.85f, 0.85f, 180.0f});
    write_fwd(ADDR_FORWARD_PARAMS_DIAGONAL, {0.3f, 0.85f, 0.85f, 127.279f});
    write_fwd(ADDR_FORWARD_PARAMS_STOP, {0.3f, 0.85f, 0.85f, 90.0f});
    write_fwd(ADDR_FORWARD_PARAMS_TURN_AROUND, {0.3f, 0.5f, 0.5f, 80.0f});
    write_fwd(ADDR_FORWARD_PARAMS_TURN_AROUND_INPLACE, {0.3f, 0.5f, 0.5f, 80.0f});
    write_fwd(ADDR_FORWARD_PARAMS_RIGHT_90_SEARCH, {0.3f, 0.85f, 0.85f, 24.0f});
    write_fwd(ADDR_FORWARD_PARAMS_LEFT_90_SEARCH, {0.3f, 0.85f, 0.85f, 24.0f});

    return OK;
}

EepromResult read_u8(uint16_t address, uint8_t* data) {
    if (!data) return ERROR;
    *data = memory[address];
    return OK;
}

EepromResult write_u8(uint16_t address, uint8_t data) {
    memory[address] = data;
    return OK;
}

EepromResult read_u16(uint16_t address, uint16_t* data) {
    if (!data || address > 0xFFFE) return ERROR;
    std::memcpy(data, &memory[address], sizeof(uint16_t));
    return OK;
}

EepromResult write_u16(uint16_t address, uint16_t data) {
    if (address > 0xFFFE) return ERROR;
    std::memcpy(&memory[address], &data, sizeof(uint16_t));
    return OK;
}

EepromResult read_u32(uint16_t address, uint32_t* data) {
    if (!data || address > 0xFFFC) return ERROR;
    std::memcpy(data, &memory[address], sizeof(uint32_t));
    return OK;
}

EepromResult write_u32(uint16_t address, uint32_t data) {
    if (address > 0xFFFC) return ERROR;
    std::memcpy(&memory[address], &data, sizeof(uint32_t));
    return OK;
}

EepromResult read_array(uint16_t address, uint8_t* data, uint16_t size) {
    if (!data || (static_cast<uint32_t>(address) + size > 0x10000)) return ERROR;
    std::memcpy(data, &memory[address], size);
    return OK;
}

EepromResult write_array(uint16_t address, uint8_t* data, uint16_t size) {
    if (!data || (static_cast<uint32_t>(address) + size > 0x10000)) return ERROR;
    std::memcpy(&memory[address], data, size);
    return OK;
}

void clear(void) {
    std::memset(memory, 0xFF, sizeof(memory));
}

void print_all(void) {}

const char* param_name(uint16_t address) {
    for (const auto& info : paramInfoArray) {
        if (info.address == address) {
            return info.name;
        }
    }
    return "UNKNOWN";
}

} // namespace bsp::eeprom
