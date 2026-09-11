#include <cstring>
#include <cstdio>
#include "bsp/eeprom.hpp"

namespace bsp::eeprom {

static uint8_t memory[0x10000] = {0};

EepromResult init(void) {
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
    std::memset(memory, 0, sizeof(memory));
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
