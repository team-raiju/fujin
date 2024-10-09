#pragma once

#include <cstdint>

namespace devices {

class EEPROM_24LC512 {
public:
    enum class Result {
        OK,
        ERROR
    };

    Result write(uint16_t write_addr, uint8_t* data, uint16_t len);
    Result read(uint16_t addr, uint8_t* read_data, uint16_t len);
};

}
