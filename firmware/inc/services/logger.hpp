#pragma once

#include <cstdint>

namespace services {

class Logger {
public:

    union LogData {

       uint8_t data[9];

        struct {
            uint16_t velocity_ms : 12;
            uint16_t target_velocity_ms : 12;

            uint16_t angular_speed_rad_s : 12;
            uint16_t target_rad_s : 12;

            uint8_t pwm_left : 8;
            uint8_t pwm_right : 8;

            uint8_t battery : 8;

        } __attribute__((packed)) fields;
    };

    struct ParamInfo {
        uint16_t max_store_value; //Max value to store
        float min_param_value; //Min measured value
        float max_param_value; //Max measured value
        float scale; // For max usage efficiency = max_store_value / max_param_value
    };

    static Logger* instance();

    void init();
    void reset();
    void update();
    void save_size();
    void print_log();

    Logger(const Logger&) = delete;

private:
    Logger() {};

    LogData logdata[7];
    uint8_t log_data_idx;
    uint32_t addr_offset;
    uint8_t ram_logger[30000]; //Max log number is 30000 / sizeof(LogData)

};

}
