#pragma once

#include <cstdint>

namespace services {

#define CONTROL_LOG_MODE 1

enum class ParamIndex : uint8_t {
    VelocityMS,
    TargetVelocityMS,
    AngularSpeedRadS,
    TargetRadS,
    PwmLeft,
    PwmRight,
    EncoderImuDiff,
#if CONTROL_LOG_MODE
    VelP,
    VelI,
    AngP,
    AngI,
    RotationFF,
    LinearFF,
#else
    Battery,
    PositionX,
    PositionY,
    Angle,
    Distance,
#endif
    COUNT
};

class Logger {
public:
    union LogData {

        #if CONTROL_LOG_MODE
        uint8_t data[20];
        #else
        uint8_t data[17];
        #endif

        struct {
            uint16_t velocity_ms : 13;
            uint16_t target_velocity_ms : 13;

            uint16_t angular_speed_rad_s : 13;
            uint16_t target_rad_s : 13;

            uint16_t pwm_left : 10;
            uint16_t pwm_right : 10;
            uint16_t encoder_imu_diff : 12;

#if CONTROL_LOG_MODE
            uint16_t vel_p : 14;
            uint16_t vel_i : 14;
            uint16_t ang_p : 14;
            uint16_t ang_i : 14;
            uint16_t rotation_ff : 10;
            uint16_t linear_ff : 10;
#else
            uint16_t battery : 8;

            uint16_t position_mm_x : 16;
            uint16_t position_mm_y : 16;
            uint16_t angle : 14;
            uint16_t distance : 14;
#endif

        } __attribute__((packed)) fields;
    };

    #if CONTROL_LOG_MODE
    static_assert(sizeof(LogData) == 20, "LogData size must be exactly 20 bytes!");
    #else
    static_assert(sizeof(LogData) == 17, "LogData size must be exactly 17 bytes!");
    #endif


    struct ParamInfo {
        uint16_t max_store_value; // Max integer value for the bitfield (e.g., 4095 for 12 bits)
        float min_param_value;    // Min measured value
        float max_param_value;    // Max measured value
        float scale;              // Scale factor, calculated as: max_store_value / (max_param_value - min_param_value)
    };

    static Logger* instance();

    void init();
    void reset();
    void update();
    void save_size();
    void print_log();
    void send_log_ble();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

private:
    Logger() {};

    float encode_value(float raw_value, const ParamInfo& info) const;
    float decode_value(float stored_value, const ParamInfo& info) const;

    LogData logdata[7];
    uint8_t log_data_idx;
    uint32_t addr_offset;
    uint8_t ram_logger[60000]; // Max log number is 60000 / sizeof(LogData)
};

}
