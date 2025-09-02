#include "utils/math.hpp"
#include <cstdio>
#include <cstring>

#include "bsp/analog_sensors.hpp"
#include "bsp/eeprom.hpp"
#include "bsp/encoders.hpp"
#include "bsp/imu.hpp"
#include "bsp/timers.hpp"
#include "services/control.hpp"
#include "services/logger.hpp"
#include "services/navigation.hpp"

/// @section Constants

/// @section Service implementation

namespace services {

const Logger::ParamInfo paramInfoArray[] = {
    {4095, -5, 10, 270.0f},     // velocity_ms
    {4095, -5, 10, 270.0f},     // target_velocity_ms
    {4095, -70, 70, 29.0f},     // angular_speed_rad_s
    {4095, -70, 70, 29.0f},     // target_rad_s
    {65535, -250, 250, 131.0f}, // position_mm_x
    {65535, -250, 250, 131.0f}, // position_mm_y
    {65535, -180, 180, 182.0f}, // angle
};

Logger* Logger::instance() {
    static Logger p;
    return &p;
}

void Logger::init() {
    reset();
}

void Logger::reset() {
    log_data_idx = 0;
    addr_offset = 0;
}

void Logger::save_size() {
    // bsp::eeprom::write_u32(bsp::eeprom::ADDR_LOGGER_SIZE, addr_offset);
}

void Logger::update() {

    if (addr_offset + sizeof(logdata) >= sizeof(ram_logger)) {
        return;
    }

    auto control = services::Control::instance();
    auto nav = services::Navigation::instance();

    float param_velocity_ms = bsp::encoders::get_filtered_velocity_m_s() + std::abs(paramInfoArray[0].min_param_value);
    param_velocity_ms *= paramInfoArray[0].scale;

    float param_target_velocity_ms = control->get_target_linear_speed() + std::abs(paramInfoArray[1].min_param_value);
    param_target_velocity_ms *= paramInfoArray[1].scale;

    float param_angular_speed_rad_s = bsp::imu::get_rad_per_s() + std::abs(paramInfoArray[2].min_param_value);
    param_angular_speed_rad_s *= paramInfoArray[2].scale;

    float param_target_rad_s = control->get_target_angular_speed() + std::abs(paramInfoArray[3].min_param_value);
    param_target_rad_s *= paramInfoArray[3].scale;

    float param_position_x = nav->get_robot_position().x + std::abs(paramInfoArray[4].min_param_value);
    param_position_x *= paramInfoArray[4].scale;

    float param_position_y = nav->get_robot_position().y + std::abs(paramInfoArray[5].min_param_value);
    param_position_y *= paramInfoArray[5].scale;

    float param_angle = (bsp::imu::get_angle() * (180.0 / M_PI)) + std::abs(paramInfoArray[6].min_param_value);
    param_angle *= paramInfoArray[6].scale;

    logdata[log_data_idx].fields.velocity_ms = std::min(param_velocity_ms, (float)paramInfoArray[0].max_store_value);
    logdata[log_data_idx].fields.target_velocity_ms =
        std::min(param_target_velocity_ms, (float)paramInfoArray[1].max_store_value);
    logdata[log_data_idx].fields.angular_speed_rad_s =
        std::min(param_angular_speed_rad_s, (float)paramInfoArray[2].max_store_value);
    logdata[log_data_idx].fields.target_rad_s = std::min(param_target_rad_s, (float)paramInfoArray[3].max_store_value);
    logdata[log_data_idx].fields.position_mm_x = std::min(param_position_x, (float)paramInfoArray[4].max_store_value);
    logdata[log_data_idx].fields.position_mm_y = std::min(param_position_y, (float)paramInfoArray[5].max_store_value);
    logdata[log_data_idx].fields.angle = std::min(param_angle, (float)paramInfoArray[6].max_store_value);

    log_data_idx++;
    if (log_data_idx >= (sizeof(logdata) / sizeof(logdata[0]))) {
        log_data_idx = 0;

        memcpy(ram_logger + addr_offset, reinterpret_cast<uint8_t*>(logdata), sizeof(logdata));

        addr_offset += sizeof(logdata);
    }
}

void Logger::print_log() {

    std::printf("t;Vel;TargetVel;AngVel;TargetAngrVel;PosX;PosY;Angle\r\n");
    bsp::delay_ms(5);

    uint32_t saved_size = addr_offset;
    // if (bsp::eeprom::read_u32(bsp::eeprom::ADDR_LOGGER_SIZE, &saved_size) != bsp::eeprom::OK) {
    //     std::printf("Error reading logger saved size\r\n");
    //     return;
    // }

    for (uint16_t i = 0; i < saved_size; i += sizeof(LogData)) {
        LogData read_logdata;

        if (i + sizeof(read_logdata.data) > sizeof(ram_logger)) {
            break;
        }

        memcpy(read_logdata.data, ram_logger + i, sizeof(read_logdata.data));

        uint16_t idx = i / sizeof(LogData);

        std::printf(
            "%d;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f\r\n", idx,
            (read_logdata.fields.velocity_ms / paramInfoArray[0].scale) - std::abs(paramInfoArray[0].min_param_value),
            (read_logdata.fields.target_velocity_ms / paramInfoArray[1].scale) -
                std::abs(paramInfoArray[1].min_param_value),
            (read_logdata.fields.angular_speed_rad_s / paramInfoArray[2].scale) -
                std::abs(paramInfoArray[2].min_param_value),

            (read_logdata.fields.target_rad_s / paramInfoArray[3].scale) - std::abs(paramInfoArray[3].min_param_value),
            (read_logdata.fields.position_mm_x / paramInfoArray[4].scale) - std::abs(paramInfoArray[4].min_param_value),
            (read_logdata.fields.position_mm_y / paramInfoArray[5].scale) - std::abs(paramInfoArray[5].min_param_value),
            (read_logdata.fields.angle / paramInfoArray[6].scale) - std::abs(paramInfoArray[6].min_param_value)

        );

        bsp::delay_ms(3);
    }
}

}
