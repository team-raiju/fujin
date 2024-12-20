#include "utils/math.hpp"
#include <cstdio>

#include "bsp/analog_sensors.hpp"
#include "bsp/eeprom.hpp"
#include "bsp/encoders.hpp"
#include "bsp/imu.hpp"
#include "bsp/timers.hpp"
#include "services/control.hpp"
#include "services/logger.hpp"

/// @section Constants

/// @section Service implementation

namespace services {

const Logger::ParamInfo paramInfoArray[] = {
    {1024, 10, 100.0f}, // velocity_ms
    {1024, 10, 100.0f}, // target_velocity_ms
    {4096, 70, 58.0f},  // angular_speed_rad_s
    {4096, 70, 58.0f},  // target_rad_s
    {16, 0, 0.0f}       // reserved
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

void Logger::update() {

    if (bsp::eeprom::ADDR_START_LOGGER + addr_offset + sizeof(logdata) >= bsp::eeprom::ADDR_MAX) {
        return;
    }

    auto control = services::Control::instance();

    logdata[log_data_idx].fields.velocity_ms = std::min(
        bsp::encoders::get_linear_velocity_m_s() * paramInfoArray[0].scale, (float)paramInfoArray[0].max_store_value);

    logdata[log_data_idx].fields.target_velocity_ms = std::min(
        control->get_target_linear_speed() * paramInfoArray[1].scale, (float)paramInfoArray[1].max_store_value);

    logdata[log_data_idx].fields.angular_speed_rad_s =
        std::min(bsp::imu::get_rad_per_s() * paramInfoArray[2].scale, (float)paramInfoArray[2].max_store_value);

    logdata[log_data_idx].fields.target_rad_s = std::min(control->get_target_angular_speed() * paramInfoArray[3].scale,
                                                         (float)paramInfoArray[3].max_store_value);

    log_data_idx++;
    if (log_data_idx >= (sizeof(logdata) / sizeof(logdata[0]))) {
        log_data_idx = 0;
        bsp::eeprom::write_array(bsp::eeprom::ADDR_START_LOGGER + addr_offset, reinterpret_cast<uint8_t*>(logdata),
                                 sizeof(logdata));

        addr_offset += sizeof(logdata);
    }
}

void Logger::print_log() {

    printf("t;Velocity;TargetVel;AngularVel;TargetAngularVel\r\n");
    bsp::delay_ms(5);

    for (uint16_t i = 0; i < addr_offset; i += sizeof(LogData)) {
        LogData read_logdata;

        bsp::eeprom::read_array(bsp::eeprom::ADDR_START_LOGGER + i, read_logdata.data, sizeof(read_logdata.data));

        uint16_t idx = i / sizeof(LogData);

        printf("%d;%0.4f;%0.4f;%0.4f;%0.4f\r\n",idx, read_logdata.fields.velocity_ms / paramInfoArray[0].scale,
               read_logdata.fields.target_velocity_ms / paramInfoArray[1].scale,
               read_logdata.fields.angular_speed_rad_s / paramInfoArray[2].scale,
               read_logdata.fields.target_rad_s / paramInfoArray[3].scale);

        bsp::delay_ms(3);
    }
}

}
