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

/// @section Constants

/// @section Service implementation

namespace services {

const Logger::ParamInfo paramInfoArray[] = {
    {4095, -5, 10, 270.0f},     // velocity_ms
    {4095, -5, 10, 270.0f},     // target_velocity_ms
    {4095, -70, 70, 29.0f},     // angular_speed_rad_s
    {4095, -70, 70, 29.0f},     // target_rad_s
    {255, -1000, 1000, 0.127f}, // pwm_left
    {255, -1000, 1000, 0.127f}, // pwm_right
    {255, 0, 13000, 0.0195f},   // battery
    {255, -100, 100, 1.274f},   // integral_vel
    {255, -500, 500, 0.254f},   // integral_angular
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

    float param_velocity_ms = bsp::encoders::get_filtered_velocity_m_s() + std::abs(paramInfoArray[0].min_param_value);
    param_velocity_ms *= paramInfoArray[0].scale;

    float param_target_velocity_ms = control->get_target_linear_speed() + std::abs(paramInfoArray[1].min_param_value);
    param_target_velocity_ms *= paramInfoArray[1].scale;

    float param_angular_speed_rad_s = bsp::imu::get_rad_per_s() + std::abs(paramInfoArray[2].min_param_value);
    param_angular_speed_rad_s *= paramInfoArray[2].scale;

    float param_target_rad_s = control->get_target_angular_speed() + std::abs(paramInfoArray[3].min_param_value);
    param_target_rad_s *= paramInfoArray[3].scale;

    float pwm_left = control->get_pwm_duty_l() + std::abs(paramInfoArray[4].min_param_value);
    pwm_left *= paramInfoArray[4].scale;

    float pwm_right = control->get_pwm_duty_r() + std::abs(paramInfoArray[5].min_param_value);
    pwm_right *= paramInfoArray[5].scale;

    float battery = bsp::analog_sensors::battery_latest_reading_mv() + std::abs(paramInfoArray[6].min_param_value);
    battery *= paramInfoArray[6].scale;

    float integral_vel = control->get_integral_vel() + std::abs(paramInfoArray[7].min_param_value);
    integral_vel *= paramInfoArray[7].scale;

    float integral_angular = control->get_integral_angular() + std::abs(paramInfoArray[8].min_param_value);
    integral_angular *= paramInfoArray[8].scale;

    logdata[log_data_idx].fields.velocity_ms = std::min(param_velocity_ms, (float)paramInfoArray[0].max_store_value);
    logdata[log_data_idx].fields.target_velocity_ms =
        std::min(param_target_velocity_ms, (float)paramInfoArray[1].max_store_value);
    logdata[log_data_idx].fields.angular_speed_rad_s =
        std::min(param_angular_speed_rad_s, (float)paramInfoArray[2].max_store_value);
    logdata[log_data_idx].fields.target_rad_s = std::min(param_target_rad_s, (float)paramInfoArray[3].max_store_value);
    logdata[log_data_idx].fields.pwm_left = std::min(pwm_left, (float)paramInfoArray[4].max_store_value);
    logdata[log_data_idx].fields.pwm_right = std::min(pwm_right, (float)paramInfoArray[5].max_store_value);
    logdata[log_data_idx].fields.battery = std::min(battery, (float)paramInfoArray[6].max_store_value);
    logdata[log_data_idx].fields.integral_vel = std::min(integral_vel, (float)paramInfoArray[7].max_store_value);
    logdata[log_data_idx].fields.integral_angular =
        std::min(integral_angular, (float)paramInfoArray[8].max_store_value);

    log_data_idx++;
    if (log_data_idx >= (sizeof(logdata) / sizeof(logdata[0]))) {
        log_data_idx = 0;

        memcpy(ram_logger + addr_offset, reinterpret_cast<uint8_t*>(logdata), sizeof(logdata));

        addr_offset += sizeof(logdata);
    }
}

void Logger::print_log() {

    std::printf("t;Vel;TargetVel;AngVel;TargetAngrVel;PWM_L;PWM_R,Bat,IntVel,IntW\r\n");
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
            "%d;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f\r\n", idx,
            (read_logdata.fields.velocity_ms / paramInfoArray[0].scale) - std::abs(paramInfoArray[0].min_param_value),
            (read_logdata.fields.target_velocity_ms / paramInfoArray[1].scale) -
                std::abs(paramInfoArray[1].min_param_value),
            (read_logdata.fields.angular_speed_rad_s / paramInfoArray[2].scale) -
                std::abs(paramInfoArray[2].min_param_value),
            (read_logdata.fields.target_rad_s / paramInfoArray[3].scale) - std::abs(paramInfoArray[3].min_param_value),
            (read_logdata.fields.pwm_left / paramInfoArray[4].scale) - std::abs(paramInfoArray[4].min_param_value),
            (read_logdata.fields.pwm_right / paramInfoArray[5].scale) - std::abs(paramInfoArray[5].min_param_value),
            (read_logdata.fields.battery / paramInfoArray[6].scale) - std::abs(paramInfoArray[6].min_param_value),
            (read_logdata.fields.integral_vel / paramInfoArray[7].scale) - std::abs(paramInfoArray[7].min_param_value),
            (read_logdata.fields.integral_angular / paramInfoArray[8].scale) -
                std::abs(paramInfoArray[8].min_param_value));

        bsp::delay_ms(3);
    }
}

}
