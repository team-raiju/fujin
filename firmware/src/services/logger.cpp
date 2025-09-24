#include "utils/math.hpp"
#include <array>
#include <cstdio>
#include <cstring>

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
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
    {255, -1000, 1000, 0.127f}, // pwm_left
    {255, -1000, 1000, 0.127f}, // pwm_right
    {255, 0, 13000, 0.0195f},   // battery
    {65535, -250, 250, 131.0f}, // position_mm_x
    {65535, -250, 250, 131.0f}, // position_mm_y
    {65535, -180, 180, 182.0f}, // angle
    {65535, -50, 1260, 50.0f},  // distance_cm
};

// This prevents bugs if a new parameter is added to one but not the other.
static_assert(static_cast<size_t>(ParamIndex::COUNT) == std::size(paramInfoArray));

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

float Logger::encode_value(float raw_value, const ParamInfo& info) const {
    float processed_value = (raw_value - info.min_param_value) * info.scale;
    return std::min(processed_value, static_cast<float>(info.max_store_value));
}

float Logger::decode_value(float stored_value, const ParamInfo& info) const {
    return (stored_value / info.scale) + info.min_param_value;
}

void Logger::update() {

    if (addr_offset + sizeof(logdata) >= sizeof(ram_logger)) {
        return;
    }

    auto control = services::Control::instance();
    auto nav = services::Navigation::instance();
    auto& current_log_entry = logdata[log_data_idx].fields;

    current_log_entry.velocity_ms = encode_value(bsp::encoders::get_filtered_velocity_m_s(),
                                                 paramInfoArray[static_cast<size_t>(ParamIndex::VelocityMS)]);
    current_log_entry.target_velocity_ms = encode_value(
        control->get_target_linear_speed(), paramInfoArray[static_cast<size_t>(ParamIndex::TargetVelocityMS)]);
    current_log_entry.angular_speed_rad_s =
        encode_value(bsp::imu::get_rad_per_s(), paramInfoArray[static_cast<size_t>(ParamIndex::AngularSpeedRadS)]);
    current_log_entry.target_rad_s =
        encode_value(control->get_target_angular_speed(), paramInfoArray[static_cast<size_t>(ParamIndex::TargetRadS)]);
    current_log_entry.pwm_left =
        encode_value(control->get_pwm_duty_l(), paramInfoArray[static_cast<size_t>(ParamIndex::PwmLeft)]);
    current_log_entry.pwm_right =
        encode_value(control->get_pwm_duty_r(), paramInfoArray[static_cast<size_t>(ParamIndex::PwmRight)]);
    current_log_entry.battery = encode_value(bsp::analog_sensors::battery_latest_reading_mv(),
                                             paramInfoArray[static_cast<size_t>(ParamIndex::Battery)]);
    current_log_entry.position_mm_x =
        encode_value(nav->get_robot_position_mm().x, paramInfoArray[static_cast<size_t>(ParamIndex::PositionX)]);
    current_log_entry.position_mm_y =
        encode_value(nav->get_robot_position_mm().y, paramInfoArray[static_cast<size_t>(ParamIndex::PositionY)]);
    const float angle_deg = bsp::imu::get_angle() * ((180.0f / M_PI));
    current_log_entry.angle = encode_value(angle_deg, paramInfoArray[static_cast<size_t>(ParamIndex::Angle)]);

    current_log_entry.distance =
        encode_value(nav->get_robot_travelled_dist_cm(), paramInfoArray[static_cast<size_t>(ParamIndex::Distance)]);

    // --- Buffer Management ---
    log_data_idx++;
    if (log_data_idx >= std::size(logdata)) {
        log_data_idx = 0;
        memcpy(ram_logger + addr_offset, reinterpret_cast<uint8_t*>(logdata), sizeof(logdata));
        addr_offset += sizeof(logdata);
    }
}

void Logger::print_log() {
    std::printf("t;Vel;TgtVel;AngVel;TgtAngVel;PWM_L;PWM_R;Batt_mV;PosX;PosY;Angle;Dist\r\n");
    bsp::delay_ms(5);

    uint32_t saved_size = addr_offset;

    for (uint16_t i = 0; i < saved_size; i += sizeof(LogData)) {
        LogData read_logdata;
        if (i + sizeof(read_logdata.data) > sizeof(ram_logger)) {
            break;
        }

        memcpy(read_logdata.data, ram_logger + i, sizeof(read_logdata.data));
        uint16_t idx = i / sizeof(LogData);

        std::printf(
            "%d;%0.4f;%0.4f;%0.4f;%0.4f;%0.f;%0.f;%0.f;%0.4f;%0.4f;%0.4f;%0.4f\r\n", idx,
            decode_value(read_logdata.fields.velocity_ms, paramInfoArray[static_cast<size_t>(ParamIndex::VelocityMS)]),
            decode_value(read_logdata.fields.target_velocity_ms,
                         paramInfoArray[static_cast<size_t>(ParamIndex::TargetVelocityMS)]),
            decode_value(read_logdata.fields.angular_speed_rad_s,
                         paramInfoArray[static_cast<size_t>(ParamIndex::AngularSpeedRadS)]),
            decode_value(read_logdata.fields.target_rad_s, paramInfoArray[static_cast<size_t>(ParamIndex::TargetRadS)]),
            decode_value(read_logdata.fields.pwm_left, paramInfoArray[static_cast<size_t>(ParamIndex::PwmLeft)]),
            decode_value(read_logdata.fields.pwm_right, paramInfoArray[static_cast<size_t>(ParamIndex::PwmRight)]),
            decode_value(read_logdata.fields.battery, paramInfoArray[static_cast<size_t>(ParamIndex::Battery)]),
            decode_value(read_logdata.fields.position_mm_x, paramInfoArray[static_cast<size_t>(ParamIndex::PositionX)]),
            decode_value(read_logdata.fields.position_mm_y, paramInfoArray[static_cast<size_t>(ParamIndex::PositionY)]),
            decode_value(read_logdata.fields.angle, paramInfoArray[static_cast<size_t>(ParamIndex::Angle)]),
            decode_value(read_logdata.fields.distance, paramInfoArray[static_cast<size_t>(ParamIndex::Distance)]));

        bsp::delay_ms(3);
    }
}

void Logger::send_log_ble() {
    uint32_t saved_size = addr_offset;
    uint8_t packet[19] = {0};
    packet[0] = bsp::ble::header;
    packet[1] = bsp::ble::BlePacketType::RequestLogData;

    for (uint16_t i = 0; i < saved_size; i += sizeof(LogData)) {
        LogData read_logdata;
        if (i + sizeof(read_logdata.data) > sizeof(ram_logger)) {
            break;
        }

        memcpy(packet + 2, ram_logger + i, sizeof(read_logdata.data));

        bsp::ble::transmit(packet, sizeof(packet));
        bsp::delay_ms(5);
    }

}
}
