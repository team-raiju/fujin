#include <algorithm>
#include <cmath>
#include <cstring>

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/debug.hpp"
#include "bsp/encoders.hpp"
#include "bsp/fan.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "fsm/state.hpp"
#include "services/config.hpp"
#include "utils/soft_timer.hpp"

namespace fsm {

PreCalib::PreCalib() {}

void PreCalib::enter() {
    using bsp::leds::Color;

    bsp::debug::print("state:PreCalib");

    bsp::leds::stripe_set(Color::Purple);

    bsp::motors::set(0, 0);
}

State* PreCalib::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreRun>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<Idle>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<CalibrationModeSelect>();
    }

    if (event.button == ButtonPressed::LONG6) {
        return &State::get<CalibrationIRSensors>();
    }

    return nullptr;
}

void PreCalib::exit() {}

CalibrationModeSelect::CalibrationModeSelect() {
    calibration_mode = IR_CALIBRATION;
}

void CalibrationModeSelect::enter() {
    bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
    calibration_mode = IR_CALIBRATION;
}

State* CalibrationModeSelect::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT2) {
        if (calibration_mode == IR_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Blue);
            calibration_mode = IR_DISTANCE_CALIBRATION;
        } else if (calibration_mode == IR_DISTANCE_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Orange);
            calibration_mode = IMU_CALIBRATION;
        } else if (calibration_mode == IMU_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Black);
            calibration_mode = FAN_CALIBRATION;
        } else if (calibration_mode == FAN_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Orange);
            calibration_mode = MOTORS_CALIBRATION;
        } else {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
            calibration_mode = IR_CALIBRATION;
        }
        return nullptr;
    }

    if (event.button == ButtonPressed::SHORT1) {
        if (calibration_mode == IR_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Orange);
            calibration_mode = MOTORS_CALIBRATION;
        } else if (calibration_mode == MOTORS_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Orange, bsp::leds::Color::Black);
            calibration_mode = FAN_CALIBRATION;
        } else if (calibration_mode == FAN_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Orange);
            calibration_mode = IMU_CALIBRATION;
        } else if (calibration_mode == IMU_CALIBRATION) {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Blue);
            calibration_mode = IR_DISTANCE_CALIBRATION;
        } else {
            bsp::leds::stripe_set(bsp::leds::Color::Blue, bsp::leds::Color::Black);
            calibration_mode = IR_CALIBRATION;
        }
        return nullptr;
    }

    if (event.button == ButtonPressed::LONG1) {
        if (calibration_mode == IR_CALIBRATION) {
            return &State::get<CalibrationIRSensors>();
        } else if (calibration_mode == IR_DISTANCE_CALIBRATION) {
            return &State::get<CalibrationIRDistance>();
        } else if (calibration_mode == IMU_CALIBRATION) {
            return &State::get<CalibrationIMU>();
        } else if (calibration_mode == FAN_CALIBRATION) {
            return &State::get<CalibrationFan>();
        } else {
            return &State::get<CalibrationMotors>();
        }
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreCalib>();
    }

    return nullptr;
}

CalibrationIRSensors::CalibrationIRSensors() {
    notification = services::Notification::instance();
}

void CalibrationIRSensors::enter() {
    bsp::debug::print("state:CalibrationIRSensors");

    bsp::leds::stripe_set(bsp::leds::Color::Black);

    bsp::leds::ir_emitter_all_on();
    bsp::analog_sensors::enable_modulation();

    bsp::motors::set(0, 0);

    for (int i = 0; i < 4; i++) {
        sample_p1[i] = CalibSample();
        sample_p2[i] = CalibSample();
    }

    bsp::buzzer::beep(150);

    bsp::ble::unlock_config_rcv();

    notification->reset();
    soft_timer::start(10, soft_timer::CONTINUOUS);

    send_calib_params();
    bsp::delay_ms(20);
    send_wall_patterns();
}

State* CalibrationIRSensors::react(BleCommand const& event) {
    if (event.packet[1] == bsp::ble::BlePacketType::RequestIrCalibParams) {
        send_calib_params();
    } else if (event.packet[1] == bsp::ble::BlePacketType::CalibrateIrSample) {
        uint8_t point_id = event.packet[3];
        if (point_id == 0) {
            handle_reset_calib(event.packet[2]);
        } else {
            handle_calib_sample(event.packet);
        }
    } else if (event.packet[1] == bsp::ble::BlePacketType::RequestIrWallPatterns) {
        send_wall_patterns(event.packet[2]);
    } else if (event.packet[1] == bsp::ble::BlePacketType::CalibrateIrWallPattern) {
        handle_wall_pattern_calib(event.packet);
    } else if (event.packet[1] == bsp::ble::BlePacketType::RequestParameters) {
        services::Config::send_parameters();
    } else if (event.packet[1] == bsp::ble::BlePacketType::UpdateParameters) {
        bsp::buzzer::beep(60);
    }
    return nullptr;
}

State* CalibrationIRSensors::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::LONG2 || event.button == ButtonPressed::SHORT1 ||
        event.button == ButtonPressed::SHORT2) {
        return &State::get<Idle>();
    }
    return nullptr;
}

State* CalibrationIRSensors::react(Timeout const&) {
    notification->update(true);
    return nullptr;
}

void CalibrationIRSensors::exit() {
    soft_timer::stop();
    bsp::ble::lock_config_rcv();
    bsp::analog_sensors::enable_modulation(false);
    bsp::leds::ir_emitter_all_off();
}

void CalibrationIRSensors::send_calib_params() {
    uint8_t packet[15] = {0};
    packet[0] = bsp::ble::header;
    packet[1] = bsp::ble::BlePacketType::RequestIrCalibParams;

    for (uint8_t i = 0; i < 4; i++) {
        auto params = bsp::analog_sensors::get_calib_params(static_cast<bsp::analog_sensors::SensingDirection>(i));
        packet[2] = i;
        std::memcpy(&packet[3], &params.a, sizeof(float));
        std::memcpy(&packet[7], &params.b, sizeof(float));
        std::memcpy(&packet[11], &params.c, sizeof(float));

        bsp::ble::transmit(packet, sizeof(packet));
        bsp::delay_ms(5);
    }
}

uint32_t CalibrationIRSensors::read_averaged_adc(bsp::analog_sensors::SensingDirection direction) {
    uint32_t sum = 0;
    uint32_t count = 0;
    uint32_t start_tick = bsp::get_tick_ms();
    while (bsp::get_tick_ms() - start_tick < 50) {
        sum += bsp::analog_sensors::ir_raw_reading(direction);
        count++;
        bsp::delay_ms(2);
    }
    return count > 0 ? (sum / count) : bsp::analog_sensors::ir_raw_reading(direction);
}

float CalibrationIRSensors::read_averaged_distance(bsp::analog_sensors::SensingDirection direction) {
    float sum = 0;
    uint32_t count = 0;
    uint32_t start_tick = bsp::get_tick_ms();
    while (bsp::get_tick_ms() - start_tick < 50) {
        sum += bsp::analog_sensors::ir_distance_mm(direction);
        count++;
        bsp::delay_ms(2);
    }
    return count > 0 ? (sum / (float)count) : bsp::analog_sensors::ir_distance_mm(direction);
}

bool CalibrationIRSensors::solve_2point_calib(uint8_t sensor_idx) {
    if (sensor_idx >= 4) {
        return false;
    }

    if (!sample_p1[sensor_idx].recorded || !sample_p2[sensor_idx].recorded) {
        return false;
    }

    float d1 = sample_p1[sensor_idx].distance;
    float r1 = static_cast<float>(sample_p1[sensor_idx].raw_adc);
    float d2 = sample_p2[sensor_idx].distance;
    float r2 = static_cast<float>(sample_p2[sensor_idx].raw_adc);

    if (std::abs(d2 - d1) < 1.0f) {
        return false;
    }

    auto dir = static_cast<bsp::analog_sensors::SensingDirection>(sensor_idx);
    auto current_params = bsp::analog_sensors::get_calib_params(dir);
    float c = current_params.c;

    float u1 = 1.0f / std::log(std::max(r1 + c, 2.0f));
    float u2 = 1.0f / std::log(std::max(r2 + c, 2.0f));
    float denom = u2 - u1;

    if (std::abs(denom) < 1e-6f){
        return false;
    }

    float a = (d2 - d1) / denom;
    float b = (a * u1) - d1;

    if (a <= 0.0f) {
        return false;
    }

    bsp::analog_sensors::IrCalibParams new_params = {a, b, c};
    bsp::analog_sensors::set_calib_params(dir, new_params);
    services::Config::save_ir_calib_to_eeprom(dir);

    return true;
}

void CalibrationIRSensors::handle_reset_calib(uint8_t sensor_target) {
    if (sensor_target < 4) {
        auto dir = static_cast<bsp::analog_sensors::SensingDirection>(sensor_target);
        bsp::analog_sensors::reset_calib_params(dir);
        services::Config::save_ir_calib_to_eeprom(dir);
        sample_p1[sensor_target] = CalibSample();
        sample_p2[sensor_target] = CalibSample();
    } else {
        bsp::analog_sensors::reset_all_calib_params();
        services::Config::save_all_ir_calib_to_eeprom();
        for (int i = 0; i < 4; i++) {
            sample_p1[i] = CalibSample();
            sample_p2[i] = CalibSample();
        }
    }
    bsp::buzzer::beep(100);
    send_calib_params();
}

void CalibrationIRSensors::send_calib_ack(uint8_t sensor_idx, uint8_t point_id, float dist, uint32_t raw_adc,
                                          uint8_t status) {
    uint8_t ack[13] = {0};
    ack[0] = bsp::ble::header;
    ack[1] = bsp::ble::BlePacketType::CalibrateIrSample;
    ack[2] = sensor_idx;
    ack[3] = point_id;
    std::memcpy(&ack[4], &dist, sizeof(float));
    std::memcpy(&ack[8], &raw_adc, sizeof(uint32_t));
    ack[12] = status;

    bsp::ble::transmit(ack, sizeof(ack));
    bsp::delay_ms(5);
}

void CalibrationIRSensors::handle_calib_sample(const uint8_t packet[bsp::ble::max_packet_size]) {
    uint8_t sensor_target = packet[2]; // 0: RIGHT, 1: FRONT_LEFT, 2: FRONT_RIGHT, 3: LEFT
    if (sensor_target >= 4) {
        return;
    }

    uint8_t point_id = packet[3]; // 1: Point 1, 2: Point 2
    if (point_id != 1 && point_id != 2) {
        return;
    }

    float dist = 0.0f;
    std::memcpy(&dist, &packet[4], sizeof(float));

    auto dir = static_cast<bsp::analog_sensors::SensingDirection>(sensor_target);
    uint32_t raw = read_averaged_adc(dir);

    if (point_id == 1) {
        sample_p1[sensor_target] = {.distance = dist, .raw_adc = raw, .recorded = true};
        send_calib_ack(sensor_target, 1, dist, raw, 0);
        bsp::buzzer::beep(80);
    } else if (point_id == 2) {
        sample_p2[sensor_target] = {.distance = dist, .raw_adc = raw, .recorded = true};
        bool success = solve_2point_calib(sensor_target);
        send_calib_ack(sensor_target, 2, dist, raw, success ? 0 : 1);

        if (success) {
            bsp::buzzer::beep_double(100, 80, 150);
            send_calib_params();
        } else {
            for (int b = 0; b < 3; b++) {
                bsp::buzzer::beep(60);
                bsp::delay_ms(60);
            }
        }
    }
}

void CalibrationIRSensors::send_wall_patterns(uint8_t pattern_idx) {
    uint8_t packet[20] = {0};
    packet[0] = bsp::ble::header;
    packet[1] = bsp::ble::BlePacketType::RequestIrWallPatterns;

    uint8_t start = (pattern_idx < 8) ? pattern_idx : 0;
    uint8_t end = (pattern_idx < 8) ? (pattern_idx + 1) : 8;

    for (uint8_t i = start; i < end; i++) {
        auto pattern = bsp::analog_sensors::get_wall_pattern(i);
        packet[2] = i;
        std::memcpy(&packet[3], &pattern.L, sizeof(float));
        std::memcpy(&packet[7], &pattern.FL, sizeof(float));
        std::memcpy(&packet[11], &pattern.FR, sizeof(float));
        std::memcpy(&packet[15], &pattern.R, sizeof(float));

        bsp::ble::transmit(packet, sizeof(packet));
        bsp::delay_ms(5);
    }
}

void CalibrationIRSensors::send_wall_pattern_ack(uint8_t pattern_idx, uint8_t status, const bsp::analog_sensors::SensingPattern& pattern) {
    uint8_t packet[20] = {0};
    packet[0] = bsp::ble::header;
    packet[1] = bsp::ble::BlePacketType::CalibrateIrWallPattern;
    packet[2] = pattern_idx;
    packet[3] = status;

    std::memcpy(&packet[4], &pattern.L, sizeof(float));
    std::memcpy(&packet[8], &pattern.FL, sizeof(float));
    std::memcpy(&packet[12], &pattern.FR, sizeof(float));
    std::memcpy(&packet[16], &pattern.R, sizeof(float));

    bsp::ble::transmit(packet, sizeof(packet));
}

void CalibrationIRSensors::handle_wall_pattern_calib(const uint8_t packet[bsp::ble::max_packet_size]) {
    uint8_t pattern_idx = packet[2];
    uint8_t action = packet[3]; // 1 = sample & save, 0 = reset

    if (action == 1) {
        if (pattern_idx >= 8) {
            bsp::analog_sensors::SensingPattern empty = {0, 0, 0, 0};
            send_wall_pattern_ack(pattern_idx, 1, empty);
            return;
        }

        float l = read_averaged_distance(bsp::analog_sensors::LEFT);
        float fl = read_averaged_distance(bsp::analog_sensors::FRONT_LEFT);
        float fr = read_averaged_distance(bsp::analog_sensors::FRONT_RIGHT);
        float r = read_averaged_distance(bsp::analog_sensors::RIGHT);

        bsp::analog_sensors::SensingPattern new_pattern = {l, fl, fr, r};
        bsp::analog_sensors::set_wall_pattern(pattern_idx, new_pattern);
        int res = services::Config::save_ir_wall_pattern_to_eeprom(pattern_idx);

        if (res == 0) {
            bsp::buzzer::beep_double(100, 80, 150);
            send_wall_pattern_ack(pattern_idx, 0, new_pattern);
        } else {
            bsp::buzzer::beep(300);
            send_wall_pattern_ack(pattern_idx, 1, new_pattern);
        }
    } else if (action == 0) {
        if (pattern_idx == 0xFF) {
            services::Config::reset_ir_wall_patterns_in_eeprom();
            bsp::buzzer::beep(100);
            send_wall_patterns(0xFF);
        } else if (pattern_idx < 8) {
            bsp::analog_sensors::reset_wall_pattern(pattern_idx);
            services::Config::save_ir_wall_pattern_to_eeprom(pattern_idx);
            bsp::buzzer::beep(100);
            auto pat = bsp::analog_sensors::get_wall_pattern(pattern_idx);
            send_wall_pattern_ack(pattern_idx, 0, pat);
        }
    }
}

CalibrationIRDistance::CalibrationIRDistance() {
    step = WAITING_PLACEMENT;
    current_distance_mm = 10;
    sample_count = 0;
    sum_raw_l = 0;
    sum_raw_fl = 0;
    sum_raw_fr = 0;
    sum_raw_r = 0;
}

void CalibrationIRDistance::print_prompt() {
    std::printf("\r\n========================================\r\n");
    bsp::delay_ms(5);
    std::printf(">>> Place micromouse at %lu mm <<<\r\n", static_cast<unsigned long>(current_distance_mm));
    bsp::delay_ms(5);
    std::printf("Press button to start 5s raw ADC sampling (Long press to exit)\r\n");
    bsp::delay_ms(5);
    std::printf("========================================\r\n");
    bsp::delay_ms(5);
}

void CalibrationIRDistance::enter() {
    bsp::debug::print("state:CalibrationIRDistance");

    bsp::leds::stripe_set(bsp::leds::Color::Orange);
    bsp::leds::ir_emitter_all_on();
    bsp::analog_sensors::enable_modulation();
    bsp::motors::set(0, 0);

    bsp::buzzer::start();
    bsp::delay_ms(150);
    bsp::buzzer::stop();

    step = WAITING_PLACEMENT;
    current_distance_mm = 10;
    sample_count = 0;
    sum_raw_l = 0;
    sum_raw_fl = 0;
    sum_raw_fr = 0;
    sum_raw_r = 0;

    print_prompt();
    soft_timer::start(services::Config::ms_to_ticks(100), soft_timer::CONTINUOUS);
}

State* CalibrationIRDistance::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::LONG1 || event.button == ButtonPressed::LONG2) {
        std::printf("\r\nCalibration aborted by user.\r\n");
        return &State::get<PreCalib>();
    }

    if (step == WAITING_PLACEMENT &&
        (event.button == ButtonPressed::SHORT1 || event.button == ButtonPressed::SHORT2)) {
        bsp::buzzer::start();
        bsp::delay_ms(60);
        bsp::buzzer::stop();

        bsp::leds::stripe_set(bsp::leds::Color::Green);

        step = SAMPLING;
        sample_count = 0;
        sum_raw_l = 0;
        sum_raw_fl = 0;
        sum_raw_fr = 0;
        sum_raw_r = 0;

        std::printf("--- Sampling raw ADC values at %lu mm (5s @ 100ms) ---\r\n",
                    static_cast<unsigned long>(current_distance_mm));
        bsp::delay_ms(5);
        std::printf("Dist_mm,Sample,L,FL,FR,R\r\n");
        bsp::delay_ms(5);
    }

    return nullptr;
}

State* CalibrationIRDistance::react(Timeout const&) {
    if (step != SAMPLING) {
        return nullptr;
    }

    const uint32_t raw_l = bsp::analog_sensors::ir_raw_reading(bsp::analog_sensors::LEFT);
    const uint32_t raw_fl = bsp::analog_sensors::ir_raw_reading(bsp::analog_sensors::FRONT_LEFT);
    const uint32_t raw_fr = bsp::analog_sensors::ir_raw_reading(bsp::analog_sensors::FRONT_RIGHT);
    const uint32_t raw_r = bsp::analog_sensors::ir_raw_reading(bsp::analog_sensors::RIGHT);

    sample_count++;
    sum_raw_l += raw_l;
    sum_raw_fl += raw_fl;
    sum_raw_fr += raw_fr;
    sum_raw_r += raw_r;

    std::printf("%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                static_cast<unsigned long>(current_distance_mm),
                static_cast<unsigned long>(sample_count),
                static_cast<unsigned long>(raw_l),
                static_cast<unsigned long>(raw_fl),
                static_cast<unsigned long>(raw_fr),
                static_cast<unsigned long>(raw_r));

    if (sample_count >= 50) {
        const uint32_t avg_l = sum_raw_l / sample_count;
        const uint32_t avg_fl = sum_raw_fl / sample_count;
        const uint32_t avg_fr = sum_raw_fr / sample_count;
        const uint32_t avg_r = sum_raw_r / sample_count;

        std::printf("--- %lu mm Summary | Avg: L=%lu FL=%lu FR=%lu R=%lu ---\r\n",
                    static_cast<unsigned long>(current_distance_mm),
                    static_cast<unsigned long>(avg_l),
                    static_cast<unsigned long>(avg_fl),
                    static_cast<unsigned long>(avg_fr),
                    static_cast<unsigned long>(avg_r));

        bsp::buzzer::start();
        bsp::delay_ms(100);
        bsp::buzzer::stop();

        if (current_distance_mm < 250) {
            current_distance_mm += 10;
            step = WAITING_PLACEMENT;
            bsp::leds::stripe_set(bsp::leds::Color::Orange);
            print_prompt();
        } else {
            bsp::delay_ms(5);
            std::printf("\r\n========================================\r\n");
            bsp::delay_ms(5);
            std::printf("*** RAW IR SENSOR CALIBRATION COMPLETE (10-250mm) ***\r\n");
            bsp::delay_ms(5);
            std::printf("========================================\r\n");
            bsp::delay_ms(5);
            return &State::get<PreCalib>();
        }
    }

    return nullptr;
}

void CalibrationIRDistance::exit() {
    soft_timer::stop();
    bsp::leds::ir_emitter_all_off();
    bsp::analog_sensors::enable_modulation(false);
    bsp::leds::stripe_set(bsp::leds::Color::Black);
    bsp::buzzer::stop();
    bsp::motors::set(0, 0);
}

CalibrationIMU::CalibrationIMU() {}

void CalibrationIMU::enter() {
    bsp::leds::stripe_set(bsp::leds::Color::Red, bsp::leds::Color::Red);
    bsp::imu::enable_motion_gc_filter(true);
    soft_timer::start(1, soft_timer::CONTINUOUS);
    bsp::imu::reset_angle();
    loop_counter = 0;
}

State* CalibrationIMU::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreCalib>();
    }

    return nullptr;
}

State* CalibrationIMU::react(Timeout const&) {
    if (loop_counter > 20000) {
        soft_timer::stop();
        return &State::get<PreCalib>();
    }

    loop_counter++;
    if (loop_counter % 400 == 0) {
        std::printf("bias: %f; Angle: %f; Loop: %d\r\n", bsp::imu::get_g_bias_z(), bsp::imu::get_angle(), loop_counter);
    }

    bsp::imu::update();

    return nullptr;
}

void CalibrationIMU::exit() {
    soft_timer::stop();
    bsp::motors::set(0, 0);

    services::Config::z_imu_bias = bsp::imu::get_g_bias_z();
    std::printf("saving z_imu_bias: %f\r\n", services::Config::z_imu_bias);
    services::Config::save_z_bias();
    bsp::imu::enable_motion_gc_filter(false);
}

CalibrationFan::CalibrationFan() {}

void CalibrationFan::enter() {
    bsp::leds::stripe_set(bsp::leds::Color::Red, bsp::leds::Color::Red);
    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    services::Control::instance()->init();
    services::Control::instance()->start_fan();
    bsp::motors::set(0, 0);

    soft_timer::start(1, soft_timer::CONTINUOUS);
    loop_counter = 0;
}

State* CalibrationFan::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::SHORT2) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::LONG1) {
        return &State::get<PreCalib>();
    }

    if (event.button == ButtonPressed::LONG2) {
        return &State::get<PreCalib>();
    }

    return nullptr;
}

State* CalibrationFan::react(Timeout const&) {
    if (loop_counter++ > 6000) {
        soft_timer::stop();
        return &State::get<PreCalib>();
    }

    return nullptr;
}

void CalibrationFan::exit() {
    soft_timer::stop();
    bsp::motors::set(0, 0);

    services::Control::instance()->stop_fan();
    bsp::fan::set(0);
}

CalibrationMotors::CalibrationMotors() {}

void CalibrationMotors::enter() {
    bsp::debug::print("state:CalibrationMotors");
    bsp::motors::set(0, 0);
    bsp::leds::stripe_set(bsp::leds::Color::Red, bsp::leds::Color::Red);
    bsp::buzzer::start();
    bsp::delay_ms(2000);
    bsp::buzzer::stop();

    loop_counter = 0;
    distance_traveled_mm = 0.0f;
    bsp::encoders::reset();
    soft_timer::start(1, soft_timer::CONTINUOUS);
}

State* CalibrationMotors::react(ButtonPressed const& event) {
    if (event.button == ButtonPressed::SHORT1) {
        bsp::debug::print("motors(50, 50)");
        bsp::motors::set(50, 50);
        return nullptr;
    }

    if (event.button == ButtonPressed::SHORT2) {
        bsp::debug::print("motors(-50, -50)");
        bsp::motors::set(-50, -50);
        return nullptr;
    }

    if (event.button == ButtonPressed::LONG2 || event.button == ButtonPressed::LONG1) {
        return &State::get<PreCalib>();
    }

    return nullptr;
}

State* CalibrationMotors::react(Timeout const&) {
    loop_counter++;

    bsp::encoders::update_velocities();

    auto left_encoder = bsp::encoders::get_data(bsp::encoders::EncoderSide::LEFT);
    auto right_encoder = bsp::encoders::get_data(bsp::encoders::EncoderSide::RIGHT);

    float estimated_delta_l_mm = (left_encoder.ticks * bsp::encoders::get_encoder_dist_mm_pulse());
    float estimated_delta_r_mm = (right_encoder.ticks * bsp::encoders::get_encoder_dist_mm_pulse());

    float delta_x_mm = (estimated_delta_l_mm + estimated_delta_r_mm) / 2.0f;
    distance_traveled_mm += delta_x_mm;
    // bsp::encoders::clear_ticks();

    if (loop_counter % 100 == 0) {
        // std::printf("Vel: %f m/s; Dist: %f mm\r\n", bsp::encoders::get_linear_velocity_m_s(), distance_traveled_mm);
        std::printf("ticks: (%ld, %ld)\r\n", left_encoder.ticks, right_encoder.ticks);
    }

    return nullptr;
}

void CalibrationMotors::exit() {
    soft_timer::stop();
    bsp::motors::set(0, 0);
}

}
