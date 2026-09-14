#pragma once

#include <array>
#include <iostream>
#include <utility>

#include "algorithms/pid.hpp"
#include "bsp/analog_sensors.hpp"
#include "fsm/event.hpp"
#include "services/logger.hpp"
#include "services/maze.hpp"
#include "services/navigation.hpp"
#include "services/notification.hpp"

namespace fsm {

class FSM;

class State {
public:
    template <typename S>
    static constexpr S& get() {
        static_assert(std::is_base_of<State, S>::value, "State::get() can only be called on States");

        static S instance;
        return instance;
    }

    virtual void enter() {}
    virtual void exit() {}

    virtual State* react(BleCommand const&) { return nullptr; };

    virtual State* react(UsbCommand const&) { return nullptr; };

    virtual State* react(ButtonPressed const&) { return nullptr; };

    virtual State* react(Timeout const&) { return nullptr; };
};

/// @section Idle States
class Idle : public State {
public:
    void enter() override;
    void exit() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;
};

/// @section Search States
class PreSearch : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
};

class SearchWaitStart : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;
};

class SearchParamSelect : public State {
public:
    SearchParamSelect();

    void enter() override;

    State* react(ButtonPressed const&) override;

private:
    enum param_type_t {
        PARAM_CUSTOM,
        PARAM_SLOW,
        PARAM_MEDIUM,
        PARAM_FAST,
    };

    param_type_t param_type;
    services::Navigation* navigation;
};

class SearchExploreModeSelect : public State {
public:
    SearchExploreModeSelect();

    void enter() override;

    State* react(ButtonPressed const&) override;

private:
    enum explore_type_t {
        EXPLORE_NORMAL,
        EXPLORE_FULL
    };

    explore_type_t explore_type;
    services::Navigation* navigation;
};

class Search : public State {
public:
    Search();

    void enter() override;
    void exit() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;

private:
    services::Navigation* navigation;
    services::Notification* notification;
    services::Maze* maze;
    bool returning;
    Point target;
    bool save_maze;
    bool stop_next_move;
    bool emergency = false;
};

/// @section Run States
class PreRun : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
};

class RunWaitStart : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;
};

class Run : public State {
public:
    Run();

    void enter() override;
    void exit() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;

private:
    services::Navigation* navigation;
    services::Maze* maze;
    services::Logger* logger;
    std::vector<Direction> target_directions;
    std::vector<std::pair<Movement, uint8_t>> target_movements;
    uint32_t move_count = 0;
    bool emergency = false;
};

class RunParamSelect : public State {
public:
    RunParamSelect();

    void enter() override;

    State* react(ButtonPressed const&) override;

private:
    enum param_type_t {
        PARAM_CUSTOM,
        PARAM_SLOW,
        PARAM_MEDIUM,
        PARAM_FAST,
        PARAM_SUPER,
    };

    param_type_t param_type;
    services::Navigation* navigation;
};

class RunMoveModeSelect : public State {
public:
    RunMoveModeSelect();

    void enter() override;

    State* react(ButtonPressed const&) override;

private:
};

class RunMapSelect : public State {
public:
    RunMapSelect();

    void enter() override;

    State* react(ButtonPressed const&) override;

private:
};

/// @section Calib States
class PreCalib : public State {
public:
    PreCalib();

    void enter() override;
    void exit() override;

    State* react(ButtonPressed const&) override;
};

class CalibrationModeSelect : public State {
public:
    CalibrationModeSelect();

    void enter() override;

    State* react(ButtonPressed const&) override;

private:
    enum calibration_mode_t { IR_CALIBRATION, IMU_CALIBRATION, FAN_CALIBRATION, MOTORS_CALIBRATION };

    calibration_mode_t calibration_mode;
};

class CalibrationIRSensors : public State {
public:
    struct CalibSample {
        float distance = 0.0f;
        uint32_t raw_adc = 0;
        bool recorded = false;
    };

    CalibrationIRSensors();

    void enter() override;
    void exit() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;

    static void send_calib_params();
    static void send_wall_patterns(uint8_t pattern_idx = 0xFF);

private:
    services::Notification* notification;
    CalibSample sample_p1[4];
    CalibSample sample_p2[4];

    void handle_calib_sample(const uint8_t packet[bsp::ble::max_packet_size]);
    void handle_reset_calib(uint8_t sensor_target);
    void send_calib_ack(uint8_t sensor_idx, uint8_t point_id, float dist, uint32_t raw_adc, uint8_t status);
    void handle_wall_pattern_calib(const uint8_t packet[bsp::ble::max_packet_size]);
    void send_wall_pattern_ack(uint8_t pattern_idx, uint8_t status, const bsp::analog_sensors::SensingPattern& pattern);
    uint32_t read_averaged_adc(bsp::analog_sensors::SensingDirection direction);
    bool solve_2point_calib(uint8_t sensor_idx);
};

class CalibrationIMU : public State {
public:
    CalibrationIMU();

    void enter() override;
    void exit() override;

    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;

private:
    int loop_counter;
};

class CalibrationFan : public State {
public:
    CalibrationFan();

    void enter() override;
    void exit() override;

    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;

private:
    int loop_counter;
};

class CalibrationMotors : public State {
public:
    CalibrationMotors();

    void enter() override;
    void exit() override;

    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;

private:
    int loop_counter;
    float distance_traveled_mm;
};
}
