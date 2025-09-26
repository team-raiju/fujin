#pragma once

#include <array>
#include <iostream>
#include <utility>

#include "algorithms/pid.hpp"
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
        PARAM_MEDIUM ,
        PARAM_FAST,
    };

    param_type_t param_type;
    services::Navigation* navigation;

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
        PARAM_MEDIUM ,
        PARAM_FAST,
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

/// @section Search States

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
    bool save_maze;
    bool stop_next_move;
    bool emergency = false;
};

///

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
    enum calibration_mode_t {
        IR_CALIBRATION,
        IMU_CALIBRATION,
        FAN_CALIBRATION
    };

    calibration_mode_t calibration_mode;
};

class CalibrationIRSensors : public State {
public:
    CalibrationIRSensors();

    void enter() override;
    void exit() override;

    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;

private:
    services::Notification* notification;
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
}
