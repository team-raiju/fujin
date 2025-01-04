#pragma once

#include <array>
#include <iostream>
#include <utility>

#include "algorithms/pid.hpp"
#include "fsm/event.hpp"
#include "services/maze.hpp"
#include "services/navigation.hpp"
#include "services/notification.hpp"
#include "services/logger.hpp"

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

class PreSearch : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;
};

class PreRun : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;
};

/// @section Run States

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
    std::array<std::pair<Movement, uint8_t>, 256> target_movements;
    int move_count = 0;
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
};

///

class PreCalib : public State {
public:
    PreCalib();

    void enter() override;
    void exit() override;

    State* react(ButtonPressed const&) override;
};

class Calib : public State {
public:
    Calib();

    void enter() override;
    void exit() override;

    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;

private:
    services::Notification* notification;
};

}
