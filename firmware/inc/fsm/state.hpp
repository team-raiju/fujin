#pragma once

#include <iostream>

#include "algorithms/pid.hpp"
#include "fsm/event.hpp"
#include "services/maze.hpp"

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

    virtual State* react(BleCommand const&) {
        return nullptr;
    };

    virtual State* react(UsbCommand const&) {
        return nullptr;
    };

    virtual State* react(ButtonPressed const&) {
        return nullptr;
    };

    virtual State* react(Timeout const&) {
        return nullptr;
    };

};

/// @section Idle States
class Idle : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;
};

class PreSearch : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
};

class PreRun : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
};

/// @section Search States

class Search : public State {
public:
    Search() : angular_vel_pid(0.0f, 0.0f, 0.0f, 0.0f),
                walls_pid(0.0f, 0.0f, 0.0f, 0.0f){}
    void enter() override;
    void exit() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;

private:
    int stop_counter;
    algorithm::PID angular_vel_pid;
    algorithm::PID walls_pid;
    Movement current_movement;
    uint8_t movement_state;
    float target_speed;
    float rotation_ratio;
    float target_travel;
};

class SearchFront : public Search {
public:
    void enter() override;
    void exit() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;
};

}
