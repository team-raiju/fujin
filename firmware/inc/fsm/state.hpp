#pragma once

#include <iostream>

#include "fsm/event.hpp"

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

class Search : public State {
public:
    void enter() override;
    void exit() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;
private:
    typedef enum {
        STRAIGHT,
        START_TURN,
        TURN_RIGHT,
        TURN_LEFT,
    } search_state_t;

    float last_error;
    search_state_t search_state;
    int counter;
    int stop_counter;
    bool last_empty_wall_right;

};

class PreRun : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
};

/// @section Run States

}
