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

/// @section Run States

}