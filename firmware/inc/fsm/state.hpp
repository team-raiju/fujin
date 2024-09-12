#pragma once

#include "fsm/event.hpp"

namespace fsm {

class FSM;

class State {
public:
    State(FSM* fsm) : fsm(fsm) {}

    virtual void enter() {}
    virtual void exit() {}

    State* react(Event const&) {
        return nullptr;
    };

    virtual State* react(BleCommand const&) {
        return nullptr;
    };

    virtual State* react(ButtonPressed const&) {
        return nullptr;
    };

    virtual State* react(Timeout const&) {
        return nullptr;
    };

private:
    FSM* fsm;
};

class Idle : public State {
public:
    void enter() override;

    State* react(BleCommand const&) override;
    State* react(ButtonPressed const&) override;
    State* react(Timeout const&) override;
};

}
