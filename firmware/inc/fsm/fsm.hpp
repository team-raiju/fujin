#pragma once

#include "fsm/state.hpp"

namespace fsm {

class FSM {
public:
    FSM();

    void start();
    void spin();

private:
    State* current_state;
};

}
