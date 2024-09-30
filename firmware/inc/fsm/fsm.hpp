#pragma once

#include "fsm/event.hpp"
#include "fsm/state.hpp"
#include "services/maze.hpp"
#include "services/navigation.hpp"
#include "utils/RingBuffer.hpp"

namespace fsm {

class FSM {
public:
    FSM();

    void start();
    void spin();

    void dispatch(Event const& event);

private:
    State* current_state;
    RingBuffer<Event, 16> event_queue;

    services::Navigation* navigation_service;
    services::Maze* maze_service;
};

}
