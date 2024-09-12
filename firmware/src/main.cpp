#include "bsp/core.hpp"
#include "fsm/fsm.hpp"

int main() {
    bsp::init();

    fsm::FSM fsm;

    fsm.start();

    for (;;) {
        fsm.spin();
    }
}
