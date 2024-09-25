#include "utils/soft_timer.hpp"

namespace soft_timer {

static uint32_t counter;
static uint32_t reload_value;
static bool running;
static timer_type_t type;
static SoftTimerCallback external_callback = NULL;


void register_callback(SoftTimerCallback callback) {
    external_callback = callback;
}

void start(uint32_t timeout_ms, timer_type_t timer_type) {
    counter = timeout_ms;
    reload_value = timeout_ms;
    running = true;
    type = timer_type;
}

void stop(void) {
    running = false;
    counter = 0;
}

void tick(void) {
    if (running) {
        if (counter > 0) {
            counter--;
        } 
        
        if (counter == 0) {
            if (type == CONTINUOUS) {
                counter = reload_value;
            } else {
                stop();
            }

            if (external_callback) {
                external_callback();
            }
        }
    }
}

}
