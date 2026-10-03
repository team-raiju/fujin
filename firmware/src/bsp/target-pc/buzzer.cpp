#include "bsp/buzzer.hpp"
#include "amaterasu/amaterasu.hpp"

namespace bsp::buzzer {

static uint16_t current_freq = 0;
static uint8_t current_vol = 0;
static bool is_running = false;

void init() {
    stop();
}

void start(void) {
    is_running = true;
    amaterasu::set_buzzer(current_freq, current_vol);
}

void stop(void) {
    is_running = false;
    amaterasu::set_buzzer(0, 0);
}

void set_volume(uint8_t volume) {
    current_vol = volume;
    if (is_running) {
        amaterasu::set_buzzer(current_freq, current_vol);
    }
}

void set_frequency(uint16_t frequency) {
    current_freq = frequency;
    if (is_running) {
        amaterasu::set_buzzer(current_freq, current_vol);
    }
}

void beep(uint32_t) {}
void beep_double(uint32_t, uint32_t, uint32_t) {}

} // namespace bsp::buzzer
