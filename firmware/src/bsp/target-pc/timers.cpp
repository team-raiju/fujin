#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>

#include "bsp/timers.hpp"
#include "amaterasu/amaterasu.hpp"
#include "services/config.hpp"

namespace bsp {

namespace timers {
void advance_sim_tick(void);
void reset_sim_time(void);
}

using timer = std::chrono::steady_clock;
static std::chrono::time_point start = timer::now();
static std::atomic<uint64_t> sim_ticks{0};

/// @section Interface implementation

void timers::init(void) {}

void timers::advance_sim_tick(void) {
    sim_ticks.fetch_add(1, std::memory_order_relaxed);
}

void timers::reset_sim_time(void) {
    sim_ticks.store(0, std::memory_order_relaxed);
    start = timer::now();
}

uint32_t get_tick_ms(void) {
    if (amaterasu::is_connected() && amaterasu::get_mode() == amaterasu::SimMode::LOCKSTEP) {
        return static_cast<uint32_t>(sim_ticks.load(std::memory_order_relaxed) * 1000 / services::Config::CONTROL_FREQUENCY_HZ);
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(timer::now() - start).count();
}

uint32_t get_tick_us(void) {
    if (amaterasu::is_connected() && amaterasu::get_mode() == amaterasu::SimMode::LOCKSTEP) {
        return static_cast<uint32_t>(sim_ticks.load(std::memory_order_relaxed) * 1000000 / services::Config::CONTROL_FREQUENCY_HZ);
    }
    return std::chrono::duration_cast<std::chrono::microseconds>(timer::now() - start).count();
}

void delay_ms(uint32_t ms) {
    if (ms == 0) {
        std::this_thread::yield();
        return;
    }
    if (amaterasu::is_connected() && amaterasu::get_mode() == amaterasu::SimMode::LOCKSTEP) {
        uint32_t start_sim = get_tick_ms();
        while ((get_tick_ms() - start_sim) < ms) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delay_us(uint32_t us) {
    if (us == 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        return;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} // namespace bsp
