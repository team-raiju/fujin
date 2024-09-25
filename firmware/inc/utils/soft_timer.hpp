#pragma once

#include <cstdint>
#include <functional>


namespace soft_timer {

/// @section Custom types

typedef enum {
    SINGLE,
    CONTINUOUS,
} timer_type_t;


/// @brief callback function timer expire
typedef std::function<void()> SoftTimerCallback;

void start(uint32_t timeout_ms, timer_type_t timer_type);
void stop(void);
void tick(void);
void register_callback(SoftTimerCallback callback);

}

